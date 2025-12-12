/*
 * Copyright (c) 2017 comsuisse AG
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <nrfx.h>

extern uint32_t SystemCoreClock;

#include <hal/nrf_tdm.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_cache.h>

#include <dmm.h>

#define FORCE_INLINE __attribute__((always_inline)) inline
#define NEVER_INLINE __attribute__ ((noinline))

#define WORD_SIZE 16U
#define NUMBER_OF_CHANNELS 2
#define FRAME_CLK_FREQ 44100

#define NUM_BLOCKS 4
#define TIMEOUT 1000

#define SAMPLES_COUNT 64

#define STOPWATCH_DEV DT_NODELABEL(stopwatch)
#define TDM_DEV DT_NODELABEL(dut_tdm)

extern volatile int g_reg_writes;
extern volatile int g_reg_reads;

/* The data_l represent a sine wave */
static int16_t data_l[SAMPLES_COUNT] = {
	  3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
	 25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
	 32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
	 20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
	 -3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
	-25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
	-32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
	-20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
};

/* The data_r represent a sine wave shifted by 90 deg to data_l sine wave */
static int16_t data_r[SAMPLES_COUNT] = {
	 32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
	 20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
	 -3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
	-25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
	-32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
	-20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
	  3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
	 25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
};

#define BLOCK_SIZE (2 * sizeof(data_l))
#ifdef CONFIG_NOCACHE_MEMORY
	#define MEM_SLAB_CACHE_ATTR __nocache
#else
	#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

#define TDM_PROFILING_USE_RAM3X

#if defined(CONFIG_HAS_NORDIC_DMM) && defined(TDM_PROFILING_USE_RAM3X)
	#define MEM_SECTION DMM_MEMORY_SECTION(TDM_DEV)
#else
	#define MEM_SECTION
#endif /* CONFIG_HAS_NORDIC_DMM */
/*
 * NUM_BLOCKS is the number of blocks used by the test. Some of the drivers,
 * permanently keep ownership of a few RX buffers. Add a two more
 * RX blocks to satisfy this requirement
 */
static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_rx_0_mem_slab[(NUM_BLOCKS + 2) * WB_UP(BLOCK_SIZE)];
STRUCT_SECTION_ITERABLE(k_mem_slab, rx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(rx_0_mem_slab, _k_mem_slab_buf_rx_0_mem_slab,
				WB_UP(BLOCK_SIZE), NUM_BLOCKS + 2);

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)] MEM_SECTION;
STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
				WB_UP(BLOCK_SIZE), NUM_BLOCKS);

static const struct device *dev_i2s = DEVICE_DT_GET_OR_NULL(TDM_DEV);

static struct i2s_config i2s_cfg = {
	.word_size = WORD_SIZE,
	.channels = NUMBER_OF_CHANNELS,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.frame_clk_freq = FRAME_CLK_FREQ,
	.block_size = BLOCK_SIZE,
	.timeout = TIMEOUT,
	.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
	.mem_slab = &tx_0_mem_slab,
};

static void raw_timer_configure(void)
{
    NRF_TIMER_Type * p_reg = (NRF_TIMER_Type *)DT_REG_ADDR(STOPWATCH_DEV);

    nrf_timer_prescaler_set(p_reg, 0); // 16 MHz timer
    nrf_timer_mode_set(p_reg, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(p_reg, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_shorts_set(p_reg, NRF_TIMER_SHORT_COMPARE0_STOP_MASK |
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    nrf_timer_int_disable(p_reg, UINT32_MAX);
}

static bool FORCE_INLINE raw_timer_cc_check(void)
{
    NRF_TIMER_Type * p_reg = (NRF_TIMER_Type *)DT_REG_ADDR(STOPWATCH_DEV);

    return nrf_timer_event_check(p_reg, NRF_TIMER_EVENT_COMPARE0);
}

static void FORCE_INLINE raw_timer_start(uint32_t cc)
{
    NRF_TIMER_Type * p_reg = (NRF_TIMER_Type *)DT_REG_ADDR(STOPWATCH_DEV);

    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_CLEAR);

    nrf_timer_event_clear(p_reg, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_cc_set(p_reg, NRF_TIMER_CC_CHANNEL0, cc);

    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_START);
}

static uint32_t FORCE_INLINE raw_timer_capture(void)
{
    NRF_TIMER_Type * p_reg = (NRF_TIMER_Type *)DT_REG_ADDR(STOPWATCH_DEV);

    nrf_timer_task_trigger(p_reg, NRF_TIMER_TASK_CAPTURE1);
    return nrf_timer_cc_get(p_reg, NRF_TIMER_CC_CHANNEL1);
}

static void NEVER_INLINE raw_tdm_reg_writes_in_second(void)
{
//     LOG_INF("Raw register write start at %llu\n", k_uptime_get());

    NRF_TDM_Type * p_reg = (NRF_TDM_Type *)DT_REG_ADDR(TDM_DEV);

    raw_timer_start(UINT32_MAX);

#define RAW_REG_WRITE_CNT 1000
#define RAW_REG_WRITER(i, _) nrf_tdm_rx_buffer_set(p_reg, NULL)
    LISTIFY(RAW_REG_WRITE_CNT, RAW_REG_WRITER, (;));

    uint32_t cc = raw_timer_capture();
    uint32_t us = cc / 16;
    uint32_t op = (cc * 1000) / (RAW_REG_WRITE_CNT * 16);

    LOG_INF("Raw register write end at %llu. Time needed for %u ops = %u [us]. One op = %u [ns]\n",
		           k_uptime_get(), RAW_REG_WRITE_CNT, us, op);
}

static void NEVER_INLINE raw_tdm_reg_reads_in_second(void)
{
//     LOG_INF("Raw register read start at %llu\n", k_uptime_get());

    NRF_TDM_Type * p_reg = (NRF_TDM_Type *)DT_REG_ADDR(TDM_DEV);

    raw_timer_start(UINT32_MAX);

#define RAW_REG_READ_CNT 1000
#define RAW_REG_READER(i, _) nrf_tdm_rx_buffer_get(p_reg)
    LISTIFY(RAW_REG_READ_CNT, RAW_REG_READER, (;));

    uint32_t cc = raw_timer_capture();
    uint32_t us = cc / 16;
    uint32_t op = (cc * 1000) / (RAW_REG_READ_CNT * 16);

    // LOG_INF("Raw register read end at %llu. Time needed for %u ops = %u [us]. One op = %u [ns]\n",
    //         k_uptime_get(), RAW_REG_READ_CNT, us, op);
}

/* Fill in TX buffer with test samples. */
static void fill_buf(int16_t *tx_block, uint8_t word_size)
{
	for (int i = 0; i < SAMPLES_COUNT; i++) {
		tx_block[2 * i] = data_l[i];
		tx_block[2 * i + 1] = data_r[i];
	}
}

static int verify_buf(int16_t *rx_block, uint8_t word_size, uint8_t channels)
{
	int sample_no = SAMPLES_COUNT;
	bool same = true;

	/* Compare received data with sent values. */
	for (int i = 0; i < sample_no; i++) {
		if (rx_block[2 * i] != data_l[i]) {
			LOG_WRN("data_l, index %d, expected 0x%x, actual 0x%x\n",
					i, data_l[i], rx_block[2 * i]);
			same = false;
		}
		if (rx_block[2 * i + 1] != data_r[i]) {
			LOG_WRN("data_r, index %d, expected 0x%x, actual 0x%x\n",
					i, data_r[i], rx_block[2 * i + 1]);
			same = false;
		}
	}

	if (!same) {
		return -1;
	} else {
		return 0;
	}
}

static void configure_stream(const struct device *dev, enum i2s_dir dir, struct i2s_config *i2s_cfg)
{
	if (dir == I2S_DIR_TX) {
		/* Configure the Transmit port as Master */
		i2s_cfg->options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	} else if (dir == I2S_DIR_RX) {
		/* Configure the Receive port as Slave */
		i2s_cfg->options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
	} else { /* dir == I2S_DIR_BOTH */
		i2s_cfg->options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	}

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		i2s_cfg->mem_slab = &tx_0_mem_slab;
		int ret = i2s_configure(dev, I2S_DIR_TX, i2s_cfg);
		if (ret < 0) {
			LOG_ERR("Failed to configure I2S TX stream (%d)", ret);
			return;
		}
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		i2s_cfg->mem_slab = &rx_0_mem_slab;
		int ret = i2s_configure(dev, I2S_DIR_RX, i2s_cfg);
		if (ret < 0) {
			LOG_ERR("Failed to configure I2S RX stream (%d)", ret);
			return;
		}
	}
}

static void zephyr_i2s_single_write(void) {


	void *tx_block;
	/* Allocate a slab for TX and fill it with samples. */
	k_mem_slab_alloc(&tx_0_mem_slab, &tx_block, K_FOREVER);
	fill_buf(tx_block, i2s_cfg.word_size);

	/* Configure I2S_DIR_TX transfer. */
	configure_stream(dev_i2s, I2S_DIR_TX, &i2s_cfg);

	/* I2S write */
	int ret = i2s_write(dev_i2s, tx_block, BLOCK_SIZE);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	/* I2S trigger */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	LOG_INF("writes: %d, reads: %d", g_reg_writes, g_reg_reads);
}

static void zephyr_i2s_transfer_dir_both(int num_blocks) {
	/* Configure I2S Dir Both transfer. */
	configure_stream(dev_i2s, I2S_DIR_BOTH, &i2s_cfg);

	void *rx_block[NUM_BLOCKS];
	void *tx_block[NUM_BLOCKS];

	/* Allocate a slab for TX and fill it with samples. */
	for (int tx_idx = 0; tx_idx < num_blocks; tx_idx++) {
		k_mem_slab_alloc(&tx_0_mem_slab, &tx_block[tx_idx], K_FOREVER);
		fill_buf((uint16_t *)tx_block[tx_idx], i2s_cfg.word_size);
	}

	int ret = i2s_write(dev_i2s, tx_block[0], BLOCK_SIZE);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	ret = i2s_write(dev_i2s, tx_block[1], BLOCK_SIZE);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	/* I2S trigger */
	ret = i2s_trigger(dev_i2s, I2S_DIR_BOTH, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	size_t rx_size;

	for (int tx_idx = 2, rx_idx = 0; tx_idx < num_blocks; tx_idx++, rx_idx++) {
		/* I2S write */
		ret = i2s_write(dev_i2s, tx_block[tx_idx], BLOCK_SIZE);
		if (ret < 0) {
			LOG_ERR("Error: %d", ret);
		}
		ret = i2s_read(dev_i2s, &rx_block[rx_idx], &rx_size);
		if (ret < 0) {
			LOG_ERR("Error: %d", ret);
		}
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_BOTH, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}


	ret = i2s_read(dev_i2s, &rx_block[num_blocks - 2], &rx_size);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	ret = i2s_read(dev_i2s, &rx_block[num_blocks - 1], &rx_size);
	if (ret < 0) {
		LOG_ERR("Error: %d", ret);
	}

	/* Free the RX slab. */
	for (int rx_idx = 0; rx_idx < num_blocks; rx_idx++) {
		k_mem_slab_free(&rx_0_mem_slab, rx_block[rx_idx]);
	}

	// LOG_INF("writes: %d, reads: %d", g_reg_writes, g_reg_reads);
}

int main(void)
{
	// LOG_INF("SystemCoreClock: %d", SystemCoreClock);

	raw_timer_configure();
	raw_tdm_reg_reads_in_second();
	raw_tdm_reg_writes_in_second();

	zephyr_i2s_single_write();
	k_msleep(1000);
	zephyr_i2s_transfer_dir_both(3);

	LOG_INF("writes: %d, reads: %d", g_reg_writes, g_reg_reads);

}
