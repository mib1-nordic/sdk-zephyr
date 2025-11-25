/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#include <nrfx_twis.h>

#include <hal/nrf_twim.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_timer.h>

#define FORCE_INLINE __attribute__((always_inline)) inline
#define NEVER_INLINE __attribute__ ((noinline))

#define TWIM_DEV DT_NODELABEL(dut_twim)
#define STOPWATCH_DEV DT_NODELABEL(stopwatch)

extern volatile uint32_t g_reg_writes;
extern volatile uint32_t g_reg_reads;

#define TWIS_INST_IDX 22

#define TWIS_ADDR 42

#define TEST_DATA_SIZE 1

static const uint8_t msg[TEST_DATA_SIZE] = {'N'};
static nrfx_twis_t twis = NRFX_TWIS_INSTANCE(NRF_TWIS_INST_GET(TWIS_INST_IDX));

static uint8_t i2c_slave_buffer[TEST_DATA_SIZE];
static uint8_t i2c_master_buffer[TEST_DATA_SIZE];

static const struct device * twim_dev = DEVICE_DT_GET(TWIM_DEV);

static void i2c_slave_handler(nrfx_twis_event_t const *p_event)
{
	switch (p_event->type) {
	case NRFX_TWIS_EVT_READ_REQ:
		nrfx_twis_tx_prepare(&twis, i2c_slave_buffer, TEST_DATA_SIZE);
		break;
	case NRFX_TWIS_EVT_READ_DONE:
		break;
	case NRFX_TWIS_EVT_WRITE_REQ:
		nrfx_twis_rx_prepare(&twis, i2c_slave_buffer, TEST_DATA_SIZE);
		break;
	case NRFX_TWIS_EVT_WRITE_DONE:
		break;
	default:
		break;
	}
}

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

static void NEVER_INLINE raw_i2c_reg_writes_in_second(void)
{
    LOG_INF("Raw register write start at %llu\n", k_uptime_get());

    NRF_TWIM_Type * p_reg = (NRF_TWIM_Type *)DT_REG_ADDR(TWIM_DEV);

    raw_timer_start(UINT32_MAX);

#define RAW_REG_WRITE_CNT 1000
#define RAW_REG_WRITER(i, _) nrf_twim_address_set(p_reg, TWIS_ADDR)
    LISTIFY(RAW_REG_WRITE_CNT, RAW_REG_WRITER, (;));

    uint32_t cc = raw_timer_capture();
    uint32_t us = cc / 16;
    uint32_t op = (cc * 1000) / (RAW_REG_WRITE_CNT * 16);

    LOG_INF("Raw register write end at %llu. Time needed for %u ops = %u [us]. One op = %u [ns]\n",
		           k_uptime_get(), RAW_REG_WRITE_CNT, us, op);
}

static void NEVER_INLINE raw_i2c_reg_reads_in_second(void)
{
    LOG_INF("Raw register read start at %llu\n", k_uptime_get());

    NRF_TWIM_Type * p_reg = (NRF_TWIM_Type *)DT_REG_ADDR(TWIM_DEV);

    raw_timer_start(UINT32_MAX);

#define RAW_REG_READ_CNT 1000
#define RAW_REG_READER(i, _) nrf_twim_shorts_get(p_reg)
    LISTIFY(RAW_REG_READ_CNT, RAW_REG_READER, (;));

    uint32_t cc = raw_timer_capture();
    uint32_t us = cc / 16;
    uint32_t op = (cc * 1000) / (RAW_REG_READ_CNT * 16);

    LOG_INF("Raw register read end at %llu. Time needed for %u ops = %u [us]. One op = %u [ns]\n",
            k_uptime_get(), RAW_REG_READ_CNT, us, op);
}

static void twis_setup(void)
{
	static nrfx_twis_config_t twis_config = NRFX_TWIS_DEFAULT_CONFIG(NRF_GPIO_PIN_MAP(1,13),
           		                                                     NRF_GPIO_PIN_MAP(1,11),
                                                                     TWIS_ADDR);

	int ret = nrfx_twis_init(&twis, &twis_config, i2c_slave_handler);
	if (ret < 0) {
		printf("TWIS initialization failed! code: %d\n", ret);
		return;
	}

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TWIS_INST_GET(TWIS_INST_IDX)), IRQ_PRIO_LOWEST,
		        nrfx_twis_irq_handler, &twis, 0);

	memcpy(i2c_slave_buffer, msg, TEST_DATA_SIZE);
	nrfx_twis_enable(&twis);
}

// static void twis_uninit(void) {
// 	nrfx_twis_error_get_and_clear();
// }

static void zephyr_reg_access_i2c_write_read(void)
{
	uint32_t reg_writes_start = g_reg_writes;
	uint32_t reg_reads_start = g_reg_reads;

	i2c_write_read(twim_dev, TWIS_ADDR, msg, TEST_DATA_SIZE, i2c_master_buffer, TEST_DATA_SIZE);

	LOG_INF(	"reg writes: %d, reg reads: %d\n", g_reg_writes - reg_writes_start, g_reg_reads - reg_reads_start);
}

static void zephyr_reg_access_i2c_read(void)
{
	/* Prepare slave data */
	memcpy(i2c_slave_buffer, msg, TEST_DATA_SIZE);

	uint32_t reg_writes_start = g_reg_writes;
	uint32_t reg_reads_start = g_reg_reads;

	i2c_read(twim_dev, i2c_master_buffer, TEST_DATA_SIZE, TWIS_ADDR);

	LOG_INF("reg writes: %d, reg reads: %d\n", g_reg_writes - reg_writes_start, g_reg_reads - reg_reads_start);
}

static void zephyr_reg_access_i2c_write(void)
{
	uint32_t reg_writes_start = g_reg_writes;
	uint32_t reg_reads_start = g_reg_reads;

	i2c_write(twim_dev, msg, TEST_DATA_SIZE, TWIS_ADDR);

	LOG_INF("reg writes: %d, reg reads: %d\n", g_reg_writes - reg_writes_start, g_reg_reads - reg_reads_start);
}

static void twis_reenable() {
	nrfx_twis_disable(&twis);
	nrfx_twis_enable(&twis);
}

int main(void)
{
	raw_timer_configure();
	raw_i2c_reg_writes_in_second();
	raw_i2c_reg_reads_in_second();

	// Uncomment for explicit PM
	(void)pm_device_runtime_get(twim_dev);

	twis_setup();
	// zephyr_reg_access_i2c_write_read();
	// twis_reenable();
	zephyr_reg_access_i2c_read();
	// twis_reenable();
	// zephyr_reg_access_i2c_write();
	(void)pm_device_runtime_put(twim_dev);
}
