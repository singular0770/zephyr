/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 Michael Estes
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MC6470_H_
#define ZEPHYR_DRIVERS_SENSOR_MC6470_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Registers */
#define MC6470_REG_INT_STATUS	0x03
#define MC6470_REG_OPSTAT 		0x04
#define MC6470_REG_ACQ_INT_EN	0x06
#define MC6470_REG_MODE			0x07
#define MC6470_REG_SRTFR		0x08
#define MC6470_REG_TAPEN		0x09
#define MC6470_REG_TTTRX		0x0A
#define MC6470_REG_TTTRY		0x0B
#define MC6470_REG_TTTRZ		0x0C
#define MC6470_REG_XOUT_L		0x0D
#define MC6470_REG_YOUT_L		0x0F
#define MC6470_REG_ZOUT_L		0x11
#define MC6470_REG_OUTCFG		0x20

#define MC6470_THRDUR_MASK		BIT(6)
#define MC6470_ACQ_INT_MASK		BIT(7)
#define MC6470_TAP_EN_MASK		BIT(7)
#define MC6470_TAP_Z_MASK		GENMASK(5, 4)
#define MC6470_TAP_Y_MASK		GENMASK(3, 2)
#define MC6470_TAP_X_MASK		GENMASK(1, 0)
#define MC6470_RANGE_MASK		GENMASK(6, 4)
#define MC6470_ODR_MASK			GENMASK(3, 0)
#define MC6470_RES_MASK			GENMASK(2, 0)

#define MC6470_ANY_MOTION_THRESH_MAX	0x7FFF
#define MC6470_SAMPLE_SIZE		3
#define MC6470_SAMPLE_READ_SIZE		(MC6470_SAMPLE_SIZE * (sizeof(int16_t)))

#define SENSOR_GRAIN_VALUE             (61LL / 1000.0)
#define SENSOR_GRAVITY_DOUBLE          (SENSOR_G / 1000000.0)

enum mc6470_trig_types {
	MC6470_TRIG_DATA_READY = 0,
	MC6470_TRIG_TAP,
	MC6470_TRIG_SIZE
};

enum mc6470_op_mode {
	MC6470_MODE_STANDBY = 0x00,
	MC6470_MODE_WAKE = 0x01
};

struct mc6470_odr_map {
	int16_t freq;
	int16_t mfreq;
};

enum mc6470_accl_range {
	MC6470_ACCL_RANGE_2G,
	MC6470_ACCL_RANGE_4G,
	MC6470_ACCL_RANGE_8G,
	MC6470_ACCL_RANGE_16G,
	MC6470_ACCL_RANGE_END
};

struct mc6470_config {
	struct i2c_dt_spec i2c;
#if defined(CONFIG_MC6470_TRIGGER)
	struct gpio_dt_spec irq_gpio;
	bool int_cfg;
#endif
};

struct mc6470_driver_data {
	uint8_t odr;
	uint16_t range;
	uint8_t res;
	struct k_sem sem;
	int16_t samples[MC6470_SAMPLE_SIZE];
#if defined(CONFIG_MC6470_TRIGGER)
	const struct device *gpio_dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trig;
#if defined(CONFIG_MC6470_TAP_TRIGGER)
	sensor_trigger_handler_t tap_handler;
	const struct sensor_trigger *tap_trig;
#endif /* CONFIG_MC6470_TAP_TRIGGER */
#if defined(CONFIG_MC6470_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MC6470_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#else
	struct k_work work;
#endif /* CONFIG_MC6470_TRIGGER_OWN_THREAD */
#endif /* CONFIG_MC6470_TRIGGER */
};

#if defined(CONFIG_MC6470_TRIGGER)
int mc6470_trigger_init(const struct device *dev);
int mc6470_configure_trigger(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler);
#endif /* CONFIG_MC6470_TRIGGER */

#endif
