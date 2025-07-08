/*
 * Copyright (c) 2025 Michael Estes
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kinetic_ktd202x

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/util.h>

#define KTD202x_REG_EN_RESET 0x00
#define KTD202X_REG_LED_EN   0x04
#define KTD202X_REG_CURRENT  0x06

#define KTD202X_MAX_LEDS   4
#define KTD202X_RESET_MASK GENMASK(2, 0)

LOG_MODULE_REGISTER(ktd202x, CONFIG_LED_LOG_LEVEL);

struct ktd202x_cfg {
	struct i2c_dt_spec i2c;
};

struct ktd202x_data {
	uint8_t enable;
};

static int ktd202x_write_buffer(const struct i2c_dt_spec *i2c, const uint8_t *buffer,
				uint32_t num_bytes)
{
	int status;

	status = i2c_write_dt(i2c, buffer, num_bytes);
	if (status < 0) {
		LOG_ERR("Could not write buffer: %i", status);
		return status;
	}

	return 0;
}

static int ktd202x_write_reg(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t val)
{
	uint8_t buffer[2] = {reg, val};

	return ktd202x_write_buffer(i2c, buffer, sizeof(buffer));
}

static uint8_t ktd202x_brightness_to_current(uint8_t brightness)
{
	return (0xBFU * brightness) / LED_BRIGHTNESS_MAX;
}

static int ktd202x_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	const struct ktd202x_cfg *config = dev->config;
	struct ktd202x_data *data = dev->data;
	uint8_t current;
	int status;

	if (led > KTD202X_MAX_LEDS - 1) {
		return -EINVAL;
	}

	WRITE_BIT(data->enable, led * 2, value != 0);
	status = ktd202x_write_reg(&config->i2c, KTD202X_REG_LED_EN, data->enable);

	if (status != 0) {
		return status;
	}

	current = ktd202x_brightness_to_current(value);
	status = ktd202x_write_reg(&config->i2c, KTD202X_REG_CURRENT + led, current);
	return status;
}

static int ktd202x_init(const struct device *dev)
{
	const struct ktd202x_cfg *config = dev->config;

	LOG_DBG("Initializing @0x%x...", config->i2c.addr);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* NACK expected - ignore return value */
	ktd202x_write_reg(&config->i2c, KTD202x_REG_EN_RESET, KTD202X_RESET_MASK);
	k_sleep(K_USEC(200));

	return 0;
}

static DEVICE_API(led, ktd202x_led_api) = {
	.set_brightness = ktd202x_led_set_brightness,
};

#define KTD202X_INIT(id)                                                                           \
	static const struct ktd202x_cfg ktd202x_##id##_cfg = {                                     \
		.i2c = I2C_DT_SPEC_INST_GET(id),                                                   \
	};                                                                                         \
	static struct ktd202x_data ktd202x_##id##_data = {0};                                      \
	DEVICE_DT_INST_DEFINE(id, &ktd202x_init, NULL, &ktd202x_##id##_data, &ktd202x_##id##_cfg,  \
			      POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &ktd202x_led_api);

DT_INST_FOREACH_STATUS_OKAY(KTD202X_INIT)
