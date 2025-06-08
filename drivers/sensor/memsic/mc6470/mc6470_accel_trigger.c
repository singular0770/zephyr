/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 Michael Estes
 */

#include "mc6470_accel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(MC6470, CONFIG_SENSOR_LOG_LEVEL);

static void mc6470_gpio_callback(const struct device *dev,
				 struct gpio_callback *cb,
				 uint32_t pin_mask)
{
	struct mc6470_driver_data *data = CONTAINER_OF(cb,
					  struct mc6470_driver_data, gpio_cb);

	const struct mc6470_config *cfg = data->gpio_dev->config;

	if ((pin_mask & BIT(cfg->irq_gpio.pin)) == 0U) {
		return;
	}

#if defined(CONFIG_MC6470_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#else
	k_work_submit(&data->work);
#endif
}

static void mc6470_process_int(const struct device *dev)
{
	int ret = 0;
	const struct mc6470_config *cfg = dev->config;
	const struct mc6470_driver_data *data = dev->data;
	uint8_t int_source = 0;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, MC6470_REG_INT_STATUS, &int_source);
	if (ret < 0) {
		LOG_ERR("Failed to read interrupt (%d)", ret);
		return;
	}

	if (int_source & MC6470_ACQ_INT_MASK) {
		if (data->drdy_handler) {
			data->drdy_handler(dev, data->drdy_trig);
		}
	}

#ifdef CONFIG_MC6470_TAP_TRIGGER
	if (int_source & (MC6470_TAP_X_MASK | MC6470_TAP_Y_MASK | MC6470_TAP_Z_MASK)) {
		if (data->tap_handler) {
			data->tap_handler(dev, data->tap_trig);
		}
	}
#endif /* CONFIG_MC6470_TAP_TRIGGER */
}

#if defined(CONFIG_MC6470_TRIGGER_OWN_THREAD)
static void mc6470_thread(struct mc6470_driver_data *data)
{
	while (1) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		mc6470_process_int(data->gpio_dev);
	}
}
#else
static void mc6470_work_cb(struct k_work *work)
{
	struct mc6470_driver_data *data = CONTAINER_OF(work,
					  struct mc6470_driver_data, work);

	mc6470_process_int(data->gpio_dev);
}
#endif

int mc6470_configure_trigger(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler)
{
	int ret = 0;
	uint8_t buf = 0;
	uint8_t reg = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	switch (trig->type) {
		case SENSOR_TRIG_DATA_READY:
			data->drdy_handler = handler;
			data->drdy_trig = trig;
			buf |= MC6470_ACQ_INT_MASK;
			reg = MC6470_REG_ACQ_INT_EN;
			break;
#ifdef CONFIG_MC6470_TAP_TRIGGER
		case SENSOR_TRIG_TAP:
			data->tap_handler = handler;
			data->tap_trig = trig;
			if (trig->chan == SENSOR_CHAN_ACCEL_X) {
				buf |= MC6470_TAP_X_MASK;
			} else if (trig->chan == SENSOR_CHAN_ACCEL_Y) {
				buf |= MC6470_TAP_Y_MASK;
			} else if (trig->chan == SENSOR_CHAN_ACCEL_Z) {
				buf |= MC6470_TAP_Z_MASK;
			} else {
				buf |= (MC6470_TAP_X_MASK | MC6470_TAP_Y_MASK | MC6470_TAP_Z_MASK);
			}
			buf |= MC6470_TAP_EN_MASK;
			reg = MC6470_REG_TAPEN;
			break;
#endif /* CONFIG_MC6470_TAP_TRIGGER */
		default:
			LOG_ERR("Unsupported sensor trigger");
			ret = -ENOTSUP;
			break;
	}

	ret = i2c_reg_update_byte_dt(&cfg->i2c, reg,
				     buf, buf);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt (%d)", ret);
		return ret;
	}

	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_FALLING);

	/* Clear pending interrupts */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, MC6470_REG_INT_STATUS, NULL);

	return ret;
}

int mc6470_trigger_init(const struct device *dev)
{
	int ret = 0;
	struct mc6470_driver_data *data = dev->data;
	const struct mc6470_config *cfg = dev->config;

	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("GPIO port %s not ready", cfg->irq_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt gpio");
		return ret;
	}

	data->gpio_dev = dev;

#if defined(CONFIG_MC6470_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, 1);
	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_mc6470_THREAD_STACK_SIZE,
			(k_thread_entry_t)mc6470_thread, data, NULL,
			NULL, K_PRIO_COOP(CONFIG_mc6470_THREAD_PRIORITY), 0,
			K_NO_WAIT);
#else
	k_work_init(&data->work, mc6470_work_cb);
#endif
	gpio_init_callback(&data->gpio_cb, mc6470_gpio_callback,
			   BIT(cfg->irq_gpio.pin));
	ret = gpio_add_callback(cfg->irq_gpio.port, &data->gpio_cb);

	if (ret < 0) {
		LOG_ERR("Failed to set int callback");
		return ret;
	}

	return 0;
}
