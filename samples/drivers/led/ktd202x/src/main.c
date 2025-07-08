/*
 * Copyright (c) 2025 Michael Estes
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define NUM_LEDS        3
#define HALF_BRIGHTNESS (LED_BRIGHTNESS_MAX / 2)
#define BLINK_DELAY_ON  500
#define BLINK_DELAY_OFF 500
#define DELAY_TIME_MS   1000
#define DELAY_TIME      K_MSEC(DELAY_TIME_MS)

int main(void)
{
	const struct device *const led_dev = DEVICE_DT_GET_ANY(kinetic_ktd202x);
	int i, ret;

	if (!led_dev) {
		LOG_ERR("No devices with compatible kinetic,ktd202x found");
		return 0;
	} else if (!device_is_ready(led_dev)) {
		LOG_ERR("LED device %s is not ready", led_dev->name);
		return 0;
	}

	LOG_INF("Found LED device %s", led_dev->name);
	LOG_INF("Testing leds");

	while (1) {
		/* Turn on LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			ret = led_on(led_dev, i);
			if (ret < 0) {
				return 0;
			}

			k_sleep(DELAY_TIME);
		}

		/* Turn off LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			ret = led_off(led_dev, i);
			if (ret < 0) {
				return 0;
			}

			k_sleep(DELAY_TIME);
		}

		/* Set the brightness to half max of LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			ret = led_set_brightness(led_dev, i, HALF_BRIGHTNESS);
			if (ret < 0) {
				return 0;
			}

			k_sleep(DELAY_TIME);
		}

		/* Turn off LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			ret = led_off(led_dev, i);
			if (ret < 0) {
				return 0;
			}

			k_sleep(DELAY_TIME);
		}
	}
	return 0;
}
