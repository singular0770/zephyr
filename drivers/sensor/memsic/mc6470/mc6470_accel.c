/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 Michael Estes
 */

#define DT_DRV_COMPAT memsic_mc6470_accel

#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mc6470_accel.h"

LOG_MODULE_REGISTER(MC6470, CONFIG_SENSOR_LOG_LEVEL);

static struct mc6470_odr_map mc6470_odr_map[] = {
	{32, 0},  /* 32 Hz   */
	{16, 0},  /* 16 Hz   */
	{8, 0},   /* 8 Hz    */
	{4, 0},   /* 4 Hz    */
	{2, 0},   /* 2 Hz    */
	{1, 0},   /* 1 Hz    */
	{0, 500}, /* 0.5 Hz  */
	{0, 250}, /* 0.25 Hz */
	{64, 0},  /* 64 Hz   */
	{128, 0}, /* 128 Hz  */
	{256, 0}  /* 256 Hz  */
};

static inline int mc6470_set_op_mode(const struct mc6470_config *cfg,
				     enum mc6470_op_mode mode)
{
	return i2c_reg_write_byte_dt(&cfg->i2c, MC6470_REG_MODE, mode);
}

static int mc6470_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	int ret = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	ret = i2c_burst_read_dt(&cfg->i2c, MC6470_REG_XOUT_L,
				(uint8_t *)data->samples,
				MC6470_SAMPLE_READ_SIZE);
	k_sem_give(&data->sem);
	return ret;
}

static int mc6470_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	int ret = 0;
	struct mc6470_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		sensor_ug_to_ms2(1000000LL * data->samples[0] * (data->range << 1) / (1 << data->res), val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		sensor_ug_to_ms2(1000000LL * data->samples[1] * (data->range << 1) / (1 << data->res), val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		sensor_ug_to_ms2(1000000LL * data->samples[2] * (data->range << 1) / (1 << data->res), val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		sensor_ug_to_ms2(1000000LL * data->samples[0] * (data->range << 1) / (1 << data->res), &val[0]);
		sensor_ug_to_ms2(1000000LL * data->samples[1] * (data->range << 1) / (1 << data->res), &val[1]);
		sensor_ug_to_ms2(1000000LL * data->samples[2] * (data->range << 1) / (1 << data->res), &val[2]);
		break;
	default:
		LOG_ERR("Unsupported channel");
		ret = -ENOTSUP;
	}

	k_sem_give(&data->sem);
	return ret;
}

static int mc6470_set_range(const struct device *dev, uint8_t range)
{
	int ret = 0;
	uint8_t value = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	switch (range) {
		case 2:
			value = 0;
			break;
		case 4:
			value = 1;
			break;
		case 8:
			value = 2;
			break;
		case 16:
			value = 3;
			break;
		default:
			LOG_ERR("Unsupported range: %d", range);
			return -ENOTSUP;
	}

	ret = i2c_reg_update_byte_dt(&cfg->i2c, MC6470_REG_OUTCFG,
				     MC6470_RANGE_MASK, value << 4);
	if (ret < 0) {
		LOG_ERR("Failed to set resolution (%d)", ret);
		return ret;
	}

	data->range = range;
	return 0;
}

static int mc6470_set_res(const struct device *dev, uint8_t res)
{
	int ret = 0;
	uint8_t value = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	switch (res) {
		case 6:
			value = 0;
			break;
		case 7:
			value = 1;
			break;
		case 8:
			value = 2;
			break;
		case 10:
			value = 3;
			break;
		case 12:
			value = 4;
			break;
		case 14:
			value = 5;
			break;
		default:
			LOG_ERR("Unsupported resolution: %d", res);
			return -ENOTSUP;
	}

	ret = i2c_reg_update_byte_dt(&cfg->i2c, MC6470_REG_OUTCFG,
				     MC6470_RES_MASK, value);
	if (ret < 0) {
		LOG_ERR("Failed to set resolution: %d", ret);
		return ret;
	}

	data->res = res;
	return 0;
}

static int mc6470_set_odr(const struct device *dev,
			  const struct sensor_value *val)
{
	int ret = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	ret = mc6470_get_odr_value(val->val1, val->val2);
	if (ret < 0) {
		LOG_ERR("Selected ODR not available: %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MC6470_REG_SRTFR,
				    ret);
	if (ret < 0) {
		LOG_ERR("Failed to set ODR (%d)", ret);
		return ret;
	}

	data->odr = ret;
	return 0;
}

static int mc6470_set_lower_thresh(const struct device *dev,
								   enum sensor_channel chan, const uint8_t val)
{
	int ret = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;
	uint8_t reg;

	ret = i2c_reg_update_byte_dt(&cfg->i2c, MC6470_REG_TAPEN,
				    MC6470_THRDUR_MASK, MC6470_THRDUR_MASK);
	if (ret < 0) {
		LOG_ERR("Failed to set threshold (%d)", ret);
		return ret;
	}

	switch (chan) {
		case SENSOR_CHAN_ACCEL_X:
			ret = i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRX, val);
			break;
		case SENSOR_CHAN_ACCEL_Y:
			ret = i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRY, val);
			break;
		case SENSOR_CHAN_ACCEL_Z:
			ret = i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRZ, val);
			break;
		case SENSOR_CHAN_ACCEL_XYZ:
			ret = i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRX, val);
			ret |= i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRY, val);
			ret |= i2c_reg_write_dt(&cfg->i2c, MC6470_REG_TTTRZ, val);
			break;
		default:
			LOG_ERR("Sensor channel not supported");
			ret -ENOTSUP;
	}

	if (ret < 0) {
		LOG_ERR("Failed to set threshold (%d)", ret);
		return ret;
	}

	return 0;
}

#if defined(CONFIG_MC6470_TRIGGER)
static int mc6470_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	int ret = 0;
	const struct mc6470_config *cfg = dev->config;
	struct mc6470_driver_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
	ret = mc6470_set_op_mode(cfg, MC6470_MODE_STANDBY);
	if (ret < 0) {
		goto exit;
	}

	ret = mc6470_configure_trigger(dev, trig, handler);
	if (ret < 0) {
		LOG_ERR("Failed to set trigger (%d)", ret);
	}

exit:
	mc6470_set_op_mode(cfg, MC6470_MODE_WAKE);

	k_sem_give(&data->sem);
	return ret;
}
#endif

static int mc6470_attr_set(const struct device *dev,
			   enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	int ret = 0;
	struct mc6470_driver_data *data = dev->data;

	if (chan != SENSOR_CHAN_ACCEL_X &&
	    chan != SENSOR_CHAN_ACCEL_Y &&
	    chan != SENSOR_CHAN_ACCEL_Z &&
	    chan != SENSOR_CHAN_ACCEL_XYZ) {
		LOG_ERR("Sensor channel not supported");
		return -ENOTSUP;
	}

	k_sem_take(&data->sem, K_FOREVER);
	ret = mc6470_set_op_mode(dev->config, MC6470_MODE_STANDBY);
	if (ret < 0) {
		goto exit;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		ret = mc6470_set_range(dev, val->val1);
		break;
	case SENSOR_ATTR_RESOLUTION:
		ret = mc6470_set_res(dev, val->val1);
		break;
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = mc6470_set_odr(dev, val);
		break;
	case SENSOR_ATTR_LOWER_THRESH:
		ret = mc6470_set_lower_thresh(dev, chan, val->val1);
		break;
	default:
		LOG_ERR("ACCEL attribute is not supported");
		ret = -EINVAL;
	}

exit:
	mc6470_set_op_mode(dev->config, MC6470_MODE_WAKE);

	k_sem_give(&data->sem);
	return ret;
}

static int mc6470_init(const struct device *dev)
{
	int ret = 0;
	struct mc6470_driver_data *data = dev->data;
	const struct mc6470_config *cfg = dev->config;

	if (!(i2c_is_ready_dt(&cfg->i2c))) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	k_sem_init(&data->sem, 1, 1);

	ret = mc6470_set_op_mode(cfg, MC6470_MODE_STANDBY);
	if (ret)
	{
		LOG_ERR("Could not set device mode");
		return ret;
	};

	LOG_INF("Setting ODR to: %d", data->odr);
	if(i2c_reg_update_byte_dt(&cfg->i2c, MC6470_REG_SRTFR, MC6470_ODR_MASK, data->odr))
	{
		LOG_ERR("Failed to set output data rate");
		return -EIO;
	}

	LOG_INF("Setting range to: %d", data->range);
	if (mc6470_set_range(dev, data->range))
	{
		LOG_ERR("Failed to set range");
		return -EIO;
	}

	LOG_INF("Setting resolution to: %d", data->res);
	if (mc6470_set_res(dev, data->res))
	{
		LOG_ERR("Failed to set resolution");
		return -EIO;
	}

#if defined(CONFIG_MC6470_TRIGGER)
	ret = mc6470_trigger_init(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize interrupts");
		return ret;
	}
#endif

	LOG_INF("MC6470 Initialized");
	return ret;
}

static DEVICE_API(sensor, mc6470_api) = {
	.attr_set = mc6470_attr_set,
#if defined(CONFIG_MC6470_TRIGGER)
	.trigger_set = mc6470_trigger_set,
#endif
	.sample_fetch = mc6470_sample_fetch,
	.channel_get = mc6470_channel_get,
};

#if defined(CONFIG_MC6470_TRIGGER)
#define MC6470_CFG_IRQ(idx)						\
	.irq_gpio = GPIO_DT_SPEC_INST_GET_OR(idx, irq_gpios, { 0 }),
#else
#define MC6470_CFG_IRQ(idx)
#endif

#define MC6470_DEFINE(idx)\
	static const struct mc6470_config mc6470_config_##idx = {\
		.i2c = I2C_DT_SPEC_INST_GET(idx),\
		MC6470_CFG_IRQ(idx)};\
	static struct mc6470_driver_data mc6470_data_##idx = {\
		.odr = DT_INST_ENUM_IDX(idx, odr),\
		.range = DT_INST_PROP(idx, range),\
		.res = DT_INST_PROP(idx, resolution)\
	};\
	SENSOR_DEVICE_DT_INST_DEFINE(idx, mc6470_init, NULL, &mc6470_data_##idx,\
				     &mc6470_config_##idx, POST_KERNEL,\
				     CONFIG_SENSOR_INIT_PRIORITY, &mc6470_api);

DT_INST_FOREACH_STATUS_OKAY(MC6470_DEFINE)