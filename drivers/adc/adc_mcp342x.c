/*
 * Copyright (c) 2023 Diodes Delight
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER 1
#include "adc_context.h"

// 1-4ch up to 18bit
#define DT_DRV_COMPAT microchip_mcp3421
// #define DT_DRV_COMPAT microchip_mcp3422
// #define DT_DRV_COMPAT microchip_mcp3423
// #define DT_DRV_COMPAT microchip_mcp3424
// 1-4ch up to 16bit
// #define DT_DRV_COMPAT microchip_mcp3425
// #define DT_DRV_COMPAT microchip_mcp3426
// #define DT_DRV_COMPAT microchip_mcp3427
// #define DT_DRV_COMPAT microchip_mcp3428

LOG_MODULE_REGISTER(MCP342X, CONFIG_ADC_LOG_LEVEL);


#define MCP342X_CONFIG_GAIN(x) ((x)&BIT_MASK(2))
#define MCP342X_CONFIG_RES(x)   (((x)&BIT_MASK(2)) << 2)
#define MCP342X_CONFIG_CM(x)   (((x)&BIT_MASK(1)) << 4)
#define MCP342X_CONFIG_MUX(x)  (((x)&BIT_MASK(2)) << 5)
#define MCP342X_CONFIG_START_CONVERSION(x)  (((x)&BIT_MASK(1)) << 7)

#define MCP342X_CONFIG_MASK_READY BIT(7)

#define MCP342X_REF_INTERNAL   2048

#define READ_BUF_SIZE 4

enum mcp342x_reg {
	MCP342X_REG_OUTPUT = 0,
	MCP342X_REG_CONFIG = 1,
};

enum {
	MCP342X_CONFIG_MUX_0 = 0,
	MCP342X_CONFIG_MUX_1 = 1,
	MCP342X_CONFIG_MUX_2 = 2,
	MCP342X_CONFIG_MUX_3 = 3,
};

enum {
	MCP342X_CONFIG_DR_RATE_240_RES_12 = 0,
	MCP342X_CONFIG_DR_RATE_60_RES_14 = 1,
	MCP342X_CONFIG_DR_RATE_15_RES_16 = 2,
	MCP342X_CONFIG_DR_RATE_3_75_RES_18 = 3,
	MCP342X_CONFIG_RES_DEFAULT = MCP342X_CONFIG_DR_RATE_3_75_RES_18,
};

enum {
	MCP342X_CONFIG_GAIN_1 = 0,
	MCP342X_CONFIG_GAIN_2 = 1,
	MCP342X_CONFIG_GAIN_4 = 2,
	MCP342X_CONFIG_GAIN_8 = 3,
};

enum {
	MCP342X_CONFIG_CM_SINGLE = 0,
	MCP342X_CONFIG_CM_CONTINUOUS = 1,
};

struct mcp342x_config {
	const struct i2c_dt_spec bus;
	uint8_t max_resolution;
};

struct mcp342x_data {
	struct adc_context ctx;
	k_timeout_t ready_time;
	uint8_t config_reg;
	uint8_t resolution;
	struct k_sem acq_sem;
	int32_t *buffer;
	int32_t *buffer_ptr;
};


static int mcp342x_read_reg(const struct device *dev, enum mcp342x_reg reg_addr, uint8_t *sample, uint8_t *status)
{
	const struct mcp342x_config *config = dev->config;
	const struct mcp342x_data *data = dev->data;
	uint8_t buf[READ_BUF_SIZE] = {0};
	int rc = i2c_read_dt(&config->bus, buf, sizeof(buf));

	//if set to 18bit, the 4th byte becomes the status register
	if(data->resolution == MCP342X_CONFIG_DR_RATE_3_75_RES_18) {
		sample[1] = buf[0];
		sample[2] = buf[1];
		sample[3] = buf[2];
		*status = buf[3];
	}else{
		sample[2] = buf[0];
		sample[3] = buf[1];
		*status = buf[2];
	}

	return rc;
}

static int mcp342x_write_reg(const struct device *dev, uint8_t val)
{
	uint8_t msg[1] = {val};
	const struct mcp342x_config *config = dev->config;

	/* It's only possible to write the config register, so the MCP342X
	 * assumes all writes are going to that register and omits the register
	 * parameter from write transactions
	 */
	return i2c_write_dt(&config->bus, msg, sizeof(msg));
}

static int set_resolution_and_acqt(const struct device *dev, uint8_t resolution)
{
	struct mcp342x_data *data = dev->data;
	switch(resolution) {
	case 12:
		data->resolution = MCP342X_CONFIG_DR_RATE_240_RES_12;
		data->ready_time = K_MSEC(1000 / 240);
		break;
	case 14:
		data->resolution = MCP342X_CONFIG_DR_RATE_60_RES_14;
		data->ready_time = K_MSEC(1000 / 60);
		break;
	case 16:
		data->resolution = MCP342X_CONFIG_DR_RATE_15_RES_16;
		data->ready_time = K_MSEC(1000 / 15);
		break;
	case 18:
		data->resolution = MCP342X_CONFIG_DR_RATE_3_75_RES_18;
		data->ready_time = K_MSEC(267);
		break;
	default:
		LOG_ERR("Resolution of %d-bit not supported", resolution);
		return -EINVAL;
	}
	data->config_reg |= MCP342X_CONFIG_RES(data->resolution);
	return 0;
}

static int mcp342x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	struct mcp342x_data *data = dev->data;
	uint8_t config = 0;
	uint8_t res = 12;
	

	switch (channel_cfg->channel_id){
	case 0:
		config |= MCP342X_CONFIG_MUX(MCP342X_CONFIG_MUX_0);
		break;
	case 1:
		config |= MCP342X_CONFIG_MUX(MCP342X_CONFIG_MUX_1);
		break;
	case 2:
		config |= MCP342X_CONFIG_MUX(MCP342X_CONFIG_MUX_2);
		break;
	case 3:
		config |= MCP342X_CONFIG_MUX(MCP342X_CONFIG_MUX_3);
		break;
	default:
		LOG_ERR("Only channel ID 0-3 supported");
		return -EINVAL;
	}

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		config |= MCP342X_CONFIG_GAIN(MCP342X_CONFIG_GAIN_1);
		break;
	case ADC_GAIN_2:
		config |= MCP342X_CONFIG_GAIN(MCP342X_CONFIG_GAIN_2);
		break;
	case ADC_GAIN_4:
		config |= MCP342X_CONFIG_GAIN(MCP342X_CONFIG_GAIN_4);
		break;
	case ADC_GAIN_8:
		config |= MCP342X_CONFIG_GAIN(MCP342X_CONFIG_GAIN_8);
		break;
	default:
		return -EINVAL;
	}

	config |= MCP342X_CONFIG_CM(MCP342X_CONFIG_CM_SINGLE); /* Only single shot supported */
	data->config_reg = config;

	return mcp342x_write_reg(dev, config);
}

static int mcp342x_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = sizeof(int32_t);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		LOG_ERR("Insufficient buffer %i < %i", sequence->buffer_size, needed);
		return -ENOMEM;
	}

	return 0;
}

static int mcp342x_validate_sequence(const struct device *dev, const struct adc_sequence *sequence)
{
	struct mcp342x_data *data = dev->data;
	int rc = 0;
	if (sequence->channels != BIT(0)) {
		LOG_ERR("Invalid Channel 0x%x", sequence->channels);
		return -EINVAL;
	}
	
	rc = set_resolution_and_acqt(dev, sequence->resolution);
	if (rc != 0) {
		return rc;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling not supported");
		return -EINVAL;
	}

	return mcp342x_validate_buffer_size(sequence);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct mcp342x_data *data = CONTAINER_OF(ctx, struct mcp342x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->buffer_ptr;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcp342x_data *data = CONTAINER_OF(ctx, struct mcp342x_data, ctx);

	data->buffer_ptr = data->buffer;
	k_sem_give(&data->acq_sem);
}

static int mcp342x_adc_start_read(const struct device *dev, const struct adc_sequence *sequence,
				  bool wait)
{
	int rc = 0;
	struct mcp342x_data *data = dev->data;

	rc = mcp342x_validate_sequence(dev, sequence);
	if (rc != 0) {
		return rc;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	if (wait) {
		rc = adc_context_wait_for_completion(&data->ctx);
	}
	return rc;
}

static int mcp342x_adc_perform_read(const struct device *dev)
{
	int rc = 0;
	struct mcp342x_data *data = dev->data;
	k_sem_take(&data->acq_sem, K_FOREVER);

	uint8_t sample[READ_BUF_SIZE] = {0};
	uint8_t status = 0xFF;
	uint8_t config = data->config_reg;

	config |= MCP342X_CONFIG_START_CONVERSION(1);
	mcp342x_write_reg(dev, config);

	k_sleep(data->ready_time);
	rc = mcp342x_read_reg(dev, MCP342X_REG_OUTPUT, sample,&status);
	if (rc != 0) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	while (((status & MCP342X_CONFIG_MASK_READY)>>7) == 1) {
		k_sleep(K_USEC(100));
		rc = mcp342x_read_reg(dev, MCP342X_REG_OUTPUT, sample,&status);
		if (rc != 0) {
			adc_context_complete(&data->ctx, rc);
			return rc;
		}
	}
	
	data->buffer[0] = sys_get_be32(sample);

	data->buffer++;

	adc_context_on_sampling_done(&data->ctx, dev);

	return rc;
}

static int mcp342x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	int rc;
	struct mcp342x_data *data = dev->data;
	adc_context_lock(&data->ctx, false, NULL);
	rc = mcp342x_adc_start_read(dev, sequence, false);

	while (rc == 0 && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
		rc = mcp342x_adc_perform_read(dev);
	}

	adc_context_release(&data->ctx, rc);
	return rc;
}

static int mcp342x_init(const struct device *dev)
{
	int rc = 0;
	uint8_t status;
	const struct mcp342x_config *config = dev->config;
	struct mcp342x_data *data = dev->data;

	adc_context_init(&data->ctx);

	k_sem_init(&data->acq_sem, 0, 1);

	if (!device_is_ready(config->bus.bus)) {
		return -ENODEV;
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return rc;
}

static const struct adc_driver_api api = {
	.channel_setup = mcp342x_channel_setup,
	.read = mcp342x_read,
	.ref_internal = MCP342X_REF_INTERNAL,
};

#define ADC_MCP342X_INST_DEFINE(n)                                                                 \
	static const struct mcp342x_config config_##n = {.bus = I2C_DT_SPEC_INST_GET(n),			 \
							.max_resolution = 18};										\
	static struct mcp342x_data data_##n;                                                       \
	DEVICE_DT_INST_DEFINE(n, mcp342x_init, NULL, &data_##n, &config_##n, POST_KERNEL,          \
			      CONFIG_ADC_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(ADC_MCP342X_INST_DEFINE);

// /*
//  * MAX11253: 16 bit, 6-channel, programmable gain amplifier, delta-sigma
//  */
// #define MAX11253_INIT(n)                                                                           \
// 	MAX1125X_INIT(11253, n, MAX1125X_ODR_DELAY_US, MAX11253_RESOLUTION, false, true)
// #undef DT_DRV_COMPAT
// #define DT_DRV_COMPAT maxim_max11253
// DT_INST_FOREACH_STATUS_OKAY(MAX11253_INIT)

// /*
//  * MAX1125X: 24 bit, 6-channel, programmable gain amplifier, delta-sigma
//  */

// #define MAX11254_INIT(n)                                                                           \
// 	MAX1125X_INIT(11254, n, MAX1125X_ODR_DELAY_US, MAX11254_RESOLUTION, false, true)
// #undef DT_DRV_COMPAT
// #define DT_DRV_COMPAT maxim_max11254
// DT_INST_FOREACH_STATUS_OKAY(MAX11254_INIT)
