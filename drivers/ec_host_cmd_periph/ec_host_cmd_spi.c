/*
 * Copyright (c) 2020 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_spi_ec_host_cmd

#include <device.h>
#include <drivers/ec_host_cmd_periph.h>
#include <string.h>

/* TODO move to header file? */
struct spi_cmd_config {
	char *spi_name;
};

struct spi_cmd_data {
	uint8_t rx_buffer[256];	//aligned?
	size_t rx_len;
	const struct device *spi_bus;
	struct k_sem handler_owns;
	struct k_sem dev_owns;
};

int ec_host_cmd_periph_spi_init(const struct device *dev,
				struct ec_host_cmd_periph_rx_ctx *rx_ctx)
{
	struct spi_cmd_data *const data = dev->data;

	if (rx_ctx == NULL) {
		return -EINVAL;
	}

	rx_ctx->buf = data->rx_buffer;
	rx_ctx->len = &data->rx_len;
	rx_ctx->dev_owns = &data->dev_owns;
	rx_ctx->handler_owns = &data->handler_owns;

	return 0;
}

int ec_host_cmd_periph_spi_send(const struct device *dev,
				const struct ec_host_cmd_periph_tx_buf *buf)
{
	// TODO, transive on SPI

	// Need to put SPI header frame on TX buffer

	return 0;
}

/* TODO handle incoming SPI transaction */

static const struct ec_host_cmd_periph_api spi_cmd_api = {
	.init = &ec_host_cmd_periph_spi_init,
	.send = &ec_host_cmd_periph_spi_send,
};

static int ec_host_cmd_spi_init(const struct device *dev)
{
	const struct spi_cmd_config *const cfg = dev->config;
	struct spi_cmd_data *const data = dev->data;

	data->spi_bus = device_get_binding(cfg->spi_name);

	if (data->spi_bus == NULL) {
		return -ENXIO;
	}

	/* Start with the device owning the buffer */
	k_sem_init(&data->dev_owns, 1, 1);
	k_sem_init(&data->handler_owns, 0, 1);

	return 0;
}

#define SPI_CMD_INIT(idx)                                                      \
	static const struct spi_cmd_config spi_cmd_cfg_##idx = {               \
		.spi_name = DT_INST_BUS_LABEL(idx),                            \
	};                                                                     \
                                                                               \
	static struct spi_cmd_data spi_cmd_data_##idx;                         \
                                                                               \
	/* Assume only one simulator */                                        \
	DEVICE_AND_API_INIT(spi_cmd_##idx, DT_INST_LABEL(idx),                 \
			    ec_host_cmd_spi_init, &spi_cmd_data_##idx,         \
			    &spi_cmd_cfg_##idx, POST_KERNEL,                   \
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &spi_cmd_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_CMD_INIT)
