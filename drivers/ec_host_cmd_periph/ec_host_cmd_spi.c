/*
 * Copyright (c) 2020 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_spi_ec_host_cmd

#include <device.h>
#include <drivers/ec_host_cmd_periph.h>
#include <drivers/spi.h>
#include <string.h>
#include <errno.h>

/* TODO move to header file? */
struct spi_cmd_config {
	char *spi_name;
};

struct spi_cmd_data {
	const struct device *spi_bus;
};

const uint8_t out_preamble[4] = {
	0xfa,
	0xfa,
	0xfa,
	0xec,
};

// TODO derived from device tree
const struct spi_config spi_cfg = {
	.frequency = 40000000, // 40Mhz
	// TODO look up SPI_MODE_CPHA. I think this is Mode 0 normally
	.operation = SPI_OP_MODE_SLAVE | SPI_TRANSFER_LSB | SPI_WORD_SET(8) |
		     SPI_LINES_SINGLE,
	.slave = 0,
	.cs = NULL,
};

int ec_host_cmd_periph_spi_init(const struct device *dev,
				struct ec_host_cmd_periph_rx_ctx *rx_ctx)
{

	/* TODO probably don't need this at add */

	return 0;
}

int ec_host_cmd_periph_spi_read(const struct device *dev,
				const struct ec_host_cmd_periph_rx_ctx *rx_ctx)
{
	struct spi_cmd_data *const data = dev->data;
	
	if (rx_ctx == NULL) {
		return -EINVAL;
	}

	const struct spi_buf bufs[] = {
		{
			.buf = rx_ctx->buf,
			.len = rx_ctx->len,
		},
	};
	const struct spi_buf_set buf_set = {
		.buffers = bufs,
		.count = ARRAY_SIZE(bufs),
	};

	const int read_rv = spi_read(data->spi_bus, &spi_cfg, &buf_set);
	// TODO multiple by framecount. Maybe it is just 1 though
	return read_rv < 0 ? read_rv : read_rv * 8;
}

int ec_host_cmd_periph_spi_send(const struct device *dev,
				const struct ec_host_cmd_periph_tx_buf *buf)
{
	struct spi_cmd_data *const data = dev->data;
	
	if (buf == NULL) {
		return -EINVAL;
	}

	/* We have to remove the const quailifer on these buffers even though
	 * write should not write to these buffers, only read from them.
	 */
	const struct spi_buf bufs[] = {
		{
			.buf = (void *)out_preamble,
			.len = ARRAY_SIZE(out_preamble),
		},
		{
			.buf = (void *)buf->buf,
			.len = buf->len,
		},
	};
	const struct spi_buf_set buf_set = {
		.buffers = bufs,
		.count = ARRAY_SIZE(bufs),
	};
	
	return spi_write(data->spi_bus, &spi_cfg, &buf_set);
}


static const struct ec_host_cmd_periph_api spi_cmd_api = {
	.init = &ec_host_cmd_periph_spi_init,
	.read = &ec_host_cmd_periph_spi_read,
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
