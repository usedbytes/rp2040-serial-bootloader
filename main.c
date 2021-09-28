/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/structs/dma.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16

#define CMD_SYNC (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_READ (('R' << 0) | ('E' << 8) | ('A' << 16) | ('D' << 24))
// TODO: CRC32 would be much better than this dumb checksum, but I
// can't make sense of the values the HW is producing. Raised
// https://github.com/raspberrypi/pico-sdk/issues/576
#define CMD_CSUM (('C' << 0) | ('S' << 8) | ('U' << 16) | ('M' << 24))

#define RSP_SYNC (('P' << 0) | ('I' << 8) | ('C' << 16) | ('O' << 24))
#define RSP_OK   (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define RSP_ERR  (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_csum(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_csum(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

struct command_desc {
	uint32_t opcode;
	uint32_t nargs;
	uint32_t resp_nargs;
	uint32_t (*size)(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
	uint32_t (*handle)(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
};

const struct command_desc cmds[] = {
	{
		.opcode = CMD_SYNC,
		.nargs = 0,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_sync,
	},
	{
		// READ addr len
		// RESP [data]
		.opcode = CMD_READ,
		.nargs = 2,
		.resp_nargs = 0,
		.size = &size_read,
		.handle = &handle_read,
	},
	{
		// CSUM addr len
		// RESP csum
		.opcode = CMD_CSUM,
		.nargs = 2,
		.resp_nargs = 1,
		.size = &size_csum,
		.handle = &handle_csum,
	},
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));
const uint32_t MAX_NARG = 4;
const uint32_t MAX_DATA_LEN = 4096;

static bool is_error(uint32_t status)
{
	return status == RSP_ERR;
}

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	return RSP_SYNC;
}

static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	uint32_t size = args_in[1];
	if (size > MAX_DATA_LEN) {
		return RSP_ERR;
	}

	// TODO: Validate address

	*data_len_out = 0;
	*resp_data_len_out = size;

	return RSP_OK;
}

static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	memcpy(resp_data_out, (void *)addr, size);

	return RSP_OK;
}

static uint32_t size_csum(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	if ((addr & 0x3) || (size & 0x3)) {
		// Must be aligned
		return RSP_ERR;
	}

	// TODO: Validate address

	*data_len_out = 0;
	*resp_data_len_out = 0;

	return RSP_OK;
}
static uint32_t handle_csum(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t dummy_dest;
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	int channel = dma_claim_unused_channel(true);

	dma_channel_config c = dma_channel_get_default_config(channel);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_sniff_enable(&c, true);

	dma_hw->sniff_data = 0;
	dma_sniffer_enable(channel, 0xf, true);

	dma_channel_configure(channel, &c, &dummy_dest, (void *)addr, size / 4, true);

	dma_channel_wait_for_finish_blocking(channel);

	dma_sniffer_disable();
	dma_channel_unclaim(channel);

	*resp_args_out = dma_hw->sniff_data;

	return RSP_OK;
}

static const struct command_desc *find_command_desc(uint32_t opcode)
{
	unsigned int i;

	for (i = 0; i < N_CMDS; i++) {
		if (cmds[i].opcode == opcode) {
			return &cmds[i];
		}
	}

	return NULL;
}

struct cmd_context {
	uint8_t *uart_buf;
	const struct command_desc *desc;
	uint32_t opcode;
	uint32_t status;
	uint32_t *args;
	uint8_t *data;
	uint32_t *resp_args;
	uint8_t *resp_data;
	uint32_t data_len;
	uint32_t resp_data_len;
};

enum state {
	STATE_WAIT_FOR_SYNC,
	STATE_READ_OPCODE,
	STATE_READ_ARGS,
	STATE_READ_DATA,
	STATE_HANDLE_DATA,
	STATE_ERROR,
};

static enum state state_wait_for_sync(struct cmd_context *ctx)
{
	int idx = 0;
	uint8_t *recv = (uint8_t *)&ctx->opcode;
	uint8_t *match = (uint8_t *)&ctx->status;

	ctx->status = CMD_SYNC;

	gpio_put(PICO_DEFAULT_LED_PIN, 1);

	while (idx < sizeof(ctx->opcode)) {
		uart_read_blocking(uart0, &recv[idx], 1);
		gpio_xor_mask((1 << PICO_DEFAULT_LED_PIN));

		if (recv[idx] != match[idx]) {
			// Start again
			idx = 0;
		} else {
			// Move on
			idx++;
		}
	}

	assert(ctx->opcode == CMD_SYNC);

	return STATE_READ_ARGS;
}

static enum state state_read_opcode(struct cmd_context *ctx)
{
	uart_read_blocking(uart0, (uint8_t *)&ctx->opcode, sizeof(ctx->opcode));

	return STATE_READ_ARGS;
}

static enum state state_read_args(struct cmd_context *ctx)
{
	const struct command_desc *desc = find_command_desc(ctx->opcode);
	if (!desc) {
		// TODO: Error handler that can do args?
		ctx->status = RSP_ERR;
		return STATE_ERROR;
	}

	ctx->desc = desc;
	ctx->args = (uint32_t *)(ctx->uart_buf + sizeof(ctx->opcode));
	ctx->data = (uint8_t *)(ctx->args + desc->nargs);
	ctx->resp_args = ctx->args;
	ctx->resp_data = (uint8_t *)(ctx->resp_args + desc->resp_nargs);

	uart_read_blocking(uart0, (uint8_t *)ctx->args, sizeof(*ctx->args) * desc->nargs);

	return STATE_READ_DATA;
}

static enum state state_read_data(struct cmd_context *ctx)
{
	const struct command_desc *desc = ctx->desc;

	if (desc->size) {
		ctx->status = desc->size(ctx->args, &ctx->data_len, &ctx->resp_data_len);
		if (is_error(ctx->status)) {
			return STATE_ERROR;
		}
	} else {
		ctx->data_len = 0;
		ctx->resp_data_len = 0;
	}

	// TODO: Check sizes

	uart_read_blocking(uart0, (uint8_t *)ctx->data, ctx->data_len);

	return STATE_HANDLE_DATA;
}

static enum state state_handle_data(struct cmd_context *ctx)
{
	const struct command_desc *desc = ctx->desc;

	if (desc->handle) {
		ctx->status = desc->handle(ctx->args, ctx->data, ctx->resp_args, ctx->resp_data);
		if (is_error(ctx->status)) {
			return STATE_ERROR;
		}
	} else {
		// TODO: Should we just assert(desc->handle)?
		ctx->status = RSP_OK;
	}

	size_t resp_len = sizeof(ctx->status) + (sizeof(*ctx->resp_args) * desc->resp_nargs) + ctx->resp_data_len;
	memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
	uart_write_blocking(uart0, ctx->uart_buf, resp_len);

	return STATE_READ_OPCODE;
}

static enum state state_error(struct cmd_context *ctx)
{
	size_t resp_len = sizeof(ctx->status);
	memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
	uart_write_blocking(uart0, ctx->uart_buf, resp_len);

	return STATE_WAIT_FOR_SYNC;
}

int main(void)
{
	sleep_ms(100);

	stdio_usb_init();

	uart_init(uart0, 115200);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_hw_flow(uart0, false, false);

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	struct cmd_context ctx;
	uint8_t uart_buf[sizeof(uint32_t) * (1 + MAX_NARG + MAX_DATA_LEN)];
	ctx.uart_buf = uart_buf;
	enum state state = STATE_WAIT_FOR_SYNC;

	while (1) {
		switch (state) {
		case STATE_WAIT_FOR_SYNC:
			printf("wait_for_sync\n");
			state = state_wait_for_sync(&ctx);
			printf("wait_for_sync done\n");
			break;
		case STATE_READ_OPCODE:
			printf("read_opcode\n");
			state = state_read_opcode(&ctx);
			printf("read_opcode done\n");
			break;
		case STATE_READ_ARGS:
			printf("read_args\n");
			state = state_read_args(&ctx);
			printf("read_args done\n");
			break;
		case STATE_READ_DATA:
			printf("read_data\n");
			state = state_read_data(&ctx);
			printf("read_data done\n");
			break;
		case STATE_HANDLE_DATA:
			printf("handle_data\n");
			state = state_handle_data(&ctx);
			printf("handle_data done\n");
			break;
		case STATE_ERROR:
			printf("error\n");
			state = state_error(&ctx);
			printf("error done\n");
			break;
		}
	}

	return 0;
}
