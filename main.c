/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>

#include "RP2040.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/structs/dma.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16

#define CMD_SYNC  (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_READ  (('R' << 0) | ('E' << 8) | ('A' << 16) | ('D' << 24))
#define CMD_CSUM  (('C' << 0) | ('S' << 8) | ('U' << 16) | ('M' << 24))
#define CMD_CRC   (('C' << 0) | ('R' << 8) | ('C' << 16) | ('C' << 24))
#define CMD_ERASE (('E' << 0) | ('R' << 8) | ('A' << 16) | ('S' << 24))
#define CMD_WRITE (('W' << 0) | ('R' << 8) | ('I' << 16) | ('T' << 24))
#define CMD_SEAL  (('S' << 0) | ('E' << 8) | ('A' << 16) | ('L' << 24))
#define CMD_GO    (('G' << 0) | ('O' << 8) | ('G' << 16) | ('O' << 24))

#define RSP_SYNC (('P' << 0) | ('I' << 8) | ('C' << 16) | ('O' << 24))
#define RSP_OK   (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define RSP_ERR  (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

static void disable_interrupts(void)
{
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICPR[0] = 0xFFFFFFFF;

	SysTick->CTRL &= ~1; /* disable the systick, which operates separately from nvic */
}

static void set_msp_and_jump(uint32_t vtor)
{
	// Dedicated function with no call to any function (appart the last call)
	// This way, there is no manipulation of the stack here, ensuring that GGC
	// didn't insert any pop from the SP after having set the MSP.
	uint32_t reset_vector = *(volatile uint32_t *)(vtor + 0x04); /* reset ptr in vector table */

	SCB->VTOR = (volatile uint32_t) (vtor);

	asm volatile("msr msp, %0"::"g"
			(*(volatile uint32_t *)vtor));

	// Inline asm branch, because otherwise the compiler was doing something
	// weird (ldmia.w) which was causing us to take an abort
	asm volatile("bx %0"::"r" (reset_vector));
}

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_csum(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_csum(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_crc(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_crc(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_erase(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_write(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_write(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_seal(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_go(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

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
		// OKOK [data]
		.opcode = CMD_READ,
		.nargs = 2,
		.resp_nargs = 0,
		.size = &size_read,
		.handle = &handle_read,
	},
	{
		// CSUM addr len
		// OKOK csum
		.opcode = CMD_CSUM,
		.nargs = 2,
		.resp_nargs = 1,
		.size = &size_csum,
		.handle = &handle_csum,
	},
	{
		// CRCC addr len
		// OKOK crc
		.opcode = CMD_CRC,
		.nargs = 2,
		.resp_nargs = 1,
		.size = &size_crc,
		.handle = &handle_crc,
	},
	{
		// ERAS addr len
		// OKOK
		.opcode = CMD_ERASE,
		.nargs = 2,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_erase,
	},
	{
		// WRIT addr len [data]
		// OKOK crc
		.opcode = CMD_WRITE,
		.nargs = 2,
		.resp_nargs = 1,
		.size = &size_write,
		.handle = &handle_write,
	},
	{
		// SEAL vtor len crc
		// OKOK
		.opcode = CMD_SEAL,
		.nargs = 3,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_seal,
	},
	{
		// GOGO vtor
		// NO RESPONSE
		.opcode = CMD_GO,
		.nargs = 1,
		.resp_nargs = 0,
		.size = NULL,
		.handle = &handle_go,
	},
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));
const uint32_t MAX_NARG = 4;
const uint32_t MAX_DATA_LEN = FLASH_SECTOR_SIZE;

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

static uint32_t size_crc(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
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

// ptr must be 4-byte aligned and len must be a multiple of 4
static uint32_t calc_crc32(void *ptr, uint32_t len)
{
	uint32_t dummy_dest, crc;

	int channel = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(channel);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_sniff_enable(&c, true);

	// Seed the CRC calculation
	dma_hw->sniff_data = 0xffffffff;

	// Mode 1, then bit-reverse the result gives the same result as
	// golang's IEEE802.3 implementation
	dma_sniffer_enable(channel, 0x1, true);
	dma_hw->sniff_ctrl |= DMA_SNIFF_CTRL_OUT_REV_BITS;

	dma_channel_configure(channel, &c, &dummy_dest, ptr, len / 4, true);

	dma_channel_wait_for_finish_blocking(channel);

	// Read the result before resetting
	crc = dma_hw->sniff_data ^ 0xffffffff;

	dma_sniffer_disable();
	dma_channel_unclaim(channel);

	return crc;
}

static uint32_t handle_crc(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	resp_args_out[0] = calc_crc32((void *)addr, size);

	return RSP_OK;
}

static uint32_t handle_erase(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	// TODO: Protect the bootloader!
	if (addr < 0x10000000) {
		// Below flash start
		return RSP_ERR;
	}

	if ((addr & (FLASH_SECTOR_SIZE - 1)) || (size & (FLASH_SECTOR_SIZE - 1))) {
		// Must be aligned
		return RSP_ERR;
	}

	flash_range_erase(addr - 0x10000000, size);

	return RSP_OK;
}

static uint32_t size_write(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	// TODO: Protect the bootloader!
	if (addr < 0x10000000) {
		// Below flash start
		return RSP_ERR;
	}

	if ((addr & (FLASH_PAGE_SIZE -1)) || (size & (FLASH_PAGE_SIZE -1))) {
		// Must be aligned
		return RSP_ERR;
	}

	if (size > MAX_DATA_LEN) {
		return RSP_ERR;
	}

	// TODO: Validate address

	*data_len_out = size;
	*resp_data_len_out = 0;

	return RSP_OK;
}

static uint32_t handle_write(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	uint32_t addr = args_in[0];
	uint32_t size = args_in[1];

	flash_range_program(addr - 0x10000000, data_in, size);

	resp_args_out[0] = calc_crc32((void *)addr, size);

	return RSP_OK;
}

struct image_header {
	uint32_t vtor;
	uint32_t size;
	uint32_t crc;
	uint8_t pad[FLASH_PAGE_SIZE - (3 * 4)];
};
static_assert(sizeof(struct image_header) == FLASH_PAGE_SIZE, "image_header must be FLASH_PAGE_SIZE bytes");

static uint32_t handle_seal(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct image_header hdr = {
		.vtor = args_in[0],
		.size = args_in[1],
		.crc = args_in[2],
	};

	if ((hdr.vtor & 0x3) || (hdr.size & 0x3)) {
		// Must be aligned
		return RSP_ERR;
	}

	uint32_t calc = calc_crc32((void *)hdr.vtor, hdr.size);

	if (calc != hdr.crc) {
		return RSP_ERR;
	}

	// Need address of a 4k page to erase and write to, from the linker?
	// flash_range_erase(IMAGE_HEADER_OFFSET, FLASH_SECTOR_SIZE);
	// flash_range_program(IMAGE_HEADER_OFFSET, &hdr, sizeof(hdr));
	// struct image_header *check = (struct image_header *)(FLASH_BASE + IMAGE_HEADER_OFFSET)
	// if (memcmp(&hdr, check, sizeof(hdr))) {
	//	return RSP_ERR;
	// }

	return RSP_ERR;
}

static uint32_t handle_go(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	disable_interrupts();

	set_msp_and_jump(args_in[0]);

	while(1);

	return RSP_ERR;
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

	//stdio_usb_init();

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
