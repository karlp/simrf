// Karl Palsson, 2013
// false.ekta.is
// Your choice of BSD 2 Clause, MIT, ISC, X11 or Apache Licensed.

#include <errno.h>
#include <stdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#include "simrf.h"
#if defined (STM32L1)
#include <libopencm3/stm32/l1/rcc.h>
#include "platform_l1.h"
#elif defined (STM32F1)
#include <libopencm3/stm32/f1/rcc.h>
#include "platform_f1.h"
#endif

static volatile int64_t ksystick;

int64_t millis(void)
{
	return ksystick;
}

/**
 * Busy loop for X ms USES INTERRUPTS
 * @param ms
 */
void delay_ms(int ms)
{
	int64_t now = millis();
	while (millis() - ms < now) {
		;
	}
}

void sys_tick_handler(void)
{
	ksystick++;
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART2, '\r');
			}
			usart_send_blocking(USART2, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void systick_setup(void)
{
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8); // 24meg / 8 = 3Mhz
	// one interrupt per ms..
	systick_set_reload(3000);
	systick_interrupt_enable();
	systick_counter_enable();
}

void usart_setup(void)
{

	usart_setup_platform();

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_TX);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

void init(void)
{
	clock_setup();
	systick_setup();
	usart_setup();

	platform_simrf_init();
	// interrupt pin from mrf
	platform_mrf_interrupt_enable();
}

static void iow(uint8_t b)
{
	usart_send_blocking(USART2, b);
}

static const uint8_t magic[] = {'S' + 'n', 0x1F, 0xFE, 'r'};

void handle_rx_snif(simrf_rx_info_t *rxinfo, uint8_t *rx_buffer)
{
	int i;
	iow(magic[0]);
	iow(magic[1]);
	iow(magic[2]);
	iow(magic[3]);
	iow(1); // version
	iow(0); // CMD_FRAME
	
	iow(rxinfo->frame_length + 3 + 2);
	iow(rxinfo->fc_raw & 0xff);
	iow(rxinfo->fc_raw >> 8);
	iow(rxinfo->sequence_number);
	for (i = 0; i < rxinfo->frame_length; ++i) {
		iow(rx_buffer[i]);
	}
	iow(rxinfo->rssi);
	// This is TI cc25xx FCS format, with FCS valid hardcoded
	iow(rxinfo->lqi | 0x80);
}

void handle_rx(simrf_rx_info_t *rxinfo, uint8_t *rx_buffer)
{
	printf("Received a packet: %u bytes long\n", rxinfo->frame_length);
	printf("headers:");
	switch (rxinfo->frame_type) {
	case MAC_FRAME_TYPE_BEACON: printf("[ft:beacon]");
		break;
	case MAC_FRAME_TYPE_DATA: printf("[ft:data]");
		break;
	case MAC_FRAME_TYPE_ACK: printf("[ft:ack]");
		break;
	case MAC_FRAME_TYPE_MACCMD: printf("[ft:mac command]");
		break;
	default: printf("[ft:reserved]");
	}
	if (rxinfo->pan_compression) {
		printf("[pan comp]");
	}
	if (rxinfo->ack_bit) {
		printf("[ack bit]");
	}
	if (rxinfo->security_enabled) {
		printf("[security]");
	}
	switch (rxinfo->dest_addr_mode) {
	case MAC_ADDRESS_MODE_NONE: printf("[dam:nopan,noaddr]");
		break;
	case MAC_ADDRESS_MODE_RESERVED: printf("[dam:reserved]");
		break;
	case MAC_ADDRESS_MODE_16: printf("[dam:16bit]");
		break;
	case MAC_ADDRESS_MODE_64: printf("[dam:64bit]");
		break;
	}
	switch (rxinfo->frame_version) {
	case MAC_FRAME_VERSION_2003: printf("[fv:std2003]");
		break;
	case MAC_FRAME_VERSION_2006: printf("[fv:std2006]");
		break;
	default: printf("[fv:future]");
		break;
	}
	switch (rxinfo->src_addr_mode) {
	case MAC_ADDRESS_MODE_NONE: printf("[sam:nopan,noaddr]");
		break;
	case MAC_ADDRESS_MODE_RESERVED: printf("[sam:reserved]");
		break;
	case MAC_ADDRESS_MODE_16: printf("[sam:16bit]");
		break;
	case MAC_ADDRESS_MODE_64: printf("[sam:64bit]");
		break;
	}

	printf("\nsequence: %d(%x)\n", rxinfo->sequence_number, rxinfo->sequence_number);

	uint8_t i = 0;
	uint16_t dest_pan = 0;
	uint16_t dest_id = 0;
	uint16_t src_id = 0;
	if (rxinfo->dest_addr_mode == MAC_ADDRESS_MODE_16
		&& rxinfo->src_addr_mode == MAC_ADDRESS_MODE_16
		&& rxinfo->pan_compression) {
		dest_pan = rx_buffer[i++];
		dest_pan |= rx_buffer[i++] << 8;
		dest_id = rx_buffer[i++];
		dest_id |= rx_buffer[i++] << 8;
		src_id = rx_buffer[i++];
		src_id |= rx_buffer[i++] << 8;
		printf("[dpan:%x]", dest_pan);
		printf("[d16:%x]", dest_id);
		printf("[s16:%x]", src_id);
		// TODO Can't move this into the library without doing more decoding in the library...
		i += 2; // as in the library, module seems to have two useless bytes after headers!
	} else {
		printf("unimplemented address decoding!\n");
	}

	// this will be whatever is still undecoded :)
	printf("Packet data, starting from %u:\n", i);
	for (; i < rxinfo->frame_length; i++) {
		printf("%02x,", rx_buffer[i]);
	}
	printf("\nLQI/RSSI=%x/%x\n", rxinfo->lqi, rxinfo->rssi);
	handle_rx_snif(rxinfo, rx_buffer);
}

int main(void)
{
	init();

	simrf_hard_reset();
	//simrf_soft_reset();
	simrf_init();

	simrf_pan_write(0xcafe);
	uint16_t pan_sanity_check = simrf_pan_read();
	printf("pan read back in as %#x\n", pan_sanity_check);
	simrf_address16_write(0x1111);
	simrf_promiscuous(1);
	while (1) {
		simrf_check_flags(&handle_rx, NULL);
	}
}

