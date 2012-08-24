// Karl Palsson, 2011
// false.ekta.is
// BSD/MIT Licensed.

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

#include <errno.h>
#include <stdio.h>
#include "simrf.h"
#include "simrf_plat.h"


int _write(int file, char *ptr, int len) {
    int i;

    if (file == 1) {
        for (i = 0; i < len; i++)
            usart_send_blocking(USART2, ptr[i]);
        return i;
    }
    errno = EIO;
    return -1;
}

void init(void) {

    platform_simrf_init();
    // interrupt pin from mrf
    platform_mrf_interrupt_enable();
}

void handle_rx(simrf_rx_info_t *rxinfo, uint8_t *rx_buffer) {
    platform_mrf_interrupt_disable();
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
    printf("\nLQI/RSSI=%d/%d\n", rxinfo->lqi, rxinfo->rssi);
    platform_mrf_interrupt_enable();
}

void handle_tx(simrf_tx_info_t *txinfo) {
    if (txinfo->tx_ok) {
        printf("TX went ok, got ack\n");
    } else {
        printf("TX failed after %d retries\n", txinfo->retries);
    }
}

int main(void) {
    init();

    printf("woke up...woo\n");
    //simrf_hard_reset();
    simrf_soft_reset();
    simrf_init();

    simrf_pan_write(0xcafe);
    simrf_address16_write(0x1111);
    simrf_promiscuous(1);
    uint32_t roughness = 0;
    while (1) {
        roughness++;
        simrf_check_flags(&handle_rx, &handle_tx);
        // about a second or so...
        if (roughness > 0x50000) {
            printf("txxxing...\n");
            simrf_send16(0x1, 4, "abcd");
            roughness = 0;
        }
    }
}
