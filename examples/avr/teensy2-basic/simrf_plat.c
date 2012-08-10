/**
 * Karl Palsson, 2012
 * Demo AVR platform driver for simrf
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <avr/io.h>
#include <util/delay.h>

#include "simrf.h"
#include "simrf_plat.h"

static volatile uint8_t *mrf_reset_port;
static uint8_t mrf_reset_pin;
static volatile uint8_t *mrf_cs_port;
static uint8_t mrf_cs_pin;

static void plat_select(bool value) {
    if (value) {
        *mrf_cs_port &= ~(_BV(mrf_cs_pin));
    } else {
        *mrf_cs_port |= (_BV(mrf_cs_pin));
    }
}

static void plat_reset(bool value) {
    if (value) {
        *mrf_reset_port &= ~(_BV(mrf_reset_pin));
    } else {
        *mrf_reset_port |= (_BV(mrf_reset_pin));
    }
}

/**
 * Internal spi handling, works on both avr tiny, with USI,
 * and also regular hardware SPI.
 *
 * For regular hardware spi, requires the spi hardware to already be setup!
 * (TODO, you can handle that yourself, or even, have a compile time flag that
 * determines whether to use internal, or provided spi_tx/rx routines)
 */
uint8_t plat_spi_tx(uint8_t cData) {

#if defined(SPDR)
    // AVR platforms with "regular" SPI hardware

    /* Start transmission */
    SPDR = cData;
    /* Wait for transmission complete */
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
#elif defined (USIDR)
    // AVR platforms with USI interfaces, capable of SPI
        /* Start transmission */
    USIDR = cData;
    USISR = (1 << USIOIF);
    do {
        USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
    } while ((USISR & (1 << USIOIF)) == 0);
    return USIDR;
//#else - stupid netbeans doesn't find the right defines :(
//#error "You don't seem to have any sort of spi hardware!"
#endif
}



void platform_simrf_init(volatile uint8_t *reset_port, uint8_t reset_pin, volatile uint8_t *cs_port, uint8_t cs_pin) {
    mrf_reset_port = reset_port;
    mrf_reset_pin = reset_pin;
    mrf_cs_port = cs_port;
    mrf_cs_pin = cs_pin;
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    plat.reset = &plat_reset;
    plat.spi_xfr = &plat_spi_tx;
    // TODO more here!
    simrf_setup(&plat);
}
