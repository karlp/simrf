/**
 * Karl Palsson, 2012
 * Demo AVR platform driver for simrf
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "simrf.h"
#include "simrf_plat.h"

#define MRF_RESET_DDR DDRB
#define MRF_RESET_PORT PORTB
#define MRF_RESET_PIN PINB4
#define MRF_CS_DDR DDRB
#define MRF_CS_PORT PORTB
#define MRF_CS_PIN PINB0

#define MRF_RESET_CONFIG  (MRF_RESET_DDR |= (1<<MRF_RESET_PIN))
#define MRF_CS_CONFIG  (MRF_CS_DDR |= (1<<MRF_CS_PIN))

#define SPI_DDR     DDRB
#define SPI_MISO        PINB3
#define SPI_MOSI        PINB2
#define SPI_SCLK        PINB1


static volatile uint8_t *mrf_reset_port;
static uint8_t mrf_reset_pin;
static volatile uint8_t *mrf_cs_port;
static uint8_t mrf_cs_pin;

void platform_mrf_interrupt_disable(void) {
    EIMSK &= ~(_BV(INT0));
}

void platform_mrf_interrupt_enable(void) {
    EIMSK |= _BV(INT0);
}

ISR(INT0_vect) {
    simrf_interrupt_handler();
}

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


void init_spi(void) {
    // outputs, also for the /SS pin, to stop it from flicking to slave
    SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCLK) | _BV(MRF_CS_PIN);
    /* Enable SPI, Master, set clock rate fck/4 */
    SPCR = (1 << SPE) | (1 << MSTR);
    // So, that's either 4Mhz, or 2Mhz, depending on whether Fosc is
    // 16mhz crystal, or 8mhz clock prescaled?
    // If I can read the mrf24j sheet properly, this should work right
    // up to 10Mhz, but let's not push it.... (yet)
}


/**
 * Internal spi handling, works on both avr tiny, with USI,
 * and also regular hardware SPI.
 *
 * For regular hardware spi, requires the spi hardware to already be setup!
 * (TODO, you can handle that yourself, or even, have a compile time flag that
 * determines whether to use internal, or provided spi_tx/rx routines)
 */
static uint8_t plat_spi_tx(uint8_t cData) {

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

void platform_simrf_init(void) {
    MRF_RESET_CONFIG;
    MRF_CS_CONFIG;
    init_spi();
    
    mrf_reset_port = &MRF_RESET_PORT;
    mrf_reset_pin = MRF_RESET_PIN;
    mrf_cs_port = &MRF_CS_PORT;
    mrf_cs_pin = MRF_CS_PIN;
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    plat.reset = &plat_reset;
    plat.spi_xfr = &plat_spi_tx;
    plat.delay_ms = &_delay_ms;
    // TODO more here!
    simrf_setup(&plat);
}
