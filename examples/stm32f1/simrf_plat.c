/**
 * Karl Palsson, 2012
 * Demo AVR platform driver for simrf
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "simrf.h"
#include "simrf_plat.h"

#define MRF_SPI SPI1
#define MRF_SELECT_PORT GPIOA
#define MRF_SELECT_PIN GPIO4
#define MRF_RESET_PORT GPIOC
#define MRF_RESET_PIN GPIO1


void platform_mrf_interrupt_disable(void) {
    // TODO
}

void platform_mrf_interrupt_enable(void) {
    // TODO
}

#if FINISHED == 1
ISR(INT0_vect) {
    simrf_interrupt_handler();
}
#endif

static void plat_select(bool value) {
    if (value) {
        gpio_set(MRF_SELECT_PORT, MRF_SELECT_PIN);
    } else {
        gpio_clear(MRF_SELECT_PORT, MRF_SELECT_PIN);
    }
}

static void plat_reset(bool value) {
    if (value) {
        gpio_set(MRF_RESET_PORT, MRF_RESET_PIN);
    } else {
        gpio_clear(MRF_RESET_PORT, MRF_RESET_PIN);
    }
}


void init_spi(void) {
    // TODO
    spi_enable_software_slave_management(MRF_SPI);
    spi_init_master()
    spi_enable(MRF_SPI);
}


static uint8_t plat_spi_tx(uint8_t cData) {
    return spi_xfer(MRF_SPI, cData);
}

void platform_simrf_init(void) {
    init_spi();
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    plat.reset = &plat_reset;
    plat.spi_xfr = &plat_spi_tx;
//    plat.delay_ms = &_delay_ms;
    // TODO more here!
    simrf_setup(&plat);
}
