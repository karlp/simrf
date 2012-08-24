/**
 * Karl Palsson, 2012
 * Demo AVR platform driver for simrf
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/systick.h>


#include "simrf.h"
#include "simrf_plat.h"

#define MRF_SPI SPI1
#define MRF_SELECT_PORT GPIOA
#define MRF_SELECT_PIN GPIO4
#define MRF_RESET_PORT GPIOC
#define MRF_RESET_PIN GPIO1
#define MRF_INTERRUPT_PORT GPIOC
#define MRF_INTERRUPT_PIN GPIO0


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
    // active low
    if (value) {
        gpio_clear(MRF_SELECT_PORT, MRF_SELECT_PIN);
    } else {
        gpio_set(MRF_SELECT_PORT, MRF_SELECT_PIN);
    }
}

static void plat_reset(bool value) {
    // active low
    if (value) {
        gpio_clear(MRF_RESET_PORT, MRF_RESET_PIN);
    } else {
        gpio_set(MRF_RESET_PORT, MRF_RESET_PIN);
    }
}

extern void delay_ms(double ms);

static uint8_t plat_spi_tx(uint8_t cData) {
    return spi_xfer(MRF_SPI, cData);
}

void spi_setup(void) {
    // SPI1 SCK
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK);
    // SPI1 MOSI
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_MOSI);
    // SPI ChipSelect
    gpio_set_mode(MRF_SELECT_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MRF_SELECT_PIN);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_SPI1_MISO);

    /* Setup SPI parameters. */
    spi_init_master(MRF_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL,
        SPI_CR1_CPHA, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    /* Ignore the stupid NSS pin. */
    spi_enable_software_slave_management(MRF_SPI);
    spi_set_nss_high(MRF_SPI);

    /* Finally enable the SPI. */
    spi_enable(MRF_SPI);
}

void mrf_gpio_setup(void) {
    // MRF reset pin
    gpio_set_mode(MRF_RESET_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MRF_RESET_PIN);
    // MRF interrupt pin
    gpio_set_mode(MRF_INTERRUPT_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, MRF_INTERRUPT_PIN);
}


void platform_simrf_init(void) {
    mrf_gpio_setup();
    spi_setup();
    
    plat_select(false);
    plat_reset(false);
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    plat.reset = &plat_reset;
    plat.spi_xfr = &plat_spi_tx;
    plat.delay_ms = &delay_ms;
    // TODO more here!
    simrf_setup(&plat);
}
