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


static uint8_t plat_spi_tx(uint8_t cData) {
    return spi_xfer(MRF_SPI, cData);
}

void clock_setup(void) {
    /* Enable clocks on all the peripherals we are going to use. */
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
    // GPIOS... spi1 and usart2 are on port A
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
    // reset and interrupts on port C
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
}

void spi_setup(void) {
    // SPI1 SCK
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
    // SPI1 MOSI
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
    // SPI ChipSelect
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4);

    /* Setup SPI parameters. */
    spi_init_master(MRF_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL,
        SPI_CR1_CPHA, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    // stm32 spi sucksorz
    spi_enable_software_slave_management(MRF_SPI);

    /* Finally enable the SPI. */
    spi_enable(MRF_SPI);
}

void usart_setup(void) {
    // NEED GPIO FOR USARTS...
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, 19200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX);

    /* Finally enable the USART. */
    usart_enable(USART2);
}

void gpio_setup(void) {
    // MRF reset pin
    gpio_set_mode(MRF_RESET_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MRF_RESET_PIN);
    // MRF interrupt pin
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

}


void platform_simrf_init(void) {
    rcc_clock_setup_in_hse_8mhz_out_24mhz();
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    plat.reset = &plat_reset;
    plat.spi_xfr = &plat_spi_tx;
//    plat.delay_ms = &_delay_ms;
    // TODO more here!
    simrf_setup(&plat);
}
