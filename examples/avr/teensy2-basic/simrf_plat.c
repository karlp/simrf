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

static volatile uint8_t *mrf_cs_port;
static uint8_t mrf_cs_pin;

static void plat_select(bool value) {
    if (value) {
        *mrf_cs_port &= ~(_BV(mrf_cs_pin));
    } else {
        *mrf_cs_port |= (_BV(mrf_cs_pin));
    }
}


void platform_simrf_init(volatile uint8_t *cs_port, uint8_t cs_pin) {
    mrf_cs_port = cs_port;
    mrf_cs_pin = cs_pin;
    
    struct simrf_platform plat;
    memset(&plat, 0, sizeof(plat));
    plat.select = &plat_select;
    // TODO more here!
    simrf_init(&plat);
}
