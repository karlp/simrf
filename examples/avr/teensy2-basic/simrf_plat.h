/* 
 * File:   simrf_plat.h
 * Author: karlp
 *
 * Created on August 9, 2012, 10:41 PM
 */

#ifndef SIMRF_PLAT_H
#define	SIMRF_PLAT_H

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     * You can also just set up the function pointers yourself.
     * This is just one way of doing it.
     * @param reset_port eg, PORTB
     * @param reset_pin eg, PINB2
     * @param cs_port eg, PORTB
     * @param cs_pin eg, PINB4
     */
void platform_simrf_init(volatile uint8_t *reset_port, uint8_t reset_pin, volatile uint8_t *cs_port, uint8_t cs_pin);


#ifdef	__cplusplus
}
#endif

#endif	/* SIMRF_PLAT_H */

