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

void platform_simrf_init(volatile uint8_t *cs_port, uint8_t cs_pin);


#ifdef	__cplusplus
}
#endif

#endif	/* SIMRF_PLAT_H */

