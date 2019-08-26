/***
 *
 * InPhase
 *
 */
#ifndef _INPHASE_CONF_H_
#define _INPHASE_CONF_H_

#include "at86rf215.h"

#ifdef __cplusplus
extern "C" {
#endif

/********* Variables *********/
extern volatile uint8_t sigSync;
extern volatile uint8_t sigSync_test;

/********* Functions *********/
extern void inphase_start(at86rf2xx_t *dev);
extern void inphase_isr(at86rf2xx_t *dev);
extern void start_timer(unsigned int);

#ifdef __cplusplus
}
#endif

#endif
