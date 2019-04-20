/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#ifndef _AT86RF215_NETDEV_H
#define _AT86RF215_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/***
 * netdev device driver.
 */
extern const netdev_driver_t at86rf2xx_driver;

#ifdef __cplusplus
}
#endif

#endif
