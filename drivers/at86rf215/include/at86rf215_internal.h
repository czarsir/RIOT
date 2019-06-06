/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#ifndef _AT86RF215_INTERNAL_H
#define _AT86RF215_INTERNAL_H

#include <stdint.h>

#include "at86rf215.h"

#ifdef __cplusplus
extern "C" {
#endif

/********* Config *********/

// TODO
/*** Max transmit power ***/
#define AT86RF215_TXPOWER_MAX           (20)

// TODO
/*** Transmit power offset ***/
#define AT86RF215_TXPOWER_OFFSET        (17)

/*** Reset ***/
/* Minimum reset pulse width */
#define AT86RF215_RESET_PULSE_WIDTH     (62U)
/* The typical transition time to TRX_OFF after reset pulse is 26 us */
#define AT86RF215_RESET_DELAY           (62U)

/*** Wakeup ***/
/* Transition time from SLEEP to TRX_OFF in us */
#define AT86RF215_WAKEUP_DELAY          (306U)

/********* Functions *********************************************************/

/********* Manipulation *********/

/*** Register ***/
uint8_t at86rf215_reg_read(const at86rf2xx_t *dev, uint16_t addr);
void at86rf215_reg_write(const at86rf2xx_t *dev, uint16_t addr, uint8_t value);

// TODO
/*** SRAM ***/
void at86rf2xx_sram_write(const at86rf2xx_t *dev, uint8_t offset,
                          const uint8_t *data, size_t len);

/*** internal Frame Buffer ***/
void at86rf215_fb_start(const at86rf2xx_t *dev);
void at86rf215_fb_read(const at86rf2xx_t *dev, uint8_t *data, size_t len);
void at86rf215_fb_stop(const at86rf2xx_t *dev);

/********* Config *********/

/* config PHY parameters */
void at86rf215_configure_phy(at86rf2xx_t *dev);

/********* State *********/

// TODO
/* Convenience function for reading the status */
uint8_t at86rf2xx_get_status(const at86rf2xx_t *dev);

// TODO
/* Make sure that device is not sleeping */
void at86rf2xx_assert_awake(at86rf2xx_t *dev);

/********* Operation *********/

/* hardware reset */
void at86rf215_hardware_reset(at86rf2xx_t *dev);


// TODO
#if defined(MODULE_AT86RF233) || defined(MODULE_AT86RF231) || defined(DOXYGEN)
/**
 * @brief   Read random data from the RNG
 *
 * @note    According to the data sheet this function only works properly in
 *          Basic Operation Mode. However, sporadic testing has shown that even
 *          in Extended Operation Mode this returns random enough data to be
 *          used as a seed for @ref sys_random if no cryptographically secure
 *          randomness is required.
 *          Any further use-case needs to be evaluated, especially if
 *          crypto-relevant randomness is required.
 *
 * @param[in] dev       device to configure
 * @param[out] data     buffer to copy the random data to
 * @param[in]  len      number of random bytes to store in data
 */
void at86rf2xx_get_random(const at86rf2xx_t *dev, uint8_t *data, size_t len);
#endif


#ifdef __cplusplus
}
#endif

#endif
