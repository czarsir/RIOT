/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#ifndef _AT86RF215_H
#define _AT86RF215_H

#include <stdint.h>
#include <stdbool.h>

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/netdev.h"
#include "net/netdev/ieee802154.h"
#include "net/gnrc/nettype.h"

#ifdef __cplusplus
extern "C" {
#endif

/********* Config *********/

/*** Maximum possible packet size in byte ***/
#define AT86RF215_MAX_PKT_LENGTH        (IEEE802154_FRAME_LEN_MAX)

// TODO subG and 2.4G
/*** Channel configuration ***/
#ifdef MODULE_AT86RF212B
/* the AT86RF212B has a sub-1GHz radio */
#define AT86RF2XX_MIN_CHANNEL           (IEEE802154_CHANNEL_MIN_SUBGHZ)
#define AT86RF2XX_MAX_CHANNEL           (IEEE802154_CHANNEL_MAX_SUBGHZ)
#define AT86RF2XX_DEFAULT_CHANNEL       (IEEE802154_DEFAULT_SUBGHZ_CHANNEL)
#define AT86RF2XX_DEFAULT_PAGE          (IEEE802154_DEFAULT_SUBGHZ_PAGE)
#else
#define AT86RF2XX_MIN_CHANNEL           (IEEE802154_CHANNEL_MIN)
#define AT86RF2XX_MAX_CHANNEL           (IEEE802154_CHANNEL_MAX)
#define AT86RF2XX_DEFAULT_CHANNEL       (IEEE802154_DEFAULT_CHANNEL)
#endif
/*** Default PAN ID ***/
#define AT86RF215_DEFAULT_PANID         (IEEE802154_DEFAULT_PANID)
/*** Default TX power (0dBm) ***/
#define AT86RF215_DEFAULT_TXPOWER       (IEEE802154_DEFAULT_TXPOWER)
/*** Base (minimal) RSSI value in dBm ***/
#define RSSI_BASE_VAL                   (-91)

// TODO
#if defined(DOXYGEN) || defined(MODULE_AT86RF232) || defined(MODULE_AT86RF233)
/**
 * @brief   Frame retry counter reporting
 *
 * The AT86RF2XX_HAVE_RETRIES flag enables support for NETOPT_TX_RETRIES NEEDED
 * operation. Required for this functionality is the XAH_CTRL_2 register which
 * contains the frame retry counter. Only the at86rf232 and the at86rf233
 * support this register.
 */
#define AT86RF2XX_HAVE_RETRIES             (1)
#else
#define AT86RF2XX_HAVE_RETRIES             (0)
#endif


/********* State *********/

/****** RF state ******/
#define AT86RF215_STATE_RF_NOP             (0x00) // NO OPERATION
#define AT86RF215_STATE_RF_SLEEP           (0x01)
#define AT86RF215_STATE_RF_TRXOFF          (0x02)
#define AT86RF215_STATE_RF_TXPREP          (0x03)
#define AT86RF215_STATE_RF_TX              (0x04)
#define AT86RF215_STATE_RF_RX              (0x05)
#define AT86RF215_STATE_RF_TRANSITION      (0x06) // only as state
#define AT86RF215_STATE_RF_RESET           (0x07)

// TODO
/*** extended mode ***/
#define AT86RF2XX_STATE_BUSY_RX_AACK   (0x11)     /**< busy receiving data (extended mode) */
#define AT86RF2XX_STATE_BUSY_TX_ARET   (0x12)     /**< busy transmitting data (extended mode) */
#define AT86RF2XX_STATE_RX_AACK_ON     (0x16)     /**< wait for incoming data */
#define AT86RF2XX_STATE_TX_ARET_ON     (0x19)     /**< ready for sending data */


/********* Option *********/

// TODO
/*** option flags ***/
/* `0x00ff` is reserved for general IEEE 802.15.4 flags */
#define AT86RF2XX_OPT_SRC_ADDR_LONG  (NETDEV_IEEE802154_SRC_MODE_LONG)  /**< legacy define */
#define AT86RF2XX_OPT_RAWDUMP        (NETDEV_IEEE802154_RAW)            /**< legacy define */
#define AT86RF215_OPT_AUTOACK        (NETDEV_IEEE802154_ACK_REQ)        /**< legacy define */
#define AT86RF2XX_OPT_ACK_PENDING    (NETDEV_IEEE802154_FRAME_PEND)     /**< legacy define */

#define AT86RF2XX_OPT_CSMA           (0x0100)       /**< CSMA active */
#define AT86RF215_OPT_PROMISCUOUS    (0x0200)       /**< promiscuous mode
                                                     *   active */
#define AT86RF2XX_OPT_PRELOADING     (0x0400)       /**< preloading enabled */
#define AT86RF2XX_OPT_TELL_TX_START  (0x0800)       /**< notify MAC layer on TX
                                                     *   start */
#define AT86RF2XX_OPT_TELL_TX_END    (0x1000)       /**< notify MAC layer on TX
                                                     *   finished */
#define AT86RF2XX_OPT_TELL_RX_START  (0x2000)       /**< notify MAC layer on RX
                                                     *   start */
#define AT86RF2XX_OPT_TELL_RX_END    (0x4000)       /**< notify MAC layer on RX
                                                     *   finished */

/********* Variables ******************************************************/

/*** for device initialization ***/
typedef struct at86rf215_params {
    spi_t spi;              /**< SPI bus the device is connected to */
    spi_clk_t spi_clk;      /**< SPI clock speed to use */
    spi_cs_t cs_pin;        /**< GPIO pin connected to chip select */
    gpio_t int_pin;         /**< GPIO pin connected to the interrupt pin */
    gpio_t sleep_pin;       /**< GPIO pin connected to the sleep pin */
    gpio_t reset_pin;       /**< GPIO pin connected to the reset pin */
} at86rf215_params_t;

/*** Device descriptor for AT86RF215 radio devices ***/
typedef struct {
    netdev_ieee802154_t netdev;             /**< netdev parent struct */
    /* device specific fields */
    at86rf215_params_t params;              /**< parameters for initialization */
    uint8_t state;                          /**< current state of the radio */
    uint8_t tx_frame_len;                   /**< length of the current TX frame */
    uint8_t idle_state;                 /**< state to return to after sending */
    uint8_t pending_tx;                 /**< keep track of pending TX calls
                                             this is required to know when to
                                             return to @ref at86rf2xx_t::idle_state */
#if AT86RF2XX_HAVE_RETRIES
    /* Only radios with the XAH_CTRL_2 register support frame retry reporting */
    uint8_t tx_retries;                 /**< Number of NOACK retransmissions */
#endif
} at86rf2xx_t;


/********* Functions *********************************************************/

/********* Config *********/

/*** Setup an AT86RF2xx based device state ***/
void at86rf215_setup(at86rf2xx_t *dev, const at86rf215_params_t *params);

/*** hardware? reset ***/
void at86rf215_reset(at86rf2xx_t *dev);

/*** short address ***/
uint16_t at86rf215_get_addr_short(const at86rf2xx_t *dev);
void at86rf215_set_addr_short(at86rf2xx_t *dev, uint16_t addr);
/*** long address ***/
uint64_t at86rf215_get_addr_long(const at86rf2xx_t *dev);

// TODO test: address reverse???
void at86rf215_set_addr_long(at86rf2xx_t *dev, uint64_t addr);

// TODO
/*** channel number ***/
uint8_t at86rf215_get_chan(const at86rf2xx_t *dev);
void at86rf215_set_chan(at86rf2xx_t *dev, uint8_t chan);

/*** channel page (NOT support) ***/
uint8_t at86rf215_get_page(const at86rf2xx_t *dev);
void at86rf215_set_page(at86rf2xx_t *dev, uint8_t page);

/*** PAN ID ***/
uint16_t at86rf215_get_pan(const at86rf2xx_t *dev);
void at86rf215_set_pan(at86rf2xx_t *dev, uint16_t pan);

// TODO
/*** transmission power [dBm] ***/
int16_t at86rf215_get_txpower(const at86rf2xx_t *dev);
void at86rf215_set_txpower(const at86rf2xx_t *dev, int16_t txpower);

// TODO
/*** maximum number of retransmissions ***/
uint8_t at86rf2xx_get_max_retries(const at86rf2xx_t *dev);
/* The maximum value is 7 */
void at86rf2xx_set_max_retries(const at86rf2xx_t *dev, uint8_t max);

/****** CSMA ******/

// TODO
/*** maximum number of channel access attempts per frame ***/
uint8_t at86rf2xx_get_csma_max_retries(const at86rf2xx_t *dev);
/* Valid values: 0 to 5, -1 means CSMA disabled */
void at86rf2xx_set_csma_max_retries(const at86rf2xx_t *dev, int8_t retries);
/***
 * min and max backoff exponent for CSMA/CA
 * - Maximum BE: 0 - 8
 * - Minimum BE: 0 - [max]
 */
void at86rf2xx_set_csma_backoff_exp(const at86rf2xx_t *dev, uint8_t min, uint8_t max);
/*** seed for CSMA random backoff ***/
void at86rf2xx_set_csma_seed(const at86rf2xx_t *dev, const uint8_t entropy[2]);

/****** CCA: channel clear assessment ******/

// TODO
/*** threshold ***/
int8_t at86rf2xx_get_cca_threshold(const at86rf2xx_t *dev);
void at86rf2xx_set_cca_threshold(const at86rf2xx_t *dev, int8_t value);

// TODO
/*** ED level measurement ***/
int8_t at86rf2xx_get_ed_level(at86rf2xx_t *dev);


/********* State *********/

uint8_t at86rf215_set_state(at86rf2xx_t *dev, uint8_t state);

/********* Option *********/

// TODO
void at86rf2xx_set_option(at86rf2xx_t *dev, uint16_t option, bool state);

/********* Operation *********/

// TODO
/***
 * Convenience function for simply sending data
 * This function ignores the PRELOADING option
 * data to send (must include IEEE802.15.4 header)
 */
size_t at86rf2xx_send(at86rf2xx_t *dev, const uint8_t *data, size_t len);

// TODO
/*** Prepare, turn to TX state ***/
void at86rf215_tx_prepare(at86rf2xx_t *dev);
/*** Load chunks of data into the transmit buffer ***/
size_t at86rf215_tx_load(at86rf2xx_t *dev, size_t offset, const uint8_t *data, size_t len);
/*** Trigger sending ***/
void at86rf215_tx_exec(const at86rf2xx_t *dev);

// TODO
/*** Perform one manual CCA ***/
bool at86rf2xx_cca(at86rf2xx_t *dev);


#ifdef __cplusplus
}
#endif

#endif /* _AT86RF2XX_H */
