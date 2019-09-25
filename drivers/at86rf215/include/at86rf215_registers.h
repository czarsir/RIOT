/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#ifndef _AT86RF215_REGISTERS_H
#define _AT86RF215_REGISTERS_H

#include "at86rf215.h"

#ifdef __cplusplus
extern "C" {
#endif

/*** Part Number list ***/
#define AT86RF212B_PARTNUM       (0x07)
#define AT86RF231_PARTNUM        (0x03)
#define AT86RF232_PARTNUM        (0x0a)
#define AT86RF233_PARTNUM        (0x0b)
#define AT86RF215_PARTNUM        (0x34)

/*** use AT86RF215 ***/
#define AT86RF2XX_PARTNUM           AT86RF215_PARTNUM

/*** SPI Protocol - cmd ***/
#define AT86RF215_ACCESS_READ                                   (0x0000)
#define AT86RF215_ACCESS_WRITE                                  (0x8000)


/****** Register addresses ***************************************************/

/****** Base ******/

#define AT86RF215_REG__PART_NUM                                 (0x000D)
#define AT86RF215_REG__VERSION                                  (0x000E)

/****** Common ******/

#define AT86RF215_REG__RF_CFG                                   (0x0006)
#define AT86RF215_REG__RF_CLKO                                  (0x0007)
#define AT86RF215_REG__RF_IQIFC1                                (0x000B)
/* IRQ status */
#define AT86RF215_REG__RF09_IRQS                                (0x0000)
#define AT86RF215_REG__RF24_IRQS                                (0x0001)
#define AT86RF215_REG__BBC0_IRQS                                (0x0002)
#define AT86RF215_REG__BBC1_IRQS                                (0x0003)

/****** Blocks ***********************************/
#define _RF09_        (0x0100)
#define _RF24_        (0x0200)
#define _BBC0_        (0x0300)
#define _BBC1_        (0x0400)

/****** RF ***************************************/

#define AT86RF215_REG__STATE                                    (0x0002)
#define AT86RF215_REG__CMD                                      (0x0003)
/* channel */
#define AT86RF215_REG__CS                                       (0x0004)
#define AT86RF215_REG__CCF0L                                    (0x0005)
#define AT86RF215_REG__CCF0H                                    (0x0006)
#define AT86RF215_REG__CNL                                      (0x0007)
#define AT86RF215_REG__CNM                                      (0x0008)
/* RX */
#define AT86RF215_REG__RXBWC                                    (0x0009)
#define AT86RF215_REG__RXDFE                                    (0x000A)
#define AT86RF215_REG__AGCC                                     (0x000B)
#define AT86RF215_REG__AGCS                                     (0x000C)
#define AT86RF215_REG__EDC                                      (0x000E)
/* TX */
#define AT86RF215_REG__TXCUTC                                   (0x0012)
#define AT86RF215_REG__TXDFE                                    (0x0013)
#define AT86RF215_REG__PAC                                      (0x0014)
/* TX DAC Overwrite */
#define AT86RF215_REG__TXDACI                                   (0x0027)
#define AT86RF215_REG__TXDACQ                                   (0x0028)

/****** BBC ***************************************/

#define AT86RF215_REG__IRQM                                     (0x0000)
#define AT86RF215_REG__PC                                       (0x0001)
/* RX Frame */
#define AT86RF215_REG__RXFLL                                    (0x0004)
#define AT86RF215_REG__RXFLH                                    (0x0005)
/* TX Frame */
#define AT86RF215_REG__TXFLL                                    (0x0006)
#define AT86RF215_REG__TXFLH                                    (0x0007)

/*** Filter ***/
#define AT86RF215_REG__AFC0                                     (0x0020)

/****** IEEE ******/
#define AT86RF215_REG__IEEE_MACEA_0                             (0x0025)
//#define AT86RF215_REG__IEEE_MACEA_1                             (0x0026)
#define AT86RF215_REG__IEEE_MACPID0_0                           (0x002D)
#define AT86RF215_REG__IEEE_MACPID0_1                           (0x002E)
#define AT86RF215_REG__IEEE_MACSHA0_0                           (0x002F)
#define AT86RF215_REG__IEEE_MACSHA0_1                           (0x0030)

/* Auto Mode */
#define AT86RF215_REG__AMCS                                     (0x0040)

/****** FSK ******/
#define AT86RF215_REG__FSKC0                                    (0x0060)
#define AT86RF215_REG__FSKC1                                    (0x0061)
#define AT86RF215_REG__FSKC2                                    (0x0062)
#define AT86RF215_REG__FSKC3                                    (0x0063)
#define AT86RF215_REG__FSKPLL                                   (0x0065)
#define AT86RF215_REG__FSKPHRTX                                 (0x006A)
#define AT86RF215_REG__FSKDM                                    (0x0072)

/*** PMU ***/
#define AT86RF215_REG__PMUC                                     (0x0080)
#define AT86RF215_REG__PMUVAL                                   (0x0081)
#define AT86RF215_REG__PMUQF                                    (0x0082)

/* RX Frame Buffer */
#define AT86RF215_REG__BBC0_FBRXS                               (0x2000)
/* TX Frame Buffer */
#define AT86RF215_REG__BBC0_FBTXS                               (0x2800)
/* RX Frame Buffer */
#define AT86RF215_REG__BBC1_FBRXS                               (0x3000)
/* TX Frame Buffer */
#define AT86RF215_REG__BBC1_FBTXS                               (0x3800)



/**
 * @name    Register addresses
 * @{
 */
#define AT86RF2XX_REG__PHY_ED_LEVEL                             (0x07)
#define AT86RF2XX_REG__CCA_THRES                                (0x09)
#define AT86RF2XX_REG__XAH_CTRL_0                               (0x2C)
#define AT86RF2XX_REG__CSMA_SEED_0                              (0x2D)
#define AT86RF2XX_REG__CSMA_SEED_1                              (0x2E)
#define AT86RF2XX_REG__CSMA_BE                                  (0x2F)
/** @} */

/****** Register masks *****************************************************/

/*** RF ***/
#define AT86RF215_RFn_STATE_MASK                                (0x07)
#define AT86RF215_RFn_DRV_MASK                                  (0x03)
#define AT86RF215_RFn_TX_PWR_MASK                               (0x1F)
#define AT86RF215_RFn_AGC_TGT_M                                 (0xE0)
#define AT86RF215_RFn_IRQM__TRXRDY_M                            (0x02)
/*** BBC ***/
#define AT86RF215_BBCn_IRQS__TXFE_M                             (0x10)
#define AT86RF215_BBCn_IRQS__RXFE_M                             (0x02)
#define AT86RF215_BBCn_IRQS__RXFS_M                             (0x01)
#define AT86RF215_BBCn_IRQM__TXFE_M                             (0x10)
#define AT86RF215_BBCn_IRQM__RXFE_M                             (0x02)
#define AT86RF215_BBCn_IRQM__RXFS_M                             (0x01)
/* FSK */
#define AT86RF215_BBCn_FSK__PDT_M                               (0x0F)
#define AT86RF215_BBCn_FSK__RXO_M                               (0x60)
#define AT86RF215_BBCn_FSK__RXPTO_M                             (0x10)


/****** Control *************************************************************/

/***
 * Command
 */


/*** PC - PHY Control ***/
/* Frame Check Sequence Filter Enable (default 0x1 enabled) */
#define AT86RF215_FCSFE_ENABLE                                  (0x40)
#define AT86RF215_FCST                                          (0x08) // 0: 32-bit, 1: 16-bit.
#define AT86RF215_BBEN_ENABLE                                   (0x04)
/* PHY Type */
#define AT86RF215_PT_M                                          (0x03)

/*** Frame Filter ***/
/* Promiscuous mode */
#define AT86RF215_PM_ENABLE                                     (0x10)

/*** AMCS – Auto Mode Configuration and Status ***/
/* automatic acknowledgement */
#define AT86RF215_AACK_ENABLE                                   (0x08)
/* CCA measurement and automatic transmit */
#define AT86RF215_CCATX_ENABLE                                  (0x02)




/*<<<<<<<<<<<<<<<<< TODO all below >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

#define AT86RF2XX_IRQ_STATUS_MASK__RX_START                     (0x04)

/**
 * @name    Bitfield definitions for the CCA_THRES register
 * @{
 */
#define AT86RF2XX_CCA_THRES_MASK__CCA_ED_THRES                  (0x0F)
#define AT86RF2XX_CCA_THRES_MASK__RSVD_HI_NIBBLE                (0xC0)

/**
 * @name    Bitfield definitions for the XAH_CTRL_0 register
 * @{
 */
#define AT86RF2XX_XAH_CTRL_0__MAX_FRAME_RETRIES                 (0xF0)
#define AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES                  (0x0E)
#define AT86RF2XX_XAH_CTRL_0__SLOTTED_OPERATION                 (0x01)

/**
 * @name    Bitfield definitions for the CSMA_SEED_1 register
 * @{
 */
#define AT86RF2XX_CSMA_SEED_1__AACK_SET_PD                      (0x20)
#define AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK                     (0x10)
#define AT86RF2XX_CSMA_SEED_1__AACK_I_AM_COORD                  (0x08)
#define AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1                      (0x07)


#ifdef __cplusplus
}
#endif

#endif
