/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#include "at86rf215.h"
#include "at86rf215_internal.h"
#include "at86rf215_registers.h"
#include "periph/spi.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


/********* IEEE802.15.4 MAC *********/

uint16_t at86rf215_get_addr_short(const at86rf2xx_t *dev)
{
    return (dev->netdev.short_addr[1] << 8) | dev->netdev.short_addr[0];
}

void at86rf215_set_addr_short(at86rf2xx_t *dev, uint16_t addr)
{
    dev->netdev.short_addr[0] = (uint8_t)(addr);
    dev->netdev.short_addr[1] = (uint8_t)(addr >> 8);
#ifdef MODULE_SIXLOWPAN
    /* https://tools.ietf.org/html/rfc4944#section-12 requires the first bit to
     * 0 for unicast addresses */
    dev->netdev.short_addr[0] &= 0x7F;
#endif
    at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__IEEE_MACSHA0_1,
                        dev->netdev.short_addr[1]);
    at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__IEEE_MACSHA0_0,
                        dev->netdev.short_addr[0]);
}

uint64_t at86rf215_get_addr_long(const at86rf2xx_t *dev)
{
    uint64_t addr;
    uint8_t *ap = (uint8_t *)(&addr);

    for (int i = 0; i < 8; i++) {
        ap[i] = dev->netdev.long_addr[i];
    }
    return addr;
}

void at86rf215_set_addr_long(at86rf2xx_t *dev, uint64_t addr)
{
    for (int i = 0; i < 8; i++) {
        dev->netdev.long_addr[i] = (uint8_t)(addr >> (i * 8));
        at86rf215_reg_write(dev, ((dev->bbc|AT86RF215_REG__IEEE_MACEA_0) + i), (addr >> ((7 - i) * 8)));
		//at86rf215_reg_write(dev, (dev->bbc|AT86RF215_REG__IEEE_MACEA_0 + i), (addr >> (i * 8)));
    }
}

uint16_t at86rf215_get_pan(const at86rf2xx_t *dev)
{
	return dev->netdev.pan;
}

void at86rf215_set_pan(at86rf2xx_t *dev, uint16_t pan)
{
	le_uint16_t le_pan = byteorder_btols(byteorder_htons(pan));

	dev->netdev.pan = pan;
	//DEBUG("[rf215] pan0: %u, pan1: %u\n", le_pan.u8[0], le_pan.u8[1]);
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__IEEE_MACPID0_0, le_pan.u8[0]);
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__IEEE_MACPID0_1, le_pan.u8[1]);
}

/********* IEEE802.15.4 channel *********/

uint8_t at86rf215_get_chan(const at86rf2xx_t *dev)
{
    return dev->netdev.chan;
}

void at86rf215_set_chan(at86rf2xx_t *dev, uint8_t channel)
{
	if(dev->rf == _RF24_) {
//		if ((channel > AT86RF215_24_CH_MAX) ||
//			(channel < AT86RF215_24_CH_MIN) ||
//			(dev->netdev.chan == channel)) {
//			return;
//		}
	} else {
		if ((channel > AT86RF215_SUB_CH_MAX) ||
			(dev->netdev.chan == channel)) {
			return;
		}
	}

    dev->netdev.chan = channel;

    at86rf215_configure_phy(dev);
}

uint8_t at86rf215_get_page(const at86rf2xx_t *dev)
{
    (void) dev;
    return 0;
}

void at86rf215_set_page(at86rf2xx_t *dev, uint8_t page)
{
    (void) dev;
    (void) page;
}

int16_t at86rf215_get_txpower(const at86rf2xx_t *dev)
{
    uint8_t txpower = at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__PAC)
                      & AT86RF215_RFn_TX_PWR_MASK;
	/* FSK: offset from -11 dBm */
    return (txpower - 11);
}

void at86rf215_set_txpower(const at86rf2xx_t *dev, int16_t txpower)
{
    txpower += AT86RF215_TXPOWER_OFFSET;

    if (txpower < 0) {
        txpower = 0;
    } else if (txpower > AT86RF215_TXPOWER_MAX) {
        txpower = AT86RF215_TXPOWER_MAX;
    }

	/* use default (max) */
    at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__PAC, 0x7f);
}

uint8_t at86rf2xx_get_max_retries(const at86rf2xx_t *dev)
{
    return (at86rf215_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0) >> 4);
}

void at86rf2xx_set_max_retries(const at86rf2xx_t *dev, uint8_t max)
{
    uint8_t tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);

    tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_FRAME_RETRIES);
    tmp |= ((max > 7) ? 7 : max) << 4;
    at86rf215_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

uint8_t at86rf2xx_get_csma_max_retries(const at86rf2xx_t *dev)
{
    uint8_t tmp;

    tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES;
    tmp >>= 1;
    return tmp;
}

void at86rf2xx_set_csma_max_retries(const at86rf2xx_t *dev, int8_t retries)
{
    retries = (retries > 5) ? 5 : retries;  /* valid values: 0-5 */
    retries = (retries < 0) ? 7 : retries;  /* max < 0 => disable CSMA (set to 7) */
    DEBUG("[at86rf2xx] opt: Set CSMA retries to %u\n", retries);

    uint8_t tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES);
    tmp |= (retries << 1);
    at86rf215_reg_write(dev, AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

void at86rf2xx_set_csma_backoff_exp(const at86rf2xx_t *dev,
                                    uint8_t min, uint8_t max)
{
    max = (max > 8) ? 8 : max;
    min = (min > max) ? max : min;
    DEBUG("[at86rf2xx] opt: Set min BE=%u, max BE=%u\n", min, max);

    at86rf215_reg_write(dev, AT86RF2XX_REG__CSMA_BE, (max << 4) | (min));
}

void at86rf2xx_set_csma_seed(const at86rf2xx_t *dev, const uint8_t entropy[2])
{
    if (entropy == NULL) {
        DEBUG("[at86rf2xx] opt: CSMA seed entropy is nullpointer\n");
        return;
    }
    DEBUG("[at86rf2xx] opt: Set CSMA seed to 0x%x 0x%x\n", entropy[0], entropy[1]);

    at86rf215_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_0, entropy[0]);

    uint8_t tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
    tmp &= ~(AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1);
    tmp |= entropy[1] & AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1;
    at86rf215_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
}

int8_t at86rf2xx_get_cca_threshold(const at86rf2xx_t *dev)
{
    int8_t tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__CCA_THRES);

    tmp &= AT86RF2XX_CCA_THRES_MASK__CCA_ED_THRES;
    tmp <<= 1;
    return (RSSI_BASE_VAL + tmp);
}

void at86rf2xx_set_cca_threshold(const at86rf2xx_t *dev, int8_t value)
{
    /* ensure the given value is negative, since a CCA threshold > 0 is
       just impossible: thus, any positive value given is considered
       to be the absolute value of the actually wanted threshold */
    if (value > 0) {
        value = -value;
    }
    /* transform the dBm value in the form
       that will fit in the AT86RF2XX_REG__CCA_THRES register */
    value -= RSSI_BASE_VAL;
    value >>= 1;
    value &= AT86RF2XX_CCA_THRES_MASK__CCA_ED_THRES;
    value |= AT86RF2XX_CCA_THRES_MASK__RSVD_HI_NIBBLE;
    at86rf215_reg_write(dev, AT86RF2XX_REG__CCA_THRES, value);
}

int8_t at86rf2xx_get_ed_level(at86rf2xx_t *dev)
{
    uint8_t tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__PHY_ED_LEVEL);

#if MODULE_AT86RF212B
    /* AT86RF212B has different scale than the other variants */
    int8_t ed = (int8_t)(((int16_t)tmp * 103) / 100) + RSSI_BASE_VAL;
#else
    int8_t ed = (int8_t)tmp + RSSI_BASE_VAL;
#endif
    return ed;
}

void at86rf2xx_set_option(at86rf2xx_t *dev, uint16_t option, bool state)
{
    uint8_t tmp;

    DEBUG("[rf215] set option 0x%x to %s\n", option, (state ? "true" : "false"));

    /* set option field */
    dev->netdev.flags = (state) ? (dev->netdev.flags |  option)
                                : (dev->netdev.flags & ~option);
    /* trigger option specific actions */
    switch (option) {
        case AT86RF2XX_OPT_CSMA:
            if (state) {
                DEBUG("[at86rf2xx] opt: enabling CSMA mode" \
                      "(4 retries, min BE: 3 max BE: 5)\n");
                /* Initialize CSMA seed with hardware address */
                at86rf2xx_set_csma_seed(dev, dev->netdev.long_addr);
                at86rf2xx_set_csma_max_retries(dev, 4);
                at86rf2xx_set_csma_backoff_exp(dev, 3, 5);
            }
            else {
                DEBUG("[at86rf2xx] opt: disabling CSMA mode\n");
                /* setting retries to -1 means CSMA disabled */
                at86rf2xx_set_csma_max_retries(dev, -1);
            }
            break;
        case AT86RF215_OPT_PROMISCUOUS:
            DEBUG("[rf215] opt: %s PROMISCUOUS mode\n",
                  (state ? "enable" : "disable"));
            /* disable/enable auto ACKs in promiscuous mode */
//            tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
//            tmp = (state) ? (tmp |  AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK)
//                          : (tmp & ~AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
//            at86rf215_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
            /* enable/disable promiscuous mode */
            tmp = at86rf215_reg_read(dev, dev->bbc|AT86RF215_REG__AFC0);
            tmp = (state) ? (tmp |  AT86RF215_PM_ENABLE)
                          : (tmp & ~AT86RF215_PM_ENABLE);
            at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__AFC0, tmp);
            break;
        case AT86RF215_OPT_AUTOACK:
            DEBUG("[rf215] opt: %s auto ACKs\n",
                  (state ? "enable" : "disable"));
            tmp = at86rf215_reg_read(dev, dev->bbc|AT86RF215_REG__AMCS);
            tmp = (state) ? (tmp |  AT86RF215_AACK_ENABLE)
                          : (tmp & ~AT86RF215_AACK_ENABLE);
            at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__AMCS, tmp);
            break;
        case AT86RF2XX_OPT_TELL_RX_START:
            DEBUG("[at86rf2xx] opt: %s SFD IRQ\n",
                  (state ? "enable" : "disable"));
            tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__IRQ_MASK);
            tmp = (state) ? (tmp |  AT86RF2XX_IRQ_STATUS_MASK__RX_START)
                          : (tmp & ~AT86RF2XX_IRQ_STATUS_MASK__RX_START);
            at86rf215_reg_write(dev, AT86RF2XX_REG__IRQ_MASK, tmp);
            break;
        case AT86RF2XX_OPT_ACK_PENDING:
            DEBUG("[at86rf2xx] opt: enabling pending ACKs\n");
            tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__CSMA_SEED_1);
            tmp = (state) ? (tmp |  AT86RF2XX_CSMA_SEED_1__AACK_SET_PD)
                          : (tmp & ~AT86RF2XX_CSMA_SEED_1__AACK_SET_PD);
            at86rf215_reg_write(dev, AT86RF2XX_REG__CSMA_SEED_1, tmp);
            break;
        default:
            /* do nothing */
            break;
    }
}

void at86rf215_set_tx_frontend(at86rf2xx_t *dev)
{
	uint8_t tmp;

	/*** digital frontend ***/
	/* RCUT | DM: Direct Modulation | SR: TX Sample Rate */
	tmp = at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__TXDFE);
	tmp = 0;
	tmp = (0x4 << 5) | (0x0 << 4) | (0x1);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__TXDFE, tmp);

	/* [FSK] Direct Modulation */
	tmp = 0x0;
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__FSKDM, tmp);

	/*** analog frontend ***/
	/* PARAMP | - | LPFCUT */
	tmp = at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__TXCUTC);
	tmp = 0;
	tmp = (0x1 << 6) | (0x9);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__TXCUTC, tmp);
}

void at86rf215_set_rx_frontend(at86rf2xx_t *dev)
{
	uint8_t tmp;

	/*** analog frontend ***/
	tmp = (0x1 << 4) | (0xb); // 0x7: 800 kHz.
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__RXBWC, tmp);

	/*** digital frontend ***/
	/* RCUT | - | SR: RX Sample Rate */
	tmp = at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__RXDFE);
	tmp = 0;
	tmp = (0x1 << 5) | (0x2);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__RXDFE, tmp);

	/*** AGC ***/
	tmp = 0x0; // 0x1: enable.
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__AGCC, tmp);
	/* target level */
	tmp = at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__AGCS);
	tmp &= ~(AT86RF215_RFn_AGC_TGT_M);
	tmp |= (0x1 << 5);
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__AGCS, tmp);

	/*** EDC ***/
	tmp = 0x3; // 0x3: disable.
	at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__EDC, tmp);
}

void at86rf215_set_bbc(at86rf2xx_t *dev)
{
	uint8_t tmp;

	/********* [FSK] *********/

	/*** [FSKC0] Index and Modulation ***/
	/* BT | MIDXS | MIDX | MOD */
	/* use default (index=1, mod=2FSK) */
	//tmp = ;
	//at86rf215_reg_write(dev, AT86RF215_REG__BBC0_FSKC0, tmp);

	/*** [FSKC1] Symbol Rate ***/
	/* FSKPLH: FSK Preamble Length High Byte | FI | - | SRATE */
	tmp = 0x5; // 0x5: 400 kHz.
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__FSKC1, tmp);

	/*** [FSKC2] ***/
	tmp = at86rf215_reg_read(dev, dev->bbc|AT86RF215_REG__FSKC2);
	/* Receiver Override */
	tmp &= ~(AT86RF215_BBCn_FSK__RXO_M);
	tmp |= (0x3 << 5); // 0x3: disable.
	/* Preamble Time Out */
	tmp &= ~(AT86RF215_BBCn_FSK__RXPTO_M);
	tmp |= (0x1 << 4); // 0x1: enable.
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__FSKC2, tmp);

	/*** FSK Preamble ***/
	/* Length Low Byte */
	tmp = 0xa; // default: 0x8; // for 400kHz: 0xa.
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__FSKPLL, tmp);
	/* preamble detector sensitivity */
//	tmp = at86rf215_reg_read(dev, AT86RF215_REG__FSKC3);
//	tmp &= ~(AT86RF215_BBCn_FSK__PDT_M);
//	tmp |= (0x5); // default: 0x5.
//	at86rf215_reg_write(dev, AT86RF215_REG__FSKC3, tmp);

	/*** SFD | DW ***/
	tmp = (0x0 << 3) | (0x1 << 2); // SFD0 // DW: enable.
	at86rf215_reg_write(dev, dev->bbc|AT86RF215_REG__FSKPHRTX, tmp);
}

/********* State *********/

uint8_t at86rf215_get_state(const at86rf2xx_t *dev)
{
	/* if sleeping immediately return state */
	if (dev->state == AT86RF215_STATE_RF_SLEEP) {
		return dev->state;
	}

	return (at86rf215_reg_read(dev, dev->rf|AT86RF215_REG__STATE)
			& AT86RF215_RFn_STATE_MASK);
}

static inline void _set_state(at86rf2xx_t *dev, uint8_t state)
{
    at86rf215_reg_write(dev, dev->rf|AT86RF215_REG__CMD, state);

    /* possible race condition in RX_AACK_ON state ??? */
	while (at86rf215_get_state(dev) != state) {}

    dev->state = state;
}

uint8_t at86rf215_set_state(at86rf2xx_t *dev, uint8_t state)
{
    uint8_t old_state;

	DEBUG("[rf215] [state] set 0x%x\n", state);

	/* make sure there is no ongoing transmission */
	/* or state transition already in progress */
    do {
        old_state = at86rf215_get_state(dev);
    } while (old_state == AT86RF215_STATE_RF_TRANSITION);

	if (state == old_state) {
		return old_state;
	}

    if (state == AT86RF215_STATE_RF_TRXOFF) {
		DEBUG("[rf215] [state] set TRX_OFF\n");
		_set_state(dev, AT86RF215_STATE_RF_TRXOFF);
    }
    else {
        /* RX <-> TX */
        if ((old_state == AT86RF215_STATE_RF_TX && state == AT86RF215_STATE_RF_RX) ||
			(old_state == AT86RF215_STATE_RF_RX && state == AT86RF215_STATE_RF_TX)) {
            _set_state(dev, AT86RF215_STATE_RF_TXPREP);
        }
        /* check if we need to wake up from sleep mode */
        if (state == AT86RF215_STATE_RF_SLEEP) {
            /* First go to TRX_OFF */
            _set_state(dev, AT86RF215_STATE_RF_TRXOFF);
            /* clear IRQ */
			at86rf215_reg_read(dev, AT86RF215_REG__RF09_IRQS);
			//at86rf215_reg_read(dev, AT86RF215_REG__BBC0_IRQS);
			at86rf215_reg_read(dev, AT86RF215_REG__RF24_IRQS);
            /* Go to SLEEP mode from TRX_OFF */
            gpio_set(dev->params.sleep_pin);
            dev->state = state;
        }
        else {
            if (old_state == AT86RF215_STATE_RF_SLEEP) {
                DEBUG("at86rf2xx: waking up from sleep mode\n");
                at86rf215_assert_awake(dev);
            }
            _set_state(dev, state);
        }
    }

    return old_state;
}
