/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#include "luid.h"
#include "byteorder.h"
#include "net/ieee802154.h"
#include "net/gnrc.h"
#include "at86rf215_registers.h"
#include "at86rf215_internal.h"
#include "at86rf215_netdev.h"

#define ENABLE_DEBUG (1)
#include "debug.h"


void at86rf215_setup(at86rf2xx_t *dev, const at86rf2xx_params_t *params)
{
    netdev_t *netdev = (netdev_t *)dev;

    netdev->driver = &at86rf2xx_driver;
    /* initialize device descriptor */
    memcpy(&dev->params, params, sizeof(at86rf2xx_params_t));
    dev->idle_state = AT86RF215_STATE_RF_TRXOFF;
    /* radio state is P_ON when first powered-on */
    dev->state = AT86RF2XX_STATE_P_ON;
    dev->pending_tx = 0;
}

void at86rf215_reset(at86rf2xx_t *dev)
{
    eui64_t addr_long;
	uint8_t tmp;

	DEBUG("[rf215] -- reset\n");

	DEBUG("[rf215] -- reset : hardware reset\n");
    at86rf215_hardware_reset(dev);

	DEBUG("[rf215] -- reset : ieee reset\n");
    netdev_ieee802154_reset(&dev->netdev);

    /* Reset state machine to ensure a known state */
    //if (dev->state == AT86RF2XX_STATE_P_ON) {
	//	DEBUG("[rf215] -- reset : set state (TRX OFF)\n");
    //    at86rf2xx_set_state(dev, AT86RF2XX_STATE_FORCE_TRX_OFF);
    //}

	DEBUG("[rf215] -- reset : set config\n");
    /* get an 8-byte unique ID to use as hardware address */
    luid_get(addr_long.uint8, IEEE802154_LONG_ADDRESS_LEN);
    /* make sure we mark the address as non-multicast and not globally unique */
    addr_long.uint8[0] &= ~(0x01);
    addr_long.uint8[0] |=  (0x02);
    /* set short and long address */
    at86rf215_set_addr_long(dev, ntohll(addr_long.uint64.u64));
    at86rf215_set_addr_short(dev, ntohs(addr_long.uint16[0].u16));

    /* set default PAN id */
    at86rf215_set_pan(dev, AT86RF2XX_DEFAULT_PANID);
    /* set default channel */
    at86rf215_set_chan(dev, AT86RF2XX_DEFAULT_CHANNEL);
    /* set default TX power */
    at86rf2xx_set_txpower(dev, AT86RF2XX_DEFAULT_TXPOWER);
    /* set default options */
    at86rf2xx_set_option(dev, AT86RF215_OPT_AUTOACK, true);
//    at86rf2xx_set_option(dev, AT86RF2XX_OPT_CSMA, true);

	/*** Frame Filter ***/
	/* using Promiscuous mode, easy for test. */
	at86rf2xx_set_option(dev, AT86RF215_OPT_PROMISCUOUS, true);

    /* enable safe mode (protect RX FIFO until reading data starts) */
//    at86rf215_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_2,
//                        AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);
#ifdef MODULE_AT86RF212B
    at86rf2xx_set_page(dev, AT86RF2XX_DEFAULT_PAGE);
#endif

    /* don't populate masked interrupt flags to IRQ_STATUS register */
//    tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__TRX_CTRL_1);
//    tmp &= ~(AT86RF2XX_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
//    at86rf215_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_1, tmp);

    /* disable clock output to save power */
    tmp = at86rf215_reg_read(dev, AT86RF2XX_REG__TRX_CTRL_0);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
//    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
//    tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
//    at86rf215_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
	at86rf215_reg_write(dev, AT86RF215_REG__BBC0_IRQM, AT86RF215_BBCn_IRQM__RXFE_M);
    /* clear interrupt flags */
	at86rf215_reg_read(dev, AT86RF215_REG__BBC0_IRQS);

	DEBUG("[rf215] -- reset : set state (TODO: stay TRXOFF)\n");
    /* go into RX state ? or TXPREP ? or stay TRXOFF ? */
    //at86rf2xx_set_state(dev, AT86RF2XX_STATE_RX_AACK_ON);

    DEBUG("[rf215] -- reset : complete.\n");
}

size_t at86rf2xx_send(at86rf2xx_t *dev, const uint8_t *data, size_t len)
{
    /* check data length */
    if (len > AT86RF2XX_MAX_PKT_LENGTH) {
        DEBUG("[at86rf2xx] Error: data to send exceeds max packet size\n");
        return 0;
    }
    at86rf2xx_tx_prepare(dev);
    at86rf2xx_tx_load(dev, data, len, 0);
    at86rf2xx_tx_exec(dev);
    return len;
}

void at86rf2xx_tx_prepare(at86rf2xx_t *dev)
{
//    uint8_t state;

	DEBUG("[rf215] -- tx_prepare \n");

    dev->pending_tx++;
	DEBUG("[rf215] -- tx_prepare : set state (TX ARET ON)\n");
    //state = at86rf2xx_set_state(dev, AT86RF2XX_STATE_TX_ARET_ON);
	uint8_t tmp = at86rf215_reg_read(dev, AT86RF215_REG__BBC0_AMCS);
	tmp |= AT86RF215_CCATX_ENABLE;
	at86rf215_reg_write(dev, AT86RF215_REG__BBC0_AMCS, tmp);
//    if (state != AT86RF2XX_STATE_TX_ARET_ON) {
//        dev->idle_state = state;
//    }
    dev->tx_frame_len = IEEE802154_FCS_LEN;
	DEBUG("[rf215] -- tx_prepare : complete.\n");
}

size_t at86rf2xx_tx_load(at86rf2xx_t *dev, const uint8_t *data,
                         size_t len, size_t offset)
{
    dev->tx_frame_len += (uint8_t)len;
    at86rf2xx_sram_write(dev, offset + 1, data, len);
    return offset + len;
}

void at86rf2xx_tx_exec(const at86rf2xx_t *dev)
{
    netdev_t *netdev = (netdev_t *)dev;
	uint8_t tmp;

    /* write frame length field in FIFO */
    //at86rf2xx_sram_write(dev, 0, &(dev->tx_frame_len), 1);
	at86rf215_reg_write(dev, AT86RF215_REG__BBC0_TXFLH, 0);
	at86rf215_reg_write(dev, AT86RF215_REG__BBC0_TXFLL, dev->tx_frame_len);
    /* trigger sending of pre-loaded frame */
    at86rf215_reg_write(dev, AT86RF215_REG__RF09_CMD,
                        AT86RF215_STATE_RF_TX);
    if (netdev->event_callback &&
        (dev->netdev.flags & AT86RF2XX_OPT_TELL_TX_START)) {
        netdev->event_callback(netdev, NETDEV_EVENT_TX_STARTED);
    }

	/*** wait until end ***/
	do {
		tmp = at86rf215_reg_read(dev, AT86RF215_REG__BBC0_IRQS);
	} while ( !(tmp|AT86RF215_BBCn_IRQS__TXFE_M) );

	/*** enable baseband ***/
	tmp = at86rf215_reg_read(dev, AT86RF215_REG__BBC0_PC);
	tmp |= AT86RF215_BBEN_ENABLE;
	at86rf215_reg_write(dev, AT86RF215_REG__BBC0_PC, tmp);
}

bool at86rf2xx_cca(at86rf2xx_t *dev)
{
    uint8_t reg;
    uint8_t old_state = at86rf2xx_set_state(dev, AT86RF215_STATE_RF_TXPREP);
    /* Disable RX path */
    uint8_t rx_syn = at86rf215_reg_read(dev, AT86RF2XX_REG__RX_SYN);

    reg = rx_syn | AT86RF2XX_RX_SYN__RX_PDT_DIS;
    at86rf215_reg_write(dev, AT86RF2XX_REG__RX_SYN, reg);
    /* Manually triggered CCA is only possible in RX_ON (basic operating mode) */
    at86rf2xx_set_state(dev, AT86RF2XX_STATE_RX_ON);
    /* Perform CCA */
    reg = at86rf215_reg_read(dev, AT86RF2XX_REG__PHY_CC_CCA);
    reg |= AT86RF2XX_PHY_CC_CCA_MASK__CCA_REQUEST;
    at86rf215_reg_write(dev, AT86RF2XX_REG__PHY_CC_CCA, reg);
    /* Spin until done (8 symbols + 12 µs = 128 µs + 12 µs for O-QPSK)*/
    do {
        reg = at86rf215_reg_read(dev, AT86RF2XX_REG__TRX_STATUS);
    } while ((reg & AT86RF2XX_TRX_STATUS_MASK__CCA_DONE) == 0);
    /* return true if channel is clear */
    bool ret = !!(reg & AT86RF2XX_TRX_STATUS_MASK__CCA_STATUS);
    /* re-enable RX */
    at86rf215_reg_write(dev, AT86RF2XX_REG__RX_SYN, rx_syn);
    /* Step back to the old state */
    at86rf2xx_set_state(dev, AT86RF215_STATE_RF_TXPREP);
    at86rf2xx_set_state(dev, old_state);
    return ret;
}
