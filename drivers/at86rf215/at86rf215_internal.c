/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "at86rf215_internal.h"
#include "at86rf215_registers.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define SPIDEV          (dev->params.spi)
#define CSPIN           (dev->params.cs_pin)

static inline void getbus(const at86rf2xx_t *dev)
{
    spi_acquire(SPIDEV, CSPIN, SPI_MODE_0, dev->params.spi_clk);
}

void at86rf215_reg_write(const at86rf2xx_t *dev, uint16_t addr, uint8_t value)
{
    uint16_t cmd = (AT86RF215_ACCESS_WRITE | addr);

    getbus(dev);
	/*** must be MSB first ***/
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
	//spi_transfer_bytes(SPIDEV, CSPIN, true, &cmd, NULL, 2);
	spi_transfer_bytes(SPIDEV, CSPIN, false, &value, NULL, 1);
    spi_release(SPIDEV);
}

uint8_t at86rf215_reg_read(const at86rf2xx_t *dev, uint16_t addr)
{
    uint16_t cmd = (AT86RF215_ACCESS_READ | addr);
    uint8_t value;

    getbus(dev);
	/*** must be MSB first ***/
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd>>8));
	spi_transfer_byte(SPIDEV, CSPIN, true, (uint8_t)(cmd));
	//spi_transfer_bytes(SPIDEV, CSPIN, true, &cmd, NULL, 2);
	value = spi_transfer_byte(SPIDEV, CSPIN, false, 0);
    spi_release(SPIDEV);

    return value;
}

void at86rf2xx_sram_read(const at86rf2xx_t *dev, uint8_t offset,
                         uint8_t *data, size_t len)
{
    uint8_t reg = (AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_READ);

    getbus(dev);
    spi_transfer_byte(SPIDEV, CSPIN, true, reg);
    spi_transfer_byte(SPIDEV, CSPIN, true, offset);
    spi_transfer_bytes(SPIDEV, CSPIN, false, NULL, data, len);
    spi_release(SPIDEV);
}

void at86rf2xx_sram_write(const at86rf2xx_t *dev, uint8_t offset,
                          const uint8_t *data, size_t len)
{
    uint8_t reg = (AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_WRITE);

    getbus(dev);
    spi_transfer_byte(SPIDEV, CSPIN, true, reg);
    spi_transfer_byte(SPIDEV, CSPIN, true, offset);
    spi_transfer_bytes(SPIDEV, CSPIN, false, data, NULL, len);
    spi_release(SPIDEV);
}

void at86rf2xx_fb_start(const at86rf2xx_t *dev)
{
    uint8_t reg = AT86RF2XX_ACCESS_FB | AT86RF2XX_ACCESS_READ;

    getbus(dev);
    spi_transfer_byte(SPIDEV, CSPIN, true, reg);
}

void at86rf2xx_fb_read(const at86rf2xx_t *dev,
                       uint8_t *data, size_t len)
{
    spi_transfer_bytes(SPIDEV, CSPIN, true, NULL, data, len);
}

void at86rf2xx_fb_stop(const at86rf2xx_t *dev)
{
    /* transfer one byte (which we ignore) to release the chip select */
    spi_transfer_byte(SPIDEV, CSPIN, false, 1);
    spi_release(SPIDEV);
}

uint8_t at86rf2xx_get_status(const at86rf2xx_t *dev)
{
    /* if sleeping immediately return state */
    if (dev->state == AT86RF2XX_STATE_SLEEP) {
        return dev->state;
    }

    return (at86rf215_reg_read(dev, AT86RF215_REG__RF09_STATE)
            & AT86RF215_RFn_STATE_MASK);
}

void at86rf2xx_assert_awake(at86rf2xx_t *dev)
{
    if (at86rf2xx_get_status(dev) == AT86RF2XX_STATE_SLEEP) {
        /* wake up and wait for transition to TRX_OFF */
        gpio_clear(dev->params.sleep_pin);
        xtimer_usleep(AT86RF2XX_WAKEUP_DELAY);

        /* update state: on some platforms, the timer behind xtimer
         * may be inaccurate or the radio itself may take longer
         * to wake up due to extra capacitance on the oscillator.
         * Spin until we are actually awake
         */
        do {
            dev->state = at86rf215_reg_read(dev, AT86RF2XX_REG__TRX_STATUS)
                         & AT86RF2XX_TRX_STATUS_MASK__TRX_STATUS;
        } while (dev->state != AT86RF2XX_TRX_STATUS__TRX_OFF);
    }
}

void at86rf2xx_hardware_reset(at86rf2xx_t *dev)
{
	DEBUG("[rf215] -- -- hardware_reset\n");

    /* trigger hardware reset */
    gpio_clear(dev->params.reset_pin);
    xtimer_usleep(AT86RF2XX_RESET_PULSE_WIDTH);
    gpio_set(dev->params.reset_pin);
    xtimer_usleep(AT86RF2XX_RESET_DELAY);

	/*** test ***/
	dev->state = at86rf215_reg_read(dev, AT86RF215_REG__RF09_STATE)
					& AT86RF215_RFn_STATE_MASK;
	DEBUG("[rf215] -- -- hardware_reset : state 0x%x\n", dev->state);

    /* update state: if the radio state was P_ON (initialization phase),
     * it remains P_ON. Otherwise, it should go to TRX_OFF
     */
    do {
        dev->state = at86rf215_reg_read(dev, AT86RF215_REG__RF09_STATE)
                     & AT86RF215_RFn_STATE_MASK;
    } while ((dev->state != AT86RF215_STATE_RF_TRXOFF)
             && (dev->state != AT86RF215_STATE_RF_RESET));
}

void at86rf2xx_configure_phy(at86rf2xx_t *dev)
{
    /* we must be in TRX_OFF before changing the PHY configuration */
    uint8_t prev_state = at86rf2xx_set_state(dev, AT86RF215_CMD_RF_TRX_OFF);

#ifdef MODULE_AT86RF212B
    /* The TX power register must be updated after changing the channel if
     * moving between bands. */
    int16_t txpower = at86rf2xx_get_txpower(dev);

    uint8_t trx_ctrl2 = at86rf215_reg_read(dev, AT86RF2XX_REG__TRX_CTRL_2);
    uint8_t rf_ctrl0 = at86rf215_reg_read(dev, AT86RF2XX_REG__RF_CTRL_0);

    /* Clear previous configuration for PHY mode */
    trx_ctrl2 &= ~(AT86RF2XX_TRX_CTRL_2_MASK__FREQ_MODE);
    /* Clear previous configuration for GC_TX_OFFS */
    rf_ctrl0 &= ~AT86RF2XX_RF_CTRL_0_MASK__GC_TX_OFFS;

    if (dev->netdev.chan != 0) {
        /* Set sub mode bit on 915 MHz as recommended by the data sheet */
        trx_ctrl2 |= AT86RF2XX_TRX_CTRL_2_MASK__SUB_MODE;
    }

    if (dev->page == 0) {
        /* BPSK coding */
        /* Data sheet recommends using a +2 dB setting for BPSK */
        rf_ctrl0 |= AT86RF2XX_RF_CTRL_0_GC_TX_OFFS__2DB;
    }
    else if (dev->page == 2) {
        /* O-QPSK coding */
        trx_ctrl2 |= AT86RF2XX_TRX_CTRL_2_MASK__BPSK_OQPSK;
        /* Data sheet recommends using a +1 dB setting for O-QPSK */
        rf_ctrl0 |= AT86RF2XX_RF_CTRL_0_GC_TX_OFFS__1DB;
    }

    at86rf215_reg_write(dev, AT86RF2XX_REG__TRX_CTRL_2, trx_ctrl2);
    at86rf215_reg_write(dev, AT86RF2XX_REG__RF_CTRL_0, rf_ctrl0);
#endif

    uint8_t phy_cc_cca = at86rf215_reg_read(dev, AT86RF2XX_REG__PHY_CC_CCA);
    /* Clear previous configuration for channel number */
    phy_cc_cca &= ~(AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);

    /* Update the channel register */
    phy_cc_cca |= (dev->netdev.chan & AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
    at86rf215_reg_write(dev, AT86RF2XX_REG__PHY_CC_CCA, phy_cc_cca);

#ifdef MODULE_AT86RF212B
    /* Update the TX power register to achieve the same power (in dBm) */
    at86rf2xx_set_txpower(dev, txpower);
#endif

    /* Return to the state we had before reconfiguring */
    at86rf2xx_set_state(dev, prev_state);
}

#if defined(MODULE_AT86RF233) || defined(MODULE_AT86RF231)
void at86rf2xx_get_random(const at86rf2xx_t *dev, uint8_t *data, size_t len)
{
    for (size_t byteCount = 0; byteCount < len; ++byteCount) {
        uint8_t rnd = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            /* bit 5 and 6 of the AT86RF2XX_REG__PHY_RSSI register contain the RND_VALUE */
            uint8_t regVal = at86rf215_reg_read(dev, AT86RF2XX_REG__PHY_RSSI)
                             & AT86RF2XX_PHY_RSSI_MASK__RND_VALUE;
            /* shift the two random bits first to the right and then to the correct position of the return byte */
            regVal = regVal >> 5;
            regVal = regVal << 2 * i;
            rnd |= regVal;
        }
        data[byteCount] = rnd;
    }
}
#endif
