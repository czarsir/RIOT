/*
 *
 * taken from AT86RF2xx based driver.
 *
 */

#ifndef _AT86RF215_PARAMS_H
#define _AT86RF215_PARAMS_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif


/*** board configuration ***/

#define AT86RF215_PARAM_SPI         (SPI_DEV(1))
#define AT86RF215_PARAM_SPI_CLK     (SPI_CLK_5MHZ)
#define AT86RF215_PARAM_CS          (GPIO_PIN(PORT_A, 4))
#define AT86RF215_PARAM_INT         (GPIO_PIN(PORT_A, 15))
#define AT86RF215_PARAM_SLEEP       (GPIO_PIN(PORT_B, 8))
#define AT86RF215_PARAM_RESET       (GPIO_PIN(PORT_B, 9))

#define AT86RF215_PARAMS            { .spi = AT86RF215_PARAM_SPI,         \
                                      .spi_clk = AT86RF215_PARAM_SPI_CLK, \
                                      .cs_pin = AT86RF215_PARAM_CS,       \
                                      .int_pin = AT86RF215_PARAM_INT,     \
                                      .sleep_pin = AT86RF215_PARAM_SLEEP, \
                                      .reset_pin = AT86RF215_PARAM_RESET }

/*** Variables ***/

static const at86rf2xx_params_t at86rf2xx_params[] =
{
    AT86RF215_PARAMS
};


#ifdef __cplusplus
}
#endif

#endif
