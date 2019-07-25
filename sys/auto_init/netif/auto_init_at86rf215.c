/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup sys_auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for at86rf2xx network interfaces
 *
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifdef MODULE_AT86RF215

/*** System ***/
#include "log.h"
#include "board.h"
#include "net/gnrc/netif/ieee802154.h"
#ifdef MODULE_GNRC_LWMAC
#include "net/gnrc/lwmac/lwmac.h"
#endif
#ifdef MODULE_GNRC_GOMACH
#include "net/gnrc/gomach/gomach.h"
#endif
#include "net/gnrc.h"

/*** Self ***/
#include "at86rf215.h"
#include "at86rf215_params.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define AT86RF215_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#ifndef AT86RF215_MAC_PRIO
#define AT86RF215_MAC_PRIO          (GNRC_NETIF_PRIO)
#endif

#define AT86RF215_NUM (sizeof(at86rf2xx_params) / sizeof(at86rf2xx_params[0]))

static at86rf2xx_t at86rf215_devs[AT86RF215_NUM][2];
static char _at86rf215_stacks[AT86RF215_NUM][2][AT86RF215_MAC_STACKSIZE];

void auto_init_at86rf215(void)
{
    for (unsigned i = 0; i < AT86RF215_NUM; i++) {
        LOG_DEBUG("[auto_init_netif] initializing at86rf215 #%u\n", i);

        at86rf215_setup(&at86rf215_devs[i][0], &at86rf215_devs[i][1], &at86rf2xx_params[i]);
#if defined(MODULE_GNRC_GOMACH)
//        gnrc_netif_gomach_create(_at86rf2xx_stacks[i],
//                                 AT86RF215_MAC_STACKSIZE,
//                                 AT86RF2XX_MAC_PRIO, "at86rf2xx-gomach",
//                                 (netdev_t *)&at86rf215_devs[i][0]);
#elif defined(MODULE_GNRC_LWMAC)
//        gnrc_netif_lwmac_create(_at86rf2xx_stacks[i],
//                                AT86RF215_MAC_STACKSIZE,
//                                AT86RF2XX_MAC_PRIO, "at86rf2xx-lwmac",
//                                (netdev_t *)&at86rf215_devs[i][0]);
#else
        gnrc_netif_ieee802154_create(_at86rf215_stacks[i][0],
                                     AT86RF215_MAC_STACKSIZE,
                                     AT86RF215_MAC_PRIO, "at86rf215-0",
                                     (netdev_t *)&at86rf215_devs[i][0]);
		gnrc_netif_ieee802154_create(_at86rf215_stacks[i][1],
									 AT86RF215_MAC_STACKSIZE,
									 AT86RF215_MAC_PRIO, "at86rf215-1",
									 (netdev_t *)&at86rf215_devs[i][1]);
#endif
    }
}
#else
typedef int dont_be_pedantic;
#endif /* MODULE_AT86RF215 */

/** @} */
