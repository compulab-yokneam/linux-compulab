/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DT_BINDINGS_REALTEK_RTL8211X_H
#define _DT_BINDINGS_REALTEK_RTL8211X_H

/* PHY LED status bitmap flags */
#define PHY_LED_ACTIVITY	(1 << 0)
#define PHY_LED_SPEED_10	(1 << 1)
#define PHY_LED_SPEED_100	(1 << 2)
#define PHY_LED_SPEED_1000	(1 << 3)
#define PHY_LED_MASK		0xf

/* Link Up (Any speed) */
#define PHY_LED_LINK		(PHY_LED_SPEED_10  | \
				 PHY_LED_SPEED_100 | \
				 PHY_LED_SPEED_1000)

/* Link Up (Any speed) + Activity (RX/TX) */
#define PHY_LED_LINK_ACTIVITY	(PHY_LED_LINK | PHY_LED_ACTIVITY)
/* Special modes */
#define PHY_LED_OFF		(0)
#define PHY_LED_DEFAULT		(PHY_LED_OFF)

#endif
