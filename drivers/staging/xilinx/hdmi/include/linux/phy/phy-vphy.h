/*
 * Xilinx VPHY header
 *
 * Copyright (C) 2016 Xilinx, Inc.
 *
 * Author: Leon Woestenberg <leon@sidebranch.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PHY_VPHY_H_
#define _PHY_VPHY_H_

/* @TODO change directory name on production release */
#include "xvphy.h"

struct phy;

#if (1 || defined(CONFIG_PHY_XILINX_VPHY))

extern XVphy *xvphy_get_xvphy(struct phy *phy);
extern void xvphy_mutex_lock(struct phy *phy);
extern void xvphy_mutex_unlock(struct phy *phy);
extern int xvphy_do_something(struct phy *phy);

#if 0
extern XVphy_PllType xvphy_get_plltype(struct phy *phy, u8 QuadId,
						 XVphy_DirectionType Dir, XVphy_ChannelId ChId);
extern int xvphy_set_hdmi_callback(struct phy *phy, XVphy_HdmiHandlerType HandlerType,
					   void *CallbackFunc, void *CallbackRef);
#endif

#else
static inline XVphy *xvphy_get_xvphy(struct phy *phy)
{
	return NULL;
}

static inline void xvphy_mutex_lock(struct phy *phy) {}
static inline void xvphy_mutex_unlock(struct phy *phy) {}

static inline int xvphy_do_something(struct phy *phy)
{
	return -ENODEV;
}

#if 0
XVphy_PllType xvphy_get_plltype(struct phy *phy, u8 QuadId,
						 XVphy_DirectionType Dir, XVphy_ChannelId ChId)
{
	return 0;
}

static inline int int xvphy_set_hdmi_callback(struct phy *phy, XVphy_HdmiHandlerType HandlerType,
					   void *CallbackFunc, void *CallbackRef)
{
	return -ENODEV;
}
#endif

#endif /* (1 || defined(CONFIG_PHY_XILINX_VPHY)) */

#endif /* _PHY_VPHY_H_ */
