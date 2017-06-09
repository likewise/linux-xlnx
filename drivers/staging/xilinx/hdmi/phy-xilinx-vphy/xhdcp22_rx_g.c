
/*******************************************************************
*
 *
 * Copyright (C) 2015, 2016, 2017 Xilinx, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details
*
* Description: Driver configuration
*
*******************************************************************/


#include "xhdcp22_rx.h"

/*
* The configuration table for devices
*/

XHdcp22_Rx_Config XHdcp22_Rx_ConfigTable[] =
{
#if XPAR_XHDCP22_RX_NUM_INSTANCES
	{
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_DEVICE_ID,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_BASEADDR,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_PROTOCOL,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_MODE,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_HDCP22_TIMER_DEVICE_ID,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_HDCP22_CIPHER_DEVICE_ID,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_HDCP22_MMULT_DEVICE_ID,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_0_HDCP22_RNG_DEVICE_ID
	}
#endif
};
