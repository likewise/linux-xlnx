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
*
* Description: Driver configuration
*
*******************************************************************/

#include "xparameters.h"
#include "xhdcp22_cipher.h"

/*
* The configuration table for devices
*/

XHdcp22_Cipher_Config XHdcp22_Cipher_ConfigTable[] =
{
#if XPAR_XHDCP22_CIPHER_NUM_INSTANCES
	{
		XPAR_HDMI_RX_SS_HDCP22_RX_SS_V_HDCP22_CIPHER_RX_0_DEVICE_ID,
		XPAR_HDMI_RX_SS_HDCP22_RX_SS_V_HDCP22_CIPHER_RX_0_CPU_BASEADDR
	},
	{
		XPAR_HDMI_TX_SS_HDCP22_TX_SS_V_HDCP22_CIPHER_TX_0_DEVICE_ID,
		XPAR_HDMI_TX_SS_HDCP22_TX_SS_V_HDCP22_CIPHER_TX_0_CPU_BASEADDR
	}
#endif
};
