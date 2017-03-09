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
#include "xhdcp22_mmult.h"

/*
* The configuration table for devices
*/

XHdcp22_mmult_Config XHdcp22_mmult_ConfigTable[] =
{
#if XPAR_XHDCP22_MMULT_NUM_INSTANCES
	{
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_SS_HDCP22_MMULT_DEVICE_ID,
		XPAR_V_HDMI_RX_SS_0_HDCP22_RX_SS_HDCP22_MMULT_BASEADDR
	}
#endif
};
