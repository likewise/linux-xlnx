/******************************************************************************
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
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xv_hdmitxss_coreinit.h
* @addtogroup v_hdmitxss
* @{
* @details
*
* This header file contains the hdmi tx subsystem sub-cores
* initialization routines and helper functions.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00         10/07/15 Initial release.
* 1.1   yh     20/01/16 Added remapper support
* 1.2   MG     03/02/16 Added HDCP support
* 1.3   MH     08/08/16 Updates to optimize out HDCP when excluded.
* </pre>
*
******************************************************************************/
#ifndef XV_HDMITXSS_COREINIT_H__  /* prevent circular inclusions */
#define XV_HDMITXSS_COREINIT_H__  /* by using protection macros */

#ifdef __cplusplus
extern "C" {
#endif
//#include <stdio.h>
#include <linux/string.h>
#include "xv_hdmitxss.h"
#include "xv_hdmitx.h"
#include "xvtc.h"
#ifdef XPAR_XHDCP_NUM_INSTANCES
#include "xtmrctr.h"
#endif
#include "xgpio.h"
#include "xv_axi4s_remap.h"
/************************** Constant Definitions *****************************/

/************************** Function Prototypes ******************************/
int XV_HdmiTxSs_SubcoreInitHdmiTx(XV_HdmiTxSs *HdmiTxSsPtr);
int XV_HdmiTxSs_SubcoreInitVtc(XV_HdmiTxSs *HdmiTxSsPtr);
#ifdef XPAR_XHDCP_NUM_INSTANCES
int XV_HdmiTxSs_SubcoreInitHdcpTimer(XV_HdmiTxSs *HdmiTxSsPtr);
int XV_HdmiTxSs_SubcoreInitHdcp14(XV_HdmiTxSs *HdmiTxSsPtr);
#endif
#ifdef XPAR_XHDCP22_TX_NUM_INSTANCES
int XV_HdmiTxSs_SubcoreInitHdcp22(XV_HdmiTxSs *HdmiTxSsPtr);
#endif
int XV_HdmiTxSs_SubcoreInitRemapperReset(XV_HdmiTxSs *HdmiTxSsPtr);
int XV_HdmiTxSs_SubcoreInitRemapper(XV_HdmiTxSs *HdmiTxSsPtr);

#ifdef __cplusplus
}
#endif

#endif
/** @} */
