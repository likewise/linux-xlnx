/******************************************************************************
*
* Copyright (C) 2015 Xilinx, Inc. All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/
/*****************************************************************************/
/**
*
* @file xv_hdmirxss.h
*
* This is main header file of the Xilinx HDMI RX Subsystem driver
*
* <b>HDMI RX Subsystem Overview</b>
*
* Video Subsystem is a collection of IP cores bounded together by software
* to provide an abstract view of the processing pipe. It hides all the
* complexities of programming the underlying cores from end user.
*
* <b>Subsystem Driver Features</b>
*
* Video Subsystem supports following features
*   - AXI Stream Input/Output interface
*   - 1, 2 or 4 pixel-wide video interface
*   - 8/10/12/16 bits per component
*   - RGB & YCbCr color space
*   - Up to 4k2k 60Hz resolution at both Input and Output interface
*   - Interlaced input support (1080i 50Hz/60Hz)
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00         10/07/15 Initial release.
* 1.1   yh     20/01/16 Added remapper support
* 1.2   yh     01/02/16 Added set_ppc api
* 1.3   MG     03/02/16 Added HDCP support
* 1.4   MH     03/15/16 Added HDCP connect event.
*                       Added HDCP authenticated callback support.
* 1.5   YH     17/03/16 Remove xintc.h as it is processor dependent
* </pre>
*
******************************************************************************/

#ifndef HDMIRXSS_H /**< prevent circular inclusions by using protection macros*/
#define HDMIRXSS_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "xstatus.h"
#include "xv_hdmirx.h"
//#include "xtmrctr.h" // -- @NOTE Rohit Consul
//#include "xhdcp1x.h" // -- @NOTE Leon Woestenberg <leon@sidebranch.com>
//#include "xhdcp22_rx.h" // -- @NOTE Leon Woestenberg <leon@sidebranch.com>
#include "xgpio.h"
#include "xv_axi4s_remap.h"

#define XV_HDMIRXSS_HDCP_KEYSEL 0x00u

/****************************** Type Definitions ******************************/
/** @name Handler Types
* @{
*/
/**
* These constants specify different types of handler and used to differentiate
* interrupt requests from peripheral.
*/
typedef enum {
  XV_HDMIRXSS_HANDLER_CONNECT = 1,       /**< A connect event interrupt type */
  XV_HDMIRXSS_HANDLER_AUX,               /**< Interrupt type for AUX peripheral */
  XV_HDMIRXSS_HANDLER_AUD,               /**< Interrupt type for AUD peripheral */
  XV_HDMIRXSS_HANDLER_LNKSTA,            /**< Interrupt type for LNKSTA peripheral */
  XV_HDMIRXSS_HANDLER_DDC,               /**< Interrupt type for DDC peripheral */
  XV_HDMIRXSS_HANDLER_STREAM_DOWN,       /**< Interrupt type for stream down */
  XV_HDMIRXSS_HANDLER_STREAM_INIT,       /**< Interrupt type for stream init */
  XV_HDMIRXSS_HANDLER_STREAM_UP,         /**< Interrupt type for stream up */
  XV_HDMIRXSS_HANDLER_HDCP,              /**< Interrupt type for hdcp */
  XV_HDMIRXSS_HANDLER_HDCP_AUTHENTICATE  /**< Interrupt type for hdcp */
} XV_HdmiRxSs_HandlerType;
/*@}*/

/**
* These constants specify the different protection schemes that can be set
*/
typedef enum
{
  XV_HDMIRXSS_HDCP_NONE,       /**< No content protection */
  XV_HDMIRXSS_HDCP_14,         /**< HDCP 1.4 */
  XV_HDMIRXSS_HDCP_22          /**< HDCP 2.2 */
} XV_HdmiRxSs_HdcpProtocol;

typedef enum
{
  XV_HDMIRXSS_HDCP_NO_EVT,
  XV_HDMIRXSS_HDCP_STREAMUP_EVT,
  XV_HDMIRXSS_HDCP_STREAMDOWN_EVT,
  XV_HDMIRXSS_HDCP_CONNECT_EVT,
  XV_HDMIRXSS_HDCP_DISCONNECT_EVT,
  XV_HDMIRXSS_HDCP_INVALID_EVT
} XV_HdmiRxSs_HdcpEvent;

typedef struct
{
  XV_HdmiRxSs_HdcpEvent Queue[16]; /**< Data */
  u8                    Tail;      /**< Tail pointer */
  u8                    Head;      /**< Head pointer */
} XV_HdmiRxSs_HdcpEventQueue;

/**
* These constants specify the HDCP key types
*/
typedef enum
{
  XV_HDMIRXSS_KEY_HDCP22_LC128,     /**< HDCP 2.2 LC128 */
  XV_HDMIRXSS_KEY_HDCP22_PRIVATE,   /**< HDCP 2.2 Private */
  XV_HDMIRXSS_KEY_HDCP14,           /**< HDCP 1.4 Key */
} XV_HdmiRxSs_HdcpKeyType;

/**
 * Sub-Core Configuration Table
 */
typedef struct
{
  u16 IsPresent;  /**< Flag to indicate if sub-core is present in the design*/
  u16 DeviceId;   /**< Device ID of the sub-core */
  uintptr_t AddrOffset; /**< sub-core offset from subsystem base address */
}XV_HdmiRxSs_SubCore;

/**
 * Video Processing Subsystem configuration structure.
 * Each subsystem device should have a configuration structure associated
 * that defines the MAX supported sub-cores within subsystem
 */

typedef struct
{
  u16 DeviceId;     /**< DeviceId is the unique ID  of the device */
  uintptr_t BaseAddress;  /**< BaseAddress is the physical base address of the
                        subsystem address range */
  uintptr_t HighAddress;  /**< HighAddress is the physical MAX address of the
                        subsystem address range */
  XVidC_PixelsPerClock Ppc;         /**< Supported Pixel per Clock */
  u8 MaxBitsPerPixel;               /**< Maximum  Supported Color Depth */
  XV_HdmiRxSs_SubCore RemapperReset;/**< Sub-core instance configuration */
  XV_HdmiRxSs_SubCore HdcpTimer;    /**< Sub-core instance configuration */
  XV_HdmiRxSs_SubCore Hdcp14;       /**< Sub-core instance configuration */
  XV_HdmiRxSs_SubCore Hdcp22;       /**< Sub-core instance configuration */
  XV_HdmiRxSs_SubCore Remapper;     /**< Sub-core instance configuration */
  XV_HdmiRxSs_SubCore HdmiRx;       /**< Sub-core instance configuration */
} XV_HdmiRxSs_Config;

/**
* Callback type for interrupt.
*
* @param  CallbackRef is a callback reference passed in by the upper
*   layer when setting the callback functions, and passed back to
*   the upper layer when the callback is invoked.
*
* @return None.
*
* @note   None.
*
*/
typedef void (*XV_HdmiRxSs_Callback)(void *CallbackRef);

/**
* The XVprocss driver instance data. The user is required to allocate a variable
* of this type for every XVprocss device in the system. A pointer to a variable
* of this type is then passed to the driver API functions.
*/
typedef struct
{
  XV_HdmiRxSs_Config Config;    /**< Hardware configuration */
  u32 IsReady;                  /**< Device and the driver instance are
                                     initialized */
  XGpio *RemapperResetPtr;        /**< handle to sub-core driver instance */
#if 0
  XTmrCtr *HdcpTimerPtr;           /**< handle to sub-core driver instance */
  XHdcp1x *Hdcp14Ptr;                /**< handle to sub-core driver instance */
  XHdcp22_Rx  *Hdcp22Ptr;           /**< handle to sub-core driver instance */
#endif
  XV_axi4s_remap *RemapperPtr;    /**< handle to sub-core driver instance */
  XV_HdmiRx *HdmiRxPtr;             /**< handle to sub-core driver instance */

  /*Callbacks */
  XV_HdmiRxSs_Callback ConnectCallback; /**< Callback for connect event
                                        * interrupt */
  void *ConnectRef; /**< To be passed to the connect interrupt callback */

  XV_HdmiRxSs_Callback AuxCallback;     /**< Callback for AUX event interrupt */
  void *AuxRef;     /**< To be passed to the AUX interrupt callback */

  XV_HdmiRxSs_Callback AudCallback;     /**< Callback for AUD event interrupt */
  void *AudRef;     /**< To be passed to the Audio interrupt callback */

  XV_HdmiRxSs_Callback LnkStaCallback;  /**< Callback for LNKSTA event
                                        * interrupt */
  void *LnkStaRef;  /**< To be passed to the LNKSTA interrupt callback */

  XV_HdmiRxSs_Callback DdcCallback;     /**< Callback for PDDC interrupt */
  void *DdcRef;     /**< To be passed to the DDC interrupt callback */

  XV_HdmiRxSs_Callback StreamDownCallback; /**< Callback for stream down
                                            * callback */
  void *StreamDownRef;  /**< To be passed to the stream down callback */

  XV_HdmiRxSs_Callback StreamInitCallback;  /**< Callback for stream init
                                            * callback */
  void *StreamInitRef;  /**< To be passed to the stream start callback */

  XV_HdmiRxSs_Callback StreamUpCallback; /**< Callback for stream up callback */
  void *StreamUpRef;    /**< To be passed to the stream up callback */
#if 0
  XV_HdmiRxSs_Callback HdcpCallback;    /**< Callback for hdcp callback */
  void *HdcpRef;        /**< To be passed to the hdcp callback */

  XV_HdmiRxSs_Callback HdcpAuthenticateCallback; /**< Callback for HDCP authenticated */
  void *HdcpAuthenticateRef;  /**< To be passed to authenticated callback */
#endif
  // Scratch pad
  u8 IsStreamConnected;         /**< HDMI RX Stream Connected */
  u8 AudioChannels;             /**< Number of Audio Channels */
  int IsLinkStatusErrMax;       /**< Link Error Status Maxed */
  u8 *EdidPtr;                     /**< Default Edid Pointer */
  u16 EdidLength;               /**< Default Edid Length */
  u8 TMDSClockRatio;            /**< HDMI RX TMDS clock ratio */

    XVidC_DelayHandler UserTimerWaitUs; /**< Custom user function for
                            delay/sleep. */
    void *UserTimerPtr;           /**< Pointer to a timer instance
                            used by the custom user
                            delay/sleep function. */
#if 0
  /**< HDCP specific */
  u32                           HdcpIsReady;    /**< HDCP ready flag */
  XV_HdmiRxSs_HdcpEventQueue    HdcpEventQueue;         /**< HDCP event queue */
  XV_HdmiRxSs_HdcpProtocol      HdcpProtocol;           /**< HDCP protect scheme */
  u8                            *Hdcp22Lc128Ptr;        /**< Pointer to HDCP 2.2 LC128 */
  u8                            *Hdcp22PrivateKeyPtr;   /**< Pointer to HDCP 2.2 Private key */
  u8                            *Hdcp14KeyPtr;          /**< Pointer to HDCP 1.4 key */
#endif
} XV_HdmiRxSs;

/************************** Macros Definitions *******************************/
#define XV_HdmiRxSs_HdcpIsReady(InstancePtr) \
  (InstancePtr)->HdcpIsReady

/************************** Function Prototypes ******************************/
XV_HdmiRxSs_Config* XV_HdmiRxSs_LookupConfig(u32 DeviceId);
void XV_HdmiRxSs_ReportCoreInfo(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_SetUserTimerHandler(XV_HdmiRxSs *InstancePtr,
        XVidC_DelayHandler CallbackFunc, void *CallbackRef);
void XV_HdmiRxSS_HdmiRxIntrHandler(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSS_HdcpIntrHandler(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSS_HdcpTimerIntrHandler(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_CfgInitialize(XV_HdmiRxSs *InstancePtr,
    XV_HdmiRxSs_Config *CfgPtr,
    uintptr_t EffectiveAddr);
void XV_HdmiRxSs_Start(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_Stop(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_Reset(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_SetCallback(XV_HdmiRxSs *InstancePtr,
    u32 HandlerType,
    void *CallbackFunc,
    void *CallbackRef);
void XV_HdmiRxSs_SetEdidParam(XV_HdmiRxSs *InstancePtr, u8 *EdidDataPtr,
                                                                u16 Length);
void XV_HdmiRxSs_LoadDefaultEdid(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_LoadEdid(XV_HdmiRxSs *InstancePtr, u8 *EdidDataPtr,
                                                                u16 Length);
void XV_HdmiRxSs_ToggleHpd(XV_HdmiRxSs *InstancePtr);
XV_HdmiRx_Aux *XV_HdmiRxSs_GetAuxiliary(XV_HdmiRxSs *InstancePtr);
u32 XV_HdmiRxSs_SetStream(XV_HdmiRxSs *InstancePtr,
    u32 Clock,
    u32 LineRate);
XVidC_VideoStream *XV_HdmiRxSs_GetVideoStream(XV_HdmiRxSs *InstancePtr);
u8 XV_HdmiRxSs_GetVideoIDCode(XV_HdmiRxSs *InstancePtr);
u8 XV_HdmiRxSs_GetVideoStreamType(XV_HdmiRxSs *InstancePtr);
u8 XV_HdmiRxSs_GetVideoStreamScramblingFlag(XV_HdmiRxSs *InstancePtr);
u8 XV_HdmiRxSs_GetAudioChannels(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_RefClockChangeInit(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_ReportTiming(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_ReportLinkQuality(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_ReportAudio(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_ReportInfoFrame(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_ReportSubcoreVersion(XV_HdmiRxSs *InstancePtr);
#if 0
int XV_HdmiRxSs_HdcpTimerStart(const XHdcp1x *InstancePtr, u16 TimeoutInMs);
int XV_HdmiRxSs_HdcpTimerStop(const XHdcp1x *InstancePtr);
int XV_HdmiRxSs_HdcpTimerBusyDelay(const XHdcp1x *InstancePtr, u16 DelayInMs);

// HDCP
void XV_HdmiRxSs_HdcpSetKey(XV_HdmiRxSs *InstancePtr, XV_HdmiRxSs_HdcpKeyType KeyType, u8 *KeyPtr);
int XV_HdmiRxSs_HdcpEnable(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpDisable(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpClearEvents(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpPushEvent(XV_HdmiRxSs *InstancePtr, XV_HdmiRxSs_HdcpEvent Event);
int XV_HdmiRxSs_HdcpPoll(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpSetProtocol(XV_HdmiRxSs *InstancePtr, XV_HdmiRxSs_HdcpProtocol Protocol);
XV_HdmiRxSs_HdcpProtocol XV_HdmiRxSs_HdcpGetProtocol(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpIsEnabled(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpIsAuthenticated(XV_HdmiRxSs *InstancePtr);
int XV_HdmiRxSs_HdcpIsEncrypted(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_HdcpInfo(XV_HdmiRxSs *InstancePtr);
void XV_HdmiRxSs_HdcpSetInfoDetail(XV_HdmiRxSs *InstancePtr, u8 Verbose);
#endif

#ifdef __cplusplus
}
#endif

#endif /* end of protection macro */
