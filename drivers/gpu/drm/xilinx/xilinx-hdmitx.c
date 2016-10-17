/*
 * Xilinx Video HDMI TX Subsystem driver
 *
 * Copyright (C) 2016 Leon Woestenberg <leon@sidebranch.com>
 * Copyright (C) 2016 Xilinx, Inc.
 *
 * Author: Leon Woestenberg <leon@sidebranch.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/interrupt.h>

#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include "linux/delay.h"

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "xilinx-hdmi-tx/xv_hdmitxss.h"
#include "xilinx-hdmi-tx/xil_printf.h"
#include "xilinx-hdmi-tx/xstatus.h"

#define NUM_SUBCORE_IRQ 2
#define HDMI_MAX_LANES	4

#if 0
#define mutex_lock(x)
#define mutex_unlock(x)
#define xvphy_mutex_lock(x)
#define xvphy_mutex_unlock(x)
#endif

struct xhdmitx_device {
	struct device *dev;
	void __iomem *iomem;
	/* video streaming bus clock */
	struct clk *clk;

	/* interrupt number */
	int irq;
	bool teardown;

	struct phy *phy[HDMI_MAX_LANES];

	/* mutex to prevent concurrent access to this structure */
	struct mutex xhdmitx_mutex;
	/* protects concurrent access from interrupt context */
	spinlock_t irq_lock;
	/* schedule (future) work */
	struct workqueue_struct *work_queue;
	struct delayed_work delayed_work_enable_hotplug;
	/* input clock */
	struct clk *tx_clk;
	bool cable_connected;
	bool hdmi_stream;

	/* configuration for the baseline subsystem driver instance */
	XV_HdmiTxSs_Config config;
	/* bookkeeping for the baseline subsystem driver instance */
	XV_HdmiTxSs xv_hdmitxss;
	/* sub core interrupt status registers */
	u32 IntrStatus[NUM_SUBCORE_IRQ];
	/* pointer to xvphy */
	XVphy *xvphy;
};

void HdmiTx_PioIntrHandler(XV_HdmiTx *InstancePtr);
//void HdmiTx_TmrIntrHandler(XV_HdmiTx *InstancePtr);
//void HdmiTx_VtdIntrHandler(XV_HdmiTx *InstancePtr);
void HdmiTx_DdcIntrHandler(XV_HdmiTx *InstancePtr);
void HdmiTx_AuxIntrHandler(XV_HdmiTx *InstancePtr);
//void HdmiTx_AudIntrHandler(XV_HdmiTx *InstancePtr);
//void HdmiTx_LinkStatusIntrHandler(XV_HdmiTx *InstancePtr);

void XV_HdmiTxSs_IntrEnable(XV_HdmiTxSs *HdmiTxSsPtr)
{
	XV_HdmiTx_PioIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_TmrIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_VtdIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
	XV_HdmiTx_DdcIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_AuxIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_AudioIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
	//XV_HdmiTx_LinkIntrEnable(HdmiTxSsPtr->HdmiTxPtr);
}

void XV_HdmiTxSs_IntrDisable(XV_HdmiTxSs *HdmiTxSsPtr)
{
	XV_HdmiTx_PioIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_TmrIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_VtdIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
	XV_HdmiTx_DdcIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_AuxIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_AudioIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
//	XV_HdmiTx_LinkIntrDisable(HdmiTxSsPtr->HdmiTxPtr);
}

/* XV_HdmiTx_IntrHandler */
static irqreturn_t hdmitx_irq_handler(int irq, void *dev_id)
{
	struct xhdmitx_device *xhdmitx;
	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;
	BUG_ON(!dev_id);
	xhdmitx = (struct xhdmitx_device *)dev_id;
	//printk(KERN_INFO "hdmitx_irq_handler()\n");
	if (!xhdmitx) {
		static int fault_count = 0;
		fault_count++;
		if (fault_count)
		dev_err(xhdmitx->dev, "irq_handler: !dev_id\n");
		spin_lock_irqsave(&xhdmitx->irq_lock, flags);
		/* mask interrupt request */
		XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
		spin_unlock_irqrestore(&xhdmitx->irq_lock, flags);
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	if (HdmiTxSsPtr->IsReady != XIL_COMPONENT_IS_READY) {
		printk(KERN_INFO "hdmitx_irq_handler(): HDMI TX SS is not initialized?!\n");
	}

#if 1
	/* read status registers */
	xhdmitx->IntrStatus[0] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_PIO_STA_OFFSET)) & (XV_HDMITX_PIO_STA_IRQ_MASK);
	xhdmitx->IntrStatus[1] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_DDC_STA_OFFSET)) & (XV_HDMITX_DDC_STA_IRQ_MASK);
#endif

	spin_lock_irqsave(&xhdmitx->irq_lock, flags);
	/* mask interrupt request */
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmitx->irq_lock, flags);

	/* call bottom-half */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hdmitx_irq_thread(int irq, void *dev_id)
{
	static int irq_count = 0;
	struct xhdmitx_device *xhdmitx;
	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;
	int i;
	char which[NUM_SUBCORE_IRQ + 1] = "012";
	int which_mask = 0;
	u32 Event, Data;
	static u32 OldData;
	static int count = 0;
	//printk(KERN_INFO "hdmitx_irq_thread()\n");

	BUG_ON(!dev_id);
	xhdmitx = (struct xhdmitx_device *)dev_id;
	if (!xhdmitx) {
		printk(KERN_INFO "irq_thread: !dev_id\n");
		return IRQ_HANDLED;
	}
	if (xhdmitx->teardown) {
		printk(KERN_INFO "irq_thread: teardown\n");
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmitx->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	mutex_lock(&xhdmitx->xhdmitx_mutex);
	/* call baremetal interrupt handler, this in turn will
	 * call the registed callbacks functions */

#if 0
	for (i = 0; i < NUM_SUBCORE_IRQ; i++) {
		which[i] = xhdmitx->IntrStatus[i]? '0' + i: '.';
		which_mask |= (xhdmitx->IntrStatus[i]? 1: 0) << i;
	}
	which[NUM_SUBCORE_IRQ] = 0;
#endif
	Data = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress,
		(XV_HDMITX_PIO_IN_OFFSET));
	count++;
	if ((Data != OldData) || ((count % 100) == 0)) {
		printk(KERN_INFO "PIO.DAT = 0x%08x, count = %d\n", (int)Data, count);
		OldData = Data;
	}
#if 0
	printk(KERN_INFO "PIO.EVT = 0x%08x, PIO.DAT = 0x%08x, DDC.EVT = 0x%08x\n",
	 xhdmitx->IntrStatus[0], (int)Data, xhdmitx->IntrStatus[0]);
#endif

	if (xhdmitx->IntrStatus[0]) HdmiTx_PioIntrHandler(HdmiTxSsPtr->HdmiTxPtr);
	if (xhdmitx->IntrStatus[1]) HdmiTx_DdcIntrHandler(HdmiTxSsPtr->HdmiTxPtr);

	//printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x\n", which, (int)which_mask);

	mutex_unlock(&xhdmitx->xhdmitx_mutex);

	spin_lock_irqsave(&xhdmitx->irq_lock, flags);
	/* unmask interrupt request */
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmitx->irq_lock, flags);
	//printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x done\n", which, (int)which_mask);

	return IRQ_HANDLED;
}

static void EnableColorBar(struct xhdmitx_device *xhdmitx,
	XVidC_VideoMode      VideoMode,
	XVidC_ColorFormat    ColorFormat,
	XVidC_ColorDepth     Bpc)
{
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;
	u32 TmdsClock = 0;
	u32 Result;
	u32 PixelClock;
	unsigned long tx_clk_rate;

	BUG_ON(!xhdmitx);

	HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmitx->xvphy;
	BUG_ON(!VphyPtr);

	mutex_lock(&xhdmitx->xhdmitx_mutex);
	xvphy_mutex_lock(xhdmitx->phy[0]);
	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);

	if (XVphy_IsBonded(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1)) {
		dev_info(xhdmitx->dev, "Both the GT RX and GT TX are clocked by the RX reference clock.\n");
	}

	if (VideoMode < XVIDC_VM_NUM_SUPPORTED) {
		dev_info(xhdmitx->dev, "Starting colorbar\n\r");

		// Disable TX TDMS clock
		XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (FALSE));

		// Get pixel clock
		PixelClock = XVidC_GetPixelClockHzByVmId(VideoMode);

		// In YUV420 the pixel clock is actually the half of the
		// reported pixel clock
		if (ColorFormat == XVIDC_CSF_YCRCB_420) {
			PixelClock = PixelClock / 2;
		}
	}

	TmdsClock = XV_HdmiTxSs_SetStream(HdmiTxSsPtr, VideoMode, ColorFormat, Bpc, NULL);

	// Set TX reference clock
	VphyPtr->HdmiTxRefClkHz = TmdsClock;

	// Set GT TX parameters
	Result = XVphy_SetHdmiTxParam(VphyPtr, 0, XVPHY_CHANNEL_ID_CHA,
					HdmiTxSsVidStreamPtr->PixPerClk,
					HdmiTxSsVidStreamPtr->ColorDepth,
					HdmiTxSsVidStreamPtr->ColorFormatId);

	if (Result == (XST_FAILURE)) {
		dev_info(xhdmitx->dev, "Unable to set requested TX video resolution.\n\r");
	}

	/* Disable RX clock forwarding */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_RX, (FALSE));

	xvphy_mutex_unlock(xhdmitx->phy[0]);
	mutex_unlock(&xhdmitx->xhdmitx_mutex);

	dev_info(xhdmitx->dev, "tx-clk: setting rate to VphyPtr->HdmiTxRefClkHz = %u\n", VphyPtr->HdmiTxRefClkHz);

	tx_clk_rate = clk_set_rate(xhdmitx->tx_clk, VphyPtr->HdmiTxRefClkHz);

	tx_clk_rate = clk_get_rate(xhdmitx->tx_clk);
	dev_info(xhdmitx->dev, "tx-clk rate = %lu\n", tx_clk_rate);
}

static void TxConnectCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	XVphy *VphyPtr = xhdmitx->xvphy;
	BUG_ON(!xhdmitx);
	BUG_ON(!HdmiTxSsPtr);
	BUG_ON(!VphyPtr);
	BUG_ON(!xhdmitx->phy[0]);
	dev_info(xhdmitx->dev, "TxConnectCallback():\n");

	xvphy_mutex_lock(xhdmitx->phy[0]);
	if (HdmiTxSsPtr->IsStreamConnected) {
		dev_info(xhdmitx->dev, "TxConnectCallback(): TX connected\n");
		/* Check HDMI sink version */
		XV_HdmiTxSs_DetectHdmi20(HdmiTxSsPtr);
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (TRUE));

		dev_info(xhdmitx->dev, "TxConnectCallback(): EnableColorBar()\n");

		EnableColorBar(xhdmitx, XVIDC_VM_1920x1080_60_P, XVIDC_CSF_RGB, XVIDC_BPC_8);


	}
	else {
		dev_info(xhdmitx->dev, "TxConnectCallback(): TX disconnected\n");
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (FALSE));
	}
	xvphy_mutex_unlock(xhdmitx->phy[0]);
}

static void TxStreamUpCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;

	BUG_ON(!xhdmitx);

	HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmitx->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(xhdmitx->dev, "TxStreamUpCallback(): TX stream is up\n");

	XVphy_PllType TxPllType;
	u64 TxLineRate;


#if 0
  XVidC_VideoStream *HdmiTxSsVidStreamPtr;

  HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(&HdmiTxSs);

  /* In passthrough copy the RX stream parameters to the TX stream */
  if (IsPassThrough) {
	  XV_HdmiTxSs_SetVideoStream(HdmiTxSsPtr, *HdmiTxSsVidStreamPtr);
  }
#endif

	xvphy_mutex_lock(xhdmitx->phy[0]);
	TxPllType = XVphy_GetPllType(VphyPtr, 0, XVPHY_DIR_TX, XVPHY_CHANNEL_ID_CH1);
	if ((TxPllType == XVPHY_PLL_TYPE_CPLL)) {
		TxLineRate = VphyPtr->Quads[0].Plls[0].LineRateHz;
	}
	else if((TxPllType == XVPHY_PLL_TYPE_QPLL) ||
		(TxPllType == XVPHY_PLL_TYPE_QPLL0) ||
		(TxPllType == XVPHY_PLL_TYPE_PLL0)) {
		TxLineRate = VphyPtr->Quads[0].Plls[XVPHY_CHANNEL_ID_CMN0 -
			XVPHY_CHANNEL_ID_CH1].LineRateHz;
	}
	else {
		TxLineRate = VphyPtr->Quads[0].Plls[XVPHY_CHANNEL_ID_CMN1 -
			XVPHY_CHANNEL_ID_CH1].LineRateHz;
	}

	/* Enable TX TMDS clock*/
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (TRUE));
	xvphy_mutex_unlock(xhdmitx->phy[0]);

	/* Copy Sampling Rate */
	XV_HdmiTxSs_SetSamplingRate(HdmiTxSsPtr, VphyPtr->HdmiTxSampleRate);

#if 0
	/* Enable audio generator */
	XhdmiAudGen_Start(&AudioGen, TRUE);

	/* Select ACR from ACR Ctrl */
	XhdmiACRCtrl_Sel(&AudioGen, ACR_SEL_GEN);

	/* Enable 2-channel audio */
	XhdmiAudGen_SetEnabChannels(&AudioGen, 2);
	XhdmiAudGen_SetPattern(&AudioGen, 1, XAUD_PAT_PING);
	XhdmiAudGen_SetPattern(&AudioGen, 2, XAUD_PAT_PING);
	XhdmiAudGen_SetSampleRate(&AudioGen,
					XV_HdmiTxSs_GetTmdsClockFreqHz(HdmiTxSsPtr),
					XAUD_SRATE_48K);
	}

	/* HDMI TX unmute audio */
	XV_HdmiTxSs_AudioMute(HdmiTxSsPtr, FALSE);
#endif
	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);
	XVidC_ReportStreamInfo(HdmiTxSsVidStreamPtr);
}


static void TxStreamDownCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;

	BUG_ON(!xhdmitx);

	HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmitx->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(xhdmitx->dev, "TX stream is down\n\r");
}

static void TxVsCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	XVphy *VphyPtr = xhdmitx->xvphy;
	BUG_ON(!xhdmitx);
	BUG_ON(!VphyPtr);
#if 0
  /* Audio Infoframe */
  /* Only when not in pass-through */
  if (!IsPassThrough) {
    XV_HdmiTxSs_SendAuxInfoframe(&HdmiTxSs, (NULL));
  }
#endif
}

static void VphyHdmiTxInitCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmitx);

	HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmitx->xvphy;
	BUG_ON(!VphyPtr);
	dev_info(xhdmitx->dev, "VphyHdmiTxInitCallback\r\n");

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(xhdmitx->phy[0]);
	mutex_lock(&xhdmitx->xhdmitx_mutex);
	xvphy_mutex_lock(xhdmitx->phy[0]);

	XV_HdmiTxSs_RefClockChangeInit(HdmiTxSsPtr);

	/* unlock RX SS but keep XVPHY locked */
	mutex_unlock(&xhdmitx->xhdmitx_mutex);
}

static void VphyHdmiTxReadyCallback(void *CallbackRef)
{
	struct xhdmitx_device *xhdmitx = (struct xhdmitx_device *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmitx);

	HdmiTxSsPtr = &xhdmitx->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmitx->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(xhdmitx->dev, "VphyHdmiTxReadyCallback\r\n");
}

#define XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_PRESENT         1

#ifndef XPAR_V_HDMI_TX_SS_0_AXI_TIMER_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_AXI_TIMER_PRESENT 0
#define XPAR_V_HDMI_TX_SS_0_AXI_TIMER_DEVICE_ID 255
#define XPAR_V_HDMI_TX_SS_0_AXI_TIMER_BASEADDR 0xFFFFFFFF
#endif

#ifndef XPAR_V_HDMI_TX_SS_0_HDCP14_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_HDCP14_PRESENT 0
#define XPAR_V_HDMI_TX_SS_0_HDCP14_DEVICE_ID 255
#define XPAR_V_HDMI_TX_SS_0_HDCP14_BASEADDR 0xFFFFFFFF
#endif

#ifndef XPAR_V_HDMI_TX_SS_0_HDCP22_TX_SS_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_HDCP22_PRESENT 0
#define XPAR_V_HDMI_TX_SS_0_HDCP22_DEVICE_ID 255
#define XPAR_V_HDMI_TX_SS_0_HDCP22_BASEADDR 0xFFFFFFFF
#endif

#ifndef XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_DEVICE_ID
#define XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_PRESENT    0
#define XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_DEVICE_ID 255
#define XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_BASEADDR  0xFFFFFFFF
#endif

#ifndef XPAR_V_HDMI_TX_SS_0_V_TC_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_V_TC_PRESENT    0
#define XPAR_V_HDMI_TX_SS_0_V_TC_DEVICE_ID  255
#define XPAR_V_HDMI_TX_SS_0_V_TC_BASEADDR   0xFFFFFFFF
#endif

#ifndef XPAR_V_HDMI_TX_SS_0_AXI_GPIO_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_AXI_GPIO_PRESENT    0
#define XPAR_V_HDMI_TX_SS_0_AXI_GPIO_DEVICE_ID  255
#define XPAR_V_HDMI_TX_SS_0_AXI_GPIO_BASEADDR   0xFFFFFFFF
#endif

#ifndef XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_DEVICE_ID
#define XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_PRESENT    0
#define XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_DEVICE_ID  255
#define XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_S_AXI_CTRL_BASEADDR   0xFFFFFFFF
#endif
static XV_HdmiTxSs_Config config =
{
	XPAR_XV_HDMITXSS_0_DEVICE_ID,
	XPAR_XV_HDMITXSS_0_BASEADDR,
	XPAR_XV_HDMITXSS_0_HIGHADDR,
	XPAR_XV_HDMITXSS_0_INPUT_PIXELS_PER_CLOCK,
	XPAR_XV_HDMITXSS_0_MAX_BITS_PER_COMPONENT,

	{
		XPAR_V_HDMI_TX_SS_0_AXI_GPIO_PRESENT,
		XPAR_V_HDMI_TX_SS_0_AXI_GPIO_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_AXI_GPIO_BASEADDR
	},
	{
		XPAR_V_HDMI_TX_SS_0_AXI_TIMER_PRESENT,
		XPAR_V_HDMI_TX_SS_0_AXI_TIMER_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_AXI_TIMER_BASEADDR
	},
	{
		XPAR_V_HDMI_TX_SS_0_HDCP14_PRESENT,
		XPAR_V_HDMI_TX_SS_0_HDCP14_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_HDCP14_BASEADDR
	},
	{
		XPAR_V_HDMI_TX_SS_0_HDCP22_PRESENT,
		XPAR_V_HDMI_TX_SS_0_HDCP22_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_HDCP22_BASEADDR
	},
	{
		XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_PRESENT,
		XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_V_AXI4S_REMAP_S_AXI_CTRL_BASEADDR
	},
	{
		XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_PRESENT,
		XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_DEVICE_ID,
		XPAR_HDMI_OUTPUT_V_HDMI_TX_SS_0_V_HDMI_TX_BASEADDR
	},
	{
		XPAR_V_HDMI_TX_SS_0_V_TC_PRESENT,
		XPAR_V_HDMI_TX_SS_0_V_TC_DEVICE_ID,
		XPAR_V_HDMI_TX_SS_0_V_TC_BASEADDR
	},
};

XVtc_Config *XVtc_LookupConfig(u16 DeviceId)
{
	BUG_ON(1);
	return (XVtc_Config *)NULL;
}

static XV_HdmiTx_Config XV_HdmiTx_FixedConfig =
{
	XPAR_XV_HDMITX_0_DEVICE_ID,
	XPAR_XV_HDMITX_0_BASEADDR
};
XV_HdmiTx_Config *XV_HdmiTx_LookupConfig(u16 DeviceId)
{
	return (XV_HdmiTx_Config *)&XV_HdmiTx_FixedConfig;
}
XGpio_Config *XGpio_LookupConfig_TX(u16 DeviceId)
{
	BUG_ON(1);
	return (XGpio_Config *)NULL;
}
XV_axi4s_remap_Config* XV_axi4s_remap_LookupConfig_TX(u16 DeviceId) {
	BUG_ON(1);
	return NULL;
}


static void hdmitx_config_init(XV_HdmiTxSs_Config *config, void __iomem *iomem)
{
	config->BaseAddress = (uintptr_t)iomem;
	config->HighAddress = (uintptr_t)iomem + 0xFFFF;
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int xhdmitx_parse_of(struct xhdmitx_device *xhdmitx, XV_HdmiTxSs_Config *config)
{
	struct device *dev = xhdmitx->dev;
	struct device_node *node = dev->of_node;
	(void)dev;
	(void)node;
#if 0
	struct device_node *ports;
	struct device_node *port;
	unsigned int nports = 0;
	bool has_endpoint = false;
#endif

#if 0 // example bool
	bool has_dre = false;
	has_dre = of_property_read_bool(node, "xlnx,include-dre");
#endif
#if 0 // example u32
	u32 value;
	int err;
	err = of_property_read_u32(node, "xlnx,datawidth", &value);
#endif

	return 0;
}

static int xhdmitx_probe(struct platform_device *pdev)
{
	struct xhdmitx_device *xhdmitx;
	int ret;
	unsigned int index;
	unsigned long tx_clk_rate;
	unsigned long flags;
	struct resource *res;

	XV_HdmiTxSs *HdmiTxSsPtr;
	u32 Status;

	/* allocate zeroed HDMI TX device structure */
	xhdmitx = devm_kzalloc(&pdev->dev, sizeof(*xhdmitx), GFP_KERNEL);
	if (!xhdmitx)
		return -ENOMEM;
	/* store pointer of the real device inside platform device */
	xhdmitx->dev = &pdev->dev;

	/* mutex that protects against concurrent access */
	mutex_init(&xhdmitx->xhdmitx_mutex);
	spin_lock_init(&xhdmitx->irq_lock);
	/* work queues */
	xhdmitx->work_queue = create_singlethread_workqueue("xilinx-hdmi-tx");
	if (!xhdmitx->work_queue) {
		dev_info(xhdmitx->dev, "Could not create work queue\n");
		return -ENOMEM;
	}
#if 0
	INIT_DELAYED_WORK(&xhdmitx->delayed_work_enable_hotplug,
		xhdmitx_delayed_work_enable_hotplug);
#endif

	/* parse open firmware device tree data */
	ret = xhdmitx_parse_of(xhdmitx, &config);
	if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xhdmitx->iomem = devm_ioremap_resource(xhdmitx->dev, res);
	if (IS_ERR(xhdmitx->iomem))
		return PTR_ERR(xhdmitx->iomem);

	/* video streaming bus clock */
	xhdmitx->clk = devm_clk_get(xhdmitx->dev, NULL);
	if (IS_ERR(xhdmitx->clk))
		return PTR_ERR(xhdmitx->clk);

	clk_prepare_enable(xhdmitx->clk);

	/* get irq */
	xhdmitx->irq = platform_get_irq(pdev, 0);
	if (xhdmitx->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		destroy_workqueue(xhdmitx->work_queue);
		return xhdmitx->irq;
	}

#if 0
	xhdmitx->tx_clk = devm_clk_get(&pdev->dev, "tx-clk0");
	if (IS_ERR(xhdmitx->tx_clk)) {
		ret = PTR_ERR(xhdmitx->tx_clk);
		xhdmitx->tx_clk = NULL;
		dev_err(&pdev->dev, "failed to get the tx-clk0.\n");
		if (ret == -EPROBE_DEFER) {
			dev_err(&pdev->dev, "defering initialization as tx-clk0 failed.\n");
		}
		return ret;
	}
#endif

	if (!xhdmitx->tx_clk) {
		xhdmitx->tx_clk = devm_clk_get(&pdev->dev, "tx-clk");
		if (IS_ERR(xhdmitx->tx_clk)) {
			ret = PTR_ERR(xhdmitx->tx_clk);
			xhdmitx->tx_clk = NULL;
			dev_err(&pdev->dev, "failed to get the tx-clk.\n");
			if (ret == -EPROBE_DEFER) {
				dev_err(&pdev->dev, "defering initialization as tx-clk failed.\n");
			}
			return ret;
		}
	}
	dev_info(&pdev->dev, "got tx-clk\n");

	ret = clk_prepare_enable(xhdmitx->tx_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tx-clk\n");
		return ret;
	}
	dev_info(&pdev->dev, "enabled tx-clk\n");

	dev_info(xhdmitx->dev, "Initializing Si5324 TX clock to 50 MHz\n");

	tx_clk_rate = clk_get_rate(xhdmitx->tx_clk);
	dev_info(&pdev->dev, "tx-clk rate = %lu\n", tx_clk_rate);
	/* http://events.linuxfoundation.org/sites/events/files/slides/gregory-clement-common-clock-framework-how-to-use-it.pdf */
	tx_clk_rate = clk_set_rate(xhdmitx->tx_clk, 50*1000*1000);
	tx_clk_rate = clk_get_rate(xhdmitx->tx_clk);
	dev_info(&pdev->dev, "tx-clk rate = %lu\n", tx_clk_rate);

	/* @TODO spread phy[index] over RX/TX as */
	index = 2;
	{
		char phy_name[32];
		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);

		index = 0;
		xhdmitx->phy[index] = devm_phy_get(xhdmitx->dev, phy_name);
		if (IS_ERR(xhdmitx->phy[index])) {
			ret = PTR_ERR(xhdmitx->phy[index]);
			dev_err(xhdmitx->dev, "failed to get phy lane %s, error %d\n",
				phy_name, ret);
			goto error_phy;
		}

		ret = phy_init(xhdmitx->phy[index]);
		if (ret) {
			dev_err(xhdmitx->dev,
				"failed to init phy lane %d\n", index);
			goto error_phy;
		}
	}

	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmitx->xv_hdmitxss;

	mutex_lock(&xhdmitx->xhdmitx_mutex);

	/* initialize the source configuration structure */
	hdmitx_config_init(&config, xhdmitx->iomem);

	printk(KERN_INFO "HdmiTxSsPtr = %p\n", HdmiTxSsPtr);
	printk(KERN_INFO "&config = %p\n", &config);
	printk(KERN_INFO "xhdmitx->iomem = %lx\n", (unsigned long)xhdmitx->iomem);

	// Initialize top level and all included sub-cores
	Status = XV_HdmiTxSs_CfgInitialize(HdmiTxSsPtr, &config,
		(uintptr_t)xhdmitx->iomem);
	if (Status != XST_SUCCESS)
	{
		dev_err(xhdmitx->dev, "initialization failed with error %d\n", Status);
		return -EINVAL;
	}

	spin_lock_irqsave(&xhdmitx->irq_lock, flags);
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmitx->irq_lock, flags);

	XV_HdmiTxSs_ReportSubcoreVersion(&xhdmitx->xv_hdmitxss);

	/* TX SS callback setup */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_CONNECT,
		TxConnectCallback, (void *)xhdmitx);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_VS,
		TxVsCallback, (void *)xhdmitx);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_UP,
		TxStreamUpCallback, (void *)xhdmitx);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_DOWN,
		TxStreamDownCallback, (void *)xhdmitx);

	/* get a reference to the XVphy data structure */
	xhdmitx->xvphy = xvphy_get_xvphy(xhdmitx->phy[0]);

	BUG_ON(!xhdmitx->xvphy);

	xvphy_mutex_lock(xhdmitx->phy[0]);
	/* the callback is not specific to a single lane, but we need to
	 * provide one of the phy's as reference */
	XVphy_SetHdmiCallback(xhdmitx->xvphy, XVPHY_HDMI_HANDLER_TXINIT,
		VphyHdmiTxInitCallback, (void *)xhdmitx);

	XVphy_SetHdmiCallback(xhdmitx->xvphy, XVPHY_HDMI_HANDLER_TXREADY,
		VphyHdmiTxReadyCallback, (void *)xhdmitx);
	xvphy_mutex_unlock(xhdmitx->phy[0]);

	platform_set_drvdata(pdev, xhdmitx);

	/* Request the interrupt */
	ret = devm_request_threaded_irq(&pdev->dev, xhdmitx->irq, hdmitx_irq_handler, hdmitx_irq_thread,
		IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss", xhdmitx/*dev_id*/);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmitx->irq);
		return ret;
	}

	mutex_unlock(&xhdmitx->xhdmitx_mutex);

	spin_lock_irqsave(&xhdmitx->irq_lock, flags);
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmitx->irq_lock, flags);

#if 0
	EnableColorBar(xhdmitx, XVIDC_VM_1920x1080_60_P, XVIDC_CSF_RGB, XVIDC_BPC_8);
#endif

	dev_info(&pdev->dev, "xhdmitx_probe() succesfull.\n");

	return 0;
error_phy:
	return ret;
}

static int xhdmitx_remove(struct platform_device *pdev)
{
	struct xhdmitx_device *xhdmitx = platform_get_drvdata(pdev);
	//if (xhdmitx->tx_clk) clk_put(xhdmitx->tx_clk);
	if (xhdmitx->work_queue) destroy_workqueue(xhdmitx->work_queue);
	return 0;
}

static SIMPLE_DEV_PM_OPS(xhdmitx_pm_ops, xhdmitx_pm_suspend, xhdmitx_pm_resume);

static const struct of_device_id xhdmitx_of_id_table[] = {
	{ .compatible = "xlnx,v-hdmitxss-2.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, xhdmitx_of_id_table);

static struct platform_driver xhdmitx_driver = {
	.driver = {
		.name		= "xilinx-hdmitxss",
		.pm		= &xhdmitx_pm_ops,
		.of_match_table	= xhdmitx_of_id_table,
	},
	.probe			= xhdmitx_probe,
	.remove			= xhdmitx_remove,
};

module_platform_driver(xhdmitx_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_DESCRIPTION("Xilinx HDMI TX DRM driver");

