/*
 * Xilinx DRM HDMI encoder driver
 *
 * Copyright (C) 2016 Leon Woestenberg <leon@sidebranch.com>
 * Copyright (C) 2014 Xilinx, Inc.
 *
 * Author: Leon Woestenberg <leon@sidebranch.com>
 *
 * Based on xilinx_drm_dp.c:
 * Author: Hyun Woo Kwon <hyunk@xilinx.com>
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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-zynqmp.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/workqueue.h>

#if 0
#include "xilinx_drm_dp_sub.h"
#endif
#include "xilinx_drm_drv.h"

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "xilinx-hdmi-tx/xv_hdmitxss.h"
#include "xilinx-hdmi-tx/xil_printf.h"
#include "xilinx-hdmi-tx/xstatus.h"

#define NUM_SUBCORE_IRQ 2
#define HDMI_MAX_LANES	4

/**
 * struct xilinx_drm_hdmi - Xilinx DisplayPort core
 * @encoder: pointer to the drm encoder structure
 * @dev: device structure
 * @iomem: device I/O memory for register access
 * @config: IP core configuration from DTS
 * @aux: aux channel
 * @dp_sub: DisplayPort subsystem
 * @aclk: clock source device for internal axi4-lite clock
 * @aud_clk: clock source device for audio clock
 * @dpms: current dpms state
 * @dpcd: DP configuration data from currently connected sink device
 * @link_config: common link configuration between IP core and sink device
 * @mode: current mode between IP core and sink device
 * @train_set: set of training data
 */
struct xilinx_drm_hdmi {
	struct drm_device *drm_dev;
	struct drm_encoder *encoder;
	struct device *dev;
	void __iomem *iomem;

	/* video streaming bus clock */
	struct clk *clk;

	/* interrupt number */
	int irq;
	bool teardown;

	struct phy *phy[HDMI_MAX_LANES];

	/* mutex to prevent concurrent access to this structure */
	struct mutex hdmi_mutex;
	/* protects concurrent access from interrupt context */
	spinlock_t irq_lock;
	/* schedule (future) work */
	struct workqueue_struct *work_queue;
	struct delayed_work delayed_work_enable_hotplug;
	/* input clock */
	struct clk *tx_clk;
	bool cable_connected;
	bool hdmi_stream_up;
	bool have_edid;
	int dpms;

	/* configuration for the baseline subsystem driver instance */
	XV_HdmiTxSs_Config config;
	/* bookkeeping for the baseline subsystem driver instance */
	XV_HdmiTxSs xv_hdmitxss;
	/* sub core interrupt status registers */
	u32 IntrStatus[NUM_SUBCORE_IRQ];
	/* pointer to xvphy */
	XVphy *xvphy;
};

static inline struct xilinx_drm_hdmi *to_hdmi(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

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
	struct xilinx_drm_hdmi *hdmi;

	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;

	BUG_ON(!dev_id);
	hdmi = (struct xilinx_drm_hdmi *)dev_id;
	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	if (HdmiTxSsPtr->IsReady != XIL_COMPONENT_IS_READY) {
		printk(KERN_INFO "hdmitx_irq_handler(): HDMI TX SS is not initialized?!\n");
	}

#if 1
	/* read status registers */
	hdmi->IntrStatus[0] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_PIO_STA_OFFSET)) & (XV_HDMITX_PIO_STA_IRQ_MASK);
	hdmi->IntrStatus[1] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_DDC_STA_OFFSET)) & (XV_HDMITX_DDC_STA_IRQ_MASK);
#endif

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	/* mask interrupt request */
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	/* call bottom-half */
	return IRQ_WAKE_THREAD;
}

/* (struct xilinx_drm_hdmi *)dev_id */

static irqreturn_t hdmitx_irq_thread(int irq, void *dev_id)
{
	struct xilinx_drm_hdmi *hdmi;

	static int irq_count = 0;
	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;
	int i;
	char which[NUM_SUBCORE_IRQ + 1] = "012";
	int which_mask = 0;
	u32 Data;
	static u32 OldData;
	static int count = 0;
	//printk(KERN_INFO "hdmitx_irq_thread()\n");

	BUG_ON(!dev_id);
	hdmi = (struct xilinx_drm_hdmi *)dev_id;
	if (!hdmi) {
		printk(KERN_INFO "irq_thread: !dev_id\n");
		return IRQ_HANDLED;
	}
	if (hdmi->teardown) {
		printk(KERN_INFO "irq_thread: teardown\n");
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	mutex_lock(&hdmi->hdmi_mutex);
	/* call baremetal interrupt handler, this in turn will
	 * call the registed callbacks functions */

#if 0
	for (i = 0; i < NUM_SUBCORE_IRQ; i++) {
		which[i] = hdmi->IntrStatus[i]? '0' + i: '.';
		which_mask |= (hdmi->IntrStatus[i]? 1: 0) << i;
	}
	which[NUM_SUBCORE_IRQ] = 0;
#endif
	Data = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress,
		(XV_HDMITX_PIO_IN_OFFSET));
	count++;
	if (Data != OldData) {
		printk(KERN_INFO "PIO.DAT = 0x%08x, HDMI TX SS interrupt count = %d\n", (int)Data, count);
		OldData = Data;
	}
#if 0
	printk(KERN_INFO "PIO.EVT = 0x%08x, PIO.DAT = 0x%08x, DDC.EVT = 0x%08x\n",
	 hdmi->IntrStatus[0], (int)Data, hdmi->IntrStatus[0]);
#endif

	if (hdmi->IntrStatus[0]) HdmiTx_PioIntrHandler(HdmiTxSsPtr->HdmiTxPtr);
	if (hdmi->IntrStatus[1]) HdmiTx_DdcIntrHandler(HdmiTxSsPtr->HdmiTxPtr);

	//printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x\n", which, (int)which_mask);

	mutex_unlock(&hdmi->hdmi_mutex);

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	/* unmask interrupt request */
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);
	//printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x done\n", which, (int)which_mask);

	return IRQ_HANDLED;
}

/* entered with both tx and vphy mutex taken */
static void EnableColorBar(struct xilinx_drm_hdmi *hdmi,
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

	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);

	if (XVphy_IsBonded(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1)) {
		dev_info(hdmi->dev, "Both the GT RX and GT TX are clocked by the RX reference clock.\n");
	}

	if (VideoMode < XVIDC_VM_NUM_SUPPORTED) {
		dev_info(hdmi->dev, "Starting colorbar\n\r");

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
		dev_info(hdmi->dev, "Unable to set requested TX video resolution.\n\r");
	}

	/* Disable RX clock forwarding */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_RX, (FALSE));

	dev_info(hdmi->dev, "tx-clk: setting rate to VphyPtr->HdmiTxRefClkHz = %u\n", VphyPtr->HdmiTxRefClkHz);

	tx_clk_rate = clk_set_rate(hdmi->tx_clk, VphyPtr->HdmiTxRefClkHz);

	//tx_clk_rate = clk_get_rate(hdmi->tx_clk);
	dev_info(hdmi->dev, "tx-clk rate = %lu\n", tx_clk_rate);
}

static void TxConnectCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	XVphy *VphyPtr = hdmi->xvphy;
	int xst_hdmi20;
	BUG_ON(!hdmi);
	BUG_ON(!HdmiTxSsPtr);
	BUG_ON(!VphyPtr);
	BUG_ON(!hdmi->phy[0]);
	dev_info(hdmi->dev, "TxConnectCallback()\n");

	xvphy_mutex_lock(hdmi->phy[0]);
	if (HdmiTxSsPtr->IsStreamConnected) {
		dev_info(hdmi->dev, "TxConnectCallback(): TX connected\n");
		hdmi->cable_connected = 1;
		/* Check HDMI sink version */
		xst_hdmi20 = XV_HdmiTxSs_DetectHdmi20(HdmiTxSsPtr);
		dev_info(hdmi->dev, "HDMI %s\n", xst_hdmi20? "2.0": "1.4");
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (TRUE));
		dev_info(hdmi->dev, "TxConnectCallback(): EnableColorBar()\n");
		EnableColorBar(hdmi, XVIDC_VM_3840x2160_30_P, XVIDC_CSF_RGB, XVIDC_BPC_8);
	}
	else {
		dev_info(hdmi->dev, "TxConnectCallback(): TX disconnected\n");
		hdmi->cable_connected = 0;
		hdmi->hdmi_stream_up = 0;
		hdmi->have_edid = 0;
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (FALSE));
	}
	xvphy_mutex_unlock(hdmi->phy[0]);
#if 0
	if (hdmi->drm_dev) {
		/* release the mutex so that our drm ops can re-acquire it */
		mutex_unlock(&hdmi->hdmi_mutex);
		dev_info(hdmi->dev, "TxConnectCallback() -> drm_kms_helper_hotplug_event()\n");
		drm_kms_helper_hotplug_event(hdmi->drm_dev);
		mutex_lock(&hdmi->hdmi_mutex);
	}
#endif
	dev_info(hdmi->dev, "TxConnectCallback() done\n");
}

static void TxStreamUpCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;
	XVphy_PllType TxPllType;
	u64 TxLineRate;

	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(hdmi->dev, "TxStreamUpCallback(): TX stream is up\n");
	hdmi->hdmi_stream_up = 1;

#if 0
  XVidC_VideoStream *HdmiTxSsVidStreamPtr;

  HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(&HdmiTxSs);

  /* In passthrough copy the RX stream parameters to the TX stream */
  if (IsPassThrough) {
	  XV_HdmiTxSs_SetVideoStream(HdmiTxSsPtr, *HdmiTxSsVidStreamPtr);
  }
#endif

	xvphy_mutex_lock(hdmi->phy[0]);
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
	xvphy_mutex_unlock(hdmi->phy[0]);

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
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;

	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(hdmi->dev, "TX stream is down\n\r");
	hdmi->hdmi_stream_up = 0;
}

static void TxVsCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	XVphy *VphyPtr = hdmi->xvphy;
	BUG_ON(!hdmi);
	BUG_ON(!VphyPtr);
#if 0
  /* Audio Infoframe */
  /* Only when not in pass-through */
  if (!IsPassThrough) {
    XV_HdmiTxSs_SendAuxInfoframe(&HdmiTxSs, (NULL));
  }
#endif
}

/* entered with vphy mutex taken */
static void VphyHdmiTxInitCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);
	dev_info(hdmi->dev, "VphyHdmiTxInitCallback\n");

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(hdmi->phy[0]);
	dev_info(hdmi->dev, "xvphy_mutex_unlock() done\n");
	mutex_lock(&hdmi->hdmi_mutex);
	dev_info(hdmi->dev, "mutex_lock() done\n");
	xvphy_mutex_lock(hdmi->phy[0]);
	dev_info(hdmi->dev, "xvphy_mutex_lock() done\n");

	XV_HdmiTxSs_RefClockChangeInit(HdmiTxSsPtr);

	/* unlock RX SS but keep XVPHY locked */
	mutex_unlock(&hdmi->hdmi_mutex);
	dev_info(hdmi->dev, "VphyHdmiTxInitCallback() done\n");

}

/* entered with vphy mutex taken */
static void VphyHdmiTxReadyCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	dev_info(hdmi->dev, "VphyHdmiTxReadyCallback\r\n");
}

/* drm_encoder_slave_funcs */
static void xilinx_drm_hdmi_dpms(struct drm_encoder *encoder, int dpms)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	mutex_lock(&hdmi->hdmi_mutex);
#if 0
	void __iomem *iomem = hdmi->iomem;
	unsigned int i;
	int ret;
#endif
	dev_info(hdmi->dev, "xilinx_drm_hdmi_dpms(dpms = %d)\n", dpms);

	if (hdmi->dpms == dpms) {
		goto done;
	}

	hdmi->dpms = dpms;

	switch (dpms) {
	case DRM_MODE_DPMS_ON:
		/* power-up */
		goto done;
	default:
		/* power-down */
		goto done;
	}
done:
	mutex_unlock(&hdmi->hdmi_mutex);

}

static void xilinx_drm_hdmi_save(struct drm_encoder *encoder)
{
	/* no op */
}

static void xilinx_drm_hdmi_restore(struct drm_encoder *encoder)
{
	/* no op */
}

static bool xilinx_drm_hdmi_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	dev_info(hdmi->dev, "xilinx_drm_hdmi_mode_fixup()\n");
	mutex_lock(&hdmi->hdmi_mutex);

#if 0
	int diff = mode->htotal - mode->hsync_end;
	/*
	 * ZynqMP DP requires horizontal backporch to be greater than 12.
	 * This limitation may conflict with the sink device.
	 */
	if (hdmi->dp_sub && diff < XILINX_DP_SUB_TX_MIN_H_BACKPORCH) {
		int vrefresh = (adjusted_mode->clock * 1000) /
			       (adjusted_mode->vtotal * adjusted_mode->htotal);

		diff = XILINX_DP_SUB_TX_MIN_H_BACKPORCH - diff;
		adjusted_mode->htotal += diff;
		adjusted_mode->clock = adjusted_mode->vtotal *
				       adjusted_mode->htotal * vrefresh / 1000;
	}
#endif
	mutex_unlock(&hdmi->hdmi_mutex);

	return true;
}

/**
 * xilinx_drm_hdmi_max_rate - Calculate and return available max pixel clock
 * @link_rate: link rate (Kilo-bytes / sec)
 * @lane_num: number of lanes
 * @bpp: bits per pixel
 *
 * Return: max pixel clock (KHz) supported by current link config.
 */
static inline int xilinx_drm_hdmi_max_rate(int link_rate, u8 lane_num, u8 bpp)
{
	return link_rate * lane_num * 8 / bpp;
}

static int xilinx_drm_hdmi_mode_valid(struct drm_encoder *encoder,
				    struct drm_display_mode *mode)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	dev_info(hdmi->dev, "xilinx_drm_hdmi_mode_valid()\n");
	drm_mode_debug_printmodeline(mode);
	mutex_lock(&hdmi->hdmi_mutex);

#if 0
	u8 max_lanes = hdmi->link_config.max_lanes;
	u8 bpp = hdmi->config.bpp;
	u32 max_pclock = hdmi->config.max_pclock;
	int max_rate = hdmi->link_config.max_rate;
	int rate;
	/* a maximum pixel clock is set and is exceeded by the mode? */
	if (max_pclock && mode->clock > max_pclock)
		return MODE_CLOCK_HIGH;

	/* rate exceeds what is possible with the current setup? */
	rate = xilinx_drm_hdmi_max_rate(max_rate, max_lanes, bpp);
	if (mode->clock > rate)
		return MODE_CLOCK_HIGH;
#endif
	mutex_unlock(&hdmi->hdmi_mutex);
	return MODE_OK;
}

static void xilinx_drm_hdmi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	dev_info(hdmi->dev, "xilinx_drm_hdmi_mode_set()\n");

	drm_mode_debug_printmodeline(mode);

	dev_info(hdmi->dev, "mode->htotal = %d\n", mode->htotal);
	dev_info(hdmi->dev, "mode->vtotal = %d\n", mode->vtotal);

	dev_info(hdmi->dev, "mode->pvsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PVSYNC));
	dev_info(hdmi->dev, "mode->phsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PHSYNC));

	dev_info(hdmi->dev, "mode->hsync_end = %d\n", mode->hsync_end);
	dev_info(hdmi->dev, "mode->hsync_start = %d\n", mode->hsync_start);
	dev_info(hdmi->dev, "mode->vsync_end = %d\n", mode->vsync_end);
	dev_info(hdmi->dev, "mode->vsync_start = %d\n", mode->vsync_start);

	dev_info(hdmi->dev, "mode->hdisplay = %d\n", mode->hdisplay);
	dev_info(hdmi->dev, "mode->vdisplay = %d\n", mode->vdisplay);

	dev_info(hdmi->dev, "mode->htotal = %d\n", mode->htotal);
	dev_info(hdmi->dev, "mode->vtotal = %d\n", mode->vtotal);

#if 0
	xilinx_drm_hdmi_mode_configure(hdmi, adjusted_mode->clock);
	xilinx_drm_hdmi_mode_set_stream(hdmi, adjusted_mode);
	xilinx_drm_hdmi_mode_set_transfer_unit(hdmi, adjusted_mode);
#endif
}

static enum drm_connector_status
xilinx_drm_hdmi_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	mutex_lock(&hdmi->hdmi_mutex);
	/* cable connected and @TODO edid retrieved? */
	if (hdmi->cable_connected /*&& have_edid*/) {
		dev_info(hdmi->dev, "xilinx_drm_hdmi_detect() = connected\n");
		mutex_unlock(&hdmi->hdmi_mutex);
		return connector_status_connected;
	}
	dev_info(hdmi->dev, "xilinx_drm_hdmi_detect() = disconnected\n");
	mutex_unlock(&hdmi->hdmi_mutex);
	return connector_status_disconnected;
}

/* callback function for drm_do_get_edid(), used in xilinx_drm_hdmi_get_modes()
 * through drm_do_get_edid() from drm/drm_edid.c.
 *
 * called with hdmi_mutex taken
 *
 * Return 0 on success, !0 otherwise
 */
static int xilinx_drm_hdmi_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len)
{
	char buffer[256];
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)data;
	XV_HdmiTxSs *HdmiTxSsPtr;
	int ret;
	int i, j;

	BUG_ON(!hdmi);
	/* out of bounds? */
	if (((block * 128) + len) > 256) return -EINVAL;

	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	if (!HdmiTxSsPtr->IsStreamConnected) {
		dev_info(hdmi->dev, "xilinx_drm_hdmi_get_edid_block() stream is not connected\n");
	}
	/* first obtain edid in local buffer */
	ret = XV_HdmiTxSs_ReadEdid(HdmiTxSsPtr, buffer);
	if (ret == XST_FAILURE) {
		mutex_unlock(&hdmi->hdmi_mutex);
		dev_info(hdmi->dev, "xilinx_drm_hdmi_get_edid_block() failed reading EDID\n");
		return -EINVAL;
	}

	for (i = 0; i < 256; i += 16) {
		u8 b[128];
		u8 *bp = b;
		if (i == 128)
			dev_info(hdmi->dev, "\n");
			for (j = i; j < i + 16; j++) {
				sprintf(bp, "0x%02x, ", buf[j]);
				bp += 6;
			}
		bp[0] = '\0';
		dev_info(hdmi->dev, "%s\n", b);
	}

	dev_info(hdmi->dev, "xilinx_drm_hdmi_get_edid_block() block #%d, len %d\n",
		block, len);
	/* then copy the correct block(s) */
	memcpy(buf, buffer + block * 128, len);
	hdmi->have_edid = 1;
	return 0;
}

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int xilinx_drm_hdmi_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	struct edid *edid;
	int ret;

	dev_info(hdmi->dev, "xilinx_drm_hdmi_get_modes()\n");
	mutex_lock(&hdmi->hdmi_mutex);

	/* When the I2C adapter connected to the DDC bus is hidden behind a device that
	* exposes a different interface to read EDID blocks this function can be used
	* to get EDID data using a custom block read function. - from drm_edid.c
	*/

	/* private data hdmi is passed to xilinx_drm_hdmi_get_edid_block(data, ...) */
	edid = drm_do_get_edid(connector, xilinx_drm_hdmi_get_edid_block, hdmi);
	mutex_unlock(&hdmi->hdmi_mutex);
	if (!edid) {
		dev_err(hdmi->dev, "xilinx_drm_hdmi_get_modes() could not obtain edid\n");
		return 0;
	}

	drm_mode_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);
	kfree(edid);
	dev_info(hdmi->dev, "xilinx_drm_hdmi_get_modes() done\n");

	return ret;
}

static struct drm_encoder_slave_funcs xilinx_drm_hdmi_encoder_funcs = {
	.dpms			= xilinx_drm_hdmi_dpms,
	.save			= xilinx_drm_hdmi_save,
	.restore		= xilinx_drm_hdmi_restore,
	.mode_fixup		= xilinx_drm_hdmi_mode_fixup,
	.mode_valid		= xilinx_drm_hdmi_mode_valid,
	.mode_set		= xilinx_drm_hdmi_mode_set,
	.detect			= xilinx_drm_hdmi_detect,
	.get_modes		= xilinx_drm_hdmi_get_modes,
};

/* forward declaration */
static XV_HdmiTxSs_Config config;

static int xilinx_drm_hdmi_encoder_init(struct platform_device *pdev,
				      struct drm_device *dev,
				      struct drm_encoder_slave *encoder)
{
	struct xilinx_drm_hdmi *hdmi = platform_get_drvdata(pdev);
	unsigned long flags;
	XV_HdmiTxSs *HdmiTxSsPtr;
	u32 Status;
	int ret;

	BUG_ON(!hdmi);

	dev_info(hdmi->dev, "xilinx_drm_hdmi_encoder_init()\n");

	encoder->slave_priv = hdmi;
	encoder->slave_funcs = &xilinx_drm_hdmi_encoder_funcs;

	hdmi->encoder = &encoder->base;
	hdmi->drm_dev = dev;

	mutex_lock(&hdmi->hdmi_mutex);

	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;

	printk(KERN_INFO "HdmiTxSsPtr = %p\n", HdmiTxSsPtr);
	BUG_ON(!HdmiTxSsPtr);

	// Initialize top level and all included sub-cores
	Status = XV_HdmiTxSs_CfgInitialize(HdmiTxSsPtr, &config,
		(uintptr_t)hdmi->iomem);
	if (Status != XST_SUCCESS)
	{
		dev_err(hdmi->dev, "initialization failed with error %d\n", Status);
		return -EINVAL;
	}

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	XV_HdmiTxSs_ReportSubcoreVersion(&hdmi->xv_hdmitxss);

	/* TX SS callback setup */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_CONNECT,
		TxConnectCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_VS,
		TxVsCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_UP,
		TxStreamUpCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_DOWN,
		TxStreamDownCallback, (void *)hdmi);

	/* get a reference to the XVphy data structure */
	hdmi->xvphy = xvphy_get_xvphy(hdmi->phy[0]);

	BUG_ON(!hdmi->xvphy);

	xvphy_mutex_lock(hdmi->phy[0]);
	/* the callback is not specific to a single lane, but we need to
	 * provide one of the phy's as reference */
	XVphy_SetHdmiCallback(hdmi->xvphy, XVPHY_HDMI_HANDLER_TXINIT,
		VphyHdmiTxInitCallback, (void *)hdmi);

	XVphy_SetHdmiCallback(hdmi->xvphy, XVPHY_HDMI_HANDLER_TXREADY,
		VphyHdmiTxReadyCallback, (void *)hdmi);
	xvphy_mutex_unlock(hdmi->phy[0]);

	/* Request the interrupt */
	ret = devm_request_threaded_irq(&pdev->dev, hdmi->irq, hdmitx_irq_handler, hdmitx_irq_thread,
		IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss", hdmi/*dev_id*/);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", hdmi->irq);
		return ret;
	}

	mutex_unlock(&hdmi->hdmi_mutex);

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	return 0;
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


static void xilinx_drm_hdmi_config_init(XV_HdmiTxSs_Config *config, void __iomem *iomem)
{
	config->BaseAddress = (uintptr_t)iomem;
	config->HighAddress = (uintptr_t)iomem + 0xFFFF;
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int xilinx_drm_hdmi_parse_of(struct xilinx_drm_hdmi *hdmi, XV_HdmiTxSs_Config *config)
{
	struct device *dev = hdmi->dev;
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

static int xilinx_drm_hdmi_probe(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *hdmi;
	int ret;
	unsigned int index;
	unsigned long tx_clk_rate;
	struct resource *res;

	/* allocate zeroed HDMI TX device structure */
	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	/* store pointer of the real device inside platform device */
	hdmi->dev = &pdev->dev;

	/* mutex that protects against concurrent access */
	mutex_init(&hdmi->hdmi_mutex);
	spin_lock_init(&hdmi->irq_lock);
	/* work queues */
	hdmi->work_queue = create_singlethread_workqueue("xilinx-hdmi-tx");
	if (!hdmi->work_queue) {
		dev_info(hdmi->dev, "Could not create work queue\n");
		return -ENOMEM;
	}

	/* parse open firmware device tree data */
	ret = xilinx_drm_hdmi_parse_of(hdmi, &config);
	if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->iomem = devm_ioremap_resource(hdmi->dev, res);
	if (IS_ERR(hdmi->iomem))
		return PTR_ERR(hdmi->iomem);

	/* video streaming bus clock */
	hdmi->clk = devm_clk_get(hdmi->dev, NULL);
	if (IS_ERR(hdmi->clk))
		return PTR_ERR(hdmi->clk);

	clk_prepare_enable(hdmi->clk);

	/* get irq */
	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		destroy_workqueue(hdmi->work_queue);
		return hdmi->irq;
	}

	if (!hdmi->tx_clk) {
		hdmi->tx_clk = devm_clk_get(&pdev->dev, "tx-clk");
		if (IS_ERR(hdmi->tx_clk)) {
			ret = PTR_ERR(hdmi->tx_clk);
			hdmi->tx_clk = NULL;
			if (ret == -EPROBE_DEFER) {
				dev_err(&pdev->dev, "defering initialization; tx-clk not (yet) available.\n");
			} else {
				dev_err(&pdev->dev, "failed to get the tx-clk.\n");
			}
			return ret;
		}
	}
	dev_info(&pdev->dev, "got tx-clk\n");

	ret = clk_prepare_enable(hdmi->tx_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tx-clk\n");
		return ret;
	}
	dev_info(&pdev->dev, "enabled tx-clk\n");

	dev_info(hdmi->dev, "Initializing Si5324 TX clock to 50 MHz\n");

	tx_clk_rate = clk_get_rate(hdmi->tx_clk);
	//dev_info(&pdev->dev, "tx-clk rate = %lu\n", tx_clk_rate);
	/* http://events.linuxfoundation.org/sites/events/files/slides/gregory-clement-common-clock-framework-how-to-use-it.pdf */
	tx_clk_rate = clk_set_rate(hdmi->tx_clk, 50*1000*1000);
	tx_clk_rate = clk_get_rate(hdmi->tx_clk);
	//dev_info(&pdev->dev, "tx-clk rate = %lu\n", tx_clk_rate);

	/* @TODO spread phy[index] over RX/TX as */
	index = 2;
	{
		char phy_name[32];
		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);

		index = 0;
		hdmi->phy[index] = devm_phy_get(hdmi->dev, phy_name);
		if (IS_ERR(hdmi->phy[index])) {
			ret = PTR_ERR(hdmi->phy[index]);
			dev_err(hdmi->dev, "failed to get phy lane %s, error %d\n",
				phy_name, ret);
			goto error_phy;
		}

		ret = phy_init(hdmi->phy[index]);
		if (ret) {
			dev_err(hdmi->dev,
				"failed to init phy lane %d\n", index);
			goto error_phy;
		}
	}

	/* initialize the source configuration structure */
	xilinx_drm_hdmi_config_init(&config, hdmi->iomem);

	printk(KERN_INFO "&config = %p\n", &config);
	printk(KERN_INFO "hdmi->iomem = %lx\n", (unsigned long)hdmi->iomem);

	platform_set_drvdata(pdev, hdmi);

	/*** consider moving this into encoder_init ***/
#if 0
	mutex_lock(&hdmi->hdmi_mutex);
	platform_set_drvdata(pdev, hdmi);

	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;
	printk(KERN_INFO "HdmiTxSsPtr = %p\n", HdmiTxSsPtr);

	// Initialize top level and all included sub-cores
	Status = XV_HdmiTxSs_CfgInitialize(HdmiTxSsPtr, &config,
		(uintptr_t)hdmi->iomem);
	if (Status != XST_SUCCESS)
	{
		dev_err(hdmi->dev, "initialization failed with error %d\n", Status);
		return -EINVAL;
	}

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

	XV_HdmiTxSs_ReportSubcoreVersion(&hdmi->xv_hdmitxss);

	/* TX SS callback setup */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_CONNECT,
		TxConnectCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_VS,
		TxVsCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_UP,
		TxStreamUpCallback, (void *)hdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_DOWN,
		TxStreamDownCallback, (void *)hdmi);

	/* get a reference to the XVphy data structure */
	hdmi->xvphy = xvphy_get_xvphy(hdmi->phy[0]);

	BUG_ON(!hdmi->xvphy);

	xvphy_mutex_lock(hdmi->phy[0]);
	/* the callback is not specific to a single lane, but we need to
	 * provide one of the phy's as reference */
	XVphy_SetHdmiCallback(hdmi->xvphy, XVPHY_HDMI_HANDLER_TXINIT,
		VphyHdmiTxInitCallback, (void *)hdmi);

	XVphy_SetHdmiCallback(hdmi->xvphy, XVPHY_HDMI_HANDLER_TXREADY,
		VphyHdmiTxReadyCallback, (void *)hdmi);
	xvphy_mutex_unlock(hdmi->phy[0]);


	/* Request the interrupt */
	ret = devm_request_threaded_irq(&pdev->dev, hdmi->irq, hdmitx_irq_handler, hdmitx_irq_thread,
		IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss", hdmi/*dev_id*/);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", hdmi->irq);
		return ret;
	}

	mutex_unlock(&hdmi->hdmi_mutex);

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);
#endif
	dev_info(&pdev->dev, "xilinx_drm_hdmi_probe() succesfull.\n");

	return 0;
error_phy:
	return ret;
}

static int xilinx_drm_hdmi_remove(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *hdmi = platform_get_drvdata(pdev);
	if (hdmi->work_queue) destroy_workqueue(hdmi->work_queue);
	return 0;
}

static const struct of_device_id xilinx_drm_hdmi_of_match[] = {
	{ .compatible = "xlnx,v-hdmitxss-2.0", },
	{ /* end of table */ },
};
MODULE_DEVICE_TABLE(of, xilinx_drm_hdmi_of_match);

static struct drm_platform_encoder_driver xilinx_drm_hdmi_driver = {
	.platform_driver = {
		.probe			= xilinx_drm_hdmi_probe,
		.remove			= xilinx_drm_hdmi_remove,
		.driver			= {
			.owner		= THIS_MODULE,
			.name		= "xilinx-drm-hdmi",
			.of_match_table	= xilinx_drm_hdmi_of_match,
		},
	},
	.encoder_init = xilinx_drm_hdmi_encoder_init,
};

static int __init xilinx_drm_hdmi_init(void)
{
	return platform_driver_register(&xilinx_drm_hdmi_driver.platform_driver);
}

static void __exit xilinx_drm_hdmi_exit(void)
{
	platform_driver_unregister(&xilinx_drm_hdmi_driver.platform_driver);
}

module_init(xilinx_drm_hdmi_init);
module_exit(xilinx_drm_hdmi_exit);

MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_DESCRIPTION("Xilinx DRM KMS HDMI Driver");
MODULE_LICENSE("GPL v2");
