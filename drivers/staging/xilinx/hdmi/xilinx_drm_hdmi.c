/*
 * Xilinx DRM HDMI encoder driver
 *
 * Copyright (C) 2016 Leon Woestenberg <leon@sidebranch.com>
 * Copyright (C) 2014 Xilinx, Inc.
 *
 * Authors: Leon Woestenberg <leon@sidebranch.com>
 *          Rohit Consul <rohitco@xilinx.com>
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

/* if both both DEBUG and DEBUG_TRACE are defined, trace_printk() is used */
#define DEBUG
//#define DEBUG_TRACE

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

#include "xilinx_drm_drv.h"

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "xilinx-hdmi-tx/xv_hdmitxss.h"
#include "xilinx-hdmi-tx/xil_printf.h"
#include "xilinx-hdmi-tx/xstatus.h"

#define NUM_SUBCORE_IRQ 2
#define HDMI_MAX_LANES	4

/* select either trace or printk logging */
#ifdef DEBUG_TRACE
#define do_hdmi_dbg(format, ...) do { \
  trace_printk("xlnx-hdmi-txss: " format, ##__VA_ARGS__); \
} while(0)
#else
#define do_hdmi_dbg(format, ...) do { \
  printk(KERN_DEBUG "xlnx-hdmi-txss: " format, ##__VA_ARGS__); \
} while(0)
#endif

/* either enable or disable debugging */
#ifdef DEBUG
#  define hdmi_dbg(x...) do_hdmi_dbg(x)
#else
#  define hdmi_dbg(x...)
#endif

/**
 * struct xilinx_drm_hdmi - Xilinx HDMI core
 * @encoder: pointer to the drm encoder structure
 * @dev: device structure
 * @iomem: device I/O memory for register access
 * @dp_sub: DisplayPort subsystem
 * @dpms: current dpms state
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
	struct clk *axi_lite_clk;

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
	/* input reference clock that we configure */
	struct clk *tx_clk;

	/* retimer that we configure by setting a clock rate */
	struct clk *retimer_clk;

	bool cable_connected;
	bool hdmi_stream_up;
	bool have_edid;
	int dpms;

	XVidC_ColorFormat xvidc_colorfmt;
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

/* @NOTE remove later; debugging purposes */
#define HDMITX_DEBUG_IRQ 0

/* (struct xilinx_drm_hdmi *)dev_id */
static irqreturn_t hdmitx_irq_thread(int irq, void *dev_id)
{
	struct xilinx_drm_hdmi *hdmi;
	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;

/* @NOTE remove later; debugging purposes */
#if (defined(HDMITX_DEBUG_IRQ) && HDMITX_DEBUG_IRQ)
	static int irq_count = 0;
	int i;
	char which[NUM_SUBCORE_IRQ + 1] = "012";
	int which_mask = 0;
	u32 Data;
	static u32 OldData;
	static int count = 0;
	//printk(KERN_INFO "hdmitx_irq_thread()\n");
#endif

	BUG_ON(!dev_id);
	hdmi = (struct xilinx_drm_hdmi *)dev_id;
	if (!hdmi) {
		printk(KERN_INFO "irq_thread: !dev_id\n");
		return IRQ_HANDLED;
	}
	/* driver is being torn down, do not process further interrupts */
	if (hdmi->teardown) {
		printk(KERN_INFO "irq_thread: teardown\n");
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	mutex_lock(&hdmi->hdmi_mutex);

/* @NOTE remove later; debugging purposes */
#if (defined(HDMITX_DEBUG_IRQ) && HDMITX_DEBUG_IRQ)
	for (i = 0; i < NUM_SUBCORE_IRQ; i++) {
		which[i] = hdmi->IntrStatus[i]? '0' + i: '.';
		which_mask |= (hdmi->IntrStatus[i]? 1: 0) << i;
	}
	which[NUM_SUBCORE_IRQ] = 0;
	/* show changes*/
	Data = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress,
		(XV_HDMITX_PIO_IN_OFFSET));
	count++;
	if (Data != OldData) {
		printk(KERN_INFO "PIO.DAT = 0x%08x, HDMI TX SS interrupt count = %d\n", (int)Data, count);
		OldData = Data;
	}
	printk(KERN_INFO "PIO.EVT = 0x%08x, PIO.DAT = 0x%08x, DDC.EVT = 0x%08x\n",
	 hdmi->IntrStatus[0], (int)Data, hdmi->IntrStatus[0]);
#endif

	/* call baremetal interrupt handler, this in turn will
	 * call the registed callbacks functions */
	if (hdmi->IntrStatus[0]) HdmiTx_PioIntrHandler(HdmiTxSsPtr->HdmiTxPtr);
	if (hdmi->IntrStatus[1]) HdmiTx_DdcIntrHandler(HdmiTxSsPtr->HdmiTxPtr);

	mutex_unlock(&hdmi->hdmi_mutex);

	spin_lock_irqsave(&hdmi->irq_lock, flags);
	/* unmask interrupt request */
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&hdmi->irq_lock, flags);

/* @NOTE remove later; debugging purposes */
#if (defined(HDMITX_DEBUG_IRQ) && HDMITX_DEBUG_IRQ)
	printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x done\n", which, (int)which_mask);
#endif

	return IRQ_HANDLED;
}

static void TxConnectCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	XVphy *VphyPtr = hdmi->xvphy;
	BUG_ON(!hdmi);
	BUG_ON(!HdmiTxSsPtr);
	BUG_ON(!VphyPtr);
	BUG_ON(!hdmi->phy[0]);
	hdmi_dbg("TxConnectCallback()\n");

	xvphy_mutex_lock(hdmi->phy[0]);
	if (HdmiTxSsPtr->IsStreamConnected) {
		int xst_hdmi20;
		hdmi->cable_connected = 1;
		/* Check HDMI sink version */
		xst_hdmi20 = XV_HdmiTxSs_DetectHdmi20(HdmiTxSsPtr);
		hdmi_dbg("TxConnectCallback(): TX connected to HDMI %s Sink Device\n",
			(xst_hdmi20 == XST_SUCCESS)? "2.0": "1.4");
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (TRUE));
	}
	else {
		hdmi_dbg("TxConnectCallback(): TX disconnected\n");
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
		hdmi_dbg("TxConnectCallback() -> drm_kms_helper_hotplug_event()\n");
		drm_kms_helper_hotplug_event(hdmi->drm_dev);
		mutex_lock(&hdmi->hdmi_mutex);
	}
#endif
	hdmi_dbg("TxConnectCallback() done\n");
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

	hdmi_dbg("TxStreamUpCallback(): TX stream is up\n");
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

	/* configure an external retimer through a (virtual) CCF clock
	 * (this was tested against the DP159 misc retimer driver) */
	if (hdmi->retimer_clk) {
		hdmi_dbg("retimer: clk_set_rate(hdmi->retimer_clk, TxLineRate=%lld\n", TxLineRate);
		(void)clk_set_rate(hdmi->retimer_clk, (signed long long)TxLineRate);
	}

	/* Enable TX TMDS clock*/
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (TRUE));

	/* Copy Sampling Rate */
	XV_HdmiTxSs_SetSamplingRate(HdmiTxSsPtr, VphyPtr->HdmiTxSampleRate);
	xvphy_mutex_unlock(hdmi->phy[0]);

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
#if 1
	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);
	XVidC_ReportStreamInfo(HdmiTxSsVidStreamPtr);
#endif
#if 0
	XV_HdmiTx_DebugInfo(HdmiTxSsPtr->HdmiTxPtr);
#endif
}

static void TxStreamDownCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	//XVidC_VideoStream *HdmiTxSsVidStreamPtr;

	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_dbg("TxStreamDownCallback(): TX stream is down\n\r");
	hdmi->hdmi_stream_up = 0;
}

static void TxVsCallback(void *CallbackRef)
{
#if 0
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	XVphy *VphyPtr = hdmi->xvphy;
	BUG_ON(!hdmi);
	BUG_ON(!VphyPtr);
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
	//hdmi_dbg("VphyHdmiTxInitCallback\n");

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(hdmi->phy[0]);
	//hdmi_dbg("xvphy_mutex_unlock() done\n");
	mutex_lock(&hdmi->hdmi_mutex);
	//hdmi_dbg("mutex_lock() done\n");
	xvphy_mutex_lock(hdmi->phy[0]);
	//hdmi_dbg("xvphy_mutex_lock() done\n");

	hdmi_dbg("VphyHdmiTxInitCallback(): XV_HdmiTxSs_RefClockChangeInit()\n");

	XV_HdmiTxSs_RefClockChangeInit(HdmiTxSsPtr);

	/* unlock RX SS but keep XVPHY locked */
	mutex_unlock(&hdmi->hdmi_mutex);
	//hdmi_dbg("VphyHdmiTxInitCallback() done\n");
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

	hdmi_dbg("VphyHdmiTxReadyCallback(): NOP\n");
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
	hdmi_dbg("xilinx_drm_hdmi_dpms(dpms = %d)\n", dpms);

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

/* if SI5324 is defined, the SI5324 clock is changed after xilinx_drm_hdmi_mode_set() has
 * completed, like in bare-metal.
 *
 * if SI5324 is commented out, the SI5324 clock is changed before  xilinx_drm_hdmi_mode_set()
 * is run, THIS IS THE LINUX DEFAULT AND LINUX DOES NOT ALLOW OTHER SEQUENCES OFFICIALLY. */
//#define SI5324_LAST

#ifdef SI5324_LAST
/* prototype */
static void xilinx_drm_hdmi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode);
#endif

static bool xilinx_drm_hdmi_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	XVphy *VphyPtr;
	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	/* @NOTE LEON: we are calling mode_set here, just before the si5324 clock is changed */

	hdmi_dbg("xilinx_drm_hdmi_mode_fixup()\n");
#ifdef SI5324_LAST
	xilinx_drm_hdmi_mode_set(encoder, mode, adjusted_mode);
#endif

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
	hdmi_dbg("xilinx_drm_hdmi_mode_valid()\n");
	drm_mode_debug_printmodeline(mode);
	mutex_lock(&hdmi->hdmi_mutex);

#if 0
	if (mode->clock > hdmi_max_rate)
		return MODE_CLOCK_HIGH;
#endif
	mutex_unlock(&hdmi->hdmi_mutex);
	return MODE_OK;
}

#ifdef SI5324_LAST
static void xilinx_drm_hdmi_mode_set_nop(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	/* nop */
}
#endif

static void xilinx_drm_hdmi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	XVidC_VideoTiming vt;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;
	u32 TmdsClock = 0;
	u32 Result;
	//u32 PixelClock;
	XVidC_VideoMode VmId;

	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	hdmi_dbg("xilinx_drm_hdmi_mode_set()\n");
	BUG_ON(!hdmi);

	HdmiTxSsPtr = &hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = hdmi->xvphy;
	BUG_ON(!VphyPtr);

	mutex_lock(&hdmi->hdmi_mutex);

	xvphy_mutex_lock(hdmi->phy[0]);

	drm_mode_debug_printmodeline(mode);

	hdmi_dbg("mode->clock = %d\n", mode->clock * 1000);
	hdmi_dbg("mode->crtc_clock = %d\n", mode->crtc_clock * 1000);


	hdmi_dbg("mode->htotal = %d\n", mode->htotal);
	hdmi_dbg("mode->vtotal = %d\n", mode->vtotal);

	hdmi_dbg("mode->pvsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PVSYNC));
	hdmi_dbg("mode->phsync = %d\n",
		!!(mode->flags & DRM_MODE_FLAG_PHSYNC));

	hdmi_dbg("mode->hsync_end = %d\n", mode->hsync_end);
	hdmi_dbg("mode->hsync_start = %d\n", mode->hsync_start);
	hdmi_dbg("mode->vsync_end = %d\n", mode->vsync_end);
	hdmi_dbg("mode->vsync_start = %d\n", mode->vsync_start);

	hdmi_dbg("mode->hdisplay = %d\n", mode->hdisplay);
	hdmi_dbg("mode->vdisplay = %d\n", mode->vdisplay);

	hdmi_dbg("mode->htotal = %d\n", mode->htotal);
	hdmi_dbg("mode->vtotal = %d\n", mode->vtotal);

#if 0
	xilinx_drm_hdmi_mode_configure(hdmi, adjusted_mode->clock);
	xilinx_drm_hdmi_mode_set_stream(hdmi, adjusted_mode);
	xilinx_drm_hdmi_mode_set_transfer_unit(hdmi, adjusted_mode);
#endif
	/* see slide 20 of http://events.linuxfoundation.org/sites/events/files/slides/brezillon-drm-kms.pdf */
	vt.HActive = mode->hdisplay;
	vt.HFrontPorch = mode->hsync_start - mode->hdisplay;
	vt.HSyncWidth = mode->hsync_end - mode->hsync_start;
	vt.HBackPorch = mode->htotal - mode->hsync_end;
	vt.HTotal = mode->htotal;
	vt.HSyncPolarity = !!(mode->flags & DRM_MODE_FLAG_PHSYNC);

	vt.VActive = mode->vdisplay;
	vt.F0PVFrontPorch = mode->vsync_start - mode->vdisplay;
	vt.F0PVSyncWidth = mode->vsync_end - mode->vsync_start;
	vt.F0PVBackPorch = mode->vtotal - mode->vsync_end;
	vt.F0PVTotal = mode->vtotal;
	vt.F1VFrontPorch = mode->vsync_start - mode->vdisplay;
	vt.F1VSyncWidth = mode->vsync_end - mode->vsync_start;
	vt.F1VBackPorch = mode->vtotal - mode->vsync_end;
	vt.F1VTotal = mode->vtotal;
	vt.VSyncPolarity = !!(mode->flags & DRM_MODE_FLAG_PVSYNC);

	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);

	if (XVphy_IsBonded(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1)) {
		hdmi_dbg("Both the GT RX and GT TX are clocked by the RX reference clock.\n");
		return;
	}

	/* Disable TX TDMS clock */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (FALSE));

#if 0
	// Get pixel clock
	PixelClock = XVidC_GetPixelClockHzByVmId(VideoMode);

	// In YUV420 the pixel clock is actually the half of the
	// reported pixel clock
	if (ColorFormat == XVIDC_CSF_YCRCB_420) {
		PixelClock = PixelClock / 2;
	}
#endif

	hdmi_dbg("mode->vrefresh = %d\n", mode->vrefresh);
	VmId = XVidC_GetVideoModeId(mode->hdisplay, mode->vdisplay, mode->vrefresh, FALSE);
	hdmi_dbg("VmId = %d\n", VmId);
	if (VmId == XVIDC_VM_NOT_SUPPORTED) { //no match found in timing table
		hdmi_dbg("Tx Video Mode not supported. Using DRM TIming\n");
		HdmiTxSsVidStreamPtr->VmId = XVIDC_VM_CUSTOM;
		HdmiTxSsVidStreamPtr->Timing = vt; //overwrite with drm detected timing
	}
	TmdsClock = XV_HdmiTxSs_SetStream(HdmiTxSsPtr, VmId, hdmi->xvidc_colorfmt, XVIDC_BPC_8, NULL);

	// Set TX reference clock
	VphyPtr->HdmiTxRefClkHz = mode->crtc_clock * 1000;
	hdmi_dbg("Setting VphyPtr->HdmiTxRefClkHz (from mode->crtc_clock) = %d\n", VphyPtr->HdmiTxRefClkHz);
	hdmi_dbg("(TmdsClock = %u, from XV_HdmiTxSs_SetStream())\n", TmdsClock);

	hdmi_dbg("XVphy_SetHdmiTxParam(PixPerClk = %d, ColorDepth = %d, ColorFormatId=%d)\n",
		(int)HdmiTxSsVidStreamPtr->PixPerClk, (int)HdmiTxSsVidStreamPtr->ColorDepth,
		(int)HdmiTxSsVidStreamPtr->ColorFormatId);

	// Set GT TX parameters
	Result = XVphy_SetHdmiTxParam(VphyPtr, 0, XVPHY_CHANNEL_ID_CHA,
					HdmiTxSsVidStreamPtr->PixPerClk,
					HdmiTxSsVidStreamPtr->ColorDepth,
					HdmiTxSsVidStreamPtr->ColorFormatId);

	if (Result == (XST_FAILURE)) {
		hdmi_dbg("Unable to set requested TX video resolution.\n\r");
	}

	/* Disable RX clock forwarding */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_RX, (FALSE));

	/* @NOTE in bare-metal, here the Si5324 clock is changed */

	xvphy_mutex_unlock(hdmi->phy[0]);

	//XVidC_ReportStreamInfo(HdmiTxSsVidStreamPtr);
	XV_HdmiTx_DebugInfo(HdmiTxSsPtr->HdmiTxPtr);
	mutex_unlock(&hdmi->hdmi_mutex);
}

static enum drm_connector_status
xilinx_drm_hdmi_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	mutex_lock(&hdmi->hdmi_mutex);
	/* cable connected and @TODO edid retrieved? */
	if (hdmi->cable_connected /*&& have_edid*/) {
		//hdmi_dbg("xilinx_drm_hdmi_detect() = connected\n");
		mutex_unlock(&hdmi->hdmi_mutex);
		return connector_status_connected;
	}
	//hdmi_dbg("xilinx_drm_hdmi_detect() = disconnected\n");
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
	u8 *buffer;
	struct xilinx_drm_hdmi *hdmi = (struct xilinx_drm_hdmi *)data;
	XV_HdmiTxSs *HdmiTxSsPtr;
	int ret;

	BUG_ON(!hdmi);
	/* out of bounds? */
	if (((block * 128) + len) > 256) return -EINVAL;

	buffer = kzalloc(256, GFP_KERNEL);
	if (!buffer) return -ENOMEM;


	HdmiTxSsPtr = (XV_HdmiTxSs *)&hdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	if (!HdmiTxSsPtr->IsStreamConnected) {
		hdmi_dbg("xilinx_drm_hdmi_get_edid_block() stream is not connected\n");
	}
#if 0
	XV_HdmiTxSs_ShowEdid(HdmiTxSsPtr);
#endif
	/* first obtain edid in local buffer */
	ret = XV_HdmiTxSs_ReadEdid(HdmiTxSsPtr, buffer);
	if (ret == XST_FAILURE) {
		hdmi_dbg("xilinx_drm_hdmi_get_edid_block() failed reading EDID\n");
		return -EINVAL;
	}

	/* then copy the requested 128-byte block(s) */
	memcpy(buf, buffer + block * 128, len);
	/* free our local buffer */
	kfree(buffer);
	return 0;
}

static const struct drm_display_mode xilinx_drm_hdmi_hardcode_modes[] = {

	/* 16 - 1920x1080@60Hz copied from drm_edid.c/edid_cea_modes */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
		   2052, 2200, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
#if 1
	/* 1 - 3840x2160@30Hz copied from from edid_4k_modes */
	{ DRM_MODE("3840x2160", DRM_MODE_TYPE_DRIVER, 297000,
		   3840, 4016, 4104, 4400, 0,
		   2160, 2168, 2178, 2250, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 30, },
#endif
};

static int xilinx_drm_hdmi_hardcode(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *newmode;
	//struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);

	int i;

	for (i = 0; i < ARRAY_SIZE(xilinx_drm_hdmi_hardcode_modes); i++) {
		newmode = drm_mode_duplicate(dev, &xilinx_drm_hdmi_hardcode_modes[0]);
		if (!newmode)
			return 0;
		printk(KERN_INFO "Adding hardcoded video mode %d\n", i);

		//hdmi_dbg("Adding hardcoded video mode %d\n", i);
		drm_mode_probed_add(connector, newmode);
	}
	return 0;
}

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int xilinx_drm_hdmi_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *hdmi = to_hdmi(encoder);
	struct edid *edid = NULL;
	int ret;

	hdmi_dbg("xilinx_drm_hdmi_get_modes()\n");
	mutex_lock(&hdmi->hdmi_mutex);

	/* When the I2C adapter connected to the DDC bus is hidden behind a device that
	* exposes a different interface to read EDID blocks this function can be used
	* to get EDID data using a custom block read function. - from drm_edid.c
	*/

#if 0
	hdmi_dbg("HDMI EDID probe disabled for now.\n");
#else
	/* private data hdmi is passed to xilinx_drm_hdmi_get_edid_block(data, ...) */
	edid = drm_do_get_edid(connector, xilinx_drm_hdmi_get_edid_block, hdmi);
#endif
	mutex_unlock(&hdmi->hdmi_mutex);
	if (!edid) {
		hdmi->have_edid = 0;
		dev_err(hdmi->dev, "xilinx_drm_hdmi_get_modes() could not obtain edid, assume <= 1024x768 works.\n");

		drm_add_modes_noedid(connector, 1024, 768);
		//xilinx_drm_hdmi_hardcode(connector);

		return 0;
	}
#if 0 // @TODO remove, this is used during side-by-side testing of DP/HDMI on the same screen
	/* always add 1080p */
	xilinx_drm_hdmi_hardcode(connector);
#endif
	hdmi->have_edid = 1;

	drm_mode_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);
	kfree(edid);
	hdmi_dbg("xilinx_drm_hdmi_get_modes() done\n");

	return ret;
}

static struct drm_encoder_slave_funcs xilinx_drm_hdmi_encoder_funcs = {
	.dpms			= xilinx_drm_hdmi_dpms,
	.save			= xilinx_drm_hdmi_save,
	.restore		= xilinx_drm_hdmi_restore,
	.mode_fixup		= xilinx_drm_hdmi_mode_fixup,
	.mode_valid		= xilinx_drm_hdmi_mode_valid,
#ifdef SI5324_LAST
	.mode_set		= xilinx_drm_hdmi_mode_set_nop,
#else
	.mode_set		= xilinx_drm_hdmi_mode_set,
#endif
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

	hdmi_dbg("xilinx_drm_hdmi_encoder_init()\n");

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

	/* TX SS callback setup */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_CONNECT,
		TxConnectCallback, (void *)hdmi);
#if 0 /* @TODO Add for HDCP */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_TOGGLE,
		TxToggleCallback, (void *)hdmi);
#endif
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
	 * provide one of the phys as reference */
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

static XV_HdmiTxSs_Config config =
{
	.DeviceId = 0,
	.BaseAddress = 0,
	.HighAddress = 0,
	.Ppc = 2,
	.MaxBitsPerPixel = 8,
	/* .AxiLiteClkFreq */
	.RemapperReset = {
		.IsPresent = 0,
		.DeviceId = 255,
		.AddrOffset = 0xFFFFFFFF
	},
	.HdcpTimer = {
		.IsPresent = 0,
		.DeviceId = 255,
		.AddrOffset = 0xFFFFFFFF
	},
	.Hdcp14 = {
		.IsPresent = 0,
		.DeviceId = 255,
		.AddrOffset = 0xFFFFFFFF
	},
	.Hdcp22 = {
		.IsPresent = 0,
		.DeviceId = 255,
		.AddrOffset = 0xFFFFFFFF
	},
	.Remapper = {
		.IsPresent = 0,
		.DeviceId = 255,
		.AddrOffset = 0xFFFFFFFF
	},
	.HdmiTx = {
		.IsPresent = 1,
		.DeviceId = 0,
		.AddrOffset = 0
	},
	.Vtc = {
		.IsPresent = 1,
		.DeviceId = 0,
		.AddrOffset = 0x10000,
	},
};

static XVtc_Config vtc_config = {
	.DeviceId = 0,
	.BaseAddress = 0x10000
};
XVtc_Config *XVtc_LookupConfig(u16 DeviceId)
{
	return &vtc_config;
}

static XV_HdmiTx_Config XV_HdmiTx_FixedConfig =
{
	0,
	0
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

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int xilinx_drm_hdmi_parse_of(struct xilinx_drm_hdmi *hdmi, XV_HdmiTxSs_Config *config)
{
	struct device *dev = hdmi->dev;
	struct device_node *node = dev->of_node;
	int rc;
	u32 val;
	const char *format;

	rc = of_property_read_u32(node, "xlnx,input-pixels-per-clock", &val);
	if (rc < 0)
		goto error_dt;
	config->Ppc = val;

	rc = of_property_read_u32(node, "xlnx,max-bits-per-component", &val);
	if (rc < 0)
		goto error_dt;
	config->MaxBitsPerPixel = val;

	//"xlnx,gfx-fmt";
	rc = of_property_read_u32(node, "xlnx,vtc-offset", &val);
	if (rc < 0) {
		hdmi_dbg("Not using an internal VTC.");
		config->Vtc.IsPresent = 0;
	} else if (rc == 0) {
		config->Vtc.IsPresent = 1;
		config->Vtc.DeviceId = vtc_config.DeviceId = 0;
		config->Vtc.AddrOffset = vtc_config.BaseAddress = val;
	}

	/* NOTE new */
	rc = of_property_read_string(node, "xlnx,pixel-format", &format);
	if (rc < 0) {
		dev_err(hdmi->dev, "xlnx,pixel-format must be specified (\"yuv422\" or \"argb8888\")\n");
		goto error_dt;
	} else
	if (strcmp(format, "yuv422") == 0) {
		hdmi->xvidc_colorfmt = XVIDC_CSF_YCRCB_422;
		hdmi_dbg("yuv422-> XVIDC_CSF_YCRCB_422\n");
	} else if (strcmp(format, "argb8888") == 0) {
		hdmi->xvidc_colorfmt = XVIDC_CSF_RGB;
		hdmi_dbg("argb8888-> XVIDC_CSF_RGB\n");
#if 0 /* @TODO untested */
	} else if (strcmp(format, "yuv420") == 0) {
		hdmi->xvidc_colorfmt = XVIDC_CSF_YCRCB_420;
#endif
	} else {
		dev_err(hdmi->dev, "Unsupported xlnx,pixel-format\n");
		goto error_dt;
	}

	return 0;

error_dt:
	dev_err(hdmi->dev, "Error parsing device tree");
	return rc;
}

static int xilinx_drm_hdmi_probe(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *hdmi;
	int ret;
	unsigned int index;
	struct resource *res;
	unsigned long axi_clk_rate;

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
		dev_err(hdmi->dev, "Could not create work queue\n");
		return -ENOMEM;
	}

	hdmi_dbg("xilinx_drm_hdmi DT parse start\n");
	/* parse open firmware device tree data */
	ret = xilinx_drm_hdmi_parse_of(hdmi, &config);
	hdmi_dbg("xilinx_drm_hdmi DT parse done\n");
	if (ret < 0)
		return ret;

	index = 2;
	{
		char phy_name[32];
		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);

		index = 0;
		hdmi->phy[index] = devm_phy_get(hdmi->dev, phy_name);
		if (IS_ERR(hdmi->phy[index])) {
			ret = PTR_ERR(hdmi->phy[index]);
			if (ret != -EPROBE_DEFER)
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

	hdmi_dbg("config.Vtc.AddrOffset =  0x%x.", (int)config.Vtc.AddrOffset);
	hdmi_dbg("config->Ppc = %d\n", (int)config.Ppc);
	hdmi_dbg("config->MaxBitsPerPixel = %d\n", (int)config.MaxBitsPerPixel);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->iomem = devm_ioremap_resource(hdmi->dev, res);
	if (IS_ERR(hdmi->iomem))
		return PTR_ERR(hdmi->iomem);

	config.BaseAddress = (uintptr_t)hdmi->iomem;
	config.HighAddress = config.BaseAddress + resource_size(res) - 1;

	hdmi_dbg("config.BaseAddress =  %p.", config.BaseAddress);
	hdmi_dbg("config.HighAddress =  %p.", config.HighAddress);

	/* video streaming bus clock */
	hdmi->clk = devm_clk_get(hdmi->dev, "video");
	if (IS_ERR(hdmi->clk)) {
		ret = PTR_ERR(hdmi->clk);
		if (ret != -EPROBE_DEFER)
				dev_err(hdmi->dev, "failed to get video clk\n");
		return ret;
	}

	clk_prepare_enable(hdmi->clk);

	/* AXI lite register bus clock */
	hdmi->axi_lite_clk = devm_clk_get(hdmi->dev, "axi-lite");
	if (IS_ERR(hdmi->axi_lite_clk)) {
		ret = PTR_ERR(hdmi->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(hdmi->dev, "failed to get axi-lite clk\n");
		return ret;
	}

	clk_prepare_enable(hdmi->axi_lite_clk);
	axi_clk_rate = clk_get_rate(hdmi->axi_lite_clk);

	/* get irq */
	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		destroy_workqueue(hdmi->work_queue);
		return hdmi->irq;
	}

	/* support to drive an external retimer IC on the TX path, depending on TX clock line rate */
	hdmi->retimer_clk = devm_clk_get(&pdev->dev, "retimer-clk");
	if (IS_ERR(hdmi->retimer_clk)) {
		ret = PTR_ERR(hdmi->retimer_clk);
		hdmi->retimer_clk = NULL;
		if (ret != -EPROBE_DEFER)
			hdmi_dbg("Did not find a retimer-clk, not driving an external retimer device driver.\n");
	} else if (hdmi->retimer_clk) {
		hdmi_dbg("got retimer-clk\n");
		ret = clk_prepare_enable(hdmi->retimer_clk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable retimer-clk\n");
			return ret;
		}
		hdmi_dbg("prepared and enabled retimer-clk\n");
	} else {
		hdmi_dbg("no retimer-clk specified\n");
	}

	hdmi_dbg("axi_clk_rate = %lu Hz\n", axi_clk_rate);

	config.AxiLiteClkFreq = axi_clk_rate;

	hdmi_dbg("&config = %p\n", &config);
	hdmi_dbg("hdmi->iomem = %lx\n", (unsigned long)hdmi->iomem);

	platform_set_drvdata(pdev, hdmi);

	/* remainder of initialization is in encoder_init() */

	hdmi_dbg("xilinx_drm_hdmi_probe() succesfull.\n");

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
	{ .compatible = "xlnx,v-hdmi-tx-ss-2.0", },
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
