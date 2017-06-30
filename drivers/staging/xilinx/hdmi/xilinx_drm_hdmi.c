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

#define DEBUG_MUTEX

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
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "xilinx_drm_drv.h"

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "xilinx-hdmi-tx/xv_hdmitxss.h"

#ifdef USE_HDCP /* WIP HDCP */
#include "../../../../../hdcp_private_keys.h"
#endif

/* @TODO: should be made public through linux/phy/phy-vphy.h */
//#include "phy-xilinx-vphy/xhdcp22_tx.h"

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

#if (defined(DEBUG_MUTEX) && defined(DEBUG))
/* storage for source code line number where mutex was last locked, -1 otherwise */
static int hdmi_mutex_line = -1;
/* If mutex is locked, print the line number of where it was locked. lock the mutex.
 * Please keep this macro on a single line, so that the C __LINE__ macro is correct.
 */
#  define hdmi_mutex_lock(x) do { if (mutex_is_locked(x)) { hdmi_dbg("@line %d waiting for mutex owner @line %d\n", __LINE__, hdmi_mutex_line); } mutex_lock(x); hdmi_mutex_line = __LINE__; } while(0)
#  define hdmi_mutex_unlock(x) do { hdmi_mutex_line = -1; mutex_unlock(x); } while(0)
/* non-debug variant */
#else
#  define hdmi_mutex_lock(x) mutex_lock(x)
#  define hdmi_mutex_unlock(x) mutex_unlock(x)
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
#ifdef USE_HDCP
	void __iomem *hdcp1x_keymngmt_iomem;
#endif
	/* video streaming bus clock */
	struct clk *clk;
	struct clk *axi_lite_clk;

	/* HDMI TXSS interrupt number */
	int irq;
#ifdef USE_HDCP
	/* HDCP interrupt numbers */
	int hdcp1x_irq;
	int hdcp1x_timer_irq;
	int hdcp22_irq;
	int hdcp22_timer_irq;
	/* controls */
	bool hdcp_authenticate;
	bool hdcp_encrypt;
	/* status */
	bool hdcp_authenticated;
	bool hdcp_encrypted;
	/* delayed work to drive HDCP poll */
	struct delayed_work delayed_work_hdcp_poll;
	struct delayed_work delayed_work_hdcp_info;
#endif

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
	bool is_hdmi_20_sink;
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

#if 0 /* future work for enable/disable HDCP */
static ssize_t show_hdcp_authenticate(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", xhdmi->authenticate);
}

static ssize_t store_hdcp_authenticate(struct kobject *kobj, struct attribute *attr,
                                  const char *buf, size_t count)
{
	int ret, enable;
	u32 Status;

	ret = sscanf(buf, "%d", &enable);
	if (ret != 1 || enable < 0 || enable > 1)
		return -EINVAL;
	if (!xhdmi->hdcp_authenticate && enable) {
		u32 Status;
		xhdmi->hdcp_authenticate = 1;
		Status = XV_HdmiTxSs_HdcpAuthRequest(HdmiTxSsPtr);
		hdmi_dbg("TxStreamUpCallback(): TX XV_HdmiTxSs_HdcpAuthRequest() %s\n",
			Status == XST_SUCCESS? "Succesfull": "FAILED");

		/* immediately start the HDCP polling action */
		cancel_delayed_work_sync(&xhdmi->delayed_work_hdcp_poll);
		schedule_delayed_work(&xhdmi->delayed_work_hdcp_poll, 0);
	}
	return count;
}
static DEVICE_ATTR(hdcp_authenticate, 0644, show_hdcp_authenticate, store_hdcp_authenticate);

static struct attribute *attrs[] = {
	&dev_attr_hdcp_authenticate.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs,
	.name = "hdcp"
};
#endif

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
	struct xilinx_drm_hdmi *xhdmi;

	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;

	BUG_ON(!dev_id);
	xhdmi = (struct xilinx_drm_hdmi *)dev_id;
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	if (HdmiTxSsPtr->IsReady != XIL_COMPONENT_IS_READY) {
		printk(KERN_INFO "hdmitx_irq_handler(): HDMI TX SS is not initialized?!\n");
	}

#if 1
	/* read status registers */
	xhdmi->IntrStatus[0] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_PIO_STA_OFFSET)) & (XV_HDMITX_PIO_STA_IRQ_MASK);
	xhdmi->IntrStatus[1] = XV_HdmiTx_ReadReg(HdmiTxSsPtr->HdmiTxPtr->Config.BaseAddress, (XV_HDMITX_DDC_STA_OFFSET)) & (XV_HDMITX_DDC_STA_IRQ_MASK);
#endif

	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	/* mask interrupt request */
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);

	/* call bottom-half */
	return IRQ_WAKE_THREAD;
}

/* @NOTE remove later; debugging purposes */
#define HDMITX_DEBUG_IRQ 0

/* (struct xilinx_drm_hdmi *)dev_id */
static irqreturn_t hdmitx_irq_thread(int irq, void *dev_id)
{
	struct xilinx_drm_hdmi *xhdmi;
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
	xhdmi = (struct xilinx_drm_hdmi *)dev_id;
	if (!xhdmi) {
		printk(KERN_INFO "irq_thread: !dev_id\n");
		return IRQ_HANDLED;
	}
	/* driver is being torn down, do not process further interrupts */
	if (xhdmi->teardown) {
		printk(KERN_INFO "irq_thread: teardown\n");
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	hdmi_mutex_lock(&xhdmi->hdmi_mutex);

/* @NOTE remove later; debugging purposes */
#if (defined(HDMITX_DEBUG_IRQ) && HDMITX_DEBUG_IRQ)
	for (i = 0; i < NUM_SUBCORE_IRQ; i++) {
		which[i] = xhdmi->IntrStatus[i]? '0' + i: '.';
		which_mask |= (xhdmi->IntrStatus[i]? 1: 0) << i;
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
	 xhdmi->IntrStatus[0], (int)Data, xhdmi->IntrStatus[0]);
#endif

	/* call baremetal interrupt handler, this in turn will
	 * call the registed callbacks functions */
	if (xhdmi->IntrStatus[0]) HdmiTx_PioIntrHandler(HdmiTxSsPtr->HdmiTxPtr);
	if (xhdmi->IntrStatus[1]) HdmiTx_DdcIntrHandler(HdmiTxSsPtr->HdmiTxPtr);

	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);

	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	/* unmask interrupt request */
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	/* @TODO unmask HDCP1x interrupt requests */
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);

/* @NOTE remove later; debugging purposes */
#if (defined(HDMITX_DEBUG_IRQ) && HDMITX_DEBUG_IRQ)
	printk(KERN_INFO "hdmitx_irq_thread() %s 0x%08x done\n", which, (int)which_mask);
#endif

	return IRQ_HANDLED;
}

/* if defined, the HDCP handling runs in interrupt context and thus the
 * DDC might race against EDID */
//#define RUN_HDCP_IN_INTERRUPT_CONTEXT

/* top-half interrupt handler for HDMI TX HDCP */
static irqreturn_t hdmitx_hdcp_irq_handler(int irq, void *dev_id)
{
	struct xilinx_drm_hdmi *xhdmi;

	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;

	BUG_ON(!dev_id);
	xhdmi = (struct xilinx_drm_hdmi *)dev_id;
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

 /* use desired threaded irq handler approach*/
#ifndef RUN_HDCP_IN_INTERRUPT_CONTEXT
	if (irq == xhdmi->hdcp1x_irq) {
		disable_irq_nosync(irq);
		/* mask/disable interrupt request from HDCP1x link status update */
		//XHdcp1x_WriteReg(HdmiTxSsPtr->Hdcp14Ptr->Config.BaseAddress,
		//	XHDCP1X_CIPHER_REG_INTERRUPT_MASK, (u32)0xFFFFFFFFu);
	}
	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	/* mask/disable interrupt requests from timers */
	if (irq == xhdmi->hdcp1x_timer_irq) {
		XTmrCtr_DisableIntr(HdmiTxSsPtr->HdcpTimerPtr->BaseAddress, 0);
	} else if (irq == xhdmi->hdcp22_timer_irq) {
		XTmrCtr_DisableIntr(HdmiTxSsPtr->Hdcp22Ptr->Timer.TmrCtr.BaseAddress, 0);
	}
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);
	//printk(KERN_INFO "hdmitx_hdcp_irq_handler(irq = %d) wake thread\n", irq);
	/* call bottom-half */
	return IRQ_WAKE_THREAD;
#else
	/* run HDCP interrupt handler functions in interrupt context */
	XV_HdmiTxSS_HdcpIntrHandler(HdmiTxSsPtr);
	XV_HdmiTxSS_HdcpTimerIntrHandler(HdmiTxSsPtr);
	XV_HdmiTxSS_Hdcp22TimerIntrHandler(HdmiTxSsPtr);
	/* will not result in bottom-half to run */
	return IRQ_HANDLED;
#endif
}

/* (struct xilinx_drm_hdmi *)dev_id */
/* @TODO this function is currently unused, as there is no bare-metal API
 * mask/unmask functionality exposed required to run the interrupt handlers in a top and bottom half */
static irqreturn_t hdmitx_hdcp_irq_thread(int irq, void *dev_id)
{
	struct xilinx_drm_hdmi *xhdmi;
	XV_HdmiTxSs *HdmiTxSsPtr;
	unsigned long flags;

	BUG_ON(!dev_id);
	xhdmi = (struct xilinx_drm_hdmi *)dev_id;
	if (!xhdmi) {
		printk(KERN_INFO "irq_thread: !dev_id\n");
		return IRQ_HANDLED;
	}
	/* driver is being torn down, do not process further interrupts */
	if (xhdmi->teardown) {
		printk(KERN_INFO "irq_thread: teardown\n");
		return IRQ_HANDLED;
	}
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr->HdmiTxPtr);

	//printk(KERN_INFO "hdmitx_hdcp_irq_thread(irq = %d)\n", irq);

	/* invoke the bare-metal interrupt handler under mutex lock */
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	if (irq == xhdmi->hdcp1x_irq) {
		XV_HdmiTxSS_HdcpIntrHandler(HdmiTxSsPtr);
	} else if (irq == xhdmi->hdcp1x_timer_irq) {
		XV_HdmiTxSS_HdcpTimerIntrHandler(HdmiTxSsPtr);
	} else if (irq == xhdmi->hdcp22_timer_irq) {
		XV_HdmiTxSS_Hdcp22TimerIntrHandler(HdmiTxSsPtr);
	}
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);

	/* re-enable interrupt requests */

	/* unmask/enable interrupt request from HDCP1x link status update */
	if (irq == xhdmi->hdcp1x_irq) {
		enable_irq(irq);
		//XHdcp1x_WriteReg(HdmiTxSsPtr->Hdcp14Ptr->Config.BaseAddress,
		//	XHDCP1X_CIPHER_REG_INTERRUPT_MASK, (u32)0xFFFFFFFDu);
	}

	/* unmask/enable interrupt requests from timers */
	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	if (irq == xhdmi->hdcp1x_timer_irq) {
		XTmrCtr_EnableIntr(HdmiTxSsPtr->HdcpTimerPtr->BaseAddress, 0);
	} else if (irq == xhdmi->hdcp22_timer_irq) {
		XTmrCtr_EnableIntr(HdmiTxSsPtr->Hdcp22Ptr->Timer.TmrCtr.BaseAddress, 0);
	}
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);

	//printk(KERN_INFO "hdmitx_irq_thread() done\n");

	return IRQ_HANDLED;
}

static void XHdcp_Authenticate(XV_HdmiTxSs *HdmiTxSsPtr)
{
	u32 Status;
	hdmi_dbg("XHdcp_Authenticate()\n");
#if 1
	if ((!(XV_HdmiTxSs_HdcpIsAuthenticated(HdmiTxSsPtr)) ||
		XV_HdmiTxSs_IsStreamToggled(HdmiTxSsPtr)) &&
		!(XV_HdmiTxSs_HdcpIsInProgress(HdmiTxSsPtr))) {
			XV_HdmiTxSs_HdcpPushEvent(HdmiTxSsPtr, XV_HDMITXSS_HDCP_AUTHENTICATE_EVT);
		hdmi_dbg("XV_HdmiTxSs_HdcpPushEvent(XV_HDMITXSS_HDCP_AUTHENTICATE_EVT)\n");
	}
#else
	Status = XV_HdmiTxSs_HdcpAuthRequest(HdmiTxSsPtr);
	hdmi_dbg("TxHdcpUnauthenticatedCallback(): TX XV_HdmiTxSs_HdcpAuthRequest() %s\n",
		Status == XST_SUCCESS? "Succesfull": "FAILED");
#endif
}

static void TxToggleCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	XVphy *VphyPtr = xhdmi->xvphy;
	BUG_ON(!xhdmi);
	BUG_ON(!HdmiTxSsPtr);
	BUG_ON(!VphyPtr);
	BUG_ON(!xhdmi->phy[0]);
	hdmi_dbg("TxToggleCallback()\n");

	if (xhdmi->hdcp_authenticate) {
		XHdcp_Authenticate(HdmiTxSsPtr);
	}
}

static void TxConnectCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	XVphy *VphyPtr = xhdmi->xvphy;
	BUG_ON(!xhdmi);
	BUG_ON(!HdmiTxSsPtr);
	BUG_ON(!VphyPtr);
	BUG_ON(!xhdmi->phy[0]);
	hdmi_dbg("TxConnectCallback()\n");

	xvphy_mutex_lock(xhdmi->phy[0]);
	if (HdmiTxSsPtr->IsStreamConnected) {
		int xst_hdmi20;
		xhdmi->cable_connected = 1;
		/* Check HDMI sink version */
		xst_hdmi20 = XV_HdmiTxSs_DetectHdmi20(HdmiTxSsPtr);
		hdmi_dbg("TxConnectCallback(): TX connected to HDMI %s Sink Device\n",
			(xst_hdmi20 == XST_SUCCESS)? "2.0": "1.4");
		xhdmi->is_hdmi_20_sink = (xst_hdmi20 == XST_SUCCESS);
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (TRUE));
	}
	else {
		hdmi_dbg("TxConnectCallback(): TX disconnected\n");
		xhdmi->cable_connected = 0;
		xhdmi->hdmi_stream_up = 0;
		xhdmi->have_edid = 0;
		xhdmi->is_hdmi_20_sink = 0;
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, (FALSE));
	}
	xvphy_mutex_unlock(xhdmi->phy[0]);
#if 0
	if (xhdmi->drm_dev) {
		/* release the mutex so that our drm ops can re-acquire it */
		hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
		hdmi_dbg("TxConnectCallback() -> drm_kms_helper_hotplug_event()\n");
		drm_kms_helper_hotplug_event(xhdmi->drm_dev);
		hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	}
#endif
	hdmi_dbg("TxConnectCallback() done\n");
}

static void TxStreamUpCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	XVidC_VideoStream *HdmiTxSsVidStreamPtr;
	XVphy_PllType TxPllType;
	u64 TxLineRate;
	u32 Status;

	BUG_ON(!xhdmi);

	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_dbg("TxStreamUpCallback(): TX stream is up\n");
	xhdmi->hdmi_stream_up = 1;

#if 0
  XVidC_VideoStream *HdmiTxSsVidStreamPtr;

  HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(&HdmiTxSs);

  /* In passthrough copy the RX stream parameters to the TX stream */
  if (IsPassThrough) {
	  XV_HdmiTxSs_SetVideoStream(HdmiTxSsPtr, *HdmiTxSsVidStreamPtr);
  }
#endif

	xvphy_mutex_lock(xhdmi->phy[0]);
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
	if (xhdmi->retimer_clk) {
		hdmi_dbg("retimer: clk_set_rate(xhdmi->retimer_clk, TxLineRate=%lld\n", TxLineRate);
		(void)clk_set_rate(xhdmi->retimer_clk, (signed long long)TxLineRate);
	}

	/* Enable TX TMDS clock*/
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (TRUE));

	/* Copy Sampling Rate */
	XV_HdmiTxSs_SetSamplingRate(HdmiTxSsPtr, VphyPtr->HdmiTxSampleRate);
	xvphy_mutex_unlock(xhdmi->phy[0]);

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
#ifdef DEBUG
	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);
	XVidC_ReportStreamInfo(HdmiTxSsVidStreamPtr);
#endif
#if 0
	XV_HdmiTx_DebugInfo(HdmiTxSsPtr->HdmiTxPtr);
	hdmi_dbg("TxStreamUpCallback(): TX XV_HdmiTxSs_HdcpEnable()\n");
	XV_HdmiTxSs_HdcpEnable(HdmiTxSsPtr);
#endif
#if 1
	//cancel_delayed_work_sync(&xhdmi->delayed_work_hdcp_poll);

	if (xhdmi->hdcp_authenticate) {
		XHdcp_Authenticate(HdmiTxSsPtr);
		hdmi_dbg("TxStreamUpCallback(): TX XHdcp_Authenticate\n");

		/* immediately start the HDCP polling action */
		//schedule_delayed_work(&xhdmi->delayed_work_hdcp_poll, 0);
	}
#endif
	hdmi_dbg("TxStreamUpCallback(): done\n");
}

static void TxStreamDownCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	//XVidC_VideoStream *HdmiTxSsVidStreamPtr;

	BUG_ON(!xhdmi);

	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_dbg("TxStreamDownCallback(): TX stream is down\n\r");
	xhdmi->hdmi_stream_up = 0;

	xhdmi->hdcp_authenticated = 0;
	xhdmi->hdcp_encrypted = 0;
}

void TxHdcpAuthenticatedCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmi);
	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	xhdmi->hdcp_authenticated = 1;
	if (XV_HdmiTxSs_HdcpGetProtocol(HdmiTxSsPtr) == XV_HDMITXSS_HDCP_22) {
		hdmi_dbg("HDCP 2.2 TX authenticated.\n");
	}

	else if (XV_HdmiTxSs_HdcpGetProtocol(HdmiTxSsPtr) == XV_HDMITXSS_HDCP_14) {
		hdmi_dbg("HDCP 1.4 TX authenticated.\n");
	}

	if (xhdmi->hdcp_encrypt) {
		hdmi_dbg("Enabling Encryption.\n");
		XV_HdmiTxSs_HdcpEnableEncryption(HdmiTxSsPtr);
		xhdmi->hdcp_encrypted = 1;
	} else {
		hdmi_dbg("Not Enabling Encryption.\n");
	}
}

void TxHdcpUnauthenticatedCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmi);
	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	xhdmi->hdcp_authenticated = 0;

	//cancel_delayed_work_sync(&xhdmi->delayed_work_hdcp_poll);
	if (xhdmi->hdcp_authenticate) {
		/* retry authentication */
		if (XV_HdmiTxSs_HdcpGetProtocol(HdmiTxSsPtr) == XV_HDMITXSS_HDCP_14) {
			XHdcp_Authenticate(HdmiTxSsPtr);
			/* immediately re-start the HDCP polling action */
			//schedule_delayed_work(&xhdmi->delayed_work_hdcp_poll, 0);
		}
	}
}

static void TxVsCallback(void *CallbackRef)
{
#if 0
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XV_HdmiTxSs *HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	XVphy *VphyPtr = xhdmi->xvphy;
	BUG_ON(!xhdmi);
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
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmi);

	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_dbg("VphyHdmiTxInitCallback(): XV_HdmiTxSs_RefClockChangeInit()\n");

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(xhdmi->phy[0]);
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	xvphy_mutex_lock(xhdmi->phy[0]);

	XV_HdmiTxSs_RefClockChangeInit(HdmiTxSsPtr);

	/* unlock RX SS but keep XVPHY locked */
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
	//hdmi_dbg("VphyHdmiTxInitCallback() done\n");
}

/* entered with vphy mutex taken */
static void VphyHdmiTxReadyCallback(void *CallbackRef)
{
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)CallbackRef;
	XVphy *VphyPtr;
	XV_HdmiTxSs *HdmiTxSsPtr;
	u32 Status;
	BUG_ON(!xhdmi);

	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_dbg("VphyHdmiTxReadyCallback()\n");


	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(xhdmi->phy[0]);
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	xvphy_mutex_lock(xhdmi->phy[0]);

	/* unlock RX SS but keep XVPHY locked */
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
	hdmi_dbg("VphyHdmiTxReadyCallback() done\n");
}

/* drm_encoder_slave_funcs */
static void xilinx_drm_hdmi_dpms(struct drm_encoder *encoder, int dpms)
{
	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
#if 0
	void __iomem *iomem = xhdmi->iomem;
	unsigned int i;
	int ret;
#endif
	hdmi_dbg("xilinx_drm_hdmi_dpms(dpms = %d)\n", dpms);

	if (xhdmi->dpms == dpms) {
		goto done;
	}

	xhdmi->dpms = dpms;

	switch (dpms) {
	case DRM_MODE_DPMS_ON:
		/* power-up */
		goto done;
	default:
		/* power-down */
		goto done;
	}
done:
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
}

static void xilinx_drm_hdmi_save(struct drm_encoder *encoder)
{
	/* no op */
}

static void xilinx_drm_hdmi_restore(struct drm_encoder *encoder)
{
	/* no op */
}

/* The HDMI C API requires the reference clock rate to be changed after setting the mode.
 * However, DRM order is fixup(), set clock rate, than mode_set().
 *
 * Defining CHANGE_CLOCKRATE_LAST will do the actual mode set in the fixup(), and will
 * make mode_set() a NOP. This way, the required HDMI C API requirement is met.
 */
#define CHANGE_CLOCKRATE_LAST

#ifdef CHANGE_CLOCKRATE_LAST
/* prototype */
static void xilinx_drm_hdmi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode);
#endif

static bool xilinx_drm_hdmi_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	XVphy *VphyPtr;
	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	/* @NOTE LEON: we are calling mode_set here, just before the reference clock is changed */

	hdmi_dbg("xilinx_drm_hdmi_mode_fixup()\n");
#ifdef CHANGE_CLOCKRATE_LAST
	xilinx_drm_hdmi_mode_set(encoder, (struct drm_display_mode *)mode, adjusted_mode);
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
	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	int max_rate = 340 * 1000;
	enum drm_mode_status status = MODE_OK;

	hdmi_dbg("xilinx_drm_hdmi_mode_valid()\n");
	drm_mode_debug_printmodeline(mode);
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	/* HDMI 2.0 sink connected? */
	if (xhdmi->is_hdmi_20_sink)
		max_rate = 600 * 1000;
	/* pixel clock too high for sink? */
	if (mode->clock > max_rate)
		status = MODE_CLOCK_HIGH;
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
	return status;
}

#ifdef CHANGE_CLOCKRATE_LAST
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
	static int nudge = 0;

	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	hdmi_dbg("xilinx_drm_hdmi_mode_set()\n");
	BUG_ON(!xhdmi);

	HdmiTxSsPtr = &xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	VphyPtr = xhdmi->xvphy;
	BUG_ON(!VphyPtr);

	hdmi_mutex_lock(&xhdmi->hdmi_mutex);

	xvphy_mutex_lock(xhdmi->phy[0]);

	drm_mode_debug_printmodeline(mode);

	/* Disable VPhy Clock buffer to force a frequency change event */
	hdmi_dbg("VPhy Clock Buffer - Disabled\n");
	XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, 0);

#ifdef DEBUG
	hdmi_dbg("mode->clock = %d\n", mode->clock * 1000);
	hdmi_dbg("mode->crtc_clock = %d\n", mode->crtc_clock * 1000);


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
	hdmi_dbg("mode->vrefresh = %d\n", mode->vrefresh);
#endif
	/* see slide 20 of http://events.linuxfoundation.org/sites/events/files/slides/brezillon-drm-kms.pdf */
	vt.HActive = mode->hdisplay;
	vt.HFrontPorch = mode->hsync_start - mode->hdisplay;
	vt.HSyncWidth = mode->hsync_end - mode->hsync_start;
	vt.HBackPorch = mode->htotal - mode->hsync_end;
	vt.HTotal = mode->htotal;
	vt.HSyncPolarity = !!(mode->flags & DRM_MODE_FLAG_PHSYNC);

	vt.VActive = mode->vdisplay;
	/* Progressive timing data is stored in field 0 */
	vt.F0PVFrontPorch = mode->vsync_start - mode->vdisplay;
	vt.F0PVSyncWidth = mode->vsync_end - mode->vsync_start;
	vt.F0PVBackPorch = mode->vtotal - mode->vsync_end;
	vt.F0PVTotal = mode->vtotal;
	/* Interlaced output is not support - set field 1 to 0 */
	vt.F1VFrontPorch = 0;
	vt.F1VSyncWidth = 0;
	vt.F1VBackPorch = 0;
	vt.F1VTotal = 0;
	vt.VSyncPolarity = !!(mode->flags & DRM_MODE_FLAG_PVSYNC);

	HdmiTxSsVidStreamPtr = XV_HdmiTxSs_GetVideoStream(HdmiTxSsPtr);

	if (XVphy_IsBonded(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1)) {
		hdmi_dbg("Both the GT RX and GT TX are clocked by the RX reference clock.\n");
		xvphy_mutex_unlock(xhdmi->phy[0]);
		hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
		return;
	}

	/* Disable TX TDMS clock */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_TX, (FALSE));

#if 0
	VmId = XVidC_GetVideoModeId(mode->hdisplay, mode->vdisplay, mode->vrefresh, FALSE);
#else
	VmId = XVidC_GetVideoModeIdWBlanking(&vt, mode->vrefresh, FALSE);
#endif
	hdmi_dbg("VmId = %d\n", VmId);
	if (VmId == XVIDC_VM_NOT_SUPPORTED) { //no match found in timing table
		hdmi_dbg("Tx Video Mode not supported. Using DRM Timing\n");
		VmId = XVIDC_VM_CUSTOM;
		HdmiTxSsVidStreamPtr->FrameRate = mode->vrefresh;
		HdmiTxSsVidStreamPtr->Timing = vt; //overwrite with drm detected timing
		XVidC_ReportTiming(&HdmiTxSsVidStreamPtr->Timing, FALSE);
	}
	TmdsClock = XV_HdmiTxSs_SetStream(HdmiTxSsPtr, VmId, xhdmi->xvidc_colorfmt, XVIDC_BPC_8, NULL);

	VphyPtr->HdmiTxRefClkHz = TmdsClock;
	hdmi_dbg("(TmdsClock = %u, from XV_HdmiTxSs_SetStream())\n", TmdsClock);

	hdmi_dbg("XVphy_SetHdmiTxParam(PixPerClk = %d, ColorDepth = %d, ColorFormatId=%d)\n",
		(int)HdmiTxSsVidStreamPtr->PixPerClk, (int)HdmiTxSsVidStreamPtr->ColorDepth,
		(int)HdmiTxSsVidStreamPtr->ColorFormatId);

	// Set GT TX parameters, this might change VphyPtr->HdmiTxRefClkHz
	Result = XVphy_SetHdmiTxParam(VphyPtr, 0, XVPHY_CHANNEL_ID_CHA,
					HdmiTxSsVidStreamPtr->PixPerClk,
					HdmiTxSsVidStreamPtr->ColorDepth,
					HdmiTxSsVidStreamPtr->ColorFormatId);

	if (Result == (XST_FAILURE)) {
		hdmi_dbg("Unable to set requested TX video resolution.\n\r");
		xvphy_mutex_unlock(xhdmi->phy[0]);
		hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
		return;
	}

	/* Enable VPhy Clock buffer - Reacquire Tx Ref Clock and triggers frequency change */
	hdmi_dbg("VPhy Clock Buffer - Enabled\n");
	XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_TX, 1);

	adjusted_mode->clock = VphyPtr->HdmiTxRefClkHz / 1000;
	hdmi_dbg("adjusted_mode->clock = %u Hz\n", adjusted_mode->clock);

	if (nudge)
	{
#if 0
		adjusted_mode->clock += 1;
#endif
	}
	nudge = !nudge;

	/* Disable RX clock forwarding */
	XVphy_Clkout1OBufTdsEnable(VphyPtr, XVPHY_DIR_RX, (FALSE));

	/* @NOTE in bare-metal, here the Si5324 clock is changed. If this mode_set()
	 * is run from the fixup() call, we mimick that behaviour */

	//XVidC_ReportStreamInfo(HdmiTxSsVidStreamPtr);
	XV_HdmiTx_DebugInfo(HdmiTxSsPtr->HdmiTxPtr);
	XVphy_HdmiDebugInfo(VphyPtr, 0, XVPHY_CHANNEL_ID_CHA);
	xvphy_mutex_unlock(xhdmi->phy[0]);
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
}

static enum drm_connector_status
xilinx_drm_hdmi_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
	/* it takes HDMI 50 ms to detect connection on init */
	static int first_time_ms = 50;
	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	/* first time; wait 50 ms max until cable connected */
	while (first_time_ms && !xhdmi->cable_connected) {
		msleep(1);
		first_time_ms--;
	}
	if (first_time_ms)
	/* after first time, report immediately */
	first_time_ms = 0;
		hdmi_dbg("xilinx_drm_hdmi_detect() waited %d ms until connect.\n", 50 - first_time_ms);
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
	/* cable connected  */
	if (xhdmi->cable_connected) {
		hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
		hdmi_dbg("xilinx_drm_hdmi_detect() = connected\n");
		return connector_status_connected;
	}
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
	hdmi_dbg("xilinx_drm_hdmi_detect() = disconnected\n");
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
	struct xilinx_drm_hdmi *xhdmi = (struct xilinx_drm_hdmi *)data;
	XV_HdmiTxSs *HdmiTxSsPtr;
	int ret;

	BUG_ON(!xhdmi);
	/* out of bounds? */
	if (((block * 128) + len) > 256) return -EINVAL;

	buffer = kzalloc(256, GFP_KERNEL);
	if (!buffer) return -ENOMEM;


	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
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

/* just for testing on LG side-by-side mode, it supports these modes
 * but does not expose them on EDID, so force them in */
#if 0
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
	//struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);

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
#endif

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int xilinx_drm_hdmi_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *xhdmi = to_hdmi(encoder);
	struct edid *edid = NULL;
	int ret;

	hdmi_dbg("xilinx_drm_hdmi_get_modes()\n");
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);

	/* When the I2C adapter connected to the DDC bus is hidden behind a device that
	* exposes a different interface to read EDID blocks this function can be used
	* to get EDID data using a custom block read function. - from drm_edid.c
	*/

#if 0
	hdmi_dbg("HDMI EDID probe disabled for now.\n");
#else
	/* private data hdmi is passed to xilinx_drm_hdmi_get_edid_block(data, ...) */
	edid = drm_do_get_edid(connector, xilinx_drm_hdmi_get_edid_block, xhdmi);
#endif
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
	if (!edid) {
		xhdmi->have_edid = 0;
		dev_err(xhdmi->dev, "xilinx_drm_hdmi_get_modes() could not obtain edid, assume <= 1024x768 works.\n");
#if 0
		drm_add_modes_noedid(connector, 1024, 768);
		//xilinx_drm_hdmi_hardcode(connector);
#endif
		return 0;
	}
#if 0 // @TODO remove, this is used during side-by-side testing of DP/HDMI on the same screen
	/* always add 1080p */
	xilinx_drm_hdmi_hardcode(connector);
#endif
	xhdmi->have_edid = 1;

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
#ifdef CHANGE_CLOCKRATE_LAST
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
	struct xilinx_drm_hdmi *xhdmi = platform_get_drvdata(pdev);
	unsigned long flags;
	XV_HdmiTxSs *HdmiTxSsPtr;
	u32 Status;
	int ret;

	BUG_ON(!xhdmi);

	hdmi_dbg("xilinx_drm_hdmi_encoder_init()\n");

	encoder->slave_priv = xhdmi;
	encoder->slave_funcs = &xilinx_drm_hdmi_encoder_funcs;

	xhdmi->encoder = &encoder->base;
	xhdmi->drm_dev = dev;

	hdmi_mutex_lock(&xhdmi->hdmi_mutex);

	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;

	BUG_ON(!HdmiTxSsPtr);

	Status = XV_HdmiTxSs_CfgInitialize(HdmiTxSsPtr, &config, (uintptr_t)xhdmi->iomem);
	if (Status != XST_SUCCESS)
	{
		dev_err(xhdmi->dev, "initialization failed with error %d\n", Status);
		return -EINVAL;
	}

	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrDisable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);

	/* TX SS callback setup */
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_CONNECT,
		TxConnectCallback, (void *)xhdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_TOGGLE,
		TxToggleCallback, (void *)xhdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_VS,
		TxVsCallback, (void *)xhdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_UP,
		TxStreamUpCallback, (void *)xhdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_STREAM_DOWN,
		TxStreamDownCallback, (void *)xhdmi);

	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_HDCP_AUTHENTICATED,
		TxHdcpAuthenticatedCallback, (void *)xhdmi);
	XV_HdmiTxSs_SetCallback(HdmiTxSsPtr, XV_HDMITXSS_HANDLER_HDCP_UNAUTHENTICATED,
		TxHdcpUnauthenticatedCallback, (void *)xhdmi);

	/* get a reference to the XVphy data structure */
	xhdmi->xvphy = xvphy_get_xvphy(xhdmi->phy[0]);

	BUG_ON(!xhdmi->xvphy);

	xvphy_mutex_lock(xhdmi->phy[0]);
	/* the callback is not specific to a single lane, but we need to
	 * provide one of the phys as reference */
	XVphy_SetHdmiCallback(xhdmi->xvphy, XVPHY_HDMI_HANDLER_TXINIT,
		VphyHdmiTxInitCallback, (void *)xhdmi);

	XVphy_SetHdmiCallback(xhdmi->xvphy, XVPHY_HDMI_HANDLER_TXREADY,
		VphyHdmiTxReadyCallback, (void *)xhdmi);
	xvphy_mutex_unlock(xhdmi->phy[0]);

	/* Request the HDMI TX interrupt */
	ret = devm_request_threaded_irq(&pdev->dev, xhdmi->irq, hdmitx_irq_handler, hdmitx_irq_thread,
		IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss", xhdmi/*dev_id*/);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmi->irq);
		return ret;
	}

	/* HDCP 1.4 Cipher interrupt */
	if (xhdmi->hdcp1x_irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, xhdmi->hdcp1x_irq, hdmitx_hdcp_irq_handler, hdmitx_hdcp_irq_thread,
			IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss-hdcp1x-cipher", xhdmi/*dev_id*/);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmi->hdcp1x_irq);
			return ret;
		}
	}

	/* HDCP 1.4 Timer interrupt */
	if (xhdmi->hdcp1x_timer_irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, xhdmi->hdcp1x_timer_irq, hdmitx_hdcp_irq_handler, hdmitx_hdcp_irq_thread,
			IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss-hdcp1x-timer", xhdmi/*dev_id*/);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmi->hdcp1x_timer_irq);
			return ret;
		}
	}

	/* HDCP 2.2 Timer interrupt */
	if (xhdmi->hdcp22_timer_irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, xhdmi->hdcp22_timer_irq, hdmitx_hdcp_irq_handler, hdmitx_hdcp_irq_thread,
			IRQF_TRIGGER_HIGH /*| IRQF_SHARED*/, "xilinx-hdmitxss-hdcp22-timer", xhdmi/*dev_id*/);
		if (ret) {
			dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmi->hdcp22_timer_irq);
			return ret;
		}
	}

	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);

	spin_lock_irqsave(&xhdmi->irq_lock, flags);
	XV_HdmiTxSs_IntrEnable(HdmiTxSsPtr);
	spin_unlock_irqrestore(&xhdmi->irq_lock, flags);

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
	/* HDCP 1.4 timer, AddrOffset is relative to TxSs */
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
		.IsPresent = 0,
		.DeviceId = 0,
		.AddrOffset = 0,
	},
};

XVtc_Config XVtc_ConfigTable[XPAR_XVTC_NUM_INSTANCES];

#if 0
static XVtc_Config vtc_config = {
	.DeviceId = 0,
	.BaseAddress = 0x10000
};
XVtc_Config *XVtc_LookupConfig(u16 DeviceId)
{
	return &vtc_config;
}
#endif

XV_HdmiTx_Config XV_HdmiTx_ConfigTable[XPAR_XV_HDMITX_NUM_INSTANCES];

/* Just to make sure the function symbol exist, not used */
XGpio_Config *XGpio_LookupConfig_TX(u16 DeviceId)
{
	BUG_ON(1);
	return (XGpio_Config *)NULL;
}
/* Just to make sure the function symbol exist, not used */
XV_axi4s_remap_Config* XV_axi4s_remap_LookupConfig_TX(u16 DeviceId) {
	BUG_ON(1);
	return NULL;
}

#ifdef USE_HDCP /* WIP HDCP */
extern XHdcp22_Cipher_Config XHdcp22_Cipher_ConfigTable[];
extern XHdcp22_Rng_Config XHdcp22_Rng_ConfigTable[];
extern XHdcp1x_Config XHdcp1x_ConfigTable[];
extern XTmrCtr_Config XTmrCtr_ConfigTable[];
extern XHdcp22_Tx_Config XHdcp22_Tx_ConfigTable[];

static int XHdcp_KeyManagerInit(uintptr_t BaseAddress, u8 *Hdcp14Key)
{
	u32 RegValue;
	u8 Row;
	u8 i;
	u8 *KeyPtr;
	u8 Status;

	/* Assign key pointer */
	KeyPtr = Hdcp14Key;

	/* Reset */
	Xil_Out32((BaseAddress + 0x0c), (1<<31));

	// There are 41 rows
	for (Row=0; Row<41; Row++)
	{
		/* Set write enable */
		Xil_Out32((BaseAddress + 0x20), 1);

		/* High data */
		RegValue = 0;
		for (i=0; i<4; i++)
		{
			RegValue <<= 8;
			RegValue |= *KeyPtr;
			KeyPtr++;
		}

		/* Write high data */
		Xil_Out32((BaseAddress + 0x2c), RegValue);

		/* Low data */
		RegValue = 0;
		for (i=0; i<4; i++)
		{
			RegValue <<= 8;
			RegValue |= *KeyPtr;
			KeyPtr++;
		}

		/* Write low data */
		Xil_Out32((BaseAddress + 0x30), RegValue);

		/* Table / Row Address */
		Xil_Out32((BaseAddress + 0x28), Row);

		// Write in progress
		do
		{
			RegValue = Xil_In32(BaseAddress + 0x24);
			RegValue &= 1;
		} while (RegValue != 0);
	}

	// Verify

	/* Re-Assign key pointer */
	KeyPtr = Hdcp14Key;

	/* Default Status */
	Status = XST_SUCCESS;

	/* Start at row 0 */
	Row = 0;

	do
	{
		/* Set read enable */
		Xil_Out32((BaseAddress + 0x20), (1<<1));

		/* Table / Row Address */
		Xil_Out32((BaseAddress + 0x28), Row);

		// Read in progress
		do
		{
			RegValue = Xil_In32(BaseAddress + 0x24);
			RegValue &= 1;
		} while (RegValue != 0);

		/* High data */
		RegValue = 0;
		for (i=0; i<4; i++)
		{
			RegValue <<= 8;
			RegValue |= *KeyPtr;
			KeyPtr++;
		}

		if (RegValue != Xil_In32(BaseAddress + 0x2c))
			Status = XST_FAILURE;

		/* Low data */
		RegValue = 0;
		for (i=0; i<4; i++)
		{
			RegValue <<= 8;
			RegValue |= *KeyPtr;
			KeyPtr++;
		}

		if (RegValue != Xil_In32(BaseAddress + 0x30))
			Status = XST_FAILURE;

		/* Increment row */
		Row++;

	} while ((Row<41) && (Status == XST_SUCCESS));

	if (Status == XST_SUCCESS)
	{
		/* Set read lockout */
		Xil_Out32((BaseAddress + 0x20), (1<<31));

		/* Start AXI-Stream */
		Xil_Out32((BaseAddress + 0x0c), (1));
	}

	return Status;
}
#endif



/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */


static int instance = 0;
/* TX uses [1, 127] and RX uses [128, 254] */
/* The HDCP22 timer uses an additional offset of +64 */
#define TX_DEVICE_ID_BASE 1

static int xilinx_drm_hdmi_parse_of(struct xilinx_drm_hdmi *xhdmi, XV_HdmiTxSs_Config *config)
{
	struct device *dev = xhdmi->dev;
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

	rc = of_property_read_u32(node, "xlnx,hdmi-tx-offset", &val);
	if (rc == 0) {
		config->HdmiTx.DeviceId = TX_DEVICE_ID_BASE + instance;
		config->HdmiTx.IsPresent = 1;
		config->HdmiTx.AddrOffset = val;
		XV_HdmiTx_ConfigTable[instance].DeviceId = TX_DEVICE_ID_BASE + instance;
		XV_HdmiTx_ConfigTable[instance].BaseAddress = val;
	}

	rc = of_property_read_u32(node, "xlnx,vtc-offset", &val);
	if (rc < 0) {
		hdmi_dbg("Not using an internal VTC.");
		config->Vtc.IsPresent = 0;
	} else if (rc == 0) {
		config->Vtc.IsPresent = 1;
		config->Vtc.DeviceId = TX_DEVICE_ID_BASE + instance;
		config->Vtc.AddrOffset = val;
		XVtc_ConfigTable[instance].DeviceId = config->Vtc.DeviceId;
		XVtc_ConfigTable[instance].BaseAddress = val;
	}

#ifdef USE_HDCP /* WIP HDCP */

	rc = of_property_read_u32(node, "xlnx,hdcp1x-offset", &val);
	if (rc == 0) {
		/* make subcomponent of TXSS present */
		config->Hdcp14.IsPresent = 1;
		config->Hdcp14.DeviceId = TX_DEVICE_ID_BASE + instance;
		config->Hdcp14.AddrOffset = val;
		/* and configure it */
		XHdcp1x_ConfigTable[instance].DeviceId = config->Hdcp14.DeviceId;
		XHdcp1x_ConfigTable[instance].BaseAddress = val;
		//XHdcp1x_ConfigTable[0].SysFrequency = axi_clk_rate;
		XHdcp1x_ConfigTable[instance].IsRx = 0;
		XHdcp1x_ConfigTable[instance].IsHDMI = 1;
	}

	rc = of_property_read_u32(node, "xlnx,hdcp1x-timer-offset", &val);
	if (rc == 0) {
		/* make subcomponent of TXSS present */
		config->HdcpTimer.DeviceId = TX_DEVICE_ID_BASE + instance;
		config->HdcpTimer.IsPresent = 1;
		config->HdcpTimer.AddrOffset = val;

		dev_info(xhdmi->dev, "xlnx,hdcp1x-timer-offset = 0x%x\n", val);

		/* and configure it */
		XTmrCtr_ConfigTable[instance * 2 + 0].DeviceId = config->HdcpTimer.DeviceId;
		XTmrCtr_ConfigTable[instance * 2 + 0].BaseAddress = val;
		/* @TODO increment timer index */
	}

	rc = of_property_read_u32(node, "xlnx,hdcp22-tx-offset", &val);
	if (rc == 0) {
		dev_info(xhdmi->dev, "xlnx,hdcp22-tx-offset = 0x%x\n", val);
		config->Hdcp22.DeviceId = TX_DEVICE_ID_BASE + instance;
		config->Hdcp22.IsPresent = 1;
		config->Hdcp22.AddrOffset = val;
		XHdcp22_Tx_ConfigTable[instance].DeviceId = config->Hdcp22.DeviceId;
		XHdcp22_Tx_ConfigTable[instance].BaseAddress = val;
		XHdcp22_Tx_ConfigTable[instance].Protocol = 0;//HDCP22_TX_HDMI;
		XHdcp22_Tx_ConfigTable[instance].TimerDeviceId = TX_DEVICE_ID_BASE + 64 + instance;
		XHdcp22_Tx_ConfigTable[instance].CipherId = TX_DEVICE_ID_BASE + instance;
		XHdcp22_Tx_ConfigTable[instance].RngId = TX_DEVICE_ID_BASE + instance;;
	}

	rc = of_property_read_u32(node, "xlnx,hdcp22-timer-offset", &val);
	if (rc == 0) {
		dev_info(xhdmi->dev, "xlnx,hdcp22-timer-offset = 0x%x\n", val);
		XTmrCtr_ConfigTable[instance * 2 + 1].DeviceId = TX_DEVICE_ID_BASE + 64 + instance;
		XTmrCtr_ConfigTable[instance * 2 + 1].BaseAddress = val;
	}
	rc = of_property_read_u32(node, "xlnx,hdcp22-cipher-offset", &val);
	if (rc == 0) {
		dev_info(xhdmi->dev, "xlnx,hdcp22-cipher-offset = 0x%x\n", val);
		XHdcp22_Cipher_ConfigTable[instance].DeviceId = TX_DEVICE_ID_BASE + instance;
		XHdcp22_Cipher_ConfigTable[instance].BaseAddress = val;
	}
	rc = of_property_read_u32(node, "xlnx,hdcp22-rng-offset", &val);
	if (rc == 0) {
		dev_info(xhdmi->dev, "xlnx,hdcp22-rng-offset = 0x%x\n", val);
		XHdcp22_Rng_ConfigTable[instance].DeviceId = TX_DEVICE_ID_BASE + instance;
		XHdcp22_Rng_ConfigTable[instance].BaseAddress = val;
	}

	rc = of_property_read_u32(node, "xlnx,hdcp-authenticate", &val);
	if (rc == 0) {
		xhdmi->hdcp_authenticate = !!val;
	}
	rc = of_property_read_u32(node, "xlnx,hdcp-encrypt", &val);
	if (rc == 0) {
		xhdmi->hdcp_encrypt = !!val;
	}

#endif /* USE_HDCP */

	/* NOTE new */
	rc = of_property_read_string(node, "xlnx,pixel-format", &format);
	if (rc < 0) {
		dev_err(xhdmi->dev, "xlnx,pixel-format must be specified (\"yuv422\" or \"argb8888\")\n");
		goto error_dt;
	} else
	if (strcmp(format, "yuv422") == 0) {
		xhdmi->xvidc_colorfmt = XVIDC_CSF_YCRCB_422;
		hdmi_dbg("yuv422-> XVIDC_CSF_YCRCB_422\n");
	} else if (strcmp(format, "argb8888") == 0) {
		xhdmi->xvidc_colorfmt = XVIDC_CSF_RGB;
		hdmi_dbg("argb8888-> XVIDC_CSF_RGB\n");
	} else if (strcmp(format, "rgb8888") == 0) {
		xhdmi->xvidc_colorfmt = XVIDC_CSF_RGB;
		hdmi_dbg("rgb8888-> XVIDC_CSF_RGB\n");
#if 0 /* @TODO untested */
	} else if (strcmp(format, "yuv420") == 0) {
		xhdmi->xvidc_colorfmt = XVIDC_CSF_YCRCB_420;
#endif
	} else {
		dev_err(xhdmi->dev, "Unsupported xlnx,pixel-format\n");
		goto error_dt;
	}

	return 0;

error_dt:
	dev_err(xhdmi->dev, "Error parsing device tree");
	return rc;
}

static void hdcp_poll_work(struct work_struct *work)
{
	/* find our parent container structure */
	struct xilinx_drm_hdmi *xhdmi = container_of(work, struct xilinx_drm_hdmi,
		delayed_work_hdcp_poll.work);
	static int counter = 0;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmi);
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	//dev_info(xhdmi->dev, "hdcp_poll_work()\n");

#ifndef RUN_HDCP_IN_INTERRUPT_CONTEXT
	hdmi_mutex_lock(&xhdmi->hdmi_mutex);
#endif
	XV_HdmiTxSs_HdcpPoll(HdmiTxSsPtr);
#ifndef RUN_HDCP_IN_INTERRUPT_CONTEXT
	hdmi_mutex_unlock(&xhdmi->hdmi_mutex);
#endif

	counter++;
	if (counter >= 1000) {
		counter = 0;
		schedule_delayed_work(&xhdmi->delayed_work_hdcp_info, 0);
	}
	/* reschedule this work again in 1 millisecond */
	schedule_delayed_work(&xhdmi->delayed_work_hdcp_poll, msecs_to_jiffies(1));
	return;
}

static void hdcp_info_work(struct work_struct *work)
{
	/* find our parent container structure */
	struct xilinx_drm_hdmi *xhdmi = container_of(work, struct xilinx_drm_hdmi,
		delayed_work_hdcp_poll.work);
	static int counter = 0;
	XV_HdmiTxSs *HdmiTxSsPtr;
	BUG_ON(!xhdmi);
	HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
	BUG_ON(!HdmiTxSsPtr);

	XV_HdmiTxSs_HdcpInfo(HdmiTxSsPtr);
}


static int xilinx_drm_hdmi_probe(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *xhdmi;
	int ret;
	unsigned int index;
	struct resource *res;
	unsigned long axi_clk_rate;

	/* allocate zeroed HDMI TX device structure */
	xhdmi = devm_kzalloc(&pdev->dev, sizeof(*xhdmi), GFP_KERNEL);
	if (!xhdmi)
		return -ENOMEM;
	/* store pointer of the real device inside platform device */
	xhdmi->dev = &pdev->dev;

	/* mutex that protects against concurrent access */
	mutex_init(&xhdmi->hdmi_mutex);
	spin_lock_init(&xhdmi->irq_lock);
	/* work queues */
	xhdmi->work_queue = create_singlethread_workqueue("xilinx-hdmi-tx");
	if (!xhdmi->work_queue) {
		dev_err(xhdmi->dev, "Could not create work queue\n");
		return -ENOMEM;
	}

	hdmi_dbg("xilinx_drm_hdmi DT parse start\n");
	/* parse open firmware device tree data */
	ret = xilinx_drm_hdmi_parse_of(xhdmi, &config);
	hdmi_dbg("xilinx_drm_hdmi DT parse done\n");
	if (ret < 0)
		return ret;

	for (index = 0; index < 3; index++)
	{
		char phy_name[16];
		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);
		xhdmi->phy[index] = devm_phy_get(xhdmi->dev, phy_name);
		if (IS_ERR(xhdmi->phy[index])) {
			ret = PTR_ERR(xhdmi->phy[index]);
			xhdmi->phy[index] = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(xhdmi->dev, "failed to get phy lane %s index %d, error %d\n",
					phy_name, index, ret);
			goto error_phy;
		}

		ret = phy_init(xhdmi->phy[index]);
		if (ret) {
			dev_err(xhdmi->dev,
				"failed to init phy lane %d\n", index);
			goto error_phy;
		}
	}

	hdmi_dbg("config.Vtc.AddrOffset =  0x%x.", (int)config.Vtc.AddrOffset);
	hdmi_dbg("config->Ppc = %d\n", (int)config.Ppc);
	hdmi_dbg("config->MaxBitsPerPixel = %d\n", (int)config.MaxBitsPerPixel);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xhdmi->iomem = devm_ioremap_resource(xhdmi->dev, res);
	if (IS_ERR(xhdmi->iomem))
		return PTR_ERR(xhdmi->iomem);

	config.BaseAddress = (uintptr_t)xhdmi->iomem;
	config.HighAddress = config.BaseAddress + resource_size(res) - 1;

#ifdef USE_HDCP
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hdcp1x-keymngmt");

	if (res) {
		hdmi_dbg("Mapping HDCP1x key management block.\n");
		xhdmi->hdcp1x_keymngmt_iomem = devm_ioremap_resource(xhdmi->dev, res);
		hdmi_dbg("HDCP1x key management block @%p.\n", xhdmi->hdcp1x_keymngmt_iomem);
		if (IS_ERR(xhdmi->hdcp1x_keymngmt_iomem)) {
			hdmi_dbg("Could not ioremap hdcp1x-keymngmt.\n");
			return PTR_ERR(xhdmi->hdcp1x_keymngmt_iomem);
		}
	}

	if (config.Hdcp14.IsPresent && config.HdcpTimer.IsPresent && xhdmi->hdcp1x_keymngmt_iomem) {
		u8 Status;
		XV_HdmiTxSs *HdmiTxSsPtr;
		HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
		dev_info(xhdmi->dev, "HDCP1x components are all there.\n");
		/* Set pointer to HDCP 1.4 key */
		XV_HdmiTxSs_HdcpSetKey(HdmiTxSsPtr, XV_HDMITXSS_KEY_HDCP14, Hdcp14KeyA);
		/* Key manager Init */
		Status = XHdcp_KeyManagerInit((uintptr_t)xhdmi->hdcp1x_keymngmt_iomem, HdmiTxSsPtr->Hdcp14KeyPtr);
		if (Status != XST_SUCCESS) {
			dev_err(xhdmi->dev, "HDCP 1.4 TX Key Manager initialization error.\n");
			return -EINVAL;
		}
		dev_info(xhdmi->dev, "HDCP 1.4 TX Key Manager initializated OK.\n");
	}
	if (config.Hdcp22.IsPresent) {
		XV_HdmiTxSs *HdmiTxSsPtr;
		HdmiTxSsPtr = (XV_HdmiTxSs *)&xhdmi->xv_hdmitxss;
		/* Set pointer to HDCP 2.2 LC128 */
		XV_HdmiTxSs_HdcpSetKey(HdmiTxSsPtr, XV_HDMITXSS_KEY_HDCP22_LC128, Hdcp22Lc128);
		XV_HdmiTxSs_HdcpSetKey(HdmiTxSsPtr, XV_HDMITXSS_KEY_HDCP22_SRM, Hdcp22Srm);
	}
#endif

#if 0
	hdmi_dbg("config.BaseAddress =  %p.", config.BaseAddress);
	hdmi_dbg("config.HighAddress =  %p.", config.HighAddress);
#endif

	/* video streaming bus clock */
	xhdmi->clk = devm_clk_get(xhdmi->dev, "video");
	if (IS_ERR(xhdmi->clk)) {
		ret = PTR_ERR(xhdmi->clk);
		if (ret != -EPROBE_DEFER)
				dev_err(xhdmi->dev, "failed to get video clk\n");
		return ret;
	}

	clk_prepare_enable(xhdmi->clk);

	/* AXI lite register bus clock */
	xhdmi->axi_lite_clk = devm_clk_get(xhdmi->dev, "axi-lite");
	if (IS_ERR(xhdmi->axi_lite_clk)) {
		ret = PTR_ERR(xhdmi->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(xhdmi->dev, "failed to get axi-lite clk\n");
		return ret;
	}

	clk_prepare_enable(xhdmi->axi_lite_clk);
	axi_clk_rate = clk_get_rate(xhdmi->axi_lite_clk);

	/* @TODO we need to fill in the clock frequencies here, after asking the rate */
	XHdcp1x_ConfigTable[0].SysFrequency = axi_clk_rate;
	XTmrCtr_ConfigTable[0].SysClockFreqHz = axi_clk_rate;
	XTmrCtr_ConfigTable[1].SysClockFreqHz = axi_clk_rate;

	/* get irq */
	xhdmi->irq = platform_get_irq(pdev, 0);
	if (xhdmi->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		destroy_workqueue(xhdmi->work_queue);
		return xhdmi->irq;
	}

#ifdef USE_HDCP
	xhdmi->hdcp1x_irq = platform_get_irq_byname(pdev, "hdcp1x");
	dev_err(&pdev->dev, "xhdmi->hdcp1x_irq = %d\n", xhdmi->hdcp1x_irq);
	xhdmi->hdcp1x_timer_irq = platform_get_irq_byname(pdev, "hdcp1x-timer");
	dev_err(&pdev->dev, "xhdmi->hdcp1x_timer_irq = %d\n", xhdmi->hdcp1x_timer_irq);

	xhdmi->hdcp22_irq = platform_get_irq_byname(pdev, "hdcp22");
	dev_err(&pdev->dev, "xhdmi->hdcp1x_irq = %d\n", xhdmi->hdcp22_irq);
	xhdmi->hdcp22_timer_irq = platform_get_irq_byname(pdev, "hdcp22-timer");
	dev_err(&pdev->dev, "xhdmi->hdcp22_timer_irq = %d\n", xhdmi->hdcp22_timer_irq);

	INIT_DELAYED_WORK(&xhdmi->delayed_work_hdcp_poll, hdcp_poll_work/*function*/);
	INIT_DELAYED_WORK(&xhdmi->delayed_work_hdcp_info, hdcp_info_work/*function*/);

#endif

	/* support to drive an external retimer IC on the TX path, depending on TX clock line rate */
	xhdmi->retimer_clk = devm_clk_get(&pdev->dev, "retimer");
	if (IS_ERR(xhdmi->retimer_clk)) {
		ret = PTR_ERR(xhdmi->retimer_clk);
		xhdmi->retimer_clk = NULL;
		if (ret != -EPROBE_DEFER)
			hdmi_dbg("Did not find a retimer clk, not driving an external retimer device driver.\n");
	} else if (xhdmi->retimer_clk) {
		hdmi_dbg("got retimer clk\n");
		ret = clk_prepare_enable(xhdmi->retimer_clk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable retimer clk\n");
			return ret;
		}
		hdmi_dbg("prepared and enabled retimer clk\n");
	} else {
		hdmi_dbg("no retimer clk specified, assuming no redriver/retimer is used.\n");
	}

	hdmi_dbg("axi_clk_rate = %lu Hz\n", axi_clk_rate);

	config.AxiLiteClkFreq = axi_clk_rate;

	platform_set_drvdata(pdev, xhdmi);

	/* remainder of initialization is in encoder_init() */

	/* http://landley.net/kdocs/Documentation/driver-model/device.txt */

#if 0 /* TODO sysfs attribute to start HDCP authentication on request */
	if ( device_create_file(pdev->dev, &hdcp_authenticate1) != 0 )
	{
	}
#endif
	/* call into hdcp_poll_work, which will reschedule itself */
	hdcp_poll_work(&xhdmi->delayed_work_hdcp_poll.work);

	hdmi_dbg("xilinx_drm_hdmi_probe() succesfull.\n");

	return 0;
error_phy:
	return ret;
}

static int xilinx_drm_hdmi_remove(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *xhdmi = platform_get_drvdata(pdev);
	if (xhdmi->work_queue) destroy_workqueue(xhdmi->work_queue);
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
