/*
 * Xilinx Video HDMI RX Subsystem driver (Early Access Release)
 *
 * Copyright (C) 2016 Leon Woestenberg <leon@sidebranch.com>
 * Copyright (C) 2016 Xilinx, Inc.
 *
 * Authors: Leon Woestenberg <leon@sidebranch.com>
 *          Rohit Consul <rohitco@xilinx.com>
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
#include <linux/platform_device.h>
#include <linux/xilinx-v4l2-controls.h>

#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-dv-timings.h>

#include "xilinx-vip.h"

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "xilinx-hdmi-rx/xv_hdmirxss.h"
#include "xilinx-hdmi-rx/xil_printf.h"
#include "xilinx-hdmi-rx/xstatus.h"

#define HDMI_MAX_LANES	2

struct xhdmirx_device {
	struct xvip_device xvip;
	/* interrupt number */
	int irq;
	bool teardown;
	struct phy *phy[HDMI_MAX_LANES];

	/* mutex to prevent concurrent access to this structure */
	struct mutex xhdmirx_mutex;

	struct media_pad pad;

	/* https://linuxtv.org/downloads/v4l-dvb-apis/subdev.html#v4l2-mbus-framefmt */
	struct v4l2_mbus_framefmt detected_format;
	struct v4l2_mbus_framefmt default_format;
	const struct xvip_video_format *vip_format;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pattern;
	bool cable_connected;
	bool hdmi_stream;

	/* configuration for the baseline subsystem driver instance */
	XV_HdmiRxSs_Config config;
	/* bookkeeping for the baseline subsystem driver instance */
	XV_HdmiRxSs xv_hdmirxss;
	/* pointer to xvphy */
	XVphy *xvphy;
	/* sub core interrupt status registers */
	u32 IntrStatus[7];
};

// Xilinx EDID
static const u8 xilinx_edid[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x61, 0x98, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12,
	0x1F, 0x19, 0x01, 0x03, 0x80, 0x59, 0x32, 0x78, 0x0A, 0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26,
	0x0F, 0x50, 0x54, 0x21, 0x08, 0x00, 0x71, 0x4F, 0x81, 0xC0, 0x81, 0x00, 0x81, 0x80, 0x95, 0x00,
	0xA9, 0xC0, 0xB3, 0x00, 0x01, 0x01, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x58, 0x49, 0x4C,
	0x49, 0x4E, 0x58, 0x20, 0x48, 0x44, 0x4D, 0x49, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C,
	0x02, 0x03, 0x34, 0x71, 0x57, 0x61, 0x10, 0x1F, 0x04, 0x13, 0x05, 0x14, 0x20, 0x21, 0x22, 0x5D,
	0x5E, 0x5F, 0x60, 0x65, 0x66, 0x62, 0x63, 0x64, 0x07, 0x16, 0x03, 0x12, 0x23, 0x09, 0x07, 0x07,
	0x67, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x78, 0x3C, 0xE3, 0x0F, 0x01, 0xE0, 0x67, 0xD8, 0x5D, 0xC4,
	0x01, 0x78, 0x80, 0x07, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00,
	0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x08, 0xE8, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
	0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80,
	0xB0, 0x58, 0x8A, 0x00, 0x20, 0x52, 0x31, 0x00, 0x00, 0x1E, 0x66, 0x21, 0x56, 0xAA, 0x51, 0x00,
	0x1E, 0x30, 0x46, 0x8F, 0x33, 0x00, 0x50, 0x1D, 0x74, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x2E
};

static inline struct xhdmirx_device *to_xhdmirx(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct xhdmirx_device, xvip.subdev);
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Video Operations
 */

static int xhdmirx_s_stream(struct v4l2_subdev *subdev, int enable)
{
	printk(KERN_INFO "xhdmirx_s_stream enable = %d\n", enable);
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

 /* https://linuxtv.org/downloads/v4l-dvb-apis/vidioc-dv-timings-cap.html */

/* https://linuxtv.org/downloads/v4l-dvb-apis/vidioc-subdev-g-fmt.html */
static struct v4l2_mbus_framefmt *
__xhdmirx_get_pad_format_ptr(struct xhdmirx_device *xhdmirx,
		struct v4l2_subdev_pad_config *cfg,
		unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		printk(KERN_INFO "__xhdmirx_get_pad_format(): V4L2_SUBDEV_FORMAT_TRY\n");
		return v4l2_subdev_get_try_format(&xhdmirx->xvip.subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		printk(KERN_INFO "__xhdmirx_get_pad_format(): V4L2_SUBDEV_FORMAT_ACTIVE\n");
		printk(KERN_INFO "detected_format->width = %u\n", xhdmirx->detected_format.width);
		return &xhdmirx->detected_format;
	default:
		return NULL;
	}
}

static int xhdmirx_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct xhdmirx_device *xhdmirx = to_xhdmirx(subdev);
	printk(KERN_INFO "xhdmirx_get_format\n");

	if (fmt->pad > 0)
		return -EINVAL;

	/* copy either try or currently-active (i.e. detected) format to caller */
	fmt->format = *__xhdmirx_get_pad_format_ptr(xhdmirx, cfg, fmt->pad, fmt->which);

	printk(KERN_INFO "xhdmirx_get_format, height = %u\n", fmt->format.height);

	return 0;
}

/* we must modify the requested format to match what the hardware can provide */
static int xhdmirx_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct xhdmirx_device *xhdmirx = to_xhdmirx(subdev);
	printk(KERN_INFO "xhdmirx_set_format\n");
	if (fmt->pad > 0)
		return -EINVAL;
	/* there is nothing we can take from the format requested by the caller, we must return the active format */
	fmt->format = xhdmirx->detected_format;
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */

static int xhdmirx_enum_frame_size(struct v4l2_subdev *subdev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad > 0)
		return -EINVAL;
#if 0
	/* @TODO we want to provide all HDMI frame sizes, not that they are
	 * settable... */
	printk(KERN_INFO "xhdmirx_enum_frame_size\n");
#endif
	return 0;
}

#if 0
static const struct v4l2_dv_timings_cap xhdmirx_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 165000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

/* https://en.wikipedia.org/wiki/Extended_Display_Identification_Data#CEA-861-F */
static const struct v4l2_dv_timings xhdmirx_timings[] = {
	V4L2_DV_BT_CEA_1920X1080P6,0
};

static int xhdmirx_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&xhdmirx_timings_cap, NULL, NULL);
}
#endif

static int xhdmirx_dv_timings_cap(struct v4l2_subdev *subdev,
		struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;
#if 0
	*cap = xhdmirx_timings_cap;
#else
	cap->type = V4L2_DV_BT_656_1120;
	cap->bt.max_width = 1920;
	cap->bt.max_height = 1200;
	cap->bt.min_pixelclock = 25000000;
	cap->bt.max_pixelclock = 225000000;
	cap->bt.standards = V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			 V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT;
	cap->bt.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE |
		V4L2_DV_BT_CAP_REDUCED_BLANKING | V4L2_DV_BT_CAP_CUSTOM;
#endif
	return 0;
}

/* struct v4l2_subdev_internal_ops.open */
static int xhdmirx_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct xhdmirx_device *xhdmirx = to_xhdmirx(subdev);
	struct v4l2_mbus_framefmt *format;
	(void)xhdmirx;
	(void)format;
	printk(KERN_INFO "xhdmirx_open\n");

#if 0
	format = v4l2_subdev_get_try_format(subdev, fh->pad, 0);
	*format = xhdmirx->default_format;
#endif
	return 0;
}

/* struct v4l2_subdev_internal_ops.close */
static int xhdmirx_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	printk(KERN_INFO "xhdmirx_close\n");
	return 0;
}

static int xhdmirx_s_ctrl(struct v4l2_ctrl *ctrl)
{
	printk(KERN_INFO "xhdmirx_s_ctrl\n");
	return 0;
}

static const struct v4l2_ctrl_ops xhdmirx_ctrl_ops = {
	.s_ctrl	= xhdmirx_s_ctrl,
};

static struct v4l2_subdev_core_ops xhdmirx_core_ops = {
};

static struct v4l2_subdev_video_ops xhdmirx_video_ops = {
	.s_stream = xhdmirx_s_stream,
};

/* If the subdev driver intends to process video and integrate with the media framework,
 * it must implement format related functionality using v4l2_subdev_pad_ops instead of
 * v4l2_subdev_video_ops. */
static struct v4l2_subdev_pad_ops xhdmirx_pad_ops = {
	.enum_mbus_code		= xvip_enum_mbus_code,
	.enum_frame_size	= xhdmirx_enum_frame_size,
	.get_fmt			= xhdmirx_get_format,
	.set_fmt			= xhdmirx_set_format,

#if 0
	.enum_dv_timings	= xhdmirx_enum_dv_timings,
#endif
	.dv_timings_cap		= xhdmirx_dv_timings_cap,
};

static struct v4l2_subdev_ops xhdmirx_ops = {
	.core   = &xhdmirx_core_ops,
	.video  = &xhdmirx_video_ops,
	.pad    = &xhdmirx_pad_ops,
};

static const struct v4l2_subdev_internal_ops xhdmirx_internal_ops = {
	.open	= xhdmirx_open,
	.close	= xhdmirx_close,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations xhdmirx_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Power Management
 */

static int __maybe_unused xhdmirx_pm_suspend(struct device *dev)
{
	struct xhdmirx_device *xhdmirx = dev_get_drvdata(dev);
	xvip_suspend(&xhdmirx->xvip);
	return 0;
}

static int __maybe_unused xhdmirx_pm_resume(struct device *dev)
{
	struct xhdmirx_device *xhdmirx = dev_get_drvdata(dev);
	xvip_resume(&xhdmirx->xvip);
	return 0;
}

void HdmiRx_PioIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_TmrIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_VtdIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_DdcIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_AuxIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_AudIntrHandler(XV_HdmiRx *InstancePtr);
void HdmiRx_LinkStatusIntrHandler(XV_HdmiRx *InstancePtr);

void XV_HdmiRxSs_IntrEnable(XV_HdmiRxSs *HdmiRxSsPtr)
{
	XV_HdmiRx_PioIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_TmrIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_VtdIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_DdcIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_AuxIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_AudioIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_LinkIntrEnable(HdmiRxSsPtr->HdmiRxPtr);
}

void XV_HdmiRxSs_IntrDisable(XV_HdmiRxSs *HdmiRxSsPtr)
{
	XV_HdmiRx_PioIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_TmrIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_VtdIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_DdcIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_AuxIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_AudioIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
	XV_HdmiRx_LinkIntrDisable(HdmiRxSsPtr->HdmiRxPtr);
}

static irqreturn_t hdmirx_irq_handler(int irq, void *dev_id)
{
	struct xhdmirx_device *xhdmirx;
	XV_HdmiRxSs *HdmiRxSsPtr;
	BUG_ON(!dev_id);
	xhdmirx = (struct xhdmirx_device *)dev_id;
	if (!xhdmirx)
		return IRQ_NONE;

	HdmiRxSsPtr = (XV_HdmiRxSs *)&xhdmirx->xv_hdmirxss;
	BUG_ON(!HdmiRxSsPtr->HdmiRxPtr);

	if (HdmiRxSsPtr->IsReady != XIL_COMPONENT_IS_READY) {
		printk(KERN_INFO "hdmirx_irq_handler(): HDMI RX SS is not initialized?!\n");
	}

#if 1
	/* read status registers */
	xhdmirx->IntrStatus[0] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_PIO_STA_OFFSET)) & (XV_HDMIRX_PIO_STA_IRQ_MASK);
	xhdmirx->IntrStatus[1] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_TMR_STA_OFFSET)) & (XV_HDMIRX_TMR_STA_IRQ_MASK);
	xhdmirx->IntrStatus[2] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_VTD_STA_OFFSET)) & (XV_HDMIRX_VTD_STA_IRQ_MASK);
	xhdmirx->IntrStatus[3] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_DDC_STA_OFFSET)) & (XV_HDMIRX_DDC_STA_IRQ_MASK);
	xhdmirx->IntrStatus[4] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_AUX_STA_OFFSET)) & (XV_HDMIRX_AUX_STA_IRQ_MASK);
	xhdmirx->IntrStatus[5] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_AUD_STA_OFFSET)) & (XV_HDMIRX_AUD_STA_IRQ_MASK);
	xhdmirx->IntrStatus[6] = XV_HdmiRx_ReadReg(HdmiRxSsPtr->Config.BaseAddress, (XV_HDMIRX_LNKSTA_STA_OFFSET)) & (XV_HDMIRX_LNKSTA_STA_IRQ_MASK);
#endif

	/* mask interrupt request */
	XV_HdmiRxSs_IntrDisable(HdmiRxSsPtr);

	/* call bottom-half */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hdmirx_irq_thread(int irq, void *dev_id)
{
	static int irq_count = 0;
	struct xhdmirx_device *xhdmirx;
	XV_HdmiRxSs *HdmiRxSsPtr;

	BUG_ON(!dev_id);
	xhdmirx = (struct xhdmirx_device *)dev_id;
	if (!xhdmirx || xhdmirx->teardown)
		return IRQ_NONE;
	HdmiRxSsPtr = (XV_HdmiRxSs *)&xhdmirx->xv_hdmirxss;
	BUG_ON(!HdmiRxSsPtr->HdmiRxPtr);

	mutex_lock(&xhdmirx->xhdmirx_mutex);
	/* call baremetal interrupt handler, this in turn will
	 * call the registed callbacks functions */

#if 1
	if (xhdmirx->IntrStatus[0]) HdmiRx_PioIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[1]) HdmiRx_TmrIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[2]) HdmiRx_VtdIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[3]) HdmiRx_DdcIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[4]) HdmiRx_AuxIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[5]) HdmiRx_AudIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	if (xhdmirx->IntrStatus[6]) HdmiRx_LinkStatusIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
#else
	HdmiRx_PioIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_TmrIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_VtdIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_DdcIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_AuxIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_AudIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
	HdmiRx_LinkStatusIntrHandler(HdmiRxSsPtr->HdmiRxPtr);
#endif
	 XV_HdmiRxSs_IntrEnable(HdmiRxSsPtr);

	mutex_unlock(&xhdmirx->xhdmirx_mutex);
	return IRQ_HANDLED;
}

/* callbacks from HDMI RX SS interrupt handler */
static void RxConnectCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	XVphy *VphyPtr = xhdmirx->xvphy;
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr || !VphyPtr) return;

	xhdmirx->cable_connected = !!HdmiRxSsPtr->IsStreamConnected;
	//dev_info(xhdmirx->xvip.dev, "RxConnectCallback()\n");
	dev_info(xhdmirx->xvip.dev, "cable is %sconnected.\n", xhdmirx->cable_connected? "": "dis");

	xvphy_mutex_lock(xhdmirx->phy[0]);
	// RX cable is disconnected
	if (HdmiRxSsPtr->IsStreamConnected == (FALSE))
	{
		VphyPtr->HdmiRxTmdsClockRatio = 0; // Clear GT RX TMDS clock ratio
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_RX, (FALSE));
	}
	else
	{
		XVphy_IBufDsEnable(VphyPtr, 0, XVPHY_DIR_RX, (TRUE));
	}
	xvphy_mutex_unlock(xhdmirx->phy[0]);
}

static void RxAuxCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	u8 AuxBuffer[36];
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr) return;
	// Copy the RX packet into the local buffer
	memcpy(AuxBuffer, XV_HdmiRxSs_GetAuxiliary(HdmiRxSsPtr), sizeof(AuxBuffer));
}

static void RxAudCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;\
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr) return;
	//printk(KERN_INFO "RxAudCallback()\n");
	(void)HdmiRxSsPtr;
}

static void RxLnkStaCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr) return;
	(void)HdmiRxSsPtr;
}

static void RxStreamDownCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr) return;
	(void)HdmiRxSsPtr;
	//printk(KERN_INFO "RxStreamDownCallback()\n");
	dev_info(xhdmirx->xvip.dev, "stream is down.\n");
	xhdmirx->hdmi_stream = 0;
}

static void RxStreamInitCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	XVphy *VphyPtr = xhdmirx->xvphy;
	XVidC_VideoStream *HdmiRxSsVidStreamPtr;
	u32 Status;
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr || !VphyPtr) return;
	//xil_printf("RxStreamInitCallback\r\n");
	// Calculate RX MMCM parameters
	// In the application the YUV422 colordepth is 12 bits
	// However the HDMI transports YUV422 in 8 bits.
	// Therefore force the colordepth to 8 bits when the colorspace is YUV422

	HdmiRxSsVidStreamPtr = XV_HdmiRxSs_GetVideoStream(HdmiRxSsPtr);

	xvphy_mutex_lock(xhdmirx->phy[0]);

	if (HdmiRxSsVidStreamPtr->ColorFormatId == XVIDC_CSF_YCRCB_422) {
		Status = XVphy_HdmiCfgCalcMmcmParam(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1,
				XVPHY_DIR_RX,
				HdmiRxSsVidStreamPtr->PixPerClk,
				XVIDC_BPC_8);
	// Other colorspaces
	} else {
		Status = XVphy_HdmiCfgCalcMmcmParam(VphyPtr, 0, XVPHY_CHANNEL_ID_CH1,
				XVPHY_DIR_RX,
				HdmiRxSsVidStreamPtr->PixPerClk,
				HdmiRxSsVidStreamPtr->ColorDepth);
	}

	if (Status == XST_FAILURE) {
		xvphy_mutex_unlock(xhdmirx->phy[0]);
		return;
	}

	// Enable and configure RX MMCM
	XVphy_MmcmStart(VphyPtr, 0, XVPHY_DIR_RX);
	xvphy_mutex_unlock(xhdmirx->phy[0]);
}

static void RxStreamUpCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XV_HdmiRxSs *HdmiRxSsPtr = &xhdmirx->xv_hdmirxss;
	XVidC_VideoStream *Stream;
	BUG_ON(!xhdmirx);
	BUG_ON(!HdmiRxSsPtr);
	if (!xhdmirx || !HdmiRxSsPtr) return;
	if (!HdmiRxSsPtr) return;
	dev_info(xhdmirx->xvip.dev, "stream is up.\n");
	mutex_lock(&xhdmirx->xhdmirx_mutex);
	Stream = &HdmiRxSsPtr->HdmiRxPtr->Stream.Video;
	XVidC_ReportStreamInfo(Stream);
	xhdmirx->detected_format.width = Stream->Timing.HActive;
	xhdmirx->detected_format.height = Stream->Timing.VActive;

	/* @TODO correct mapping, see https://linuxtv.org/downloads/v4l-dvb-apis/field-order.html#v4l2-field */
	xhdmirx->detected_format.field = Stream->IsInterlaced? V4L2_FIELD_INTERLACED: V4L2_FIELD_NONE;
	/* https://linuxtv.org/downloads/v4l-dvb-apis/ch02s05.html#v4l2-colorspace */
	xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_REC709;
	if (Stream->ColorFormatId == XVIDC_CSF_RGB) {
		/* red blue green */
		xhdmirx->detected_format.code = MEDIA_BUS_FMT_RBG888_1X24;
		xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_SRGB;
	} else if (Stream->ColorFormatId == XVIDC_CSF_YCRCB_444) {
		xhdmirx->detected_format.code = MEDIA_BUS_FMT_VUY8_1X24;
		xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_REC709;
	} else if (Stream->ColorFormatId == XVIDC_CSF_YCRCB_422) {
		xhdmirx->detected_format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_REC709;
	} else if (Stream->ColorFormatId == XVIDC_CSF_YCRCB_420) {
		xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_REC709;
	}

	xhdmirx->detected_format.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	xhdmirx->detected_format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	xhdmirx->detected_format.quantization = V4L2_QUANTIZATION_DEFAULT;

	mutex_unlock(&xhdmirx->xhdmirx_mutex);
}

/* Called from non-interrupt context with xvphy mutex locked
 */
static void VphyHdmiRxInitCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XVphy *VphyPtr = xhdmirx->xvphy;
	BUG_ON(!xhdmirx);
	BUG_ON(!VphyPtr);
	BUG_ON(!xhdmirx->phy[0]);
	if (!xhdmirx || !VphyPtr) return;
	printk(KERN_INFO "VphyHdmiRxInitCallback()\n");

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(xhdmirx->phy[0]);
	mutex_lock(&xhdmirx->xhdmirx_mutex);
	xvphy_mutex_lock(xhdmirx->phy[0]);

	XV_HdmiRxSs_RefClockChangeInit(&xhdmirx->xv_hdmirxss);
	/* @NOTE maybe implement xvphy_set_hdmirx_tmds_clockratio(); */
	VphyPtr->HdmiRxTmdsClockRatio = xhdmirx->xv_hdmirxss.TMDSClockRatio;
	/* unlock RX SS but keep XVPHY locked */
	mutex_unlock(&xhdmirx->xhdmirx_mutex);
}

/* Called from non-interrupt context with xvphy mutex locked
 */
static void VphyHdmiRxReadyCallback(void *CallbackRef)
{
	struct xhdmirx_device *xhdmirx = (struct xhdmirx_device *)CallbackRef;
	XVphy *VphyPtr = xhdmirx->xvphy;
	XVphy_PllType RxPllType;
	BUG_ON(!xhdmirx);
	BUG_ON(!VphyPtr);
	BUG_ON(!xhdmirx->phy[0]);
	if (!xhdmirx || !VphyPtr) return;
	printk(KERN_INFO "VphyHdmiRxReadyCallback()\n");

#if 0
	// Enable pass-through
#if(LOOPBACK_MODE_EN != 1)
	IsPassThrough = (TRUE);
#endif
#endif

	/* a pair of mutexes must be locked in fixed order to prevent deadlock,
	 * and the order is RX SS then XVPHY, so first unlock XVPHY then lock both */
	xvphy_mutex_unlock(xhdmirx->phy[0]);
	mutex_lock(&xhdmirx->xhdmirx_mutex);
	xvphy_mutex_lock(xhdmirx->phy[0]);
	/* @NOTE too much peeking around in Vphy */
	RxPllType = XVphy_GetPllType(xhdmirx->xvphy, 0, XVPHY_DIR_RX,
		XVPHY_CHANNEL_ID_CH1);
	if (!(RxPllType == XVPHY_PLL_TYPE_CPLL)) {
		XV_HdmiRxSs_SetStream(&xhdmirx->xv_hdmirxss, VphyPtr->HdmiRxRefClkHz,
				(VphyPtr->Quads[0].Plls[XVPHY_CHANNEL_ID_CMN0 -
				XVPHY_CHANNEL_ID_CH1].LineRateHz / 1000000));
	}
	else {
		XV_HdmiRxSs_SetStream(&xhdmirx->xv_hdmirxss, VphyPtr->HdmiRxRefClkHz,
				(VphyPtr->Quads[0].Plls[0].LineRateHz / 1000000));
	}
	mutex_unlock(&xhdmirx->xhdmirx_mutex);
}

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_PRESENT         1

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_PRESENT 0
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_DEVICE_ID 255
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_BASEADDR 0

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_PRESENT 0
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_DEVICE_ID 255
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_BASEADDR 0

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_PRESENT 0
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_DEVICE_ID 255
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_BASEADDR 0

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_PRESENT 0
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_DEVICE_ID 255
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_BASEADDR 0

#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_PRESENT 0
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_DEVICE_ID 255
#define XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_BASEADDR 0

static XV_HdmiRxSs_Config config = {
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_DEVICE_ID,
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_BASEADDR,
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HIGHADDR,
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_INPUT_PIXELS_PER_CLOCK,
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_MAX_BITS_PER_COMPONENT,

	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_GPIO_0_BASEADDR
	},
	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_AXI_TIMER_0_BASEADDR
	},
	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP_0_BASEADDR
	},
	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_HDCP22_RX_0_BASEADDR
	},
	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_AXI4S_REMAP_0_BASEADDR
	},
	{
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_PRESENT,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_DEVICE_ID,
		XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_BASEADDR
	}
};

/* @TODO these must be moved into the device specific structure, then
 * initialized from device-tree so that multiple instances are possible */
static XGpio_Config XGpio_FixedConfig = {
#ifdef XPAR_XGPIO_NUM_INSTANCES
	 XPAR_GPIO_0_DEVICE_ID,
	 XPAR_GPIO_0_BASEADDR,
	 XPAR_GPIO_0_INTERRUPT_PRESENT,
	 XPAR_GPIO_0_IS_DUAL
#endif
};
XGpio_Config *XGpio_LookupConfig(u16 DeviceId) {
	return (XGpio_Config *)&XGpio_FixedConfig;
}
XV_axi4s_remap_Config* XV_axi4s_remap_LookupConfig(u16 DeviceId) {
	BUG_ON(1);
	return NULL;
}

static XV_HdmiRx_Config XV_HdmiRx_FixedConfig =
{
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_DEVICE_ID,
	XPAR_HDMI_INPUT_V_HDMI_RX_SS_0_V_HDMI_RX_BASEADDR
};
XV_HdmiRx_Config *XV_HdmiRx_LookupConfig(u16 DeviceId)
{
	return (XV_HdmiRx_Config *)&XV_HdmiRx_FixedConfig;
}

static void hdmirx_config_init(XV_HdmiRxSs_Config *config, void __iomem *iomem)
{
	config->BaseAddress = (uintptr_t)iomem;
	config->HighAddress = (uintptr_t)iomem + 0xFFFF;
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int xhdmirx_parse_of(struct xhdmirx_device *xhdmirx, XV_HdmiRxSs_Config *config)
{
	struct device *dev = xhdmirx->xvip.dev;
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

static int xhdmirx_probe(struct platform_device *pdev)
{
	struct v4l2_subdev *subdev;
	struct xhdmirx_device *xhdmirx;
	int ret;
	unsigned int index;

	XV_HdmiRxSs *HdmiRxSsPtr;
	u32 Status;

	/* allocate zeroed HDMI RX device structure */
	xhdmirx = devm_kzalloc(&pdev->dev, sizeof(*xhdmirx), GFP_KERNEL);
	if (!xhdmirx)
		return -ENOMEM;
	/* store pointer of the real device inside platform device */
	xhdmirx->xvip.dev = &pdev->dev;

	/* mutex that protects against concurrent access */
	mutex_init(&xhdmirx->xhdmirx_mutex);

	/* parse open firmware device tree data */
	ret = xhdmirx_parse_of(xhdmirx, &config);
	if (ret < 0)
		return ret;

	ret = xvip_init_resources(&xhdmirx->xvip);
	if (ret < 0)
		return ret;

	/* get irq */
	xhdmirx->irq = platform_get_irq(pdev, 0);
	if (xhdmirx->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		return xhdmirx->irq;
	}

	/* @TODO compiler issue, or am I just staring blind? if change to "index < 3", the case for index == 3 is also run.
	 * -- Leon Woestenberg <leon@sidebranch.com>
	 */
	for (index = 0; index < 2; index++) {
		printk(KERN_INFO "entering loop with index = %d\n",index);
		char phy_name[32];

		snprintf(phy_name, sizeof(phy_name), "hdmi-phy%d", index);
		dev_info(xhdmirx->xvip.dev, "%s\n", phy_name);
		xhdmirx->phy[index] = devm_phy_get(xhdmirx->xvip.dev, phy_name);
		if (IS_ERR(xhdmirx->phy[index])) {
			ret = PTR_ERR(xhdmirx->phy[index]);
			dev_err(xhdmirx->xvip.dev, "failed to get phy lane %s, error %d\n", phy_name, ret);
			goto error_phy;
		}

		ret = phy_init(xhdmirx->phy[index]);
		if (ret) {
			dev_err(xhdmirx->xvip.dev,
				"failed to init phy lane %d\n", index);
			goto error_phy;
		}
		printk(KERN_INFO "exiting loop with index = %d\n",index);
	}

	HdmiRxSsPtr = (XV_HdmiRxSs *)&xhdmirx->xv_hdmirxss;

	mutex_lock(&xhdmirx->xhdmirx_mutex);

	/* initialize the source configuration structure */
	hdmirx_config_init(&config, xhdmirx->xvip.iomem);

	XV_HdmiRxSs_SetEdidParam(HdmiRxSsPtr, (u8 *)&xilinx_edid, sizeof(xilinx_edid));

	// Initialize top level and all included sub-cores
	Status = XV_HdmiRxSs_CfgInitialize(HdmiRxSsPtr, &config,
		(uintptr_t)xhdmirx->xvip.iomem);
	if (Status != XST_SUCCESS)
	{
		dev_err(xhdmirx->xvip.dev, "initialization failed with error %d\r\n", Status);
		return -EINVAL;
	}
	XV_HdmiRxSs_IntrDisable(HdmiRxSsPtr);

	XV_HdmiRxSs_ReportSubcoreVersion(&xhdmirx->xv_hdmirxss);

	/* RX SS callback setup (from xapp1287/xhdmi_example.c:2146) */
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss, XV_HDMIRXSS_HANDLER_CONNECT,
		RxConnectCallback, (void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss,XV_HDMIRXSS_HANDLER_AUX,
		RxAuxCallback,(void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss,XV_HDMIRXSS_HANDLER_AUD,
		RxAudCallback, (void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss, XV_HDMIRXSS_HANDLER_LNKSTA,
		RxLnkStaCallback, (void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss, XV_HDMIRXSS_HANDLER_STREAM_DOWN,
		RxStreamDownCallback, (void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss, XV_HDMIRXSS_HANDLER_STREAM_INIT,
		RxStreamInitCallback, (void *)xhdmirx);
	XV_HdmiRxSs_SetCallback(&xhdmirx->xv_hdmirxss, XV_HDMIRXSS_HANDLER_STREAM_UP,
		RxStreamUpCallback, (void *)xhdmirx);

	/* get a reference to the XVphy data structure */
	xhdmirx->xvphy = xvphy_get_xvphy(xhdmirx->phy[0]);

	BUG_ON(!xhdmirx->xvphy);

	xvphy_mutex_lock(xhdmirx->phy[0]);
	/* the callback is not specific to a single lane, but we need to
	 * provide one of the phy's as reference */
	XVphy_SetHdmiCallback(xhdmirx->xvphy, XVPHY_HDMI_HANDLER_RXINIT,
		VphyHdmiRxInitCallback, (void *)xhdmirx);

	XVphy_SetHdmiCallback(xhdmirx->xvphy, XVPHY_HDMI_HANDLER_RXREADY,
		VphyHdmiRxReadyCallback, (void *)xhdmirx);
	xvphy_mutex_unlock(xhdmirx->phy[0]);

	platform_set_drvdata(pdev, xhdmirx);

	ret = devm_request_threaded_irq(&pdev->dev, xhdmirx->irq, hdmirx_irq_handler, hdmirx_irq_thread,
		IRQF_TRIGGER_HIGH, "xilinx-hdmi-rx", xhdmirx/*dev_id*/);

	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", xhdmirx->irq);
		mutex_unlock(&xhdmirx->xhdmirx_mutex);
		goto error_phy;
	}

	/* Initialize V4L2 subdevice */
	subdev = &xhdmirx->xvip.subdev;
	v4l2_subdev_init(subdev, &xhdmirx_ops);
	subdev->dev = &pdev->dev;
	subdev->internal_ops = &xhdmirx_internal_ops;
	strlcpy(subdev->name, dev_name(&pdev->dev), sizeof(subdev->name));
	v4l2_set_subdevdata(subdev, xhdmirx);
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE /* | V4L2_SUBDEV_FL_HAS_EVENTS*/;

	/* Initialize V4L2 media entity */
	xhdmirx->pad.flags = MEDIA_PAD_FL_SOURCE;
	subdev->entity.ops = &xhdmirx_media_ops;
	ret = media_entity_init(&subdev->entity, 1/*npads*/, &xhdmirx->pad, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to init media entity\n");
		mutex_unlock(&xhdmirx->xhdmirx_mutex);
		goto error_irq;
	}

	v4l2_ctrl_handler_init(&xhdmirx->ctrl_handler, 0/*controls*/);
	subdev->ctrl_handler = &xhdmirx->ctrl_handler;
	ret = v4l2_ctrl_handler_setup(&xhdmirx->ctrl_handler);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set controls\n");
		mutex_unlock(&xhdmirx->xhdmirx_mutex);
		goto error_irq;
	}

	xvip_print_version(&xhdmirx->xvip);

	/* assume detected format */
	xhdmirx->detected_format.width = 1280;
	xhdmirx->detected_format.height = 720;
	xhdmirx->detected_format.field = V4L2_FIELD_NONE;
	xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_REC709;
	xhdmirx->detected_format.code = MEDIA_BUS_FMT_RBG888_1X24;
	xhdmirx->detected_format.colorspace = V4L2_COLORSPACE_SRGB;
	xhdmirx->detected_format.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	xhdmirx->detected_format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	xhdmirx->detected_format.quantization = V4L2_QUANTIZATION_DEFAULT;

	ret = v4l2_async_register_subdev(subdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register subdev\n");
		mutex_unlock(&xhdmirx->xhdmirx_mutex);
		goto error;
	}

	XV_HdmiRxSs_IntrEnable(HdmiRxSsPtr);
	mutex_unlock(&xhdmirx->xhdmirx_mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(&xhdmirx->ctrl_handler);
	media_entity_cleanup(&subdev->entity);
error_irq:

error_phy:
	printk(KERN_INFO "xhdmirx_probe() error_phy:\n");
	for (index = 0; index < 2; index++) {
		if (xhdmirx->phy[index]) {
			printk(KERN_INFO "phy_exit() xhdmirx->phy[%d] = %p\n", index, xhdmirx->phy[index]);
			phy_exit(xhdmirx->phy[index]);
			xhdmirx->phy[index] = NULL;
		}
	}
error_resource:
	printk(KERN_INFO "xhdmirx_probe() error_resource:\n");
	xvip_cleanup_resources(&xhdmirx->xvip);
	return ret;
}

static int xhdmirx_remove(struct platform_device *pdev)
{
	struct xhdmirx_device *xhdmirx = platform_get_drvdata(pdev);
	struct v4l2_subdev *subdev = &xhdmirx->xvip.subdev;

#if 0 // @TODO mutex can not be acquired
	mutex_lock(&xhdmirx->xhdmirx_mutex);
#endif

	xhdmirx->teardown = 1;
#if 0
	mutex_unlock(&xhdmirx->xhdmirx_mutex);
#endif

	v4l2_async_unregister_subdev(subdev);
	v4l2_ctrl_handler_free(&xhdmirx->ctrl_handler);
	media_entity_cleanup(&subdev->entity);

	xvip_cleanup_resources(&xhdmirx->xvip);
	dev_info(&pdev->dev, "removed.\n");
	return 0;
}

static SIMPLE_DEV_PM_OPS(xhdmirx_pm_ops, xhdmirx_pm_suspend, xhdmirx_pm_resume);

static const struct of_device_id xhdmirx_of_id_table[] = {
	{ .compatible = "xlnx,v-hdmirxss-2.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, xhdmirx_of_id_table);

static struct platform_driver xhdmirx_driver = {
	.driver = {
		.name		= "xilinx-hdmi-rx",
		.pm		= &xhdmirx_pm_ops,
		.of_match_table	= xhdmirx_of_id_table,
	},
	.probe			= xhdmirx_probe,
	.remove			= xhdmirx_remove,
};

module_platform_driver(xhdmirx_driver);

MODULE_DESCRIPTION("Xilinx HDMI RXSS V4L2 driver");
MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_LICENSE("GPL v2");
