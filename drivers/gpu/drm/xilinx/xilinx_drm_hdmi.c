/*
 * Xilinx DRM DisplayPort encoder driver for Xilinx
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

#include "xilinx_drm_dp_sub.h"
#include "xilinx_drm_drv.h"

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
	struct drm_encoder *encoder;
	struct device *dev;
	void __iomem *iomem;

	struct xilinx_drm_hdmi_config config;
	struct drm_dp_aux aux;
	struct xilinx_drm_dp_sub *dp_sub;
	struct phy *phy[DP_MAX_LANES];
	struct clk *aclk;
	struct clk *aud_clk;

	int dpms;
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	struct xilinx_drm_dp_link_config link_config;
	struct xilinx_drm_dp_mode mode;
	u8 train_set[DP_MAX_LANES];
};

static inline struct xilinx_drm_hdmi *to_hdmi(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

/* drm_encoder_slave_funcs */

static void xilinx_drm_hdmi_dpms(struct drm_encoder *encoder, int dpms)
{
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);
	void __iomem *iomem = dp->iomem;
	unsigned int i;
	int ret;

	if (dp->dpms == dpms)
		return;

	dp->dpms = dpms;

	switch (dpms) {
	case DRM_MODE_DPMS_ON:
		/* power-up */
		return;
	default:
		/* power-down */
		return;
	}
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
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);
	int diff = mode->htotal - mode->hsync_end;
#if 0
	/*
	 * ZynqMP DP requires horizontal backporch to be greater than 12.
	 * This limitation may conflict with the sink device.
	 */
	if (dp->dp_sub && diff < XILINX_DP_SUB_TX_MIN_H_BACKPORCH) {
		int vrefresh = (adjusted_mode->clock * 1000) /
			       (adjusted_mode->vtotal * adjusted_mode->htotal);

		diff = XILINX_DP_SUB_TX_MIN_H_BACKPORCH - diff;
		adjusted_mode->htotal += diff;
		adjusted_mode->clock = adjusted_mode->vtotal *
				       adjusted_mode->htotal * vrefresh / 1000;
	}
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
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);
	u8 max_lanes = dp->link_config.max_lanes;
	u8 bpp = dp->config.bpp;
	u32 max_pclock = dp->config.max_pclock;
	int max_rate = dp->link_config.max_rate;
	int rate;

	/* a maximum pixel clock is set and is exceeded by the mode? */
	if (max_pclock && mode->clock > max_pclock)
		return MODE_CLOCK_HIGH;

	/* rate exceeds what is possible with the current setup? */
	rate = xilinx_drm_hdmi_max_rate(max_rate, max_lanes, bpp);
	if (mode->clock > rate)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void xilinx_drm_hdmi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);
#if 0
	xilinx_drm_hdmi_mode_configure(dp, adjusted_mode->clock);
	xilinx_drm_hdmi_mode_set_stream(dp, adjusted_mode);
	xilinx_drm_hdmi_mode_set_transfer_unit(dp, adjusted_mode);
#endif
}

static enum drm_connector_status
xilinx_drm_hdmi_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);

	if (1) {
		return connector_status_connected;
	}

	return connector_status_disconnected;
}

/* xilinx_drm_hdmi_get_edid_block */
static int xilinx_drm_hdmi_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len)
{
	//memcpy(buf, adv7511->edid_buf, len);
	return 0;
}

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int xilinx_drm_hdmi_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
	struct xilinx_drm_hdmi *dp = to_hdmi(encoder);
	struct edid *edid;
	int ret;

	/* When the I2C adapter connected to the DDC bus is hidden behind a device that
	* exposes a different interface to read EDID blocks this function can be used
	* to get EDID data using a custom block read function. - from drm_edid.c
	*/

	/* private data dp is passed to xilinx_drm_hdmi_get_edid_block(data, ...) */
	edid = drm_do_get_edid(connector, xilinx_drm_hdmi_get_edid_block, dp);

	drm_mode_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);

	kfree(edid);

	return ret;
}

static struct drm_encoder_slave_funcs xilinx_drm_hdmi_encoder_funcs = {
	.dpms			= xilinx_drm_hdmi_hdmims,
	.save			= xilinx_drm_hdmi_save,
	.restore		= xilinx_drm_hdmi_restore,
	.mode_fixup		= xilinx_drm_hdmi_mode_fixup,
	.mode_valid		= xilinx_drm_hdmi_mode_valid,
	.mode_set		= xilinx_drm_hdmi_mode_set,
	.detect			= xilinx_drm_hdmi_detect,
	.get_modes		= xilinx_drm_hdmi_get_modes,
};

static int xilinx_drm_hdmi_encoder_init(struct platform_device *pdev,
				      struct drm_device *dev,
				      struct drm_encoder_slave *encoder)
{
	struct xilinx_drm_hdmi *dp = platform_get_drvdata(pdev);
	int clock_rate;
	u32 reg, w;

	encoder->slave_priv = dp;
	encoder->slave_funcs = &xilinx_drm_hdmi_encoder_funcs;

	dp->encoder = &encoder->base;

	/* Get aclk rate */
	clock_rate = clk_get_rate(dp->aclk);
	if (clock_rate < XILINX_DP_TX_CLK_DIVIDER_MHZ) {
		DRM_ERROR("aclk should be higher than 1MHz\n");
		return -EINVAL;
	}
	/* enable TX here */
	return 0;
}

static irqreturn_t xilinx_drm_hdmi_irq_handler(int irq, void *data)
{
	struct xilinx_drm_hdmi *dp = (struct xilinx_drm_hdmi *)data;
	u32 reg, status;

#if 0
	xilinx_drm_writel(dp->iomem, reg, status);

	if (status & XILINX_DP_TX_INTR_VBLANK_START)
		xilinx_drm_dp_sub_handle_vblank(dp->dp_sub);

	if (status & XILINX_DP_TX_INTR_HPD_EVENT)
		drm_helper_hpd_irq_event(dp->encoder->dev);

	if (status & XILINX_DP_TX_INTR_HPD_IRQ) {
		u8 status[DP_LINK_STATUS_SIZE + 2];

		drm_dp_dpcd_read(&dp->aux, DP_SINK_COUNT, status,
				 DP_LINK_STATUS_SIZE + 2);

		if (status[4] & DP_LINK_STATUS_UPDATED ||
		    !drm_dp_clock_recovery_ok(&status[2], dp->mode.lane_cnt) ||
		    !drm_dp_channel_eq_ok(&status[2], dp->mode.lane_cnt))
			xilinx_drm_dp_train(dp);
	}
#endif
	return IRQ_HANDLED;
}

static ssize_t
xilinx_drm_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct xilinx_drm_dp *dp = container_of(aux, struct xilinx_drm_dp, aux);
	int ret;

	ret = xilinx_drm_dp_aux_cmd_submit(dp, msg->request, msg->address,
					   msg->buffer, msg->size, &msg->reply);
	if (ret < 0)
		return ret;

	return msg->size;
}

static int xilinx_drm_hdmi_parse_of(struct xilinx_drm_hdmi *dp)
{
	struct device_node *node = dp->dev->of_node;
	return 0;
}

static int xilinx_drm_hdmi_probe(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *dp;
	struct resource *res;
	u32 version, i;
	int irq, ret;

	dp = devm_kzalloc(&pdev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dpms = DRM_MODE_DPMS_OFF;
	dp->dev = &pdev->dev;

	ret = xilinx_drm_hdmi_parse_of(dp);
	if (ret < 0)
		return ret;

	dp->aclk = devm_clk_get(dp->dev, "aclk");
	if (IS_ERR(dp->aclk))
		return PTR_ERR(dp->aclk);

	ret = clk_prepare_enable(dp->aclk);
	if (ret) {
		dev_err(dp->dev, "failed to enable the aclk\n");
		return ret;
	}

	dp->aud_clk = devm_clk_get(dp->dev, "aud_clk");
	if (IS_ERR(dp->aud_clk)) {
		ret = PTR_ERR(dp->aud_clk);
		if (ret == -EPROBE_DEFER)
			goto error_aclk;
		dp->aud_clk = NULL;
		dev_dbg(dp->dev, "failed to get the aud_clk:\n");
	} else {
		ret = clk_prepare_enable(dp->aud_clk);
		if (ret) {
			dev_err(dp->dev, "failed to enable aud_clk\n");
			goto error_aclk;
		}
	}

	dp->dp_sub = xilinx_drm_dp_sub_of_get(pdev->dev.of_node);
	if (IS_ERR(dp->dp_sub)) {
		ret = PTR_ERR(dp->dp_sub);
		goto error_aud_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dp->iomem = devm_ioremap_resource(dp->dev, res);
	if (IS_ERR(dp->iomem)) {
		ret = PTR_ERR(dp->iomem);
		goto error_dp_sub;
	}

	platform_set_drvdata(pdev, dp);

	xilinx_drm_writel(dp->iomem, XILINX_DP_TX_PHY_POWER_DOWN,
			  XILINX_DP_TX_PHY_POWER_DOWN_ALL);
	xilinx_drm_set(dp->iomem, XILINX_DP_TX_PHY_CONFIG,
		       XILINX_DP_TX_PHY_CONFIG_ALL_RESET);
	xilinx_drm_writel(dp->iomem, XILINX_DP_TX_FORCE_SCRAMBLER_RESET, 1);
	xilinx_drm_writel(dp->iomem, XILINX_DP_TX_ENABLE, 0);

	if (dp->dp_sub) {
		for (i = 0; i < dp->config.max_lanes; i++) {
			char phy_name[16];

			snprintf(phy_name, sizeof(phy_name), "dp-phy%d", i);
			dp->phy[i] = devm_phy_get(dp->dev, phy_name);
			if (IS_ERR(dp->phy[i])) {
				dev_err(dp->dev, "failed to get phy lane\n");
				ret = PTR_ERR(dp->phy[i]);
				goto error_dp_sub;
			}

			ret = phy_init(dp->phy[i]);
			if (ret) {
				dev_err(dp->dev,
					"failed to init phy lane %d\n", i);
				goto error_dp_sub;
			}
		}

		xilinx_drm_writel(dp->iomem, XILINX_DP_SUB_TX_INTR_DS,
				  XILINX_DP_TX_INTR_ALL);
		xilinx_drm_clr(dp->iomem, XILINX_DP_TX_PHY_CONFIG,
			       XILINX_DP_TX_PHY_CONFIG_ALL_RESET);
		xilinx_drm_writel(dp->iomem, XILINX_DP_TX_PHY_POWER_DOWN, 0);

		/* Wait for PLL to be locked for the primary (1st) */
		if (dp->phy[0]) {
			ret = xpsgtr_wait_pll_lock(dp->phy[0]);
			if (ret) {
				dev_err(dp->dev, "failed to lock pll\n");
				goto error_dp_sub;
			}
		}
	} else {
		xilinx_drm_writel(dp->iomem, XILINX_DP_TX_INTR_MASK,
				  XILINX_DP_TX_INTR_ALL);
		xilinx_drm_clr(dp->iomem, XILINX_DP_TX_PHY_CONFIG,
				XILINX_DP_TX_PHY_CONFIG_ALL_RESET);
		xilinx_drm_writel(dp->iomem, XILINX_DP_TX_PHY_POWER_DOWN, 0);
	}

#if 0
	dp->aux.name = "Xilinx DP AUX";
	dp->aux.dev = dp->dev;
	dp->aux.transfer = xilinx_drm_dp_aux_transfer;
	ret = drm_dp_aux_register(&dp->aux);
	if (ret < 0) {
		dev_err(dp->dev, "failed to initialize DP aux\n");
		return ret;
	}
#endif

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto error;
	}

	ret = devm_request_threaded_irq(dp->dev, irq, NULL,
					xilinx_drm_dp_irq_handler, IRQF_ONESHOT,
					dev_name(dp->dev), dp);
	if (ret < 0)
		goto error;

	return 0;

error:
#if 0
	drm_dp_aux_unregister(&dp->aux);
#endif
error_dp_sub:
	if (dp->dp_sub) {
		for (i = 0; i < dp->config.max_lanes; i++) {
			if (dp->phy[i]) {
				phy_exit(dp->phy[i]);
				dp->phy[i] = NULL;
			}
		}
	}
	xilinx_drm_dp_sub_put(dp->dp_sub);
error_aud_clk:
	if (dp->aud_clk)
		clk_disable_unprepare(dp->aud_clk);
error_aclk:
	clk_disable_unprepare(dp->aclk);
	return ret;
}

static int xilinx_drm_hdmi_remove(struct platform_device *pdev)
{
	struct xilinx_drm_hdmi *dp = platform_get_drvdata(pdev);
	unsigned int i;

	xilinx_drm_writel(dp->iomem, XILINX_DP_TX_ENABLE, 0);

	drm_dp_aux_unregister(&dp->aux);

	if (dp->dp_sub) {
		for (i = 0; i < dp->config.max_lanes; i++) {
			if (dp->phy[i]) {
				phy_exit(dp->phy[i]);
				dp->phy[i] = NULL;
			}
		}
	}

	xilinx_drm_dp_sub_put(dp->dp_sub);

	if (dp->aud_clk)
		clk_disable_unprepare(dp->aud_clk);
	clk_disable_unprepare(dp->aclk);

	return 0;
}

static const struct of_device_id xilinx_drm_hdmi_of_match[] = {
	{ .compatible = "xlnx,v-dp", },
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
