/*
 * dp159 redriver and retimer
 * Copyright (C) 2016 Leon Woestenberg <leon@sidebranch.com>
 *
 * based on dp159.c
 * Copyright (C) 2007 Hans Verkuil
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/of.h>

MODULE_DESCRIPTION("i2c device driver for dp159 redriver and retimer");
MODULE_AUTHOR("Leon Woestenberg");
MODULE_LICENSE("GPL");

static bool debug;

module_param(debug, bool, 0644);

MODULE_PARM_DESC(debug, "Debugging messages, 0=Off (default), 1=On");

struct dp159_state {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
};

//static int dp159_program(struct v4l2_subdev *sd, int mode);

static inline struct dp159_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct dp159_state, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct dp159_state, hdl)->sd;
}

/* ----------------------------------------------------------------------- */

static inline int dp159_write(struct v4l2_subdev *sd, u8 reg, u8 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_write_byte_data(client, reg, value);
}

static inline int dp159_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_read_byte_data(client, reg);
}

static int dp159_program(struct v4l2_subdev *sd, int mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int r;
	if (client->addr == 0x2C) {
		switch (mode) {
		/* HDMI 1.4 (250Mbps - 1.2Gbps) */
		case 0 :
			// Select page 1
			r = dp159_write(sd, 0xff, 0x01);

			// PLL_FBDIV is 280
			r = dp159_write(sd, 0x04, 0x80);
			r = dp159_write(sd, 0x05, 0x02);

			// PLL_PREDIV is 2
			r = dp159_write(sd, 0x08, 0x02);

			// CDR_CONFIG[4:0]
			r = dp159_write(sd, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(sd, 0x01, 0x81);
			usleep_range(10000, 11000);

			// Enable Bandgap
			r = dp159_write(sd, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(sd, 0x00, 0x03);

			// Enable TX
			r = dp159_write(sd, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(sd, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(sd, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(sd, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(sd, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(sd, 0x13, 0x0f);
			r = dp159_write(sd, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(sd, 0x11, 0xC0);

			// Enable receivers
			r = dp159_write(sd, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(sd, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(sd, 0x31, 0xC0);

			// Disable offset correction
			r = dp159_write(sd, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(sd, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(sd, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(sd, 0x4D, 0x38);

			// Enable Adaptive EQ
			r = dp159_write(sd, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(sd, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(sd, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x00);
			return 0;
			break;

		case 1 : // HDMI 1.4 (1.2Gbps - 3Gbps)
			// Select page 1
			r = dp159_write(sd, 0xff, 0x01);

			// PLL_FBDIV is 140
			r = dp159_write(sd, 0x04, 0x40);
			r = dp159_write(sd, 0x05, 0x01);

			// PLL_PREDIV is 4
			r = dp159_write(sd, 0x08, 0x04);

			// CDR_CONFIG[4:0]
			r = dp159_write(sd, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(sd, 0x01, 0x81);
			usleep_range(10000, 11000);
			// Enable Bandgap
			r = dp159_write(sd, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(sd, 0x00, 0x03);

			// Enable TX
			r = dp159_write(sd, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(sd, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(sd, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(sd, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(sd, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(sd, 0x13, 0x0f);
			r = dp159_write(sd, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(sd, 0x11, 0x70);

			// Enable receivers
			r = dp159_write(sd, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(sd, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(sd, 0x31, 0x40);

			// Disable offset correction
			r = dp159_write(sd, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(sd, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(sd, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(sd, 0x4D, 0x28);

			// Enable Adaptive EQ
			r = dp159_write(sd, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(sd, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(sd, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x00);
			return 0;
			break;

		case 2 : // HDMI 2.0 (3.4Gbps - 6 Gbps)

			 // Select page 1
			r = dp159_write(sd, 0xff, 0x01);

			// PLL_FBDIV is 280
			r = dp159_write(sd, 0x04, 0x80);
			r = dp159_write(sd, 0x05, 0x02);

			// PLL_PREDIV is 4
			r = dp159_write(sd, 0x08, 0x04);

			// CDR_CONFIG[4:0]
			r = dp159_write(sd, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(sd, 0x01, 0x81);
			usleep_range(10000, 11000);
			// Enable Bandgap
			r = dp159_write(sd, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(sd, 0x00, 0x03);

			// Enable TX
			r = dp159_write(sd, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(sd, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(sd, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(sd, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(sd, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(sd, 0x13, 0x0f);
			r = dp159_write(sd, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(sd, 0x11, 0x30);

			// Enable receivers
			r = dp159_write(sd, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(sd, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(sd, 0x31, 0x00);

			// Disable offset correction
			r = dp159_write(sd, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(sd, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(sd, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(sd, 0x4D, 0x18);

			// Enable Adaptive EQ
			r = dp159_write(sd, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(sd, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(sd, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(sd, 0x09, 0x00);
			return 0;
			break;
		} /* switch (mode) */
	/* DP159 ES? */
	} else if (client->addr == 0x5C) {
		if (mode == 2) {
			r = dp159_write(sd, 0x0A, 0x36);	// Automatic retimer for HDMI 2.0
			r = dp159_write(sd, 0x0B, 0x1a);

			r = dp159_write(sd, 0x0C, 0xa1);
			r = dp159_write(sd, 0x0D, 0x00);
		} else {
			r = dp159_write(sd, 0x0A, 0x35);			// Automatic redriver to retimer crossover at 1.0 Gbps
			//r = dp159_write(sd, 0x0A, 0x34);			// The redriver mode must be selected to support low video rates
			r = dp159_write(sd, 0x0B, 0x01);
			r = dp159_write(sd, 0x0C, 0xA0);				// Set VSWING data decrease by 24%
			r = dp159_write(sd, 0x0D, 0x00);
		}
	}
}

static int dp159_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	int mode = 0;
	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		if ((ctrl->val / 1000000) > 3400) {
			mode = 2;
		} else if ((ctrl->val / 1000000) > 1200) {
			mode = 1;
		}
		return dp159_program(sd, mode);
	}
	return -EINVAL;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int dp159_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	reg->size = 1;
	reg->val = dp159_read(sd, reg->reg);
	return 0;
}

static int dp159_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	dp159_write(sd, reg->reg, reg->val & 0xff);
	return 0;
}
#endif

static int dp159_log_status(struct v4l2_subdev *sd)
{
#if 0
	u8 v = dp159_read(sd, 0x09) & 7;
	u8 m = dp159_read(sd, 0x04);
	int vol = dp159_read(sd, 0x08) & 0x3f;

	v4l2_info(sd, "Input:  %d%s\n", v,
			(m & 0x80) ? " (muted)" : "");
	if (vol >= 32)
		vol = vol - 64;
	v4l2_info(sd, "Volume: %d dB\n", vol);
#endif
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops dp159_ctrl_ops = {
	.s_ctrl = dp159_s_ctrl,
};

static const struct v4l2_subdev_core_ops dp159_core_ops = {
	.log_status = dp159_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = dp159_g_register,
	.s_register = dp159_s_register,
#endif
};

static const struct v4l2_subdev_ops dp159_ops = {
	.core = &dp159_core_ops,
};

/* ----------------------------------------------------------------------- */

static int dp159_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct dp159_state *state;
	struct v4l2_subdev *sd;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &dp159_ops);

	v4l2_ctrl_handler_init(&state->hdl, 1);
	v4l2_ctrl_new_std(&state->hdl, &dp159_ctrl_ops,
			V4L2_CID_LINK_FREQ, 0, 8000*1000*1000LL, 1, 0);
	sd->ctrl_handler = &state->hdl;
	if (state->hdl.error) {
		int err = state->hdl.error;

		v4l2_ctrl_handler_free(&state->hdl);
		return err;
	}
	/* set volume/mute */
	v4l2_ctrl_handler_setup(&state->hdl);
	return 0;
}

/* ----------------------------------------------------------------------- */

static int dp159_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dp159_state *state = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&state->hdl);
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id dp159_id[] = {
	{ "dp159", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dp159_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id dp159_of_match[] = {
        { .compatible = "ti,dp159", },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dp159_of_match);
#endif

static struct i2c_driver dp159_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "dp159",
//#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(dp159_of_match),
//#endif
	},
	.probe		= dp159_probe,
	.remove		= dp159_remove,
	.id_table	= dp159_id,
};

module_i2c_driver(dp159_driver);
