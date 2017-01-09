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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of.h>
#include <linux/clk-provider.h>

MODULE_DESCRIPTION("i2c device driver for dp159 redriver and retimer");
MODULE_AUTHOR("Leon Woestenberg");
MODULE_LICENSE("GPL");

static bool debug;

module_param(debug, bool, 0644);

MODULE_PARM_DESC(debug, "Debugging messages, 0=Off (default), 1=On");

struct clk_tx_linerate {
	struct clk_hw hw;
	struct i2c_client *client;
};

static inline int dp159_write(struct i2c_client *client, u8 reg, u8 value)
{

#if 0
	struct i2c_client *client = v4l2_get_subdevdata(client);
#endif
	return i2c_smbus_write_byte_data(client, reg, value);
}

static inline int dp159_read(struct i2c_client *client, u8 reg)
{

	return i2c_smbus_read_byte_data(client, reg);
}

static int dp159_program(struct i2c_client *client, int mode)
{
	int r;
	if (client->addr == 0x2C) {
		switch (mode) {
		/* HDMI 1.4 (250Mbps - 1.2Gbps) */
		case 0 :
			// Select page 1
			r = dp159_write(client, 0xff, 0x01);

			// PLL_FBDIV is 280
			r = dp159_write(client, 0x04, 0x80);
			r = dp159_write(client, 0x05, 0x02);

			// PLL_PREDIV is 2
			r = dp159_write(client, 0x08, 0x02);

			// CDR_CONFIG[4:0]
			r = dp159_write(client, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(client, 0x01, 0x81);
			usleep_range(10000, 11000);

			// Enable Bandgap
			r = dp159_write(client, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(client, 0x00, 0x03);

			// Enable TX
			r = dp159_write(client, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(client, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(client, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(client, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(client, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(client, 0x13, 0x0f);
			r = dp159_write(client, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(client, 0x11, 0xC0);

			// Enable receivers
			r = dp159_write(client, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(client, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(client, 0x31, 0xC0);

			// Disable offset correction
			r = dp159_write(client, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(client, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(client, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(client, 0x4D, 0x38);

			// Enable Adaptive EQ
			r = dp159_write(client, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(client, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(client, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(client, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(client, 0x09, 0x00);
			return 0;
			break;

		case 1 : // HDMI 1.4 (1.2Gbps - 3Gbps)
			// Select page 1
			r = dp159_write(client, 0xff, 0x01);

			// PLL_FBDIV is 140
			r = dp159_write(client, 0x04, 0x40);
			r = dp159_write(client, 0x05, 0x01);

			// PLL_PREDIV is 4
			r = dp159_write(client, 0x08, 0x04);

			// CDR_CONFIG[4:0]
			r = dp159_write(client, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(client, 0x01, 0x81);
			usleep_range(10000, 11000);
			// Enable Bandgap
			r = dp159_write(client, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(client, 0x00, 0x03);

			// Enable TX
			r = dp159_write(client, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(client, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(client, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(client, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(client, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(client, 0x13, 0x0f);
			r = dp159_write(client, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(client, 0x11, 0x70);

			// Enable receivers
			r = dp159_write(client, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(client, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(client, 0x31, 0x40);

			// Disable offset correction
			r = dp159_write(client, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(client, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(client, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(client, 0x4D, 0x28);

			// Enable Adaptive EQ
			r = dp159_write(client, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(client, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(client, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(client, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(client, 0x09, 0x00);
			return 0;
			break;

		case 2 : // HDMI 2.0 (3.4Gbps - 6 Gbps)

			 // Select page 1
			r = dp159_write(client, 0xff, 0x01);

			// PLL_FBDIV is 280
			r = dp159_write(client, 0x04, 0x80);
			r = dp159_write(client, 0x05, 0x02);

			// PLL_PREDIV is 4
			r = dp159_write(client, 0x08, 0x04);

			// CDR_CONFIG[4:0]
			r = dp159_write(client, 0x0e, 0x10);

			// CP_CURRENT
			r = dp159_write(client, 0x01, 0x81);
			usleep_range(10000, 11000);
			// Enable Bandgap
			r = dp159_write(client, 0x00, 0x02);
			usleep_range(10000, 11000);
			// Enable PLL
			r = dp159_write(client, 0x00, 0x03);

			// Enable TX
			r = dp159_write(client, 0x10, 0x0f);

			// HDMI_TWPST1
			r = dp159_write(client, 0x14, 0x10);

			// DP_TWPST1
			r = dp159_write(client, 0x16, 0x10);

			// DP_TWPST2
			r = dp159_write(client, 0x17, 0x00);

			// Slew CTRL
			r = dp159_write(client, 0x12, 0x28);

			// FIR_UPD
			r = dp159_write(client, 0x13, 0x0f);
			r = dp159_write(client, 0x13, 0x00);

			// TX_RATE
			r = dp159_write(client, 0x11, 0x30);

			// Enable receivers
			r = dp159_write(client, 0x30, 0x0f);

			// PD_RXINT
			r = dp159_write(client, 0x32, 0x00);

			// RX_RATE
			r = dp159_write(client, 0x31, 0x00);

			// Disable offset correction
			r = dp159_write(client, 0x34, 0x00);

			// Change default of CDR_STL
			r = dp159_write(client, 0x3c, 0x04);

			// Change default of CDR_SO_TR
			r = dp159_write(client, 0x3D, 0x06);

			// EQFTC
			r = dp159_write(client, 0x4D, 0x18);

			// Enable Adaptive EQ
			r = dp159_write(client, 0x4c, 0x03);

			// Select page 0
			r = dp159_write(client, 0xff, 0x00);

			// Gate HPD_SNK
			r = dp159_write(client, 0x09, 0x01);

			// Set GPIO
			r = dp159_write(client, 0xe0, 0x01);

			// Un gate HPD_SNK
			r = dp159_write(client, 0x09, 0x00);
			return 0;
			break;
		} /* switch (mode) */
	/* DP159 ES? */
	} else if (client->addr == 0x5C) {
		if (mode == 2) {
			r = dp159_write(client, 0x0A, 0x36);	// Automatic retimer for HDMI 2.0
			r = dp159_write(client, 0x0B, 0x1a);

			r = dp159_write(client, 0x0C, 0xa1);
			r = dp159_write(client, 0x0D, 0x00);
		} else {
			r = dp159_write(client, 0x0A, 0x35);			// Automatic redriver to retimer crossover at 1.0 Gbps
			//r = dp159_write(client, 0x0A, 0x34);			// The redriver mode must be selected to support low video rates
			r = dp159_write(client, 0x0B, 0x01);
			r = dp159_write(client, 0x0C, 0xA0);				// Set VSWING data decrease by 24%
			r = dp159_write(client, 0x0D, 0x00);
		}
	}
}

#define to_clk_tx_linerate(_hw) container_of(_hw, struct clk_tx_linerate, hw)

int clk_tx_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct clk_tx_linerate *clk;
	clk = to_clk_tx_linerate(hw);
	return 0;
};

int clk_tx_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_tx_linerate *clk;
	clk = to_clk_tx_linerate(hw);
	return 0;
};

long clk_tx_round_rate(struct clk_hw *hw,
	unsigned long rate,	unsigned long *parent_rate)
{
	struct clk_tx_linerate *clk;
	clk = to_clk_tx_linerate(hw);
	return 0;
};

struct clk_ops clk_tx_rate_ops = {
	.set_rate 		= &clk_tx_set_rate,
	.recalc_rate	= &clk_tx_recalc_rate,
	.round_rate		= &clk_tx_round_rate,
};

static int dp159_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct clk_tx_linerate *clk_tx;
	struct clk_init_data init;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	/* allocate fixed-rate clock */
	clk_tx = kzalloc(sizeof(*clk_tx), GFP_KERNEL);
	if (!clk_tx)
		return -ENOMEM;

	init.name = "clk_tx_linerate";
	init.ops = &clk_tx_rate_ops;
	init.flags = /*flags |*/ CLK_IS_BASIC;
	init.parent_names = NULL;
	init.num_parents = 0;

	/* register the clock */
	clk_tx = clk_register(&client->dev, &clk_tx->hw);
	if (IS_ERR(clk_tx)) {
		kfree(clk_tx);
		return ERR_PTR(clk_tx);
	}
	/* reference to client in clock */
	clk_tx->client = client;
	/* reference to clk in client */
	i2c_set_clientdata(client, (void *)clk_tx);

	return 0;
}

static int dp159_remove(struct i2c_client *client)
{
	struct clk_tx_linerate *clk_tx;
	clk_tx = (struct clk_tx_linerate *)i2c_get_clientdata(client);
	if (clk_tx)
		clk_unregister(clk_tx);
	return 0;
}

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
		.of_match_table = of_match_ptr(dp159_of_match),
	},
	.probe		= dp159_probe,
	.remove		= dp159_remove,
	.id_table	= dp159_id,
};

module_i2c_driver(dp159_driver);
