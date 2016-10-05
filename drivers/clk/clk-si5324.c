/*
 * clk-si5324.c: Silicon Laboratories Si5324 Clock Multiplier / Jitter Attenuator
 *
 * Leon Woestenberg <leon@sidebranch.com>
 *
 * References:
 * [1] "Si5324 Data Sheet"
 *     https://www.silabs.com/Support%20Documents/TechnicalDocs/Si5324.pdf
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/rational.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/platform_data/si5324.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/div64.h>

#include "clk-si5324.h"
#include "si5324drv.h"

struct si5324_driver_data;

struct si5324_parameters {
	// Current Si5342 parameters
	u32 n1_min;
	u32 n1_max;
	u32 n1_hs;
	u32 nc_ls_min;
	u32 nc_ls_max;
	u32 nc_ls;
	u32 n2_hs;
	u32 n2_ls_min;
	u32 n2_ls_max;
	u32 n2_ls;
	u32 n3_min;
	u32 n3_max;
	u32 n3;
	// Current frequencies (fixed point 36.28 notation)
	u64 fin;
	u64 fout;
	u64 fosc;
	// Best settings found
	u64 best_delta_fout;
	u64 best_fout;
	u32 best_n1_hs;
	u32 best_nc_ls;
	u32 best_n2_hs;
	u32 best_n2_ls;
	u32 best_n3;
	int valid;
};

struct si5324_hw_data {
	struct clk_hw			hw;
	struct si5324_driver_data	*drvdata;
	struct si5324_parameters	params;
	unsigned char			num;
};

struct si5324_driver_data {
	struct i2c_client	*client;
	struct regmap		*regmap;
	struct clk_onecell_data onecell;


	struct clk		*pxtal;
	const char		*pxtal_name;
	struct clk_hw		xtal;

	struct clk		*pclkin1;
	const char		*pclkin1_name;
	struct clk_hw		clkin1;

	struct clk		*pclkin2;
	const char		*pclkin2_name;
	struct clk_hw		clkin2;

	struct si5324_hw_data	pll;
	struct si5324_hw_data	*clkout;
};

static const char * const si5324_input_names[] = {
	"xtal", "clkin1", "clkin2"
};

static const char * const si5324_pll_name = "pll";

static const char * const si5324_clkout_names[] = {
	"clk0", "clk1"
};

/*
 * Si5324 i2c regmap
 */
static inline u8 si5324_reg_read(struct si5324_driver_data *drvdata, u8 reg)
{
	u32 val;
	int ret;

	ret = regmap_read(drvdata->regmap, reg, &val);
	if (ret) {
		dev_err(&drvdata->client->dev,
			"unable to read from reg%02x\n", reg);
		return 0;
	} else {
		dev_info(&drvdata->client->dev, "Read value 0x%02x from reg0x@%02x\n",
			(int)val, (int)reg);
	}

	return (u8)val;
}

static inline int si5324_bulk_read(struct si5324_driver_data *drvdata,
				   u8 reg, u8 count, u8 *buf)
{
	return regmap_bulk_read(drvdata->regmap, reg, buf, count);
}

static inline int si5324_reg_write(struct si5324_driver_data *drvdata,
				   u8 reg, u8 val)
{
	dev_info(&drvdata->client->dev, "si5324_reg_write() 0x%02x @0x%02x\n", (int)val, (int)reg);

	return regmap_write(drvdata->regmap, reg, val);
}

static inline int si5324_bulk_write(struct si5324_driver_data *drvdata,
				    u8 reg, u8 count, const u8 *buf)
{
	return regmap_raw_write(drvdata->regmap, reg, buf, count);
}

static inline int si5324_set_bits(struct si5324_driver_data *drvdata,
				  u8 reg, u8 mask, u8 val)
{
	return regmap_update_bits(drvdata->regmap, reg, mask, val);
}

/* similar to Si5324_DoSettings() */
static inline int si5324_bulk_scatter_write(struct si5324_driver_data *drvdata,
					u8 count/*number of reg/val pairs*/, const u8 *buf)
{
	int i;
	for (i = 0; i < count; i++) {
		si5324_reg_write(drvdata, buf[i * 2]/*reg*/, buf[i * 2 + 1]/*val*/);
	}
}

static void si5324_initialize(struct si5324_driver_data *drvdata)
{
	/* keep RST_REG asserted for 10 ms */
	si5324_set_bits(drvdata, SI5324_RESET,
		SI5324_RST_REG, SI5324_RST_REG);
	msleep(10);
	si5324_set_bits(drvdata, SI5324_RESET,
		SI5324_RST_REG, 0);
	msleep(10);

	// Disable output clocks during calibration (bit 4 SQ_ICAL=1),
	// other bits are default
	si5324_reg_write(drvdata, 3, 0x15);

	// Auto select clock (automatic revertive) (bit 7:6 AUTOSEL_REG)10
	// History delay default
	si5324_reg_write(drvdata, 4, 0x92);
	// Disable CKOUT2 (SFOUT2_REG=001)
	// set CKOUT1 to LVDS (SFOUT1_REG=111)
	// (default is LVPECL for both)
	si5324_reg_write(drvdata, 6, 0x0F);
	// Enable CKOUT1 output (bit 2 DSBL1_REG = 1)
	// disable CKOUT2 output (bit 3 DSBL2_REG=0)
	si5324_reg_write(drvdata, 10, 0x08);
	// Disable CKIN2 input buffer (bit 1 PD_CK2=1)
	// enable CKIN1 buffer (bit 0 PD_CK1=0)
	// (bit 6 is reserved, write default value)
	si5324_reg_write(drvdata, 11, 0x42);
	// Set lock time to 53ms as recommended (bits 2:0 LOCKT=001)
	// other bits are default
	si5324_reg_write(drvdata, 19, 0x2f);  // 0x29
	// Enable fast locking (bit 0 FASTLOCK=1)
	si5324_reg_write(drvdata, 137, 0x01);   // FASTLOCK=1 (enable fast locking)
}

#define SI5324_PARAMETERS_LENGTH		30

static void si5324_read_parameters(struct si5324_driver_data *drvdata,
				   u8 reg, struct si5324_parameters *params)
{
#if 0
	u8 buf[SI5324_PARAMETERS_LENGTH];

	switch (reg) {
	case SI5324_CLK6_PARAMETERS:
	case SI5324_CLK7_PARAMETERS:
		buf[0] = si5324_reg_read(drvdata, reg);
		params->p1 = buf[0];
		params->p2 = 0;
		params->p3 = 1;
		break;
	default:
		si5324_bulk_read(drvdata, reg, SI5324_PARAMETERS_LENGTH, buf);
		params->p1 = ((buf[2] & 0x03) << 16) | (buf[3] << 8) | buf[4];
		params->p2 = ((buf[5] & 0x0f) << 16) | (buf[6] << 8) | buf[7];
		params->p3 = ((buf[5] & 0xf0) << 12) | (buf[0] << 8) | buf[1];
	}
#endif
	params->valid = 1;
}

static void si5324_write_parameters(struct si5324_driver_data *drvdata,
				    u8 reg, struct si5324_parameters *params)
{
#if 0
	u8 buf[SI5324_PARAMETERS_LENGTH];
	switch (reg) {
	case SI5324_CLK6_PARAMETERS:
	case SI5324_CLK7_PARAMETERS:
		buf[0] = params->p1 & 0xff;
		si5324_reg_write(drvdata, reg, buf[0]);
		break;
	default:
		buf[0] = ((params->p3 & 0x0ff00) >> 8) & 0xff;
		buf[1] = params->p3 & 0xff;
		/* save rdiv and divby4 */
		buf[2] = si5324_reg_read(drvdata, reg + 2) & ~0x03;
		buf[2] |= ((params->p1 & 0x30000) >> 16) & 0x03;
		buf[3] = ((params->p1 & 0x0ff00) >> 8) & 0xff;
		buf[4] = params->p1 & 0xff;
		buf[5] = ((params->p3 & 0xf0000) >> 12) |
			((params->p2 & 0xf0000) >> 16);
		buf[6] = ((params->p2 & 0x0ff00) >> 8) & 0xff;
		buf[7] = params->p2 & 0xff;
		si5324_bulk_write(drvdata, reg, SI5324_PARAMETERS_LENGTH, buf);
	}
#endif
}

static bool si5324_regmap_is_volatile(struct device *dev, unsigned int reg)
{
#if 0
	switch (reg) {
	case SI5324_INTERRUPT_STATUS:
	case SI5324_PLL_RESET:
		return true;
	}
#endif
	return false;
}

static bool si5324_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	/* reserved registers */
	if (reg >= 12 && reg <= 18)
		return false;
	if (reg >= 26 && reg <= 30)
		return false;
	if (reg >= 37 && reg <= 39)
		return false;
	if (reg >= 49 && reg <= 54)
		return false;
	if (reg >= 56 && reg <= 127)
		return false;
	if (reg >= 144)
		return false;
	/* read-only */
	if (reg >= 128 && reg <= 130)
		return false;
	if (reg >= 134 && reg <= 135)
		return false;
	if (reg == 137)
		return false;
	return true;
}

static const struct regmap_config si5324_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 144,
	.writeable_reg = si5324_regmap_is_writeable,
	.volatile_reg = si5324_regmap_is_volatile,
};

/*
 * Si5324 xtal clock input
 */
static int si5324_xtal_prepare(struct clk_hw *hw)
{
	struct si5324_driver_data *drvdata =
		container_of(hw, struct si5324_driver_data, xtal);
	/* enable free-run */
	si5324_set_bits(drvdata, SI5324_REG0,
			SI5324_REG0_FREE_RUN, SI5324_REG0_FREE_RUN);
	/* clkin2 powered, clkin1 powered-down, xtal connects to clkin2 */
	si5324_set_bits(drvdata, SI5324_POWERDOWN,
			SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK1);
	return 0;
}

static void si5324_xtal_unprepare(struct clk_hw *hw)
{
	struct si5324_driver_data *drvdata =
		container_of(hw, struct si5324_driver_data, xtal);
}

static const struct clk_ops si5324_xtal_ops = {
	.prepare = si5324_xtal_prepare,
	.unprepare = si5324_xtal_unprepare,
};

/*
 * Si5324 clkin1/clkin2 clock input
 */
static int si5324_clkin_prepare(struct clk_hw *hw)
{
	struct si5324_driver_data *drvdata;
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);

	/* clkin1? */
	if (hwdata->num == 0) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin1);
		/* disable free-run */
		si5324_set_bits(drvdata, SI5324_REG0, SI5324_REG0_FREE_RUN, 0);
		/* clkin1 powered, clkin2 powered-down*/
		si5324_set_bits(drvdata, SI5324_POWERDOWN,
			SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK2);
	} else if (hwdata->num == 1) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin2);
		/* disable free-run */
		si5324_set_bits(drvdata, SI5324_REG0, SI5324_REG0_FREE_RUN, 0);
		/* clkin2 powered, clkin1 powered-down*/
		si5324_set_bits(drvdata, SI5324_POWERDOWN,
			SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK1);
	} else {
	}
	return 0;
}

static void si5324_clkin_unprepare(struct clk_hw *hw)
{
	struct si5324_driver_data *drvdata;
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	if (hwdata->num == 0) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin1);
	} else if (hwdata->num == 1) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin2);
	} else {
	}
}

/*
 * CMOS clock source constraints:
 * The input frequency range of the PLL is 10Mhz to 40MHz.
 * If CLKIN is >40MHz, the input divider must be used.
 */
static unsigned long si5324_clkin_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct si5324_driver_data *drvdata;
	unsigned long rate;
	unsigned char idiv;
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	(void)rate;
	(void)idiv;
	if (hwdata->num == 0) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin1);
	} else if (hwdata->num == 1) {
		drvdata = container_of(hw, struct si5324_driver_data, clkin2);
	} else {
	}
	rate = parent_rate;
#if 0
	if (parent_rate > 160000000) {
		idiv = SI5324_CLKIN_DIV_8;
		rate /= 8;
	} else if (parent_rate > 80000000) {
		idiv = SI5324_CLKIN_DIV_4;
		rate /= 4;
	} else if (parent_rate > 40000000) {
		idiv = SI5324_CLKIN_DIV_2;
		rate /= 2;
	} else {
		idiv = SI5324_CLKIN_DIV_1;
	}

	si5324_set_bits(drvdata, SI5324_PLL_INPUT_SOURCE,
			SI5324_CLKIN_DIV_MASK, idiv);

	dev_dbg(&drvdata->client->dev, "%s - clkin div = %d, rate = %lu\n",
		__func__, (1 << (idiv >> 6)), rate);
#endif
	return rate;
}

static const struct clk_ops si5324_clkin_ops = {
	.prepare = si5324_clkin_prepare,
	.unprepare = si5324_clkin_unprepare,
	.recalc_rate = si5324_clkin_recalc_rate,
};

/* Select other clock input to the PLL
 */
static int _si5324_pll_reparent(struct si5324_driver_data *drvdata,
				int num, enum si5324_pll_src parent)
{
	if (parent == SI5324_PLL_SRC_DEFAULT)
		return 0;

	if (parent == SI5324_PLL_SRC_XTAL) {
		/* enable free-run */
		si5324_set_bits(drvdata, SI5324_REG0,
				SI5324_REG0_FREE_RUN, SI5324_REG0_FREE_RUN);
		/* clkin2 powered, clkin1 powered-down, xtal connects to clkin2 */
		si5324_set_bits(drvdata, SI5324_POWERDOWN,
				SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK1);
		/* select clkin2 */
		si5324_set_bits(drvdata, SI5324_CKSEL,
				3 << 6, 1 << 6);
	} else if (parent == SI5324_PLL_SRC_CLKIN1) {
		/* disable free-run */
		si5324_set_bits(drvdata, SI5324_REG0,
				SI5324_REG0_FREE_RUN, 0);
		/* clkin1 powered, clkin2 powered-down */
		si5324_set_bits(drvdata, SI5324_POWERDOWN,
				SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK2);
		/* select clkin1 */
		si5324_set_bits(drvdata, SI5324_CKSEL,
				3 << 6, 0);
	} else if (parent == SI5324_PLL_SRC_CLKIN2) {
		/* disable free-run */
		si5324_set_bits(drvdata, SI5324_REG0,
				SI5324_REG0_FREE_RUN, 0);
		/* clkin2 powered, clkin1 powered-down */
		si5324_set_bits(drvdata, SI5324_POWERDOWN,
				SI5324_PD_CK1 || SI5324_PD_CK2, SI5324_PD_CK1);
		/* select clkin2 */
		si5324_set_bits(drvdata, SI5324_CKSEL,
				3 << 6, 1 << 6);
	}
	return 0;
}

static unsigned char si5324_pll_get_parent(struct clk_hw *hw)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	return 0;
}

static int si5324_pll_set_parent(struct clk_hw *hw, u8 index)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);

	if (index > 2)
		return -EINVAL;

	return _si5324_pll_reparent(hwdata->drvdata, hwdata->num,
			     (index == 0) ? SI5324_PLL_SRC_XTAL :
			     SI5324_PLL_SRC_CLKIN1);
}

static unsigned long si5324_pll_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	return parent_rate;
#if 0
	u8 reg = (hwdata->num == 0) ? SI5324_PLLA_PARAMETERS :
		SI5324_PLLB_PARAMETERS;
	unsigned long long rate;

	if (!hwdata->params.valid)
		si5324_read_parameters(hwdata->drvdata, reg, &hwdata->params);

	if (hwdata->params.p3 == 0)
		return parent_rate;

	/* fVCO = fIN * (P1*P3 + 512*P3 + P2)/(128*P3) */
	rate  = hwdata->params.p1 * hwdata->params.p3;
	rate += 512 * hwdata->params.p3;
	rate += hwdata->params.p2;
	rate *= parent_rate;
	do_div(rate, 128 * hwdata->params.p3);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw),
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		parent_rate, (unsigned long)rate);
	return (unsigned long)rate;
#endif
}

static long si5324_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
#if 0
	unsigned long rfrac, denom, a, b, c;
	unsigned long long lltmp;

	if (rate < SI5324_PLL_VCO_MIN)
		rate = SI5324_PLL_VCO_MIN;
	if (rate > SI5324_PLL_VCO_MAX)
		rate = SI5324_PLL_VCO_MAX;

	/* determine integer part of feedback equation */
	a = rate / *parent_rate;

	if (a < SI5324_PLL_A_MIN)
		rate = *parent_rate * SI5324_PLL_A_MIN;
	if (a > SI5324_PLL_A_MAX)
		rate = *parent_rate * SI5324_PLL_A_MAX;

	/* find best approximation for b/c = fVCO mod fIN */
	denom = 1000 * 1000;
	lltmp = rate % (*parent_rate);
	lltmp *= denom;
	do_div(lltmp, *parent_rate);
	rfrac = (unsigned long)lltmp;

	b = 0;
	c = 1;
	if (rfrac)
		rational_best_approximation(rfrac, denom,
				    SI5324_PLL_B_MAX, SI5324_PLL_C_MAX, &b, &c);

	/* calculate parameters */
	hwdata->params.p3  = c;
	hwdata->params.p2  = (128 * b) % c;
	hwdata->params.p1  = 128 * a;
	hwdata->params.p1 += (128 * b / c);
	hwdata->params.p1 -= 512;

	/* recalculate rate by fIN * (a + b/c) */
	lltmp  = *parent_rate;
	lltmp *= b;
	do_div(lltmp, c);

	rate  = (unsigned long)lltmp;
	rate += *parent_rate * a;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: a = %lu, b = %lu, c = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw), a, b, c,
		*parent_rate, rate);
#endif
	return rate;
}

static int si5324_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
#if 0
	u8 reg = (hwdata->num == 0) ? SI5324_PLLA_PARAMETERS :
		SI5324_PLLB_PARAMETERS;

	/* write multisynth parameters */
	si5324_write_parameters(hwdata->drvdata, reg, &hwdata->params);

	/* plla/pllb ctrl is in clk6/clk7 ctrl registers */
	si5324_set_bits(hwdata->drvdata, SI5324_CLK6_CTRL + hwdata->num,
		SI5324_CLK_INTEGER_MODE,
		(hwdata->params.p2 == 0) ? SI5324_CLK_INTEGER_MODE : 0);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw),
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		parent_rate, rate);
#endif
	return 0;
}

static const struct clk_ops si5324_pll_ops = {
	.set_parent = si5324_pll_set_parent,
	.get_parent = si5324_pll_get_parent,
	.recalc_rate = si5324_pll_recalc_rate,
	.round_rate = si5324_pll_round_rate,
	.set_rate = si5324_pll_set_rate,
};

static int _si5324_clkout_set_drive_strength(
	struct si5324_driver_data *drvdata, int num,
	enum si5324_drive_strength drive)
{
#if 0
	u8 mask;

	if (num > 8)
		return -EINVAL;

	switch (drive) {
	case SI5324_DRIVE_2MA:
		mask = SI5324_CLK_DRIVE_STRENGTH_2MA;
		break;
	case SI5324_DRIVE_4MA:
		mask = SI5324_CLK_DRIVE_STRENGTH_4MA;
		break;
	case SI5324_DRIVE_6MA:
		mask = SI5324_CLK_DRIVE_STRENGTH_6MA;
		break;
	case SI5324_DRIVE_8MA:
		mask = SI5324_CLK_DRIVE_STRENGTH_8MA;
		break;
	default:
		return 0;
	}

	si5324_set_bits(drvdata, SI5324_CLK0_CTRL + num,
			SI5324_CLK_DRIVE_STRENGTH_MASK, mask);
#endif
	return 0;
}

static int _si5324_clkout_set_disable_state(
	struct si5324_driver_data *drvdata, int num,
	enum si5324_disable_state state)
{
#if 0
	u8 reg = (num < 4) ? SI5324_CLK3_0_DISABLE_STATE :
		SI5324_CLK7_4_DISABLE_STATE;
	u8 shift = (num < 4) ? (2 * num) : (2 * (num-4));
	u8 mask = SI5324_CLK_DISABLE_STATE_MASK << shift;
	u8 val;

	if (num > 8)
		return -EINVAL;

	switch (state) {
	case SI5324_DISABLE_LOW:
		val = SI5324_CLK_DISABLE_STATE_LOW;
		break;
	case SI5324_DISABLE_HIGH:
		val = SI5324_CLK_DISABLE_STATE_HIGH;
		break;
	case SI5324_DISABLE_FLOATING:
		val = SI5324_CLK_DISABLE_STATE_FLOAT;
		break;
	case SI5324_DISABLE_NEVER:
		val = SI5324_CLK_DISABLE_STATE_NEVER;
		break;
	default:
		return 0;
	}

	si5324_set_bits(drvdata, reg, mask, val << shift);
#endif
	return 0;
}

static int si5324_clkout_prepare(struct clk_hw *hw)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);

	/* clear power-down bit for output clock num */
	si5324_set_bits(hwdata->drvdata, SI5324_DSBL_CLKOUT,
			1 << (hwdata->num + 2), 0);
	return 0;
}

static void si5324_clkout_unprepare(struct clk_hw *hw)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);

	/* set power-down bit for output clock num */
	si5324_set_bits(hwdata->drvdata, SI5324_DSBL_CLKOUT,
			1 << (hwdata->num + 2), 1 << (hwdata->num + 2));
}

/*
 * Si5324 clkout divider
 */
static unsigned long si5324_clkout_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	/* clkout2 divider is three registers higher up clkout1 divider */
	unsigned char reg = SI5324_NC1_LS_H + (hwdata->num * 3);
	/* common divider */
	unsigned char n1_hs;
	/* output clock specific divider */
	unsigned char nc_ls_h;
	unsigned char nc_ls_m;
	unsigned char nc_ls_l;
	unsigned long nc_ls;

	/* obtain divider values from hardware */
	n1_hs = si5324_reg_read(hwdata->drvdata, SI5324_N1_HS_OUTPUT_DIVIDER) >> 5;
	n1_hs += 4;

	nc_ls_h = si5324_reg_read(hwdata->drvdata, reg);
	nc_ls_m = si5324_reg_read(hwdata->drvdata, reg + 1);
	nc_ls_l = si5324_reg_read(hwdata->drvdata, reg + 2);
	nc_ls = (nc_ls_h << 16) || (nc_ls_m << 8) || nc_ls_l;
	nc_ls += 1;
	return (parent_rate / n1_hs) / nc_ls;
}

static long si5324_clkout_round_rate_bsp(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	u32 NCn_ls, N2_ls, N3n;
	u8  N1_hs, N2_hs, BwSel;
	int result;
	u8  buf[14*2]; // Need to set 14 registers
	int i;

	// Calculate the frequency settings for the Si5324
	result = Si5324_CalcFreqSettings(114285000, 114285000,
	                                 &N1_hs, &NCn_ls, &N2_hs, &N2_ls, &N3n,
	                                 &BwSel);
}

/* round_rate selects the rate closest to the requested one.
determine_rate does the same but even better by changing the clock’s
parent. The actual setting is done by set_rate. recalc_rate is called
when a parent changes rate.  */
static long si5324_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);
	unsigned char rdiv;

#if 0
	if (rate > SI5324_CLKOUT_MAX_FREQ)
		rate = SI5324_CLKOUT_MAX_FREQ;
	if (rate < SI5324_CLKOUT_MIN_FREQ)
		rate = SI5324_CLKOUT_MIN_FREQ;

	/* request frequency if multisync master */
	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		/* use r divider for frequencies below 1MHz */
		rdiv = SI5324_OUTPUT_CLK_DIV_1;
		while (rate < SI5324_MULTISYNTH_MIN_FREQ &&
		       rdiv < SI5324_OUTPUT_CLK_DIV_128) {
			rdiv += 1;
			rate *= 2;
		}
		*parent_rate = rate;
	} else {
		unsigned long new_rate, new_err, err;

		/* round to closed rdiv */
		rdiv = SI5324_OUTPUT_CLK_DIV_1;
		new_rate = *parent_rate;
		err = abs(new_rate - rate);
		do {
			new_rate >>= 1;
			new_err = abs(new_rate - rate);
			if (new_err > err || rdiv == SI5324_OUTPUT_CLK_DIV_128)
				break;
			rdiv++;
			err = new_err;
		} while (1);
	}
	rate = *parent_rate >> rdiv;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %u, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw), (1 << rdiv),
		*parent_rate, rate);
#endif
	return rate;
}

static int si5324_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct si5324_hw_data *hwdata =
		container_of(hw, struct si5324_hw_data, hw);

	u32 NCn_ls, N2_ls, N3n;
	u8  N1_hs, N2_hs, BwSel;
	int result;
	u8  buf[14*2]; // Need to set 14 registers
	int i;

	// Calculate the frequency settings for the Si5324
	result = Si5324_CalcFreqSettings(114285000, 114285000,
	                                 &N1_hs, &NCn_ls, &N2_hs, &N2_ls, &N3n,
	                                 &BwSel);

	    i = 0;

	// Free running mode or use a reference clock
	buf[i] = 0;
	    // Enable free running mode
	    buf[i+1] = 0x54;
	i += 2;

	// Loop bandwidth
	buf[i]   = 2;
	buf[i+1] = (BwSel << 4) | 0x02;
	i += 2;

	// Enable reference clock 2 in free running mode
	buf[i] = 11;
	    //Enable input clock 2
	    buf[i+1] = 0x40;
	i += 2;

	// N1_HS
	buf[i]   = 25;
	buf[i+1] = N1_hs << 5;
	i += 2;

	// NC1_LS
	buf[i]   = 31;
	buf[i+1] = (u8)((NCn_ls & 0x000F0000) >> 16);
	buf[i+2] = 32;
	buf[i+3] = (u8)((NCn_ls & 0x0000FF00) >>  8);
	buf[i+4] = 33;
	buf[i+5] = (u8)( NCn_ls & 0x000000FF       );
	i += 6;

	// N2_HS and N2_LS
	buf[i]    = 40;
	buf[i+1]  = (N2_hs << 5);
	// N2_LS upper bits (same register as N2_HS)
	buf[i+1] |= (u8)((N2_ls & 0x000F0000) >> 16);
	buf[i+2]  = 41;
	buf[i+3]  = (u8)((N2_ls & 0x0000FF00) >>  8);
	buf[i+4]  = 42;
	buf[i+5]  = (u8)( N2_ls & 0x000000FF       );
	i += 6;

	    // N32
	    buf[i]   = 46;
	    buf[i+2] = 47;
	    buf[i+4] = 48;
	buf[i+1] = (u8)((N3n & 0x00070000) >> 16);
	buf[i+3] = (u8)((N3n & 0x0000FF00) >>  8);
	buf[i+5] = (u8)( N3n & 0x000000FF       );
	i += 6;

	// Start calibration
	buf[i]   = 136;
	buf[i+1] = 0x40;
	i += 2;

	return si5324_bulk_scatter_write(hwdata->drvdata, 14, buf);
}

static const struct clk_ops si5324_clkout_ops = {
	.prepare = si5324_clkout_prepare,
	.unprepare = si5324_clkout_unprepare,
	.recalc_rate = si5324_clkout_recalc_rate,
	.round_rate = si5324_clkout_round_rate,
	.set_rate = si5324_clkout_set_rate,
};

/*
 * Si5324 i2c probe and DT
 */
#ifdef CONFIG_OF
static const struct of_device_id si5324_dt_ids[] = {
	{ .compatible = "silabs,si5324", },
	{ }
};
MODULE_DEVICE_TABLE(of, si5324_dt_ids);

static int si5324_dt_parse(struct i2c_client *client)
{
	struct device_node *child, *np = client->dev.of_node;
	struct si5324_platform_data *pdata;
	struct property *prop;
	const __be32 *p;
	int num = 0;
	u32 val;

	if (np == NULL)
		return 0;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/*
	 * property silabs,pll-source : <num src>, [<..>]
	 * allow to selectively set pll source
	 */
	of_property_for_each_u32(np, "silabs,pll-source", prop, p, num) {
		if (num >= 1) {
			dev_err(&client->dev,
				"invalid pll %d on pll-source prop\n", num);
			return -EINVAL;
		}

		p = of_prop_next_u32(prop, p, &val);
		if (!p) {
			dev_err(&client->dev,
				"missing pll-source for pll %d\n", num);
			return -EINVAL;
		}

		switch (val) {
		case 0:
			pdata->pll_src = SI5324_PLL_SRC_XTAL;
			break;
		case 1:
			pdata->pll_src = SI5324_PLL_SRC_CLKIN1;
			break;
		case 2:
			pdata->pll_src = SI5324_PLL_SRC_CLKIN2;
			break;
		default:
			dev_err(&client->dev,
				 "invalid parent %d for pll %d\n", val, num);
			return -EINVAL;
		}
	}

	/* per clkout properties */
	for_each_child_of_node(np, child) {
		if (of_property_read_u32(child, "reg", &num)) {
			dev_err(&client->dev, "missing reg property of %s\n",
				child->name);
			goto put_child;
		}

		if (num >= 2) {
			dev_err(&client->dev, "invalid clkout %d\n", num);
			goto put_child;
		}

		if (!of_property_read_u32(child, "silabs,drive-strength",
					  &val)) {
			switch (val) {
			case SI5324_DRIVE_2MA:
			case SI5324_DRIVE_4MA:
			case SI5324_DRIVE_6MA:
			case SI5324_DRIVE_8MA:
				pdata->clkout[num].drive = val;
				break;
			default:
				dev_err(&client->dev,
					"invalid drive strength %d for clkout %d\n",
					val, num);
				goto put_child;
			}
		}

		if (!of_property_read_u32(child, "silabs,disable-state",
					  &val)) {
			switch (val) {
			case 0:
				pdata->clkout[num].disable_state =
					SI5324_DISABLE_LOW;
				break;
			case 1:
				pdata->clkout[num].disable_state =
					SI5324_DISABLE_HIGH;
				break;
			case 2:
				pdata->clkout[num].disable_state =
					SI5324_DISABLE_FLOATING;
				break;
			case 3:
				pdata->clkout[num].disable_state =
					SI5324_DISABLE_NEVER;
				break;
			default:
				dev_err(&client->dev,
					"invalid disable state %d for clkout %d\n",
					val, num);
				goto put_child;
			}
		}

		if (!of_property_read_u32(child, "clock-frequency", &val))
			pdata->clkout[num].rate = val;

		pdata->clkout[num].pll_master =
			of_property_read_bool(child, "silabs,pll-master");
	}
	client->dev.platform_data = pdata;

	return 0;
put_child:
	of_node_put(child);
	return -EINVAL;
}
#else
static int si5324_dt_parse(struct i2c_client *client)
{
	return 0;
}
#endif /* CONFIG_OF */

static int si5324_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct si5324_platform_data *pdata;
	struct si5324_driver_data *drvdata;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_names[3];
	u8 val;
	u8 num_parents, num_clocks;
	int ret, n;

	ret = si5324_dt_parse(client);
	if (ret)
		return ret;

	pdata = client->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	drvdata = devm_kzalloc(&client->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&client->dev, "unable to allocate driver data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, drvdata);
	drvdata->client = client;
	drvdata->pxtal = devm_clk_get(&client->dev, "xtal");
	drvdata->pclkin1 = devm_clk_get(&client->dev, "clkin1");
	drvdata->pclkin2 = devm_clk_get(&client->dev, "clkin2");

	if (PTR_ERR(drvdata->pxtal) == -EPROBE_DEFER ||
		PTR_ERR(drvdata->pclkin1) == -EPROBE_DEFER ||
		PTR_ERR(drvdata->pclkin2) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	drvdata->regmap = devm_regmap_init_i2c(client, &si5324_regmap_config);
	if (IS_ERR(drvdata->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(drvdata->regmap);
	}

	if ((si5324_reg_read(drvdata, 134) != 0x01) || (si5324_reg_read(drvdata, 135) != 0x82)) {
		dev_err(&client->dev, "Identification registers do not indicate Si5324 presence.\n");
		return -ENODEV;
	}

	si5324_initialize(drvdata);

	/* setup clock configuration */
	ret = _si5324_pll_reparent(drvdata, 0, pdata->pll_src);
	if (ret) {
		dev_err(&client->dev,
			"failed to reparent pll to %d\n",
			pdata->pll_src);
		return ret;
	}

	for (n = 0; n < 2; n++) {

		ret = _si5324_clkout_set_drive_strength(drvdata, n,
							pdata->clkout[n].drive);
		if (ret) {
			dev_err(&client->dev,
				"failed set drive strength of clkout%d to %d\n",
				n, pdata->clkout[n].drive);
			return ret;
		}

		ret = _si5324_clkout_set_disable_state(drvdata, n,
						pdata->clkout[n].disable_state);
		if (ret) {
			dev_err(&client->dev,
				"failed set disable state of clkout%d to %d\n",
				n, pdata->clkout[n].disable_state);
			return ret;
		}
	}

	if (!IS_ERR(drvdata->pxtal))
		clk_prepare_enable(drvdata->pxtal);
	if (!IS_ERR(drvdata->pclkin1))
		clk_prepare_enable(drvdata->pclkin1);
	if (!IS_ERR(drvdata->pclkin2))
		clk_prepare_enable(drvdata->pclkin2);

	/* register xtal input clock gate */
	memset(&init, 0, sizeof(init));
	init.name = si5324_input_names[0];
	init.ops = &si5324_xtal_ops;
	init.flags = 0;
	if (!IS_ERR(drvdata->pxtal)) {
		drvdata->pxtal_name = __clk_get_name(drvdata->pxtal);
		init.parent_names = &drvdata->pxtal_name;
		init.num_parents = 1;
	}
	drvdata->xtal.init = &init;
	clk = devm_clk_register(&client->dev, &drvdata->xtal);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n", init.name);
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	/* register clkin1 input clock gate */
	memset(&init, 0, sizeof(init));
	init.name = si5324_input_names[1];
	init.ops = &si5324_clkin_ops;
	if (!IS_ERR(drvdata->pclkin1)) {
		drvdata->pclkin1_name = __clk_get_name(drvdata->pclkin1);
		init.parent_names = &drvdata->pclkin1_name;
		init.num_parents = 1;
	}
	drvdata->clkin1.init = &init;
	clk = devm_clk_register(&client->dev, &drvdata->clkin1);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n",
			init.name);
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	/* register clkin2 input clock gate */
	memset(&init, 0, sizeof(init));
	init.name = si5324_input_names[2];
	init.ops = &si5324_clkin_ops;
	if (!IS_ERR(drvdata->pclkin2)) {
		drvdata->pclkin2_name = __clk_get_name(drvdata->pclkin2);
		init.parent_names = &drvdata->pclkin2_name;
		init.num_parents = 1;
	}
	drvdata->clkin2.init = &init;
	clk = devm_clk_register(&client->dev, &drvdata->clkin2);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n",
			init.name);
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	/* Si5324 allows to mux xtal or clkin1 or clkin2 to PLL input */
	num_parents = 3;
	parent_names[0] = si5324_input_names[0];
	parent_names[1] = si5324_input_names[1];
	parent_names[2] = si5324_input_names[2];

	/* register PLL */
	drvdata->pll.num = 0;
	drvdata->pll.drvdata = drvdata;
	drvdata->pll.hw.init = &init;
	memset(&init, 0, sizeof(init));
	init.name = si5324_pll_name;
	init.ops = &si5324_pll_ops;
	init.flags = 0;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	clk = devm_clk_register(&client->dev, &drvdata->pll.hw);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n", init.name);
		ret = PTR_ERR(clk);
		goto err_clk;
	}

	/* register clk multisync and clk out divider */
	num_clocks = 2;
	num_parents = 1;
	parent_names[0] = si5324_pll_name;

	drvdata->clkout = devm_kzalloc(&client->dev, num_clocks *
				       sizeof(*drvdata->clkout), GFP_KERNEL);

	drvdata->onecell.clk_num = num_clocks;
	drvdata->onecell.clks = devm_kzalloc(&client->dev,
		num_clocks * sizeof(*drvdata->onecell.clks), GFP_KERNEL);

	if (WARN_ON(!drvdata->clkout) || (!drvdata->onecell.clks)) {
		ret = -ENOMEM;
		goto err_clk;
	}

	for (n = 0; n < num_clocks; n++) {
		drvdata->clkout[n].num = n;
		drvdata->clkout[n].drvdata = drvdata;
		drvdata->clkout[n].hw.init = &init;
		memset(&init, 0, sizeof(init));
		init.name = si5324_clkout_names[n];
		init.ops = &si5324_clkout_ops;
		init.flags = 0;
		init.flags |= CLK_SET_RATE_PARENT;
		init.parent_names = parent_names;
		init.num_parents = num_parents;
		clk = devm_clk_register(&client->dev, &drvdata->clkout[n].hw);
		if (IS_ERR(clk)) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			ret = PTR_ERR(clk);
			goto err_clk;
		}
		/* refer to output clock in onecell */
		drvdata->onecell.clks[n] = clk;

		/* set initial clkout rate */
		if (pdata->clkout[n].rate != 0) {
			int ret;
			ret = clk_set_rate(clk, pdata->clkout[n].rate);
			if (ret != 0) {
				dev_err(&client->dev, "Cannot set rate : %d\n",
					ret);
			}
		}
	}

	ret = of_clk_add_provider(client->dev.of_node, of_clk_src_onecell_get,
				  &drvdata->onecell);
	if (ret) {
		dev_err(&client->dev, "unable to add clk provider\n");
		goto err_clk;
	}
	dev_info(&client->dev, "Initialized Si5324.\n");

	return 0;

err_clk:
	if (!IS_ERR(drvdata->pxtal))
		clk_disable_unprepare(drvdata->pxtal);
	if (!IS_ERR(drvdata->pclkin1))
		clk_disable_unprepare(drvdata->pclkin1);
	if (!IS_ERR(drvdata->pclkin2))
		clk_disable_unprepare(drvdata->pclkin2);
	return ret;
}

static const struct i2c_device_id si5324_i2c_ids[] = {
	{ "si5324", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5324_i2c_ids);

static struct i2c_driver si5324_driver = {
	.driver = {
		.name = "si5324",
		.of_match_table = of_match_ptr(si5324_dt_ids),
	},
	.probe = si5324_i2c_probe,
	.id_table = si5324_i2c_ids,
};
module_i2c_driver(si5324_driver);

MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_DESCRIPTION("Silicon Labs Si5324 jitter attenuating clock multiplier driver");
MODULE_LICENSE("GPL");
