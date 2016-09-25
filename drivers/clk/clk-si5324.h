/*
 * clk-si5324.h: Silicon Laboratories Si5324A/B/C I2C Clock Generator
 *
 * Leon Woestenberg <leon@sidebranch.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _CLK_SI5324_H_
#define _CLK_SI5324_H_

#define SI5324_BUS_BASE_ADDR			0x68

#define SI5324_REG0			0
#define SI5324_REG0_FREE_RUN			(1<<7)

#define SI5324_CKSEL 3

#define SI5324_DSBL_CLKOUT 10

#define SI5324_POWERDOWN		11
#define SI5324_PD_CK1 (1<<0)
#define SI5324_PD_CK2 (1<<1)

/* output clock dividers */
#define SI5324_N1_HS_OUTPUT_DIVIDER 25
#define SI5324_NC1_LS_H 31
#define SI5324_NC1_LS_M 32
#define SI5324_NC1_LS_L 33

#define SI5324_NC2_LS_H 34
#define SI5324_NC2_LS_M 35
#define SI5324_NC2_LS_L 36

#define SI5324_RESET 136
#define SI5324_RST_REG (1<<7)

/* selects 2kHz to 710 MHz */
#define SI5324_CLKIN_MIN_FREQ			2000
#define SI5324_CLKIN_MAX_FREQ			(710 * 1000 * 1000)


/* generates 2kHz to 945 MHz */
#define SI5324_CLKOUT_MIN_FREQ			2000
#define SI5324_CLKOUT_MAX_FREQ			(945 * 1000 * 1000)

/**
 * The following constants define the limits of the divider settings.
 */
#define SI5324_N1_HS_MIN  6        /**< Minimum N1_HS setting (4 and 5 are for higher output frequencies than we support */
#define SI5324_N1_HS_MAX 11        /**< Maximum N1_HS setting */
#define SI5324_NC_LS_MIN  1        /**< Minimum NCn_LS setting (1 and even values) */
#define SI5324_NC_LS_MAX 0x100000  /**< Maximum NCn_LS setting (1 and even values) */
#define SI5324_N2_HS_MIN  4        /**< Minimum NC2_HS setting */
#define SI5324_N2_HS_MAX 11        /**< Maximum NC2_HS setting */
#define SI5324_N2_LS_MIN  2        /**< Minimum NC2_LS setting (even values only) */
#define SI5324_N2_LS_MAX 0x100000  /**< Maximum NC2_LS setting (even values only) */
#define SI5324_N3_MIN     1        /**< Minimum N3n setting */
#define SI5324_N3_MAX    0x080000  /**< Maximum N3n setting */

/* 5351 legacy */
#if 0
#define SI5324_PLL_MIN			15
#define SI5324_PLL_MAX			90

#define SI5324_INTERRUPT_STATUS			1
#define SI5324_INTERRUPT_MASK			2
#define  SI5324_STATUS_SYS_INIT			(1<<7)
#define  SI5324_STATUS_LOL_B			(1<<6)
#define  SI5324_STATUS_LOL_A			(1<<5)
#define  SI5324_STATUS_LOS			(1<<4)
#define SI5324_OUTPUT_ENABLE_CTRL		3
#define SI5324_OEB_PIN_ENABLE_CTRL		9
#define SI5324_PLL_INPUT_SOURCE			15
#define  SI5324_CLKIN_DIV_MASK			(3<<6)
#define  SI5324_CLKIN_DIV_1			(0<<6)
#define  SI5324_CLKIN_DIV_2			(1<<6)
#define  SI5324_CLKIN_DIV_4			(2<<6)
#define  SI5324_CLKIN_DIV_8			(3<<6)
#define  SI5324_PLLB_SOURCE			(1<<3)
#define  SI5324_PLLA_SOURCE			(1<<2)

#define SI5324_CLK0_CTRL			16
#define SI5324_CLK1_CTRL			17
#define SI5324_CLK2_CTRL			18
#define SI5324_CLK3_CTRL			19
#define SI5324_CLK4_CTRL			20
#define SI5324_CLK5_CTRL			21
#define SI5324_CLK6_CTRL			22
#define SI5324_CLK7_CTRL			23
#define  SI5324_CLK_POWERDOWN			(1<<7)
#define  SI5324_CLK_INTEGER_MODE		(1<<6)
#define  SI5324_CLK_PLL_SELECT			(1<<5)
#define  SI5324_CLK_INVERT			(1<<4)
#define  SI5324_CLK_INPUT_MASK			(3<<2)
#define  SI5324_CLK_INPUT_XTAL			(0<<2)
#define  SI5324_CLK_INPUT_CLKIN			(1<<2)
#define  SI5324_CLK_INPUT_MULTISYNTH_0_4	(2<<2)
#define  SI5324_CLK_INPUT_MULTISYNTH_N		(3<<2)
#define  SI5324_CLK_DRIVE_STRENGTH_MASK		(3<<0)
#define  SI5324_CLK_DRIVE_STRENGTH_2MA		(0<<0)
#define  SI5324_CLK_DRIVE_STRENGTH_4MA		(1<<0)
#define  SI5324_CLK_DRIVE_STRENGTH_6MA		(2<<0)
#define  SI5324_CLK_DRIVE_STRENGTH_8MA		(3<<0)

#define SI5324_CLK3_0_DISABLE_STATE		24
#define SI5324_CLK7_4_DISABLE_STATE		25
#define  SI5324_CLK_DISABLE_STATE_MASK		3
#define  SI5324_CLK_DISABLE_STATE_LOW		0
#define  SI5324_CLK_DISABLE_STATE_HIGH		1
#define  SI5324_CLK_DISABLE_STATE_FLOAT		2
#define  SI5324_CLK_DISABLE_STATE_NEVER		3

#endif

#if 0
#define SI5324_PLLA_PARAMETERS			26
#define SI5324_PLLB_PARAMETERS			34
#define SI5324_CLK0_PARAMETERS			42
#define SI5324_CLK1_PARAMETERS			50
#define SI5324_CLK2_PARAMETERS			58
#define SI5324_CLK3_PARAMETERS			66
#define SI5324_CLK4_PARAMETERS			74
#define SI5324_CLK5_PARAMETERS			82
#define SI5324_CLK6_PARAMETERS			90
#define SI5324_CLK7_PARAMETERS			91
#define SI5324_CLK6_7_OUTPUT_DIVIDER		92
#define  SI5324_OUTPUT_CLK_DIV_MASK		(7 << 4)
#define  SI5324_OUTPUT_CLK6_DIV_MASK		(7 << 0)
#define  SI5324_OUTPUT_CLK_DIV_SHIFT		4
#define  SI5324_OUTPUT_CLK_DIV6_SHIFT		0
#define  SI5324_OUTPUT_CLK_DIV_1		0
#define  SI5324_OUTPUT_CLK_DIV_2		1
#define  SI5324_OUTPUT_CLK_DIV_4		2
#define  SI5324_OUTPUT_CLK_DIV_8		3
#define  SI5324_OUTPUT_CLK_DIV_16		4
#define  SI5324_OUTPUT_CLK_DIV_32		5
#define  SI5324_OUTPUT_CLK_DIV_64		6
#define  SI5324_OUTPUT_CLK_DIV_128		7
#define  SI5324_OUTPUT_CLK_DIVBY4		(3<<2)

#define SI5324_SSC_PARAM0			149
#define SI5324_SSC_PARAM1			150
#define SI5324_SSC_PARAM2			151
#define SI5324_SSC_PARAM3			152
#define SI5324_SSC_PARAM4			153
#define SI5324_SSC_PARAM5			154
#define SI5324_SSC_PARAM6			155
#define SI5324_SSC_PARAM7			156
#define SI5324_SSC_PARAM8			157
#define SI5324_SSC_PARAM9			158
#define SI5324_SSC_PARAM10			159
#define SI5324_SSC_PARAM11			160
#define SI5324_SSC_PARAM12			161

#define SI5324_VXCO_PARAMETERS_LOW		162
#define SI5324_VXCO_PARAMETERS_MID		163
#define SI5324_VXCO_PARAMETERS_HIGH		164

#define SI5324_CLK0_PHASE_OFFSET		165
#define SI5324_CLK1_PHASE_OFFSET		166
#define SI5324_CLK2_PHASE_OFFSET		167
#define SI5324_CLK3_PHASE_OFFSET		168
#define SI5324_CLK4_PHASE_OFFSET		169
#define SI5324_CLK5_PHASE_OFFSET		170

#define SI5324_PLL_RESET			177
#define  SI5324_PLL_RESET_B			(1<<7)
#define  SI5324_PLL_RESET_A			(1<<5)

#define SI5324_CRYSTAL_LOAD			183
#define  SI5324_CRYSTAL_LOAD_MASK		(3<<6)
#define  SI5324_CRYSTAL_LOAD_6PF		(1<<6)
#define  SI5324_CRYSTAL_LOAD_8PF		(2<<6)
#define  SI5324_CRYSTAL_LOAD_10PF		(3<<6)

#define SI5324_FANOUT_ENABLE			187
#define  SI5324_CLKIN_ENABLE			(1<<7)
#define  SI5324_XTAL_ENABLE			(1<<6)
#define  SI5324_MULTISYNTH_ENABLE		(1<<4)

#endif

#endif
