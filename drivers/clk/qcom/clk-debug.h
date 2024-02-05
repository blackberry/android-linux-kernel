/*
 * Copyright (c) 2014, 2016-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QCOM_CLK_DEBUG_H__
#define __QCOM_CLK_DEBUG_H__

#include "../clk.h"

/* Debugfs Measure Clocks */

/**
 * struct measure_clk_data - Structure of clk measure
 *
 * @cxo:		XO clock.
 * @xo_div4_cbcr:	offset of debug XO/4 div register.
 * @ctl_reg:		offset of debug control register.
 * @status_reg:		offset of debug status register.
 *
 */
struct measure_clk_data {
	struct clk *cxo;
	u32 xo_div4_cbcr;
	u32 ctl_reg;
	u32 status_reg;
};

/**
 * List of Debug clock controllers.
 */
enum debug_cc {
	GCC,
	MMCC,
	GPU,
	CPU,
};

/**
 * struct clk_src - Struture of clock source for debug mux
 *
 * @parents:	clock name to be used as parent for debug mux.
 * @sel:	debug mux index at global clock controller.
 * @dbg_cc:     indicates the clock controller for recursive debug clock
 *		controllers.
 * @next_sel:	indicates the debug mux index at recursive debug mux.
 * @mask:	indicates the mask required at recursive debug mux.
 * @shift:	indicates the shift required at recursive debug mux.
 * @en_mask:	indicates the enable bit mask at recursive debug mux.
 *		Incase the recursive debug mux does not have a enable bit,
 *		0xFF should be used to indicate the same, otherwise global
 *		enable bit would be used.
 * @post_div_mask: indicates the post div mask to be used at debug/recursive
 *		   debug mux.
 * @post_div_val: indicates the post div value to be used at debug/recursive
 *		  debug mux.
 */
struct clk_src {
	const char  *parents;
	int sel;
	enum debug_cc dbg_cc;
	int next_sel;
	u32 mask;
	u32 shift;
	u32 en_mask;
	u32 post_div_mask;
	u32 post_div_val;
};

#define MUX_SRC_LIST(...) \
	.parent = (struct clk_src[]){__VA_ARGS__}, \
	.num_parents = ARRAY_SIZE(((struct clk_src[]){__VA_ARGS__}))

/**
 * struct clk_debug_mux - Struture of clock debug mux
 *
 * @parent:		structure of clk_src
 * @num_parents:	number of parents
 * @regmap:		regmaps of debug mux
 * @num_parent_regmap:	number of regmap of debug mux
 * @priv:		private measure_clk_data to be used by debug mux
 * @en_mask:		indicates the enable bit mask at global clock
 *			controller debug mux.
 * @mask:		indicates the mask to be used at global clock
 *			controller debug mux.
 * @debug_offset:	debug mux offset.
 * @hw:			handle between common and hardware-specific interfaces.
 * @multiplier:		internally used by debug mux as post div multiplier.
 */
struct clk_debug_mux {
	struct clk_src *parent;
	int num_parents;
	struct regmap **regmap;
	int num_parent_regmap;
	void *priv;
	u32 en_mask;
	u32 mask;
	u32 debug_offset;
	struct clk_hw hw;

	/* internal */
	u32 multiplier;
};

#define to_clk_measure(_hw) container_of((_hw), struct clk_debug_mux, hw)

extern const struct clk_ops clk_debug_mux_ops;

int clk_register_debug(struct clk_hw *hw);
int clk_debug_measure_add(struct clk_hw *hw, struct dentry *dentry);

#endif
