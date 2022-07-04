/*
 *
 * Author: Cao Rongrong <rrcao@ambarella.com>
 *
 * Copyright (C) 2012-2016, Ambarella, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/rational.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

/*
 * The RCT doc said max vco frequency is 1.8GHz, but there is no
 * much margine left when vco frequency is 1.8GHz, so we limit max
 * vco frequency to 1.6GHz.
 */
#define MAX_VCO_FREQ				1600000000ULL
#define REF_CLK_FREQ				24000000ULL

union ctrl_reg_u {
	struct {
		u32 write_enable		: 1;	/* [0] */
		u32 reserved1			: 1;	/* [1] */
		u32 bypass			: 1;	/* [2] */
		u32 frac_mode			: 1;	/* [3] */
		u32 force_reset			: 1;	/* [4] */
		u32 power_down			: 1;	/* [5] */
		u32 halt_vco			: 1;	/* [6] */
		u32 tristate			: 1;	/* [7] */
		u32 tout_async			: 4;	/* [11:8] */
		u32 sdiv			: 4;	/* [15:12] */
		u32 sout			: 4;	/* [19:16] */
		u32 force_lock			: 1;	/* [20] */
		u32 force_bypass		: 1;	/* [21] */
		u32 reserved2			: 2;	/* [23:22] */
		u32 intp			: 7;	/* [30:24] */
		u32 reserved3			: 1;	/* [31] */
	} s;

	u32 w;
};

union frac_reg_u {
	struct {
		u32 frac			: 31;	/* [30:0] */
		u32 nega			: 1;	/* [31] */
	} s;

	u32 w;
};

union ctrl2_reg_u {
	struct {
		u32 vco_div			: 4;	/* [3:0] */
		u32 fsdiv			: 4;	/* [7:4] */
		u32 fsout			: 4;	/* [11:8] */
		u32 bypass_hs_div		: 1;	/* [12] */
		u32 bypass_mdiv			: 1;	/* [13] */
		u32 diff_vco_en			: 1;	/* [14] */
		u32 duty_cycle_tune		: 1;	/* [15] */
		u32 charge_pump_cur		: 8;	/* [23:16] */
		u32 reserved1			: 4;	/* [27:24] */
		u32 change_pump_dc_bias		: 2;	/* [29:28] */
		u32 dsm_mode_ctrl		: 2;	/* [31:30] */
	} s;

	u32 w;
};

union ctrl3_reg_u {
	struct {
		u32 reserved1			: 1;	/* [0] */
		u32 pll_vco_range		: 2;	/* [2:1] */
		u32 pll_vco_clamp		: 2;	/* [4:3] */
		u32 reserved2			: 2;	/* [6:5] */
		u32 dsm_dither_gain		: 2;	/* [8:7] */
		u32 reserved3			: 4;	/* [12:9] */
		u32 ff_zero_resistor_sel	: 4;	/* [16:13] */
		u32 bias_current_ctrl		: 3;	/* [19:17] */
		u32 bypass_jdiv			: 1;	/* [20] */
		u32 bypass_jout			: 1;	/* [21] */
		u32 reserved4			: 10;	/* [31:22] */
	} s;

	u32 w;
};

enum {
	PHANDLE_OFFSET = 0,
	CTRL_OFFSET,
	FRAC_OFFSET,
	CTRL2_OFFSET,
	CTRL3_OFFSET,
	PRES_OFFSET,
	POST_OFFSET,
	REG_NUM,
};

struct amb_clk_pll {
	struct clk_hw hw;
	void __iomem *ctrl_reg;
	void __iomem *frac_reg;
	void __iomem *ctrl2_reg;
	void __iomem *ctrl3_reg;
	void __iomem *pres_reg;
	void __iomem *post_reg;
	struct regmap *pll_regmap;
	u32 reg_offset[REG_NUM];
	u32 extra_pre_scaler : 1;
	u32 extra_post_scaler : 1;
	u32 frac_mode : 1;
	u32 frac_nega : 1;
	u32 ctrl2_val;
	u32 ctrl3_val;
	u32 fix_divider;
	unsigned long max_vco;
	struct device_node *np;
};

#define PLL_DTS_TABLE_SIZE 16
struct pll_from_dts {
	u32 pll_freq;
	u32 pre_scaler_val;
	u32 ctrl_val;
	u32 frac_val;
	u32 ctrl2_val;
	u32 ctrl3_val;
	u32 post_scaler_val;
};
static struct pll_from_dts pll_from_dts_table[PLL_DTS_TABLE_SIZE];

#define to_amb_clk_pll(_hw) container_of(_hw, struct amb_clk_pll, hw)
#define rct_writel_en(v, p)		\
		do {writel(v, p); writel((v | 0x1), p); writel(v, p);} while (0)
#define rct_regmap_en(r, o, v) 	do {		\
		regmap_write(r, o, v);		\
		regmap_write(r, o, v | 0x1);	\
		regmap_write(r, o, v);		\
	} while (0)

static void ambarella_pll_set_ctrl3(struct amb_clk_pll *clk_pll, unsigned long pre_scaler,
		unsigned long intp, unsigned long sdiv, unsigned long parent_rate, unsigned long rate)
{
	struct regmap *map = clk_pll->pll_regmap;
	u32 *reg = clk_pll->reg_offset;
	union ctrl3_reg_u ctrl3_val;
	unsigned long pllvco;

	// tmp hard code for HDMI 4kp60 clock
	if (rate == 5940000000ULL) {
		ctrl3_val.w = 0x88006;
		goto exit;
	}

	if (clk_pll->ctrl3_val != 0) {
		ctrl3_val.w =clk_pll->ctrl3_val;
		goto exit;
	}

	if (map)
		regmap_read(map, reg[CTRL3_OFFSET], &ctrl3_val.w);
	else
		ctrl3_val.w = readl(clk_pll->ctrl3_reg);

	if (clk_pll->frac_nega) {
		if (clk_pll->frac_mode)
			ctrl3_val.w |= (1 << 12);
		else
			ctrl3_val.w &= ~(1 << 12);
	} else {
		pllvco = parent_rate / pre_scaler * intp * sdiv;

		if (pllvco > 980000000ULL)
			ctrl3_val.s.pll_vco_range = 3;
		else if (pllvco > 700000000ULL)
			ctrl3_val.s.pll_vco_range = 2;
		else if (pllvco > 530000000ULL)
			ctrl3_val.s.pll_vco_range = 1;
		else
			ctrl3_val.s.pll_vco_range = 0;
	}
exit:
	if (map)
		regmap_write(map, reg[CTRL3_OFFSET], ctrl3_val.w);
	else
		writel(ctrl3_val.w, clk_pll->ctrl3_reg);
}

static void ambarella_pll_set_ctrl2(struct amb_clk_pll *clk_pll, unsigned long rate)
{
	struct regmap *map = clk_pll->pll_regmap;
	u32 *reg = clk_pll->reg_offset, ctrl2_val;

	// tmp hard code for HDMI 4kp60 clock
	if (rate == 5940000000ULL) {
		ctrl2_val = 0x30520040;
	} else if (clk_pll->ctrl2_val != 0) {
		ctrl2_val = clk_pll->ctrl2_val;
	} else {
		if (map)
			regmap_read(map, reg[CTRL2_OFFSET], &ctrl2_val);
		else
			ctrl2_val = readl(clk_pll->ctrl2_reg);
	}

	if (map)
		regmap_write(map, reg[CTRL2_OFFSET], ctrl2_val);
	else
		writel(ctrl2_val, clk_pll->ctrl2_reg);
}

static unsigned long ambarella_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct amb_clk_pll *clk_pll = to_amb_clk_pll(hw);
	struct regmap *map = clk_pll->pll_regmap;
	u32 *reg = clk_pll->reg_offset;
	u32 pre_scaler, post_scaler, intp, sdiv, sout;
	u32 ctrl2_8, ctrl2_9, ctrl2_11, ctrl2_12;
	u64 dividend, divider, frac;
	unsigned long rate_ref;
	union ctrl_reg_u ctrl_val;
	union ctrl2_reg_u ctrl2_val;
	union frac_reg_u frac_val;

	if (map) {
		regmap_read(map, reg[CTRL_OFFSET], &ctrl_val.w);
		regmap_read(map, reg[CTRL2_OFFSET], &ctrl2_val.w);
		regmap_read(map, reg[FRAC_OFFSET], &frac_val.w);
	} else {
		ctrl_val.w = readl(clk_pll->ctrl_reg);
		ctrl2_val.w = readl(clk_pll->ctrl2_reg);
		frac_val.w = readl(clk_pll->frac_reg);
	}

	if ((ctrl_val.s.power_down == 1) || (ctrl_val.s.halt_vco == 1))
		return 0;

	if (clk_pll->pres_reg != NULL || reg[PRES_OFFSET] != 0) {
		if (map)
			regmap_read(map, reg[PRES_OFFSET], &pre_scaler);
		else
			pre_scaler = readl(clk_pll->pres_reg);
		if (clk_pll->extra_pre_scaler) {
			pre_scaler >>= 4;
			pre_scaler++;
		}
	} else {
		pre_scaler = 1;
	}

	if (clk_pll->post_reg != NULL || reg[POST_OFFSET] != 0) {
		if (map)
			regmap_read(map, reg[POST_OFFSET], &post_scaler);
		else
			post_scaler = readl(clk_pll->post_reg);
		if (clk_pll->extra_post_scaler) {
			post_scaler >>= 4;
			post_scaler++;
		}
	} else {
		post_scaler = 1;
	}

	if (ctrl_val.s.bypass || ctrl_val.s.force_bypass)
		return parent_rate / pre_scaler / post_scaler;

	rate_ref = (ctrl2_val.s.fsdiv == 4) ? parent_rate * 2 : parent_rate;

	ctrl2_12 = ((ctrl2_val.w >> 12) & 0x1);
	ctrl2_11 = ((ctrl2_val.w >> 11) & 0x1) + 1;
	ctrl2_9 = ((ctrl2_val.w >> 9) & 0x1) + 1;
	ctrl2_8 = ((ctrl2_val.w >> 8) & 0x1) + 1;

	intp = ctrl_val.s.intp + 1;
	sdiv = ctrl_val.s.sdiv + 1;
	sout = ctrl_val.s.sout + 1;

	dividend = rate_ref;
	dividend *= (u64)intp;
	dividend *= (u64)sdiv;
	dividend *= (u64)ctrl2_8;
	dividend *= (u64)ctrl2_9;

	if (ctrl_val.s.frac_mode) {
		if (clk_pll->frac_nega) {
			if (frac_val.s.nega) {
				/* Negative */
				frac = 0x80000000 - frac_val.s.frac;
				frac = rate_ref * frac * sdiv;
				frac >>= 32;
				dividend = dividend - frac;
			} else {
				/* Positive */
				frac = frac_val.s.frac;
				frac = rate_ref * frac * sdiv;
				frac >>= 32;
				dividend = dividend + frac;
			}
		} else {
			frac = rate_ref * frac_val.w * sdiv;
			frac >>= 32;
			dividend = dividend + frac;
		}
	}

	if (ctrl2_12)
		divider = clk_pll->fix_divider;
	else
		divider = pre_scaler * sout * ctrl2_8 * ctrl2_11 * post_scaler * clk_pll->fix_divider;
	BUG_ON(divider == 0);

	do_div(dividend, divider);

	return dividend;
}

static long ambarella_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	unsigned int half_refclk = REF_CLK_FREQ / 2;

	if (to_amb_clk_pll(hw)->frac_mode)
		return rate;
	else
		return roundup(rate, half_refclk);
}

extern uint32_t amba_secure_clk_pll_set_rate(const char *name,
			unsigned long rate, unsigned long parent_rate);

static int ambarella_pll_find_pll_dts_table(struct device_node *np,
			unsigned long rate, struct pll_from_dts *pll_dts)
{
	int i = 0;
	int rval = -1;

	for (i = 0; i < PLL_DTS_TABLE_SIZE; i++) {
		if (pll_from_dts_table[i].pll_freq == (rate / 1000)){
			rval = 0;
			pll_dts->pre_scaler_val = pll_from_dts_table[i].pre_scaler_val;
			pll_dts->post_scaler_val = pll_from_dts_table[i].post_scaler_val;
			pll_dts->ctrl_val = pll_from_dts_table[i].ctrl_val;
			pll_dts->ctrl2_val = pll_from_dts_table[i].ctrl2_val;
			pll_dts->ctrl3_val = pll_from_dts_table[i].ctrl3_val;
			pll_dts->frac_val = pll_from_dts_table[i].frac_val;
		}
	}

	return rval;
}

static int ambarella_pll_set_from_dts(struct amb_clk_pll *clk_pll,
			char *prop_name, unsigned long rate)
{
	int rval = -1, num = 0;
	struct device_node *np = clk_pll->np;
	struct regmap *map = clk_pll->pll_regmap;
	struct pll_from_dts pll_dts;
	u32 *reg = clk_pll->reg_offset;

	if(of_find_property(np, prop_name, NULL)){
		num = of_property_count_elems_of_size(np, prop_name, sizeof(struct pll_from_dts));
		if (num) {
				rval = of_property_read_u32_array(np, prop_name,
					  (u32 *)pll_from_dts_table, num * sizeof(struct pll_from_dts) / sizeof(u32));
				if (rval) {
					pr_err("%s: failed to get pll dts table\n", np->name);
					goto END;
				} else {
					rval = ambarella_pll_find_pll_dts_table(np, rate, &pll_dts);
					if (rval)
						goto END;

					/* set pre_scaler */
					if (clk_pll->pres_reg){
						if (map) {
							if (clk_pll->extra_pre_scaler == 1)
								rct_regmap_en(map, reg[PRES_OFFSET], (pll_dts.pre_scaler_val - 1) << 4);
							else
								regmap_write(map, reg[PRES_OFFSET], pll_dts.pre_scaler_val);
						} else {
							if (clk_pll->extra_pre_scaler == 1)
								rct_writel_en((pll_dts.pre_scaler_val - 1) << 4, clk_pll->pres_reg);
							else
								writel(pll_dts.pre_scaler_val, clk_pll->pres_reg);
						}
					}

					/* set post_scaler */
					if (clk_pll->post_reg){
						if (map) {
							if (clk_pll->extra_post_scaler)
								rct_regmap_en(map, reg[POST_OFFSET], (pll_dts.post_scaler_val - 1) << 4);
							else
								regmap_write(map, reg[POST_OFFSET], pll_dts.post_scaler_val);
						} else {
							if (clk_pll->extra_post_scaler)
								rct_writel_en((pll_dts.post_scaler_val - 1) << 4, clk_pll->post_reg);
							else
								writel(pll_dts.post_scaler_val, clk_pll->post_reg);
						}
					}

					/* set ctrl reg */
					if (map)
						rct_regmap_en(map, reg[CTRL_OFFSET], pll_dts.ctrl_val);
					else
						rct_writel_en(pll_dts.ctrl_val, clk_pll->ctrl_reg);

					/* set frac reg */
					if (map)
						regmap_write(map, reg[FRAC_OFFSET], pll_dts.frac_val);
					else
						writel(pll_dts.frac_val, clk_pll->frac_reg);

					/* set ctrl2 reg */
					if (map)
						regmap_write(map, reg[CTRL2_OFFSET], pll_dts.ctrl2_val);
					else
						writel(pll_dts.ctrl2_val, clk_pll->ctrl2_reg);

					/* set ctrl3 reg */
					if (map)
						regmap_write(map, reg[CTRL3_OFFSET], pll_dts.ctrl3_val);
					else
						writel(pll_dts.ctrl3_val, clk_pll->ctrl3_reg);

					rval = 0;
				}
		}
	}

END:
	return rval;
}

static int ambarella_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct amb_clk_pll *clk_pll = to_amb_clk_pll(hw);
	struct regmap *map = clk_pll->pll_regmap;
	u32 *reg = clk_pll->reg_offset;
	unsigned long max_numerator, max_denominator;
	unsigned long rate_ref, rate_tmp, rate_resolution, pre_scaler = 1, post_scaler = 1;
	unsigned long intp, sdiv = 1, sout = 1, intp_tmp, sout_tmp;
	u64 dividend, divider, diff;
	union ctrl_reg_u ctrl_val;
	union frac_reg_u frac_val;

	if (amba_secure_clk_pll_set_rate(clk_hw_get_name(hw), rate, parent_rate))
		return 0;

	if (rate == 0) {
		if (map) {
			regmap_read(map, reg[CTRL_OFFSET], &ctrl_val.w);
			ctrl_val.s.power_down = 1;
			ctrl_val.s.halt_vco = 1;
			rct_regmap_en(map, reg[CTRL_OFFSET], ctrl_val.w);
		} else {
			ctrl_val.w = readl(clk_pll->ctrl_reg);
			ctrl_val.s.power_down = 1;
			ctrl_val.s.halt_vco = 1;
			rct_writel_en(ctrl_val.w, clk_pll->ctrl_reg);
		}
		return 0;
	}

	if (ambarella_pll_set_from_dts(clk_pll, "amb,set-from-dts", rate) == 0)
		return 0;

	rate *= clk_pll->fix_divider;

	/*
	 * The RCT doc said:
	 *   If VCO frequency exceeds 3GHz, use PLL_*_CTRL2_REG[2] = 1 and use
	 *   half the frequency setting in PLL_*_CTRL_REG".
	 */
	rate_ref = rate > 3000000000ULL ? parent_rate * 2 : parent_rate;

	if (rate < rate_ref && (clk_pll->post_reg != NULL || reg[POST_OFFSET] != 0)) {
		rate *= 16;
		post_scaler = 16;
	}

	if (rate < rate_ref) {
		pr_err("%s: Error: target rate is too slow: %ld!\n",
				clk_hw_get_name(hw), rate);
		return -EINVAL;
	}

	/*
	 * Non-HDMI fvco should be less than 1.6GHz and higher than 700MHz.
	 * HDMI fvco should be less than 5.5GHz, and much higher than 700MHz,
	 * probably higher than 2GHz, but not sure, need VLSI's confirmation.
	 *
	 * The pll_vco_range in CTRL3 register need to match the VCO frequency.
	 * Note: If the VCO frequency is larger than 1.6GHz, please take a look
	        at the CTRL2 and CTRL3 again(Especially the pll_vco_range field).
	 *
	 * In addition, for 10nm and later chips, the formula of PLL calculation
	 * is changed, there is no negative frac any more.
	 */
	rate_tmp = rate;
	max_numerator = min(128ULL, clk_pll->max_vco / REF_CLK_FREQ);
	max_denominator = 16;
	rational_best_approximation(rate_tmp, rate_ref, max_numerator, max_denominator,
				&intp, &sout);

	if (!clk_pll->frac_nega) {
		rate_resolution = rate_ref / post_scaler / 16;
		/*
		 * 10nm chips don't have negative fraction mode, so the
		 * calculated rate must be less than the required rate.
		 */
		while (rate_ref * intp * sdiv / sout > rate) {
			rate_tmp -= rate_resolution;
			rational_best_approximation(rate_tmp, rate_ref, max_numerator, max_denominator,
						&intp, &sout);
		}
	}

	intp_tmp = intp;
	sout_tmp = sout;

	while (rate_ref / 1000000 * intp * sdiv / pre_scaler < 700) {
		if (sout > 8 || intp > 64)
			break;
		intp += intp_tmp;
		sout += sout_tmp;
	}

	BUG_ON(intp > max_numerator || sout > max_denominator || sdiv > 16);
	BUG_ON(pre_scaler > 16 || post_scaler > 16);

	if (clk_pll->pres_reg != NULL || reg[PRES_OFFSET] != 0) {
		if (map) {
			if (clk_pll->extra_pre_scaler == 1)
				rct_regmap_en(map, reg[PRES_OFFSET], (pre_scaler - 1) << 4);
			else
				regmap_write(map, reg[PRES_OFFSET], pre_scaler);

		} else {
			if (clk_pll->extra_pre_scaler == 1)
				rct_writel_en((pre_scaler - 1) << 4, clk_pll->pres_reg);
			else
				writel(pre_scaler, clk_pll->pres_reg);
		}
	}

	if (clk_pll->post_reg != NULL || reg[POST_OFFSET] != 0) {
		if (map) {
			if (clk_pll->extra_post_scaler)
				rct_regmap_en(map, reg[POST_OFFSET], (post_scaler - 1) << 4);
			else
				regmap_write(map, reg[POST_OFFSET], post_scaler);
		} else {
			if (clk_pll->extra_post_scaler)
				rct_writel_en((post_scaler - 1) << 4, clk_pll->post_reg);
			else
				writel(post_scaler, clk_pll->post_reg);
		}
	}

	if (map) {
		regmap_read(map, reg[CTRL_OFFSET], &ctrl_val.w);
		if(ctrl_val.s.frac_mode) {
			ctrl_val.s.force_reset = 1;
			rct_regmap_en(map, reg[CTRL_OFFSET], ctrl_val.w);
		}
	} else {
		ctrl_val.w = readl(clk_pll->ctrl_reg);
		if(ctrl_val.s.frac_mode) {
			ctrl_val.s.force_reset = 1;
			rct_writel_en(ctrl_val.w, clk_pll->ctrl_reg);
		}
	}

	ctrl_val.s.intp = intp - 1;
	ctrl_val.s.sdiv = sdiv - 1;
	ctrl_val.s.sout = sout - 1;
	ctrl_val.s.bypass = 0;
	ctrl_val.s.frac_mode = 0;
	ctrl_val.s.force_reset = 0;
	ctrl_val.s.power_down = 0;
	ctrl_val.s.halt_vco = 0;
	ctrl_val.s.tristate = 0;
	ctrl_val.s.force_lock = 1;
	ctrl_val.s.force_bypass = 0;
	ctrl_val.s.write_enable = 0;
	if (map)
		rct_regmap_en(map, reg[CTRL_OFFSET], ctrl_val.w);
	else
		rct_writel_en(ctrl_val.w, clk_pll->ctrl_reg);

	ambarella_pll_set_ctrl2(clk_pll, rate);
	ambarella_pll_set_ctrl3(clk_pll, pre_scaler, intp, sdiv, rate_ref, rate);

	if (clk_pll->frac_mode) {
		rate_tmp = ambarella_pll_recalc_rate(hw, parent_rate);
		rate_tmp *= clk_pll->fix_divider * post_scaler;
		if (rate_tmp <= rate)
			diff = rate - rate_tmp;
		else
			diff = rate_tmp - rate;

		if (diff) {
			dividend = diff * pre_scaler * sout;
			dividend = dividend << 32;
			divider = (u64)sdiv * rate_ref;
			dividend = DIV_ROUND_CLOSEST_ULL(dividend, divider);
			if (clk_pll->frac_nega) {
				if (rate_tmp <= rate) {
					frac_val.s.nega	= 0;
					frac_val.s.frac	= dividend;
				} else {
					frac_val.s.nega	= 1;
					frac_val.s.frac	= 0x80000000 - dividend;
				}
			} else {
				frac_val.w = dividend;
			}
			if (map)
				regmap_write(map, reg[FRAC_OFFSET], frac_val.w);
			else
				writel(frac_val.w, clk_pll->frac_reg);

			ctrl_val.s.frac_mode = 1;
		}

		if (map)
			rct_regmap_en(map, reg[CTRL_OFFSET], ctrl_val.w);
		else
			rct_writel_en(ctrl_val.w, clk_pll->ctrl_reg);
	}

	/* check if result rate is precise or not */
	rate_tmp = ambarella_pll_recalc_rate(hw, parent_rate);
	if (abs(rate_tmp - rate / clk_pll->fix_divider / post_scaler) > 10) {
		pr_warn("[Warning] %s: request %ld, but got %ld\n",
			clk_hw_get_name(hw),
			rate / clk_pll->fix_divider / post_scaler, rate_tmp);
	}

	return 0;
}

static const struct clk_ops pll_ops = {
	.recalc_rate = ambarella_pll_recalc_rate,
	.round_rate = ambarella_pll_round_rate,
	.set_rate = ambarella_pll_set_rate,
};

static int ambarella_pll_get_reg(struct device_node *np,
		struct amb_clk_pll *clk_pll)
{
	clk_pll->ctrl_reg = of_iomap(np, 0);
	if (!clk_pll->ctrl_reg)
		return -ENXIO;

	clk_pll->frac_reg = of_iomap(np, 1);
	if (!clk_pll->frac_reg)
		return -ENXIO;

	clk_pll->ctrl2_reg = of_iomap(np, 2);
	if (!clk_pll->ctrl2_reg)
		return -ENXIO;

	clk_pll->ctrl3_reg = of_iomap(np, 3);
	if (!clk_pll->ctrl3_reg)
		return -ENXIO;

	/* pre and post scaler registers are allowed non-exist */
	clk_pll->pres_reg = of_iomap(np, 4);
	clk_pll->post_reg = of_iomap(np, 5);

	return 0;
}

static int ambarella_pll_get_regmap(struct device_node *np,
		struct amb_clk_pll *clk_pll)
{
	int rval;

	clk_pll->pll_regmap = syscon_regmap_lookup_by_phandle(np, "amb,clk-regmap");
	if (IS_ERR(clk_pll->pll_regmap)) {
		pr_err("%s: failed to get pll regmap\n", np->name);
		return -ENXIO;
	}

	rval = of_property_read_u32_array(np, "amb,clk-regmap",
			clk_pll->reg_offset, ARRAY_SIZE(clk_pll->reg_offset));
	if (rval < 0) {
		pr_err("%s: failed to get regs offset\n", np->name);
		return rval;
	}

	return 0;
}

static void __init ambarella_pll_clocks_init(struct device_node *np)
{
	struct amb_clk_pll *clk_pll;
	struct clk *clk;
	struct clk_init_data init;
	const char *name, *parent_name;
	u32 max_vco;
	int num_parents;

	num_parents = of_clk_get_parent_count(np);
	if (num_parents < 1) {
		pr_err("%s: no parent found\n", np->name);
		return;
	}

	clk_pll = kzalloc(sizeof(struct amb_clk_pll), GFP_KERNEL);
	if (!clk_pll)
		return;

	if (ambarella_pll_get_reg(np, clk_pll) < 0)
		ambarella_pll_get_regmap(np, clk_pll);

	if (of_property_read_string(np, "clock-output-names", &name))
		name = np->name;

	parent_name = of_clk_get_parent_name(np, 0);

	clk_pll->extra_pre_scaler = !!of_find_property(np, "amb,extra-pre-scaler", NULL);
	clk_pll->extra_post_scaler = !!of_find_property(np, "amb,extra-post-scaler", NULL);
	clk_pll->frac_mode = !!of_find_property(np, "amb,frac-mode", NULL);
	clk_pll->frac_nega = !!of_find_property(np, "amb,frac-nega", NULL);

	if (of_property_read_u32(np, "amb,ctrl2-val", &clk_pll->ctrl2_val))
		clk_pll->ctrl2_val = 0;

	if (of_property_read_u32(np, "amb,ctrl3-val", &clk_pll->ctrl3_val))
		clk_pll->ctrl3_val = 0;

	if (of_property_read_u32(np, "amb,fix-divider", &clk_pll->fix_divider))
		clk_pll->fix_divider = 1;

	if (of_property_read_u32(np, "amb,max-vco", &max_vco) == 0)
		clk_pll->max_vco = max_vco;
	else if (of_property_read_u32(np, "amb,max-vco-mhz", &max_vco) == 0)
		clk_pll->max_vco = (unsigned long)max_vco * 1000000ULL;
	else
		clk_pll->max_vco = MAX_VCO_FREQ;

	clk_pll->np = np;

	init.name = name;
	init.ops = &pll_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.parent_names = &parent_name;
	init.num_parents = num_parents;
	clk_pll->hw.init = &init;

	clk = clk_register(NULL, &clk_pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register %s pll clock (%ld)\n",
		       __func__, name, PTR_ERR(clk));
		kfree(clk_pll);
		return;
	}

	of_clk_add_provider(np, of_clk_src_simple_get, clk);
	clk_register_clkdev(clk, name, NULL);
}
CLK_OF_DECLARE(ambarella_clk_pll, "ambarella,pll-clock", ambarella_pll_clocks_init);

