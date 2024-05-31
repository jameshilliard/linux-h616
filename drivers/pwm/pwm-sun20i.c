// SPDX-License-Identifier: GPL-2.0
/*
 * PWM Controller Driver for sunxi platforms (D1, T113-S3, R329 and H616)
 *
 * Limitations:
 * - When the parameters change, current running period will not be completed
 *   and run new settings immediately.
 * - It output HIGH-Z state when PWM channel disabled.
 *
 * Copyright (c) 2023 Aleksandr Shubin <privatesub2@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define SUN20I_PWM_REG_OFFSET_PER_D1		(0x0080)
#define SUN20I_PWM_REG_OFFSET_PCR_D1		(0x0100 + 0x0000)
#define SUN20I_PWM_REG_OFFSET_PPR_D1		(0x0100 + 0x0004)
#define SUN20I_PWM_REG_OFFSET_PER_H616		(0x0040)
#define SUN20I_PWM_REG_OFFSET_PCR_H616		(0x0060 + 0x0000)
#define SUN20I_PWM_REG_OFFSET_PPR_H616		(0x0060 + 0x0004)

#define SUN20I_PWM_CLK_CFG(chan)		(0x20 + (((chan) >> 1) * 0x4))
#define SUN20I_PWM_CLK_CFG_SRC			GENMASK(8, 7)
#define SUN20I_PWM_CLK_CFG_BYPASS(chan)		BIT(5 + ((chan) & 1))
#define SUN20I_PWM_CLK_CFG_GATING		BIT(4)
#define SUN20I_PWM_CLK_CFG_DIV_M		GENMASK(3, 0)
#define SUN20I_PWM_CLK_DIV_M_MAX		8

#define SUN20I_PWM_CLK_GATE			0x40
#define SUN20I_PWM_CLK_GATE_BYPASS(chan)	BIT((chan) + 16)
#define SUN20I_PWM_CLK_GATE_GATING(chan)	BIT(chan)

#define SUN20I_PWM_ENABLE(chip)			((chip)->data->reg_per)
#define SUN20I_PWM_ENABLE_EN(chan)		BIT(chan)

#define SUN20I_PWM_CTL(chip, chan)		((chip)->data->reg_pcr + (chan) * 0x20)
#define SUN20I_PWM_CTL_ACT_STA			BIT(8)
#define SUN20I_PWM_CTL_PRESCAL_K		GENMASK(7, 0)
#define SUN20I_PWM_CTL_PRESCAL_K_MAX		field_max(SUN20I_PWM_CTL_PRESCAL_K)

#define SUN20I_PWM_PERIOD(chip, chan)		((chip)->data->reg_ppr + (chan) * 0x20)
#define SUN20I_PWM_PERIOD_ENTIRE_CYCLE		GENMASK(31, 16)
#define SUN20I_PWM_PERIOD_ACT_CYCLE		GENMASK(15, 0)

#define SUN20I_PWM_PCNTR_SIZE			BIT(16)

#define SUN20I_PWM_CLOCK_SRC_HOSC		(0)
#define SUN20I_PWM_CLOCK_SRC_APB		(1)
#define SUN20I_PWM_CLOCK_SRC_DEFAULT		SUN20I_PWM_CLOCK_SRC_HOSC
#define SUN20I_PWM_DIV_M_SHIFT_DEFAULT		(0)

#define SUN20I_PWM_CHANNELS_MAX			(16)
#define SUN20I_PWM_ENTIRE_CYCLE_MAX		(0xffff)

struct sun20i_pwm_data {
	unsigned long reg_per;
	unsigned long reg_pcr;
	unsigned long reg_ppr;
	bool has_pcgr;
};

struct sun20i_pwm_chip {
	struct clk *clk_bus, *clk_hosc, *clk_apb;
	struct reset_control *rst;
	struct pwm_chip chip;
	void __iomem *base;
	/* Mutex to protect pwm apply state */
	struct mutex mutex;
	const struct sun20i_pwm_data *data;

	u32 clk_src_reg[(SUN20I_PWM_CHANNELS_MAX + 1) / 2];
	u32 div_m_shift_reg[(SUN20I_PWM_CHANNELS_MAX + 1) / 2];
};

static inline struct sun20i_pwm_chip *to_sun20i_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 sun20i_pwm_readl(struct sun20i_pwm_chip *chip,
				   unsigned long offset)
{
	return readl(chip->base + offset);
}

static inline void sun20i_pwm_writel(struct sun20i_pwm_chip *chip,
				     u32 val, unsigned long offset)
{
	writel(val, chip->base + offset);
}

static int sun20i_pwm_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u32 ent_cycle, act_cycle;
	u16 prescale_k;
	u64 clk_rate, tmp;
	u8 div_m;
	u32 val;

	mutex_lock(&sun20i_chip->mutex);

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
	div_m = FIELD_GET(SUN20I_PWM_CLK_CFG_DIV_M, val);
	if (div_m > SUN20I_PWM_CLK_DIV_M_MAX)
		div_m = SUN20I_PWM_CLK_DIV_M_MAX;

	if (FIELD_GET(SUN20I_PWM_CLK_CFG_SRC, val) == 0)
		clk_rate = clk_get_rate(sun20i_chip->clk_hosc);
	else
		clk_rate = clk_get_rate(sun20i_chip->clk_apb);

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CTL(sun20i_chip, pwm->hwpwm));
	state->polarity = (SUN20I_PWM_CTL_ACT_STA & val) ?
			   PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;

	prescale_k = FIELD_GET(SUN20I_PWM_CTL_PRESCAL_K, val) + 1;

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_ENABLE(sun20i_chip));
	state->enabled = (SUN20I_PWM_ENABLE_EN(pwm->hwpwm) & val) ? true : false;

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_PERIOD(sun20i_chip, pwm->hwpwm));

	mutex_unlock(&sun20i_chip->mutex);

	act_cycle = FIELD_GET(SUN20I_PWM_PERIOD_ACT_CYCLE, val);
	ent_cycle = FIELD_GET(SUN20I_PWM_PERIOD_ENTIRE_CYCLE, val) + 1;

	/*
	 * The duration of the active phase should not be longer
	 * than the duration of the period
	 */
	if (act_cycle > ent_cycle)
		act_cycle = ent_cycle;

	/*
	 * We have act_cycle <= ent_cycle <= 0xffff, prescale_k <= 0x100,
	 * div_m <= 8. So the multiplication fits into an u64 without
	 * overflow.
	 */
	tmp = ((u64)(act_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_UP_ULL(tmp, clk_rate);
	tmp = ((u64)(ent_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->period = DIV_ROUND_UP_ULL(tmp, clk_rate);

	return 0;
}

static int sun20i_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u64 bus_rate, hosc_rate, ent_cycle, act_cycle;
	u32 clk_gate, clk_cfg, pwm_en, ctl, reg_period, clk_rate;
	u32 prescale_k, div_m, div_m_shift;
	bool use_bus_clk;
	int ret = 0;

	mutex_lock(&sun20i_chip->mutex);

	pwm_en = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_ENABLE(sun20i_chip));

	if (state->enabled != pwm->state.enabled && !state->enabled) {
		if (sun20i_chip->data->has_pcgr) {
			/* Disabling the gate via PWM Clock Gating Register */
			clk_gate = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_GATE);
			clk_gate &= ~SUN20I_PWM_CLK_GATE_GATING(pwm->hwpwm);
			sun20i_pwm_writel(sun20i_chip, clk_gate, SUN20I_PWM_CLK_GATE);
		} else if (!(pwm_en & SUN20I_PWM_ENABLE_EN(pwm->hwpwm ^ 1))) {
			/*
			 * Disabling the gate via PWM Clock Configuration Register
			 * if and only if the counterpart channel is disabled
			 */
			clk_cfg = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
			clk_cfg &= ~SUN20I_PWM_CLK_CFG_GATING;
			sun20i_pwm_writel(sun20i_chip, clk_cfg, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
		}

		pwm_en &= ~SUN20I_PWM_ENABLE_EN(pwm->hwpwm);
		sun20i_pwm_writel(sun20i_chip, pwm_en, sun20i_chip->data->reg_per);
	}

	if (state->polarity != pwm->state.polarity ||
	    state->duty_cycle != pwm->state.duty_cycle ||
	    state->period != pwm->state.period) {
		int idx = pwm->hwpwm / 2;

		hosc_rate = clk_get_rate(sun20i_chip->clk_hosc);
		bus_rate = clk_get_rate(sun20i_chip->clk_apb);

		use_bus_clk = sun20i_chip->clk_src_reg[idx] == SUN20I_PWM_CLOCK_SRC_APB;
		clk_rate = use_bus_clk ? bus_rate : hosc_rate;
		div_m_shift = sun20i_chip->div_m_shift_reg[idx];
		div_m = 1 << div_m_shift;

		if (state->period > U64_MAX / clk_rate || state->duty_cycle > state->period) {
			ret = -EINVAL;
			goto unlock_mutex;
		}
		ent_cycle = DIV_ROUND_CLOSEST(state->period * clk_rate, NSEC_PER_SEC * div_m);
		act_cycle =
			min(DIV_ROUND_CLOSEST(state->duty_cycle * clk_rate, NSEC_PER_SEC * div_m),
			    ent_cycle);
		if (ent_cycle == 0 ||
		    ent_cycle > SUN20I_PWM_ENTIRE_CYCLE_MAX * SUN20I_PWM_CTL_PRESCAL_K_MAX) {
			ret = -EINVAL;
			goto unlock_mutex;
		}
		prescale_k = clamp(DIV_ROUND_UP_ULL(ent_cycle, SUN20I_PWM_ENTIRE_CYCLE_MAX), 1,
				   SUN20I_PWM_CTL_PRESCAL_K_MAX);
		ent_cycle = clamp(DIV_ROUND_CLOSEST_ULL(ent_cycle, prescale_k), 1,
				  SUN20I_PWM_ENTIRE_CYCLE_MAX);
		act_cycle = clamp(DIV_ROUND_CLOSEST_ULL(act_cycle, prescale_k), 0, ent_cycle);

		clk_cfg = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
		clk_cfg &= ~(SUN20I_PWM_CLK_CFG_DIV_M | SUN20I_PWM_CLK_CFG_SRC);
		clk_cfg |= FIELD_PREP(SUN20I_PWM_CLK_CFG_DIV_M, div_m_shift);
		clk_cfg |= FIELD_PREP(SUN20I_PWM_CLK_CFG_SRC, use_bus_clk);
		sun20i_pwm_writel(sun20i_chip, clk_cfg, SUN20I_PWM_CLK_CFG(pwm->hwpwm));

		reg_period = FIELD_PREP(SUN20I_PWM_PERIOD_ENTIRE_CYCLE, ent_cycle - 1);
		reg_period |= FIELD_PREP(SUN20I_PWM_PERIOD_ACT_CYCLE, act_cycle);
		sun20i_pwm_writel(sun20i_chip, reg_period,
			SUN20I_PWM_PERIOD(sun20i_chip, pwm->hwpwm));

		ctl = FIELD_PREP(SUN20I_PWM_CTL_PRESCAL_K, prescale_k - 1);
		if (state->polarity == PWM_POLARITY_NORMAL)
			ctl |= SUN20I_PWM_CTL_ACT_STA;
		sun20i_pwm_writel(sun20i_chip, ctl, SUN20I_PWM_CTL(sun20i_chip, pwm->hwpwm));
	}

	if (state->enabled != pwm->state.enabled && state->enabled) {
		if (sun20i_chip->data->has_pcgr) {
			/* Enabling the gate via PWM Clock Gating Register */
			clk_gate = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_GATE);
			clk_gate &= ~SUN20I_PWM_CLK_GATE_BYPASS(pwm->hwpwm);
			clk_gate |= SUN20I_PWM_CLK_GATE_GATING(pwm->hwpwm);
			sun20i_pwm_writel(sun20i_chip, clk_gate, SUN20I_PWM_CLK_GATE);
		} else {
			/* Enabling the gate via PWM Clock Configuration Register */
			clk_cfg = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
			clk_cfg &= ~SUN20I_PWM_CLK_CFG_BYPASS(pwm->hwpwm);
			clk_cfg |= SUN20I_PWM_CLK_CFG_GATING;
			sun20i_pwm_writel(sun20i_chip, clk_cfg, SUN20I_PWM_CLK_CFG(pwm->hwpwm));
		}

		pwm_en |= SUN20I_PWM_ENABLE_EN(pwm->hwpwm);
		sun20i_pwm_writel(sun20i_chip, pwm_en, SUN20I_PWM_ENABLE(sun20i_chip));
	}

unlock_mutex:
	mutex_unlock(&sun20i_chip->mutex);

	return ret;
}

static const struct pwm_ops sun20i_pwm_ops = {
	.apply = sun20i_pwm_apply,
	.get_state = sun20i_pwm_get_state,
};

static const struct sun20i_pwm_data sun20i_d1_pwm_data = {
	.reg_per = SUN20I_PWM_REG_OFFSET_PER_D1,
	.reg_pcr = SUN20I_PWM_REG_OFFSET_PCR_D1,
	.reg_ppr = SUN20I_PWM_REG_OFFSET_PPR_D1,
	.has_pcgr = true,
};

static const struct sun20i_pwm_data sun50i_h616_pwm_data = {
	.reg_per = SUN20I_PWM_REG_OFFSET_PER_H616,
	.reg_pcr = SUN20I_PWM_REG_OFFSET_PCR_H616,
	.reg_ppr = SUN20I_PWM_REG_OFFSET_PPR_H616,
	.has_pcgr = false,
};

static const struct of_device_id sun20i_pwm_dt_ids[] = {
	{
		.compatible = "allwinner,sun20i-d1-pwm",
		.data = &sun20i_d1_pwm_data
	},
	{
		.compatible = "allwinner,sun50i-h616-pwm",
		.data = &sun50i_h616_pwm_data
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sun20i_pwm_dt_ids);

static void sun20i_pwm_reset_ctrl_release(void *data)
{
	struct reset_control *rst = data;

	reset_control_assert(rst);
}

static int sun20i_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct sun20i_pwm_chip *sun20i_chip;
	const struct sun20i_pwm_data *data;
	u32 npwm;
	int ret;

	data = of_device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	ret = of_property_read_u32(pdev->dev.of_node, "allwinner,pwm-channels", &npwm);
	if (ret)
		npwm = 8;

	if (npwm > SUN20I_PWM_CHANNELS_MAX) {
		dev_info(&pdev->dev, "Limiting number of PWM lines from %u to %u", npwm,
			 SUN20I_PWM_CHANNELS_MAX);
		npwm = SUN20I_PWM_CHANNELS_MAX;
	}

	chip = devm_pwmchip_alloc(&pdev->dev, npwm, sizeof(*sun20i_chip));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	sun20i_chip = to_sun20i_pwm_chip(chip);

	sun20i_chip->data = data;

	sun20i_chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sun20i_chip->base))
		return PTR_ERR(sun20i_chip->base);

	sun20i_chip->data = data;

	sun20i_chip->clk_bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(sun20i_chip->clk_bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_bus),
				     "failed to get bus clock\n");

	sun20i_chip->clk_hosc = devm_clk_get_enabled(&pdev->dev, "hosc");
	if (IS_ERR(sun20i_chip->clk_hosc))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_hosc),
				     "failed to get hosc clock\n");

	sun20i_chip->clk_apb = devm_clk_get_enabled(&pdev->dev, "apb");
	if (IS_ERR(sun20i_chip->clk_apb))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_apb),
				     "failed to get apb clock\n");

	sun20i_chip->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(sun20i_chip->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->rst),
				     "failed to get bus reset\n");

	for (int i = 0; i < (npwm + 1) / 2; i++) {
		const char *source;
		u32 div_m;

		sun20i_chip->clk_src_reg[i] = SUN20I_PWM_CLOCK_SRC_DEFAULT;
		sun20i_chip->div_m_shift_reg[i] = SUN20I_PWM_DIV_M_SHIFT_DEFAULT;

		ret = of_property_read_string_index(pdev->dev.of_node,
						    "allwinner,pwm-pair-clock-sources", i, &source);
		if (!ret) {
			if (!strcasecmp(source, "hosc"))
				sun20i_chip->clk_src_reg[i] = SUN20I_PWM_CLOCK_SRC_HOSC;
			else if (!strcasecmp(source, "apb"))
				sun20i_chip->clk_src_reg[i] = SUN20I_PWM_CLOCK_SRC_APB;
			else
				return dev_err_probe(&pdev->dev, -EINVAL,
						     "Unknown clock source: %s\n", source);
		}

		ret = of_property_read_u32_index(pdev->dev.of_node,
						 "allwinner,pwm-pair-clock-prescales", i, &div_m);
		if (!ret) {
			if (div_m <= SUN20I_PWM_CLK_DIV_M_MAX)
				sun20i_chip->div_m_shift_reg[i] = div_m;
			else
				return dev_err_probe(&pdev->dev, -EINVAL,
						     "Invalid prescale value: %u\n", div_m);
		}
	}

	/* Deassert reset */
	ret = reset_control_deassert(sun20i_chip->rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to deassert reset\n");

	ret = devm_add_action_or_reset(&pdev->dev, sun20i_pwm_reset_ctrl_release, sun20i_chip->rst);
	if (ret)
		return ret;

	chip->ops = &sun20i_pwm_ops;

	mutex_init(&sun20i_chip->mutex);

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "failed to add PWM chip\n");

	return 0;
}

static struct platform_driver sun20i_pwm_driver = {
	.driver = {
		.name = "sun20i-pwm",
		.of_match_table = sun20i_pwm_dt_ids,
	},
	.probe = sun20i_pwm_probe,
};
module_platform_driver(sun20i_pwm_driver);

MODULE_AUTHOR("Aleksandr Shubin <privatesub2@gmail.com>");
MODULE_AUTHOR("Hironori KIKUCHI <kikuchan98@gmail.com>");
MODULE_DESCRIPTION("Allwinner sun20i PWM driver");
MODULE_LICENSE("GPL");
