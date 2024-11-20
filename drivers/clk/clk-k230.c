// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Bo Wang <Juvenoia@...>
 * Author: Xukai Wang <kingxukai@zohomail.com>
 * Kendryte Canaan K230 Clock Drivers
 */
#define pr_fmt(fmt)     "K230-clk: " fmt

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of_platform.h>

#include <dt-bindings/clock/k230-clk.h>

/*
 * PLL control register bits.
 */
#define K230_PLL_BYPASS_ENABLE		(1 << 19)
#define K230_PLL_GATE_ENABLE		(1 << 2)
#define K230_PLL_GATE_WRITE_ENABLE	(1 << 18)
#define K230_PLL_OD_SHIFT		24
#define K230_PLL_OD_MASK		0xF
#define K230_PLL_R_SHIFT		16
#define K230_PLL_R_MASK			0x3F
#define K230_PLL_F_SHIFT		0
#define K230_PLL_F_MASK			0x1FFFF
#define K230_PLL0_OFFSET_BASE		0x00
#define K230_PLL1_OFFSET_BASE		0x10
#define K230_PLL2_OFFSET_BASE		0x20
#define K230_PLL3_OFFSET_BASE		0x30
#define K230_PLL_DIV_REG_OFFSET		0x00
#define K230_PLL_BYPASS_REG_OFFSET	0x04
#define K230_PLL_GATE_REG_OFFSET	0x08
#define K230_PLL_LOCK_REG_OFFSET	0x0C
/*
 * PLL lock register bits.
 */
#define K230_PLL_STATUS_MASK	(1 << 0)

/* K230 CLK MACROS */
#define K230_GATE_FORMAT(_reg, _bit, _reverse, _have_gate)                      \
	.gate_reg_off = (_reg),                                                 \
	.gate_bit_enable = (_bit),                                              \
	.gate_bit_reverse = (_reverse),                                         \
        .have_gate = (_have_gate)

#define K230_RATE_FORMAT(_mul_min, _mul_max, _mul_shift, _mul_mask,             \
                         _div_min, _div_max, _div_shift, _div_mask,             \
                         _reg, _bit, _method, _reg_c, _bit_c,                   \
                        _mul_min_c, _mul_max_c, _mul_shift_c, _mul_mask_c,      \
                        _have_rate, _have_rate_c)                               \
        .rate_mul_min = (_mul_min),                                             \
        .rate_mul_max = (_mul_max),                                             \
        .rate_mul_shift = (_mul_shift),                                         \
        .rate_mul_mask = (_mul_mask),                                           \
        .rate_mul_min_c = (_mul_min_c),                                         \
        .rate_mul_max_c = (_mul_max_c),                                         \
        .rate_mul_shift_c = (_mul_shift_c),                                     \
        .rate_mul_mask_c = (_mul_mask_c),                                       \
        .rate_div_min = (_div_min),                                             \
        .rate_div_max = (_div_max),                                             \
        .rate_div_shift = (_div_shift),                                         \
        .rate_div_mask = (_div_mask),                                           \
        .rate_reg_off = (_reg),                                                 \
        .rate_reg_off_c = (_reg_c),                                             \
        .rate_write_enable_bit = (_bit),                                        \
        .rate_write_enable_bit_c = (_bit_c),                                    \
        .method = (_method),                                                    \
        .have_rate = (_have_rate),                                              \
        .have_rate_c = (_have_rate_c)

#define K230_MUX_FORMAT(_reg, _shift, _mask, _have_mux)                         \
	.mux_reg_off = (_reg),                                                  \
	.mux_reg_shift = (_shift),                                              \
	.mux_reg_mask = (_mask),                                                \
        .have_mux = (_have_mux)

#define K230_GATE_FORMAT_ZERO K230_GATE_FORMAT(0, 0, 0, 0)
#define K230_RATE_FORMAT_ZERO K230_RATE_FORMAT(0, 0, 0, 0, 0, 0,                \
                                               0, 0, 0, 0, 0, 0,                \
                                               0, 0, 0, 0, 0, 0, 0)
#define K230_MUX_FORMAT_ZERO K230_MUX_FORMAT(0, 0, 0, 0)

struct K230_sysclk;

/* K230 PLLs. */
enum K230_pll_id {
	K230_PLL0, K230_PLL1, K230_PLL2, K230_PLL3, K230_PLL_NUM
};

struct K230_pll {
	enum K230_pll_id id;
	struct K230_sysclk *ksc;
	void __iomem *div, *bypass, *gate, *lock;
	struct clk_hw hw;
};
#define to_K230_pll(_hw)	container_of(_hw, struct K230_pll, hw)

struct K230_pll_cfg {
	u32 reg;
};

static struct K230_pll_cfg K230_plls_cfg[] = {
	{ K230_PLL0_OFFSET_BASE },
	{ K230_PLL1_OFFSET_BASE },
	{ K230_PLL2_OFFSET_BASE },
	{ K230_PLL3_OFFSET_BASE },
};

/* K230 PLL_DIVS. */
enum K230_pll_div_type {
	K230_PLL0_DIV2,
	K230_PLL0_DIV3,
	K230_PLL0_DIV4,
	K230_PLL1_DIV2,
	K230_PLL1_DIV3,
	K230_PLL1_DIV4,
	K230_PLL2_DIV2,
	K230_PLL2_DIV3,
	K230_PLL2_DIV4,
	K230_PLL3_DIV2,
	K230_PLL3_DIV3,
	K230_PLL3_DIV4,
	K230_PLL_DIV_NUM,
};

struct K230_pll_div {
	struct K230_sysclk *ksc;
	struct clk_hw *hw;
};

struct K230_pll_div_configs {
	const char *parent_name, *name;
	int div;
};

static struct K230_pll_div_configs K230_pll_div_cfgs[K230_PLL_DIV_NUM] = {
	[K230_PLL0_DIV2] = { "pll0", "pll0_div2", 2},
	[K230_PLL0_DIV3] = { "pll0", "pll0_div3", 3},
	[K230_PLL0_DIV4] = { "pll0", "pll0_div4", 4},
	[K230_PLL1_DIV2] = { "pll1", "pll1_div2", 2},
	[K230_PLL1_DIV3] = { "pll1", "pll1_div3", 3},
	[K230_PLL1_DIV4] = { "pll1", "pll1_div4", 4},
	[K230_PLL2_DIV2] = { "pll2", "pll2_div2", 2},
	[K230_PLL2_DIV3] = { "pll2", "pll2_div3", 3},
	[K230_PLL2_DIV4] = { "pll2", "pll2_div4", 4},
	[K230_PLL3_DIV2] = { "pll3", "pll3_div2", 2},
	[K230_PLL3_DIV3] = { "pll3", "pll3_div3", 3},
	[K230_PLL3_DIV4] = { "pll3", "pll3_div4", 4},
};

/* K230 CLK registers offset */
#define K230_CLK_AUDIO_CLKDIV_OFFSET 0x34
#define K230_CLK_PDM_CLKDIV_OFFSET 0x40
#define K230_CLK_CODEC_ADC_MCLKDIV_OFFSET 0x38
#define K230_CLK_CODEC_DAC_MCLKDIV_OFFSET 0x3c

/* K230 CLKS. */

struct K230_clk {
	int id;
	struct K230_sysclk *ksc;
	struct clk_hw hw;
};
#define to_K230_clk(_hw)	container_of(_hw, struct K230_clk, hw)

enum K230_clk_div_type {
	K230_MUL,
	K230_DIV,
	K230_MUL_DIV,
};

struct K230_clk_cfg {
/* attr */
	const char *name;
        bool read_only;         /* 0-read & write; 1-only read */

/* rate */
        /* info */
        u32 rate_reg_off;
        u32 rate_reg_off_c;
        u32 rate_write_enable_bit;
        u32 rate_write_enable_bit_c;
        enum K230_clk_div_type method;
        bool have_rate;
        bool have_rate_c;
        /* mul */
        u32 rate_mul_min;
        u32 rate_mul_max;
        u32 rate_mul_shift;
        u32 rate_mul_mask;
        /* mul-changable */
        u32 rate_mul_min_c;
        u32 rate_mul_max_c;
        u32 rate_mul_shift_c;
        u32 rate_mul_mask_c;
        /* div */
        u32 rate_div_min;
        u32 rate_div_max;
        u32 rate_div_shift;
        u32 rate_div_mask;
        /* reg */
        void __iomem *rate_reg;
        void __iomem *rate_reg_c;

/* gate */
        bool have_gate;
        u32 gate_reg_off;
        u32 gate_bit_enable;
        u32 gate_bit_reverse;
        /* reg */
        void __iomem *gate_reg;

/* mux */
        bool have_mux;
        u32 mux_reg_off; 
        u32 mux_reg_shift; 
        u32 mux_reg_mask;
        /* reg */
        void __iomem *mux_reg;
};

/* SRC: K230-SDK-DTS,
 * SEE: /src/little/linux/arch/riscv/boot/dts/kendryte/clock-provider.dtsi
 */
static struct K230_clk_cfg K230_clk_cfgs[K230_NUM_CLKS] = {
        [K230_CPU0_SRC] = {
                .name = "cpu0_src",
                .read_only = 0,
                K230_RATE_FORMAT(1, 16, 0, 0,
                                 16, 16, 1, 0xF,
                                 0x0, 31, K230_MUL, 0, 0,
                                 0, 0, 0, 0,
                                 1, 0),
                K230_GATE_FORMAT(0, 0, 0, 1),
                K230_MUX_FORMAT_ZERO,
        },
        [K230_CPU0_ACLK] = {
                .name = "cpu0_aclk",
                .read_only = 0,
                K230_RATE_FORMAT(1, 1, 0, 0,
                                 1, 8, 7, 0x7,
                                 0x0, 31, K230_MUL, 0, 0,
                                 0, 0, 0, 0,
                                 1, 0),
                K230_GATE_FORMAT_ZERO,
                K230_MUX_FORMAT_ZERO,
        },
        [K230_CPU0_PLIC] = {
                .name = "cpu0_plic",
                .read_only = 0,
                K230_RATE_FORMAT(1, 1, 0, 0,
                                 1, 8, 10, 0x7,
                                 0x0, 31, K230_DIV, 0, 0,
                                 0, 0, 0, 0,
                                 1, 0),
                K230_GATE_FORMAT(0, 9, 0, 1),
                K230_MUX_FORMAT_ZERO,
        },
        [K230_CPU0_NOC_DDRCP4] = {
                .name = "cpu0_noc_ddrcp4",
                .read_only = 0,
                K230_RATE_FORMAT_ZERO,
                K230_GATE_FORMAT(0x60, 7, 0, 1),
                K230_MUX_FORMAT_ZERO,
        },
        [K230_CPU0_PCLK] = {
                .name = "cpu0_pclk",
                .read_only = 0,
                K230_RATE_FORMAT(1, 1, 0, 0,
                                 1, 8, 15, 0x7,
                                 0x0, 31, K230_DIV, 0, 0,
                                 0, 0, 0, 0,
                                 1, 0),
                K230_GATE_FORMAT(0, 13, 0, 1),
                K230_MUX_FORMAT_ZERO,
        },
};

struct K230_sysclk {
	void __iomem			*pll_regs, *regs;
	spinlock_t			pll_lock, clk_lock;
	struct K230_pll			plls[K230_PLL_NUM];
	struct K230_clk			clks[K230_NUM_CLKS];
	struct K230_pll_div		dclks[K230_PLL_DIV_NUM];
} clksrc;

static void K230_init_pll(void __iomem *regs, enum K230_pll_id pllid,
			  struct K230_pll *pll)
{
	void __iomem *base;

	pll->id = pllid;
	base = regs + K230_plls_cfg[pllid].reg;
	pll->div = base + K230_PLL_DIV_REG_OFFSET;
	pll->bypass = base + K230_PLL_BYPASS_REG_OFFSET;
	pll->gate = base + K230_PLL_GATE_REG_OFFSET;
	pll->lock = base + K230_PLL_LOCK_REG_OFFSET;
}

#if 0
static void K230_pll_wait_for_lock(struct K230_pll *pll)
{
	u32 reg, mask = K230_PLL_STATUS_MASK;

	while (true) {
		reg = readl(pll->lock);
		if ((reg & mask) == mask)
			break;

		reg ^= mask;
		writel(reg, pll->lock);
	}
}
#endif /* static void K230_pll_wait_for_lock */

static bool K230_pll_hw_is_enabled(struct K230_pll *pll)
{
	u32 reg = readl(pll->gate);
	u32 mask = K230_PLL_GATE_ENABLE;

	return (reg & mask) == mask;
}

static void K230_pll_enable_hw(void __iomem *regs, struct K230_pll *pll)
{
	u32 reg;

	if (K230_pll_hw_is_enabled(pll))
		return;

	/* Set PLL factors */
	reg = readl(pll->gate);
	reg |= (K230_PLL_GATE_ENABLE | K230_PLL_GATE_WRITE_ENABLE);
	writel(reg, pll->gate);
}

static int K230_pll_enable(struct clk_hw *hw)
{
	struct K230_pll *pll = to_K230_pll(hw);
	struct K230_sysclk *ksc = pll->ksc;
	unsigned long flags;

	spin_lock_irqsave(&ksc->pll_lock, flags);

	K230_pll_enable_hw(ksc->regs, pll);

	spin_unlock_irqrestore(&ksc->pll_lock, flags);

	return 0;
}

static void K230_pll_disable(struct clk_hw *hw)
{

	struct K230_pll *pll = to_K230_pll(hw);
	struct K230_sysclk *ksc = pll->ksc;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ksc->pll_lock, flags);

	reg = readl(pll->gate);

	reg &= ~(K230_PLL_GATE_ENABLE);
	reg |= (K230_PLL_GATE_WRITE_ENABLE);

	writel(reg, pll->gate);

	spin_unlock_irqrestore(&ksc->pll_lock, flags);
}

static int K230_pll_is_enabled(struct clk_hw *hw)
{
	return K230_pll_hw_is_enabled(to_K230_pll(hw));
}

static unsigned long K230_pll_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct K230_pll *pll = to_K230_pll(hw);
	u32 reg = readl(pll->bypass);
	u32 r, f, od;

	if (reg & K230_PLL_BYPASS_ENABLE)
		return parent_rate;

	reg = readl(pll->lock);
	if (reg & (K230_PLL_LOCK_REG_OFFSET)) /* now locked */
		return 0;
	reg = readl(pll->div);

	r = ((reg >> K230_PLL_R_SHIFT) & K230_PLL_R_MASK) + 1;
	f = ((reg >> K230_PLL_F_SHIFT) & K230_PLL_F_MASK) + 1;
	od = ((reg >> K230_PLL_OD_SHIFT) & K230_PLL_OD_MASK) + 1;

	return div_u64((u64)parent_rate * f, r * od);
}

static const struct clk_ops K230_pll_ops = {
        /* gate */
	.enable	= K230_pll_enable,
	.disable	= K230_pll_disable,
	.is_enabled	= K230_pll_is_enabled,
        /* rate */
	.recalc_rate	= K230_pll_get_rate,
};

static int K230_register_pll(struct device_node *np,
				    struct K230_sysclk *ksc,
				    enum K230_pll_id pllid, const char *name,
				    int num_parents, const struct clk_ops *ops)
{
	struct K230_pll *pll = &ksc->plls[pllid];
	struct clk_init_data init = {};
	const struct clk_parent_data parent_data[] = {
		{ /* .index = 0 for osc24m */ }, /* pll0-3 are son of osc24m. */
	};

	init.name = name;
	init.parent_data = parent_data;
	init.num_parents = num_parents;
	init.ops = ops;

	pll->hw.init = &init;
	pll->ksc = ksc;

	return of_clk_hw_register(np, &pll->hw);
}

static int K230_register_plls(struct device_node *np,
				     struct K230_sysclk *ksc)
{
	int i, ret;

	for (i = 0; i < K230_PLL_NUM; i++)
		K230_init_pll(ksc->pll_regs, i, &ksc->plls[i]);

	ret = K230_register_pll(np, ksc, K230_PLL0, "pll0", 1, &K230_pll_ops);

	if (ret) {
		pr_err("%pOFP: register PLL0 failed\n", np);
		return ret;
	}

	ret = K230_register_pll(np, ksc, K230_PLL1, "pll1", 1, &K230_pll_ops);

	if (ret) {
		pr_err("%pOFP: register PLL1 failed\n", np);
		return ret;
	}
	ret = K230_register_pll(np, ksc, K230_PLL2, "pll2", 1, &K230_pll_ops);

	if (ret) {
		pr_err("%pOFP: register PLL2 failed\n", np);
		return ret;
	}
	ret = K230_register_pll(np, ksc, K230_PLL3, "pll3", 1, &K230_pll_ops);

	if (ret) {
		pr_err("%pOFP: register PLL3 failed\n", np);
		return ret;
	}

        return 0;
}

static int K230_register_pll_divs(struct device_node *np,
					 struct K230_sysclk *ksc)
{
	for (int i = 0; i < K230_PLL_DIV_NUM; i++) {
		ksc->dclks[i].hw = clk_hw_register_fixed_factor(NULL, K230_pll_div_cfgs[i].name,
								K230_pll_div_cfgs[i].parent_name,
								0, 1, K230_pll_div_cfgs[i].div);
		ksc->dclks[i].ksc = ksc;
	}
	return 0;
}

static int K230_clk_enable(struct clk_hw *hw)
{
	struct K230_clk *kclk = to_K230_clk(hw);
	struct K230_sysclk *ksc = kclk->ksc;
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[kclk->id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ksc->clk_lock, flags);
	reg = readl(cfg->gate_reg);
	if (cfg->gate_bit_reverse)
		reg &= ~BIT(cfg->gate_bit_enable);
	else
		reg |= BIT(cfg->gate_bit_enable);
	writel(reg, cfg->gate_reg);
	spin_unlock_irqrestore(&ksc->clk_lock, flags);

	return 0;
}

static void K230_clk_disable(struct clk_hw *hw)
{
	struct K230_clk *kclk = to_K230_clk(hw);
	struct K230_sysclk *ksc = kclk->ksc;
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[kclk->id];
	unsigned long flags;
	u32 reg;

	if (!(cfg->have_gate) && (!cfg->gate_reg)) {
                pr_err("gate_reg is not set\n");
		return;
        }

	spin_lock_irqsave(&ksc->clk_lock, flags);
	reg = readl(cfg->gate_reg);

	if (cfg->gate_bit_reverse)
		reg |= BIT(cfg->gate_bit_enable);
	else
		reg &= ~BIT(cfg->gate_bit_enable);

	writel(reg, cfg->gate_reg);
	spin_unlock_irqrestore(&ksc->clk_lock, flags);
}

static int K230_clk_is_enabled(struct clk_hw *hw)
{
        struct K230_clk *kclk = to_K230_clk(hw);
        struct K230_sysclk *ksc = kclk->ksc;
        struct K230_clk_cfg *cfg = &K230_clk_cfgs[kclk->id];
        unsigned long flags;
        u32 reg;
        int ret;

        if ((!cfg->have_gate) && (!cfg->gate_reg)) {
                pr_err("gate_reg is not set\n");
                return 0;
        }

	spin_lock_irqsave(&ksc->clk_lock, flags);
	reg = readl(cfg->gate_reg);

	if (cfg->gate_bit_reverse) {
                if (BIT(cfg->gate_bit_enable) & reg)
                        ret = 1;
                else
                        ret = 0;
        }
	else {
                if (BIT(cfg->gate_bit_enable) & (~reg) )
                        ret = 1;
                else
                        ret = 0;
        }

	spin_unlock_irqrestore(&ksc->clk_lock, flags);

        return ret;
}

static int K230_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct K230_clk *kclk = to_K230_clk(hw);
	struct K230_sysclk *ksc = kclk->ksc;
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[kclk->id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ksc->clk_lock, flags);
	reg = (cfg->mux_reg_mask & index) << cfg->mux_reg_shift;
	writel(reg, cfg->mux_reg);
	spin_unlock_irqrestore(&ksc->clk_lock, flags);

	return 0;
}

static u8 K230_clk_get_parent(struct clk_hw *hw)
{
	struct K230_clk *kclk = to_K230_clk(hw);
	struct K230_sysclk *ksc = kclk->ksc;
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[kclk->id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&ksc->clk_lock, flags);
	reg = readl(cfg->mux_reg);
	spin_unlock_irqrestore(&ksc->clk_lock, flags);

	return reg;
}

static unsigned long K230_clk_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct K230_clk *clk = to_K230_clk(hw);
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[clk->id];
	u32 mul, div;
        pr_info("%d:%s", clk->id, __func__);

	if (!cfg->have_rate) /* no divider, return parents' clk */
		return parent_rate;

	switch (cfg->method) {
	        case K230_MUL:
	        	mul = (readl(cfg->rate_reg) >> cfg->rate_div_shift) & cfg->rate_div_mask;
	        	mul++;
	        	div = cfg->rate_div_max;
	        	break;
	        case K230_DIV:
	        	mul = cfg->rate_mul_max;
	        	div = (readl(cfg->rate_reg) >> cfg->rate_div_shift) & cfg->rate_div_mask;
	        	div++;
	        	break;
	        case K230_MUL_DIV:
	        	if (!cfg->rate_reg_off_c) {
	        		mul = (readl(cfg->rate_reg) >> cfg->rate_mul_shift) & cfg->rate_mul_mask;
	        		div = (readl(cfg->rate_reg) >> cfg->rate_div_shift) & cfg->rate_div_mask;
	        	} else {
	        		mul = (readl(cfg->rate_reg_c) >> cfg->rate_mul_shift_c) & cfg->rate_mul_mask_c;
	        		div = (readl(cfg->rate_reg_c) >> cfg->rate_div_shift) & cfg->rate_div_mask;
	        	}
	        	break;
	        default:
	        	return 0;
	}

        pr_info("the rate is: parent_rate:%lu mul:%lu div:%lu rate:%lu", parent_rate, mul, div, div_u64((u64)parent_rate * mul, div));
	return div_u64((u64)parent_rate * mul, div);
}

static long K230_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
        return (long)rate;
}

static int k230_clk_find_approximate(struct K230_clk *clk, u32 mul_min, u32 mul_max,
				       u32 div_min, u32 div_max, u8 method,
				       unsigned long rate, unsigned long parent_rate,
				       u32 *div, u32 *mul)
{
	long abs_min;
	long abs_current;
	/* we used interger to instead of float */
	long perfect_divide;

	struct K230_clk_cfg *cfg = &K230_clk_cfgs[clk->id];

	volatile uint32_t codec_clk[9] = {
		2048000,
		3072000,
		4096000,
		6144000,
		8192000,
		11289600,
		12288000,
		24576000,
		49152000
	};
	volatile uint32_t codec_div[9][2] = {
		{3125, 16},
		{3125, 24},
		{3125, 32},
		{3125, 48},
		{3125, 64},
		{15625, 441},
		{3125, 96},
		{3125, 192},
		{3125, 384}
	};

	volatile uint32_t pdm_clk[20] = {
		128000,
		192000,
		256000,
		384000,
		512000,
		768000,
		1024000,
		1411200,
		1536000,
		2048000,
		2822400,
		3072000,
		4096000,
		5644800,
		6144000,
		8192000,
		11289600,
		12288000,
		24576000,
		49152000
	};
	volatile uint32_t pdm_div[20][2] = {
		{3125, 1},
		{6250, 3},
		{3125, 2},
		{3125, 3},
		{3125, 4},
		{3125, 6},
		{3125, 8},
		{125000, 441},
		{3125, 12},
		{3125, 16},
		{62500, 441},
		{3125, 24},
		{3125, 32},
		{31250, 441},
		{3125, 48},
		{3125, 64},
		{15625, 441},
		{3125, 96},
		{3125, 192},
		{3125, 384}
	};

	switch (method) {
	    /* only mul can be changeable 1/12,2/12,3/12...*/
		case 0:
			perfect_divide = (long)((parent_rate * 1000) / rate);
			abs_min = abs(perfect_divide -
					(long)(((long)div_max * 1000) / (long)mul_min));

			for (u32 i = mul_min + 1; i <= mul_max; i++) {
				abs_current = abs(perfect_divide -
						(long)((long)((long)div_max * 1000) / (long)i));
				if (abs_min > abs_current) {
					abs_min = abs_current;
					*mul = i;
				}
			}

			*div = div_min;
			break;
		/* only div can be changeable, 1/1,1/2,1/3...*/
		case 1:
			perfect_divide = (long)((parent_rate * 1000) / rate);
			abs_min = abs(perfect_divide -
					(long)(((long)div_min * 1000) / (long)mul_max));
			*div = div_min;

			for (u32 i = div_min + 1; i <= div_max; i++) {
				abs_current = abs(perfect_divide -
						(long)((long)((long)i * 1000) / (long)mul_max));
				if (abs_min > abs_current) {
					abs_min = abs_current;
					*div = i;
				}
			}

			*mul = mul_min;
			break;
		/* mul and div can be changeable. */
		case 2:
			if (K230_CLK_CODEC_ADC_MCLKDIV_OFFSET == cfg->rate_reg_off || \
			    K230_CLK_CODEC_DAC_MCLKDIV_OFFSET == cfg->rate_reg_off) {
				for (u32 j = 0; j < 9; j++) {
					if (0 == (rate - codec_clk[j])) {
						*div = codec_div[j][0];
						*mul = codec_div[j][1];
					}
				}
                        } else if (K230_CLK_AUDIO_CLKDIV_OFFSET == cfg->rate_reg_off || \
                                   K230_CLK_PDM_CLKDIV_OFFSET == cfg->rate_reg_off) {
                                for (u32 j = 0; j < 20; j++) {
					if (0 == (rate - pdm_clk[j])) {
						*div = pdm_div[j][0];
						*mul = pdm_div[j][1];
					}
				}
                        } else
				return -1;
			break;

		default:
			pr_err("%s method error!\n", __func__);
			return -1;

	}
	return 0;
}

static int K230_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct K230_clk *clk = to_K230_clk(hw);
	struct K230_clk_cfg *cfg = &K230_clk_cfgs[clk->id];
        pr_info("%d:%s %lu %lu", clk->id, __func__, rate, parent_rate);

	u32 div = 0, mul = 0, reg = 0, reg_c = 0;

	if ((!cfg->have_rate) || (!cfg->rate_reg)) {
                pr_err("This clock may have no rate");
		return -1;
        }

	if ((rate > parent_rate) || (rate == 0) || (parent_rate == 0)) {
                pr_err("rate or parent_rate error");
		return -1;
        }

        if (cfg->read_only == 1){
                pr_err("This clk rate is read only");
                return -1;
        }

	if (k230_clk_find_approximate(clk,
                                      cfg->rate_mul_min, cfg->rate_mul_max,
                                      cfg->rate_div_min, cfg->rate_div_max,
                                      cfg->method, rate, parent_rate, &div, &mul)) {
		pr_err("[%s]: clk %s set rate error!\n", __func__, clk_hw_get_name(hw));
		return -1;
	}

	if (!cfg->rate_reg_off_c) {
		reg = readl(cfg->rate_reg);
		reg &= ~((cfg->rate_div_mask) << (cfg->rate_div_shift));

		if (cfg->method == K230_DIV) {
			reg &= ~((cfg->rate_mul_mask) << (cfg->rate_mul_shift));
		}
		reg |= (1 << cfg->rate_write_enable_bit);
		 
		if (cfg->method == K230_MUL_DIV)
			reg |= ((div - 1) & cfg->rate_div_mask) << (cfg->rate_div_shift);
		else if (cfg->method == K230_MUL)
			reg |= ((mul - 1) & cfg->rate_div_mask) << (cfg->rate_div_shift);
		else {
			reg |= (mul & cfg->rate_mul_mask) << (cfg->rate_mul_shift);
			reg |= (div & cfg->rate_div_mask) << (cfg->rate_div_shift);
		}
		writel(reg, cfg->rate_reg);
        } else { 
		reg = readl(cfg->rate_reg);
		reg_c = readl(cfg->rate_reg_c);
		reg &= ~((cfg->rate_div_mask) << (cfg->rate_div_shift));
		reg_c &= ~((cfg->rate_mul_mask_c) << (cfg->rate_mul_shift_c));
		reg_c |= (1 << cfg->rate_write_enable_bit_c);

		reg_c |= (mul & cfg->rate_mul_mask_c) << (cfg->rate_mul_shift_c);
		reg |= (div & cfg->rate_div_mask) << (cfg->rate_div_shift);

		writel(reg, cfg->rate_reg);
		writel(reg_c, cfg->rate_reg_c);
	}

	return 0;
}

static const struct clk_ops K230_clk_mux_ops = {
        /* gate */
	.enable		= K230_clk_enable,
	.disable	= K230_clk_disable,
        .is_enabled     = K230_clk_is_enabled,
        /* rate */
	.set_rate       = K230_clk_set_rate,
	.round_rate     = K230_clk_round_rate,
	.recalc_rate	= K230_clk_get_rate,
        /* mux */
	.set_parent	= K230_clk_set_parent,
	.get_parent	= K230_clk_get_parent,
};

static const struct clk_ops K230_clk_ops = {
        /* gate */
	.enable		= K230_clk_enable,
	.disable	= K230_clk_disable,
        .is_enabled     = K230_clk_is_enabled,
        /* rate */
	.set_rate       = K230_clk_set_rate,
	.round_rate     = K230_clk_round_rate,
	.recalc_rate	= K230_clk_get_rate,
};

static const struct clk_ops K230_clk_gate_ops = {
        /* gate */
	.enable		= K230_clk_enable,
	.disable	= K230_clk_disable,
        .is_enabled     = K230_clk_is_enabled,
};

static void K230_register_clk(struct device_node *np,
				     struct K230_sysclk *ksc, int id,
				     const struct clk_parent_data *parent_data,
				     int num_parents, unsigned long flags)
{

	struct K230_clk *kclk = &ksc->clks[id];
        struct K230_clk_cfg *cfg = &K230_clk_cfgs[id];
	struct clk_init_data init = {};
	int ret;

        if (cfg->have_rate)
                cfg->rate_reg = (void __iomem*)((unsigned long)clksrc.regs + (unsigned long)cfg->rate_reg_off);
        if (cfg->have_rate_c)
                cfg->rate_reg_c = (void __iomem*)((unsigned long)clksrc.regs + (unsigned long)cfg->rate_reg_off_c);
        if (cfg->have_mux)
                cfg->mux_reg = (void __iomem*)((unsigned long)clksrc.regs + (unsigned long)cfg->mux_reg_off);
        if (cfg->have_gate)
                cfg->gate_reg = (void __iomem*)((unsigned long)clksrc.regs + (unsigned long)cfg->gate_reg_off);

	init.name = K230_clk_cfgs[id].name;
	init.flags = flags;
	init.parent_data = parent_data;
	init.num_parents = num_parents;
	if (num_parents > 1)
		init.ops = &K230_clk_mux_ops;
	else {
                if ((cfg->have_gate) && !(cfg->have_rate)) /* only gate */
                        init.ops = &K230_clk_gate_ops;
                else
		        init.ops = &K230_clk_ops;
        }


	kclk->id = id;
	kclk->ksc = ksc;
	kclk->hw.init = &init;

	ret = of_clk_hw_register(np, &kclk->hw);

	if (ret) {
		pr_err("%pOFP: register clock %s failed\n",
		       np, K230_clk_cfgs[id].name);
		kclk->id = -1;
	}
}

static inline void K230_register_mux_clk(struct device_node *np,
						struct K230_sysclk *ksc,
						struct clk_hw *hw1, struct clk_hw *hw2,
						int id)
{
	const struct clk_parent_data parent_data[] = {
		{ .hw = hw1 },
		{ .hw = hw2 }
	};
	K230_register_clk(np, ksc, id, parent_data, 2, 0);
}

static inline void K230_register_osc24m_child(struct device_node *np,
						  struct K230_sysclk *ksc, int id)
{
	const struct clk_parent_data parent_data = {
		/* .index = 0 for osc24m.. */
	};
	K230_register_clk(np, ksc, id, &parent_data, 1, 0);
}

static inline void K230_register_pll_child(struct device_node *np,
						  struct K230_sysclk *ksc, int id,
						  enum K230_pll_id pllid,
						  unsigned long flags)
{
	const struct clk_parent_data parent_data = {
		.hw = &ksc->plls[pllid].hw,
	};

	K230_register_clk(np, ksc, id, &parent_data, 1, flags);
}

static inline void K230_register_pll_div_child(struct device_node *np,
						      struct K230_sysclk *ksc, int id,
						      enum K230_pll_div_type pll_div_type,
						      unsigned long flags)
{
	const struct clk_parent_data parent_data = {
		.hw = ksc->dclks[pll_div_type].hw,
	};
	K230_register_clk(np, ksc, id, &parent_data, 1, flags);
}

static inline void K230_register_clk_child(struct device_node *np,
						  struct K230_sysclk *ksc, int id,
						  int parent_id)
{
	const struct clk_parent_data parent_data = {
		.hw = &ksc->clks[parent_id].hw,
	};

	K230_register_clk(np, ksc, id, &parent_data, 1, 0);
}

static struct clk_hw *K230_clk_hw_onecell_get(struct of_phandle_args *clkspec,
					      void *data)
{
	struct K230_sysclk *ksc;
	unsigned int idx;

        if (clkspec->args_count != 1)
                return ERR_PTR(-EINVAL);
        idx = clkspec->args[0];

        if (!data)
                return ERR_PTR(-EINVAL);

        ksc = (struct K230_sysclk *)data;

	if (idx >= K230_NUM_CLKS)
		return ERR_PTR(-EINVAL);

	return &ksc->clks[idx].hw;
}

static void __init K230_clk_init_plls(struct device_node *np)
{
	/* plls are the very son of osc24m, and reg are from sysctl_boot */
	struct K230_sysclk *ksc = &clksrc;
	int ret;

        pr_info("%s\n", __func__);

	spin_lock_init(&ksc->pll_lock);

	ksc->pll_regs = of_iomap(np, 0);
	if (!ksc->pll_regs) {
		pr_err("%pOFP: failed to map registers\n", np);
		return;
	}

	ret = K230_register_plls(np, ksc);
	if (ret) {
                pr_err("K230 register plls error\n");
		return;
        }

	/* Registration for all pll_divs */
	ret = K230_register_pll_divs(np, ksc);
        if (ret) {
                pr_err("K230 register pll divs error\n");
                return;
        }
}
CLK_OF_DECLARE_DRIVER(k230_pll, "canaan,k230-pll", K230_clk_init_plls);

static void __init K230_clk_init_clks(struct device_node *np)
{
	struct K230_sysclk *ksc = &clksrc;
	int ret;

        pr_info("%s", __func__);

	spin_lock_init(&ksc->clk_lock);

	ksc->regs = of_iomap(np, 0);
	if (!ksc->regs) {
		pr_err("%pOFP: failed to map registers\n", np);
		return;
	}

	/* Register for pll0_div2 sons: CPU0_SRC, */
	K230_register_pll_div_child(np, ksc, K230_CPU0_SRC, K230_PLL0_DIV2, 0);

	/* Register for CPU0_SRC sons: CPU0_ACLK, CPU0_PLIC, CPU0_NOC_DDRCP4 */
	K230_register_clk_child(np, ksc, K230_CPU0_ACLK, K230_CPU0_SRC);
	K230_register_clk_child(np, ksc, K230_CPU0_PLIC, K230_CPU0_SRC);
	K230_register_clk_child(np, ksc, K230_CPU0_NOC_DDRCP4, K230_CPU0_SRC);

	/* Register for pll0_div4 sons: CPU0_PCLK*/
        K230_register_pll_div_child(np, ksc, K230_CPU0_PCLK, K230_PLL0_DIV4, 0);

	ret = of_clk_add_hw_provider(np, K230_clk_hw_onecell_get, ksc);
	if (ret) {
		pr_err("%pOFP: add clock provider failed %d\n", np, ret);
		return;
	}
}
CLK_OF_DECLARE_DRIVER(k230_clk, "canaan,k230-clk-composite", K230_clk_init_clks);

/* test part */

static int test_driver_probe(struct platform_device *pdev)
{
        int ret = 0;
        unsigned long rate = 0;

        printk("%s\n", __func__) ;
        struct device* dev = &pdev->dev;
        struct clk* clk = devm_clk_get(dev, "test_node");
        if (IS_ERR(clk)) {
                dev_err(dev, "failed to find clock provider");
                ret = PTR_ERR(clk);
                goto probe_out_free_dev;
        }

        ret = clk_prepare_enable(clk);
        if (ret){
                dev_err(dev, "failed to prepare_enable clk");
                goto probe_out_free_dev;
        }

        ret = clk_set_rate(clk, 30000);

        rate = clk_get_rate(clk);
        pr_info("get the rate:%lu", rate);
        goto out;

probe_out_free_dev:

out:
        return ret;
}

static void test_driver_remove(struct platform_device *pdev)
{
        printk("%s\n", __func__) ;
}

static const struct of_device_id test_match_table[] = {
        { .compatible = "canaan,test-node",},
        {}
};
MODULE_DEVICE_TABLE(of, test_match_table);

static struct platform_driver test_driver = {
        .driver = {
                .name = "test_driver",
                .of_match_table = test_match_table,
        },
        .probe = test_driver_probe,
        .remove = test_driver_remove,
};

static int __init test_driver_init(void)
{
        printk("%s\n", __func__) ;
        return platform_driver_register(&test_driver);
}

static void __exit test_driver_exit(void)
{
        printk("%s\n", __func__) ;
        platform_driver_unregister(&test_driver);
}

module_init(test_driver_init);
module_exit(test_driver_exit);

MODULE_LICENSE("GPL");
