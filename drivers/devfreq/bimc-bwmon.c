/*
 * Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "bimc-bwmon: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include "governor_bw_hwmon.h"

#define GLB_INT_STATUS(m)	((m)->global_base + 0x100)
#define GLB_INT_CLR(m)		((m)->global_base + 0x108)
#define	GLB_INT_EN(m)		((m)->global_base + 0x10C)
#define MON_INT_STATUS(m)	((m)->base + 0x100)
#define MON_INT_CLR(m)		((m)->base + 0x108)
#define	MON_INT_EN(m)		((m)->base + 0x10C)
#define	MON_EN(m)		((m)->base + 0x280)
#define MON_CLEAR(m)		((m)->base + 0x284)
#define MON_CNT(m)		((m)->base + 0x288)
#define MON_THRES(m)		((m)->base + 0x290)
#define MON_MASK(m)		((m)->base + 0x298)
#define MON_MATCH(m)		((m)->base + 0x29C)

#define MON2_EN(m)		((m)->base + 0x2A0)
#define MON2_CLEAR(m)		((m)->base + 0x2A4)
#define MON2_SW(m)		((m)->base + 0x2A8)
#define MON2_THRES_HI(m)	((m)->base + 0x2AC)
#define MON2_THRES_MED(m)	((m)->base + 0x2B0)
#define MON2_THRES_LO(m)	((m)->base + 0x2B4)
#define MON2_ZONE_ACTIONS(m)	((m)->base + 0x2B8)
#define MON2_ZONE_CNT_THRES(m)	((m)->base + 0x2BC)
#define MON2_BYTE_CNT(m)	((m)->base + 0x2D0)
#define MON2_WIN_TIMER(m)	((m)->base + 0x2D4)
#define MON2_ZONE_CNT(m)	((m)->base + 0x2D8)
#define MON2_ZONE_MAX(m, zone)	((m)->base + 0x2E0 + 0x4 * zone)

struct bwmon_spec {
	bool wrap_on_thres;
	bool overflow;
	bool throt_adj;
	bool hw_sampling;
};

struct bwmon {
	void __iomem *base;
	void __iomem *global_base;
	unsigned int mport;
	unsigned int irq;
	const struct bwmon_spec *spec;
	struct device *dev;
	struct bw_hwmon hw;
	u32 hw_timer_hz;
	u32 throttle_adj;
	u32 sample_size_ms;
	u32 intr_status;
};

#define to_bwmon(ptr)		container_of(ptr, struct bwmon, hw)
#define has_hw_sampling(m)		(m->spec->hw_sampling)

#define ENABLE_MASK BIT(0)
#define THROTTLE_MASK 0x1F
#define THROTTLE_SHIFT 16
#define INT_ENABLE_V1	0x1
#define INT_STATUS_MASK	0x03
#define INT_STATUS_MASK_HWS	0xF0

static DEFINE_SPINLOCK(glb_lock);
static void mon_enable(struct bwmon *m)
{
	if (has_hw_sampling(m))
		writel_relaxed((ENABLE_MASK | m->throttle_adj), MON2_EN(m));
	else
		writel_relaxed((ENABLE_MASK | m->throttle_adj), MON_EN(m));
}

static void mon_disable(struct bwmon *m)
{
	if (has_hw_sampling(m))
		writel_relaxed(m->throttle_adj, MON2_EN(m));
	else
		writel_relaxed(m->throttle_adj, MON_EN(m));
	/*
	 * mon_disable() and mon_irq_clear(),
	 * If latter goes first and count happen to trigger irq, we would
	 * have the irq line high but no one handling it.
	 */
	mb();
}

#define MON_CLEAR_BIT	0x1
#define MON_CLEAR_ALL_BIT	0x2
static void mon_clear(struct bwmon *m, bool clear_all)
{
	if (!has_hw_sampling(m)) {
		writel_relaxed(MON_CLEAR_BIT, MON_CLEAR(m));
		goto out;
	}

	if (clear_all)
		writel_relaxed(MON_CLEAR_ALL_BIT, MON2_CLEAR(m));
	else
		writel_relaxed(MON_CLEAR_BIT, MON2_CLEAR(m));

	/*
	 * The counter clear and IRQ clear bits are not in the same 4KB
	 * region. So, we need to make sure the counter clear is completed
	 * before we try to clear the IRQ or do any other counter operations.
	 */
out:
	mb();
}

#define	SAMPLE_WIN_LIM	0xFFFFF
static void mon_set_hw_sampling_window(struct bwmon *m, unsigned int sample_ms)
{
	u32 rate;

	if (unlikely(sample_ms != m->sample_size_ms)) {
		rate = mult_frac(sample_ms, m->hw_timer_hz, MSEC_PER_SEC);
		m->sample_size_ms = sample_ms;
		if (unlikely(rate > SAMPLE_WIN_LIM)) {
			rate = SAMPLE_WIN_LIM;
			pr_warn("Sample window %u larger than hw limit: %u\n",
					rate, SAMPLE_WIN_LIM);
		}
		writel_relaxed(rate, MON2_SW(m));
	}
}

static void mon_irq_enable(struct bwmon *m)
{
	u32 val;

	spin_lock(&glb_lock);
	val = readl_relaxed(GLB_INT_EN(m));
	val |= 1 << m->mport;
	writel_relaxed(val, GLB_INT_EN(m));

	val = readl_relaxed(MON_INT_EN(m));
	val |= has_hw_sampling(m) ? INT_STATUS_MASK_HWS : INT_ENABLE_V1;
	writel_relaxed(val, MON_INT_EN(m));
	spin_unlock(&glb_lock);
	/*
	 * make Sure irq enable complete for local and global
	 * to avoid race with other monitor calls
	 */
	mb();
}

static void mon_irq_disable(struct bwmon *m)
{
	u32 val;

	spin_lock(&glb_lock);
	val = readl_relaxed(GLB_INT_EN(m));
	val &= ~(1 << m->mport);
	writel_relaxed(val, GLB_INT_EN(m));

	val = readl_relaxed(MON_INT_EN(m));
	val &= has_hw_sampling(m) ? ~INT_STATUS_MASK_HWS : ~INT_ENABLE_V1;
	writel_relaxed(val, MON_INT_EN(m));
	spin_unlock(&glb_lock);
	/*
	 * make Sure irq disable complete for local and global
	 * to avoid race with other monitor calls
	 */
	mb();
}

static unsigned int mon_irq_status(struct bwmon *m)
{
	u32 mval;

	mval = readl_relaxed(MON_INT_STATUS(m));

	dev_dbg(m->dev, "IRQ status p:%x, g:%x\n", mval,
			readl_relaxed(GLB_INT_STATUS(m)));

	mval &= has_hw_sampling(m) ? INT_STATUS_MASK_HWS : INT_STATUS_MASK;

	return mval;
}

static void mon_irq_clear(struct bwmon *m)
{
	u32 intclr;

	intclr = has_hw_sampling(m) ? INT_STATUS_MASK_HWS : INT_STATUS_MASK;

	writel_relaxed(intclr, MON_INT_CLR(m));
	mb();
	writel_relaxed(1 << m->mport, GLB_INT_CLR(m));
	mb();
}

static int mon_set_throttle_adj(struct bw_hwmon *hw, uint adj)
{
	struct bwmon *m = to_bwmon(hw);

	if (adj > THROTTLE_MASK)
		return -EINVAL;

	adj = (adj & THROTTLE_MASK) << THROTTLE_SHIFT;
	m->throttle_adj = adj;

	return 0;
}

static u32 mon_get_throttle_adj(struct bw_hwmon *hw)
{
	struct bwmon *m = to_bwmon(hw);

	return m->throttle_adj >> THROTTLE_SHIFT;
}

#define ZONE1_SHIFT	8
#define ZONE2_SHIFT	16
#define ZONE3_SHIFT	24
#define ZONE0_ACTION	0x01	/* Increment zone 0 count */
#define ZONE1_ACTION	0x09	/* Increment zone 1 & clear lower zones */
#define ZONE2_ACTION	0x25	/* Increment zone 2 & clear lower zones */
#define ZONE3_ACTION	0x95	/* Increment zone 3 & clear lower zones */
static u32 calc_zone_actions(void)
{
	u32 zone_actions;

	zone_actions = ZONE0_ACTION;
	zone_actions |= ZONE1_ACTION << ZONE1_SHIFT;
	zone_actions |= ZONE2_ACTION << ZONE2_SHIFT;
	zone_actions |= ZONE3_ACTION << ZONE3_SHIFT;

	return zone_actions;
}

#define ZONE_CNT_LIM	0xFFU
#define UP_CNT_1	1
static u32 calc_zone_counts(struct bw_hwmon *hw)
{
	u32 zone_counts;

	zone_counts = ZONE_CNT_LIM;
	zone_counts |= min(hw->down_cnt, ZONE_CNT_LIM) << ZONE1_SHIFT;
	zone_counts |= ZONE_CNT_LIM << ZONE2_SHIFT;
	zone_counts |= UP_CNT_1 << ZONE3_SHIFT;

	return zone_counts;
}

static unsigned int mbps_to_mb(unsigned long mbps, unsigned int ms)
{
	mbps *= ms;
	mbps = DIV_ROUND_UP(mbps, MSEC_PER_SEC);
	return mbps;
}

/*
 * Define the 4 zones using HI, MED & LO thresholds:
 * Zone 0: byte count < THRES_LO
 * Zone 1: THRES_LO < byte count < THRES_MED
 * Zone 2: THRES_MED < byte count < THRES_HI
 * Zone 3: byte count > THRES_HI
 */
#define	THRES_LIM	0x7FFU
static void set_zone_thres(struct bwmon *m, unsigned int sample_ms)
{
	struct bw_hwmon *hw = &(m->hw);
	u32 hi, med, lo;

	hi = mbps_to_mb(hw->up_wake_mbps, sample_ms);
	med = mbps_to_mb(hw->down_wake_mbps, sample_ms);
	lo = 0;

	if (unlikely((hi > THRES_LIM) || (med > hi) || (lo > med))) {
		pr_warn("Zone thres larger than hw limit: hi:%u med:%u lo:%u\n",
				hi, med, lo);
		hi = min(hi, THRES_LIM);
		med = min(med, hi - 1);
		lo = min(lo, med-1);
	}

	writel_relaxed(hi, MON2_THRES_HI(m));
	writel_relaxed(med, MON2_THRES_MED(m));
	writel_relaxed(lo, MON2_THRES_LO(m));
	dev_dbg(m->dev, "Thres: hi:%u med:%u lo:%u\n", hi, med, lo);
}

static void mon_set_zones(struct bwmon *m, unsigned int sample_ms)
{
	struct bw_hwmon *hw = &(m->hw);
	u32 zone_cnt_thres = calc_zone_counts(hw);

	mon_set_hw_sampling_window(m, sample_ms);
	set_zone_thres(m, sample_ms);
	/* Set the zone count thresholds for interrupts */
	writel_relaxed(zone_cnt_thres, MON2_ZONE_CNT_THRES(m));

	dev_dbg(m->dev, "Zone Count Thres: %0x\n", zone_cnt_thres);
}

static void mon_set_limit(struct bwmon *m, u32 count)
{
	writel_relaxed(count, MON_THRES(m));
	dev_dbg(m->dev, "Thres: %08x\n", count);
}

static u32 mon_get_limit(struct bwmon *m)
{
	return readl_relaxed(MON_THRES(m));
}

#define THRES_HIT(status)	(status & BIT(0))
#define OVERFLOW(status)	(status & BIT(1))
static unsigned long mon_get_count(struct bwmon *m)
{
	unsigned long count, status;

	count = readl_relaxed(MON_CNT(m));
	status = mon_irq_status(m);

	dev_dbg(m->dev, "Counter: %08lx\n", count);

	if (OVERFLOW(status) && m->spec->overflow)
		count += 0xFFFFFFFF;
	if (THRES_HIT(status) && m->spec->wrap_on_thres)
		count += mon_get_limit(m);

	dev_dbg(m->dev, "Actual Count: %08lx\n", count);

	return count;
}

static unsigned int get_zone(struct bwmon *m)
{
	u32 zone_counts;
	u32 zone;

	zone = get_bitmask_order((m->intr_status & INT_STATUS_MASK_HWS) >> 4);
	if (zone) {
		zone--;
	} else {
		zone_counts = readl_relaxed(MON2_ZONE_CNT(m));
		if (zone_counts) {
			zone = get_bitmask_order(zone_counts) - 1;
			zone /= 8;
		}
	}

	m->intr_status = 0;
	return zone;
}

static unsigned long mon_get_zone_stats(struct bwmon *m)
{
	unsigned int zone;
	unsigned long count = 0;

	zone = get_zone(m);

	count = readl_relaxed(MON2_ZONE_MAX(m, zone)) + 1;
	count *= SZ_1M;

	dev_dbg(m->dev, "Zone%d Max byte count: %08lx\n", zone, count);

	return count;
}

/* ********** CPUBW specific code  ********** */

/* Returns MBps of read/writes for the sampling window. */
static unsigned int mbps_to_bytes(unsigned long mbps, unsigned int ms,
				  unsigned int tolerance_percent)
{
	mbps *= (100 + tolerance_percent) * ms;
	mbps /= 100;
	mbps = DIV_ROUND_UP(mbps, MSEC_PER_SEC);
	mbps *= SZ_1M;
	return mbps;
}

static unsigned long get_bytes_and_clear(struct bw_hwmon *hw)
{
	struct bwmon *m = to_bwmon(hw);
	unsigned long count;

	mon_disable(m);
	count = has_hw_sampling(m) ? mon_get_zone_stats(m) : mon_get_count(m);
	mon_clear(m, false);
	mon_irq_clear(m);
	mon_enable(m);

	return count;
}

static unsigned long set_thres(struct bw_hwmon *hw, unsigned long bytes)
{
	unsigned long count;
	u32 limit;
	struct bwmon *m = to_bwmon(hw);

	mon_disable(m);
	count = mon_get_count(m);
	mon_clear(m, false);
	mon_irq_clear(m);

	if (likely(!m->spec->wrap_on_thres))
		limit = bytes;
	else
		limit = max(bytes, 500000UL);

	mon_set_limit(m, limit);
	mon_enable(m);

	return count;
}

static unsigned long set_hw_events(struct bw_hwmon *hw, unsigned sample_ms)
{
	struct bwmon *m = to_bwmon(hw);

	mon_disable(m);
	mon_clear(m, false);
	mon_irq_clear(m);

	mon_set_zones(m, sample_ms);
	mon_enable(m);

	return 0;
}

static irqreturn_t bwmon_intr_handler(int irq, void *dev)
{
	struct bwmon *m = dev;

	m->intr_status = mon_irq_status(m);
	if (!m->intr_status)
		return IRQ_NONE;

	if (bw_hwmon_sample_end(&m->hw) > 0)
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

static irqreturn_t bwmon_intr_thread(int irq, void *dev)
{
	struct bwmon *m = dev;

	update_bw_hwmon(&m->hw);
	return IRQ_HANDLED;
}

static int start_bw_hwmon(struct bw_hwmon *hw, unsigned long mbps)
{
	struct bwmon *m = to_bwmon(hw);
	u32 limit;
	u32 zone_actions = calc_zone_actions();
	int ret;

	ret = request_threaded_irq(m->irq, bwmon_intr_handler,
				  bwmon_intr_thread,
				  IRQF_ONESHOT | IRQF_SHARED,
				  dev_name(m->dev), m);
	if (ret) {
		dev_err(m->dev, "Unable to register interrupt handler! (%d)\n",
				ret);
		return ret;
	}

	mon_disable(m);

	mon_clear(m, true);
	limit = mbps_to_bytes(mbps, hw->df->profile->polling_ms, 0);
	if (has_hw_sampling(m)) {
		mon_set_zones(m, hw->df->profile->polling_ms);
		/* Set the zone actions to increment appropriate counters */
		writel_relaxed(zone_actions, MON2_ZONE_ACTIONS(m));
	} else {
		mon_set_limit(m, limit);
	}

	mon_irq_clear(m);
	mon_irq_enable(m);
	mon_enable(m);

	return 0;
}

static void stop_bw_hwmon(struct bw_hwmon *hw)
{
	struct bwmon *m = to_bwmon(hw);

	mon_irq_disable(m);
	free_irq(m->irq, m);
	mon_disable(m);
	mon_clear(m, true);
	mon_irq_clear(m);
}

static int suspend_bw_hwmon(struct bw_hwmon *hw)
{
	struct bwmon *m = to_bwmon(hw);

	mon_irq_disable(m);
	free_irq(m->irq, m);
	mon_disable(m);
	mon_irq_clear(m);

	return 0;
}

static int resume_bw_hwmon(struct bw_hwmon *hw)
{
	struct bwmon *m = to_bwmon(hw);
	int ret;

	mon_clear(m, false);
	ret = request_threaded_irq(m->irq, bwmon_intr_handler,
				  bwmon_intr_thread,
				  IRQF_ONESHOT | IRQF_SHARED,
				  dev_name(m->dev), m);
	if (ret) {
		dev_err(m->dev, "Unable to register interrupt handler! (%d)\n",
				ret);
		return ret;
	}

	mon_irq_enable(m);
	mon_enable(m);

	return 0;
}

/*************************************************************************/

static const struct bwmon_spec spec[] = {
	{ .wrap_on_thres = true, .overflow = false, .throt_adj = false,
		.hw_sampling = false},
	{ .wrap_on_thres = false, .overflow = true, .throt_adj = false,
		.hw_sampling = false},
	{ .wrap_on_thres = false, .overflow = true, .throt_adj = true,
		.hw_sampling = false},
	{ .wrap_on_thres = false, .overflow = true, .throt_adj = true,
		.hw_sampling = true},
};

static struct of_device_id match_table[] = {
	{ .compatible = "qcom,bimc-bwmon", .data = &spec[0] },
	{ .compatible = "qcom,bimc-bwmon2", .data = &spec[1] },
	{ .compatible = "qcom,bimc-bwmon3", .data = &spec[2] },
	{ .compatible = "qcom,bimc-bwmon4", .data = &spec[3] },
	{}
};

static int bimc_bwmon_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct bwmon *m;
	const struct of_device_id *id;
	int ret;
	u32 data;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->dev = dev;

	ret = of_property_read_u32(dev->of_node, "qcom,mport", &data);
	if (ret) {
		dev_err(dev, "mport not found!\n");
		return ret;
	}
	m->mport = data;

	id = of_match_device(match_table, dev);
	if (!id) {
		dev_err(dev, "Unknown device type!\n");
		return -ENODEV;
	}
	m->spec = id->data;

	if (has_hw_sampling(m)) {
		ret = of_property_read_u32(dev->of_node,
				"qcom,hw-timer-hz", &data);
		if (ret) {
			dev_err(dev, "HW sampling rate not specified!\n");
			return ret;
		}
		m->hw_timer_hz = data;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (!res) {
		dev_err(dev, "base not found!\n");
		return -EINVAL;
	}
	m->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!m->base) {
		dev_err(dev, "Unable map base!\n");
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "global_base");
	if (!res) {
		dev_err(dev, "global_base not found!\n");
		return -EINVAL;
	}
	m->global_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!m->global_base) {
		dev_err(dev, "Unable map global_base!\n");
		return -ENOMEM;
	}

	m->irq = platform_get_irq(pdev, 0);
	if (m->irq < 0) {
		dev_err(dev, "Unable to get IRQ number\n");
		return m->irq;
	}

	m->hw.of_node = of_parse_phandle(dev->of_node, "qcom,target-dev", 0);
	if (!m->hw.of_node)
		return -EINVAL;
	m->hw.start_hwmon = &start_bw_hwmon;
	m->hw.stop_hwmon = &stop_bw_hwmon;
	m->hw.suspend_hwmon = &suspend_bw_hwmon;
	m->hw.resume_hwmon = &resume_bw_hwmon;
	m->hw.get_bytes_and_clear = &get_bytes_and_clear;
	m->hw.set_thres =  &set_thres;
	if (has_hw_sampling(m))
		m->hw.set_hw_events = &set_hw_events;
	if (m->spec->throt_adj) {
		m->hw.set_throttle_adj = &mon_set_throttle_adj;
		m->hw.get_throttle_adj = &mon_get_throttle_adj;
	}

	ret = register_bw_hwmon(dev, &m->hw);
	if (ret) {
		dev_err(dev, "Dev BW hwmon registration failed\n");
		return ret;
	}

	return 0;
}

static struct platform_driver bimc_bwmon_driver = {
	.probe = bimc_bwmon_driver_probe,
	.driver = {
		.name = "bimc-bwmon",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init bimc_bwmon_init(void)
{
	return platform_driver_register(&bimc_bwmon_driver);
}
module_init(bimc_bwmon_init);

static void __exit bimc_bwmon_exit(void)
{
	platform_driver_unregister(&bimc_bwmon_driver);
}
module_exit(bimc_bwmon_exit);

MODULE_DESCRIPTION("BIMC bandwidth monitor driver");
MODULE_LICENSE("GPL v2");
