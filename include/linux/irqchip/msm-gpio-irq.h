/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#ifndef MSM_GPIO_IRQ_H
#define MSM_GPIO_IRQ_H

#include <linux/irq.h>

#if (defined(CONFIG_GPIO_MSM_V1) || defined(CONFIG_GPIO_MSM_V2) \
	|| defined(CONFIG_GPIO_MSM_V3) && !defined(CONFIG_USE_PINCTRL_IRQ))
int __init msm_gpio_of_init(struct device_node *node,
					struct device_node *parent);
extern struct irq_chip msm_gpio_irq_extn;
static inline int __init msm_tlmm_of_irq_init(struct device_node *node,
					struct device_node *parent)
{
	return 0;
}
#elif defined(CONFIG_PINCTRL_MSM_TLMM)
int __init msm_tlmm_of_irq_init(struct device_node *node,
					struct device_node *parent);
extern struct irq_chip mpm_tlmm_irq_extn;
static inline int __init msm_gpio_of_init(struct device_node *node,
					struct device_node *parent)
{
	return 0;
}
#else
extern struct irq_chip mpm_pinctrl_extn;
static inline int __init msm_tlmm_of_irq_init(struct device_node *node,
					      struct device_node *parent)
{
	return 0;
}

static inline int __init msm_gpio_of_init(struct device_node *node,
					struct device_node *parent)
{
	return 0;
}
#endif
#endif
