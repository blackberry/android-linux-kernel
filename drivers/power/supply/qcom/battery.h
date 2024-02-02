/* Copyright (c) 2017 The Linux Foundation. All rights reserved.
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

#ifndef __BATTERY_H
#define __BATTERY_H
int qcom_batt_init(void);
void qcom_batt_deinit(void);

#if defined(CONFIG_TCT_SDM660_COMMON)
#define QC2_ICL_MAX (1800000)
#define QC2_FCC_MAX (3000000)
#endif

#endif /* __BATTERY_H */
