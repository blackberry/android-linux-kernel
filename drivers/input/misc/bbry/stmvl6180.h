/*
 * Copyright (C) 2014 BlackBerry Limited
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef	__STMVL6180_H__
#define	__STMVL6180_H__

#define VL6180_DEV_NAME "stmvl6180"
#define VL6180_ALS_DEV_NAME "stmvl6180_als"
#define VL6180_RANGING_DEV_NAME "stmvl6180_ranging"

#define RESULT_RANGE_STATUS         0x04D
#define RESULT_RANGE_VAL            0x062

#define RANGE_START                 0x018
#define SYSALS_MODE_SELECT_SINGLE   (0u << 1)
#define SYSALS_STARTSTOP_START      (1u << 0)

typedef enum {
	LIGHT = 0,
	PROX,
	NUM_SAMPLE_TYPES
} sample_type_e;

struct vl6180_platform_data {
	int gpio_int;
	unsigned int poll_interval_ranging;
	unsigned int poll_interval_als;
	unsigned int min_interval_ranging;
	unsigned int min_interval_als;
};

#endif
