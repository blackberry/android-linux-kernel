/*
 * Copyright (C) 2015 Blackberry
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SYNAPTICS_DSX_WAKEUP_H_
#define SYNAPTICS_DSX_WAKEUP_H_

#include "synaptics_dsx_ddt.h"

struct synaptics_wakeup_criteria_t {
	int min_speed;
	int max_speed;

	/* Max ratio.
	 * The ratio between swipe in the X-axis and Y-axis.
	 * Is directly related to the angle of the swipe.
	 * Note: To simplify ratio calculation both dx and dy incremented by 1.
	 * This represents 10*x/(y+1)
	 *  0 - parallel to Y bezel
	 *  8 - 45 degrees (approx)
	 * -1  any angle
	 */
	int max_ratio;
	int min_z;
	int max_z;
	int max_area;
	int fingers;
	bool large_object;
};

struct synaptics_wakeup_data_t {
	int dx;
	int dy;
	int x;
	int y;
	int z;
	int w;
	int h;
	int vel;
	int fingers;
	bool large_object;
};

typedef void synaptics_quip_t;

#define SYNAPTICS_WAKEUP_START                     3500
#define SYNAPTICS_WAKEUP_MAX_RATIO                    5
#define SYNAPTICS_WAKEUP_SCAN_SNOOZE                  1
/* above min distance, no unit, not actual mills per second */
#define SYNAPTICS_WAKEUP_MAX_WAKE_SPEED               8
#define SYNAPTICS_WAKEUP_MIN_Z                       35
#define SYNAPTICS_WAKEUP_MAX_Z                      125

/* default wakeup control  (read synaptics.h for more info) */
#define SYNAPTICS_FIRMWARE_MIN_WAKE_DISTANCE         15
#define SYNAPTICS_FIRMWARE_MIN_WAKE_SPEED             2

#define SYNAPTICS_FIRMWARE_ZONE_TOP_LEFT_X          287
#define SYNAPTICS_FIRMWARE_ZONE_TOP_LEFT_Y          462
#define SYNAPTICS_FIRMWARE_ZONE_BOTTOM_RIGHT_X     1152
#define SYNAPTICS_FIRMWARE_ZONE_BOTTOM_RIGHT_Y     1440

#define SYNAPTICS_GESTURE_WAKEUP_DEBOUNCE_PERIOD   (500)  /* in ms */

extern const struct synaptics_wakeup_criteria_t default_wakeup_criteria;

int
synaptics_is_wakeup_swipe(
	const struct synaptics_wakeup_criteria_t *criteria,
	struct synaptics_wakeup_data_t *data, synaptics_quip_t *quip);

#endif /* SYNAPTICS_DSX_WAKEUP_H_ */
