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

#ifndef SYNAPTICS_DSX_WAKEUP_C_
#define SYNAPTICS_DSX_WAKEUP_C_

#include "synaptics_dsx_wakeup.h"
#include "synaptics_dsx_ddt.h"
#include "synaptics_dsx_logging.h"

#define FAILED_SWIPE_MSG                "Swipe failed wakeup criteria"
#define POSSIBLE_WAKEUP_MSG             "Possible wakeup detected"
#define SWIPE_DATA_PARAMS               "dx=%d, dy=%d, x=%d, y=%d, z=%d, area=%d, vel=%d, fingers=%d"
#define SWIPE_CRITERIA_PARAMS           "min_speed=%d, max_speed=%d, max_ratio=%d, min_z=%d, max_z=%d, max_area=%d"

const struct synaptics_wakeup_criteria_t default_wakeup_criteria = {
	.min_speed = -1,
	.max_speed = SYNAPTICS_WAKEUP_MAX_WAKE_SPEED,
	.max_ratio = SYNAPTICS_WAKEUP_MAX_RATIO,
	.min_z     = SYNAPTICS_WAKEUP_MIN_Z,
	.max_z     = SYNAPTICS_WAKEUP_MAX_Z,
	.max_area  = 75,
	.fingers   = 1,
	.large_object = true,
};

#define log_possible_wakeup(criteria, data)  { \
	synaptics_info(POSSIBLE_WAKEUP_MSG " Data = " SWIPE_DATA_PARAMS, \
		data->dx, \
		data->dy, \
		data->x, \
		data->y, \
		data->z, \
		data->w * data->h, \
		data->vel, \
		data->fingers); \
				\
	synaptics_info(POSSIBLE_WAKEUP_MSG " Criteria = " SWIPE_CRITERIA_PARAMS, \
		criteria->min_speed, \
		criteria->max_speed, \
		criteria->max_ratio, \
		criteria->min_z, \
		criteria->max_z, \
		criteria->max_area); \
} \

#define WRONG_DIRECTION_SWIPE_PARAMS        "dy=%d"
#define WRONG_DIRECTION_FAILED              FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_WRONG_DIRECTION" " " WRONG_DIRECTION_SWIPE_PARAMS

#define OVER_X_SWIPE_PARAMS                 "dx=%d, dy=%d, ratio=%d, max_ratio=%d"
#define OVER_X_SWIPE_FAILED                 FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_OVER_X" " " OVER_X_SWIPE_PARAMS

#define TOO_FAST_SWIPE_PARAMS               "dx=%d, dy=%d, vel=%d, max_speed=%d"
#define TOO_FAST_FAILED                     FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_TOO_FAST" " " TOO_FAST_SWIPE_PARAMS

#define TOO_SLOW_SWIPE_PARAMS               "dx=%d, dy=%d, vel=%d, min_speed=%d"
#define TOO_SLOW_FAILED                     FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_TOO_SLOW" " " TOO_SLOW_SWIPE_PARAMS

#define OVER_Z_SWIPE_PARAMS                 "z=%d, max_z=%d"
#define OVER_Z_FAILED                       FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_OVER_Z" " " OVER_Z_SWIPE_PARAMS

#define UNDER_Z_SWIPE_PARAMS                "z=%d, min_z=%d"
#define UNDER_Z_FAILED                      FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_UNDER_Z" " " UNDER_Z_SWIPE_PARAMS

#define LARGE_OBJECT_SWIPE_PARAMS           "large_object=%d"
#define LARGE_OBJECT_FAILED                 FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_LARGE_OBJECT" " " LARGE_OBJECT_SWIPE_PARAMS

#define AREA_SWIPE_PARAMS                   "area=%d"
#define AREA_FAILED                         FAILED_SWIPE_MSG " " "SYNAPTICS_AREA_TOO_BIG" " " AREA_SWIPE_PARAMS

#define INVALID_NUM_FINGERS_SWIPE_PARAMS    "fingers=%d"
#define INVALID_NUM_FINGERS_FAILED          FAILED_SWIPE_MSG " " "SYNAPTICS_WAKEUP_INVALID_NUM_FINGERS" " " INVALID_NUM_FINGERS_SWIPE_PARAMS

int
synaptics_is_wakeup_swipe(
	const struct synaptics_wakeup_criteria_t *criteria,
	struct synaptics_wakeup_data_t *data, synaptics_quip_t *quip)
{
	int ratio;

	/*
	 * Wake up swipe moves bottom to top (0 >= dy) and is mostly parallel to x bezel (+/- angle ratio )
	 */
	log_possible_wakeup(criteria, data);

	/* Check for number of fingers */
	if (criteria->fingers >= 0 && data->fingers != criteria->fingers) {
		synaptics_warn(INVALID_NUM_FINGERS_FAILED, data->fingers);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_INVALID_NUM_FINGERS, criteria, data);
		return 0;
	}

	/* Check for large object */
	if (criteria->large_object && data->large_object) {
		synaptics_warn(LARGE_OBJECT_FAILED, data->large_object);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_LARGE_OBJECT, criteria, data);
		return 0;
	}

	/* Check area */
	if (criteria->max_area >= 0 && (data->w * data->h) > criteria->max_area) {
		synaptics_warn(AREA_FAILED, data->w * data->h);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_TOO_BIG, criteria, data);
		return 0;
	}

	if (0 <= data->dy)  {
		synaptics_warn(WRONG_DIRECTION_FAILED, data->dy);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_WRONG_DIRECTION, criteria, data);
		return 0;
	}

	ratio = (abs(data->dx) * 10) / abs(data->dy);

	if (criteria->max_ratio >= 0 && ratio > criteria->max_ratio) {
		synaptics_warn(OVER_X_SWIPE_FAILED, data->dx, data->dy, ratio, criteria->max_ratio);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_OVER_X, criteria, data);
		return 0;
	}

	/* Swipe min speed gate */
	if (criteria->min_speed >= 0 && data->vel < criteria->min_speed) {
		synaptics_warn(TOO_SLOW_FAILED, data->dx, data->dy, data->vel, criteria->min_speed);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_TOO_SLOW, criteria, data);
		return 0;
	}

	/* Swipe max speed gate */
	if (criteria->max_speed >= 0 && data->vel > criteria->max_speed) {
		synaptics_warn(TOO_FAST_FAILED, data->dx, data->dy, data->vel, criteria->max_speed);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_TOO_FAST, criteria, data);
		return 0;
	}

	/* Swipe min signal strength (min finger size) */
	if (criteria->min_z >= 0 && data->z <= criteria->min_z) {
		synaptics_warn(UNDER_Z_FAILED, data->z, criteria->min_z);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_UNDER_Z, criteria, data);
		return 0;
	}

	/* Swipe max signal strength (max finger size) */
	if (criteria->max_z >= 0 && data->z >= criteria->max_z) {
		synaptics_warn(OVER_Z_FAILED, data->z, criteria->max_z);
		synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_OVER_Z, criteria, data);
		return 0;
	}

	/* Swipe accepted */
	synaptics_info("Swipe accepted");
	synaptics_report_wakeup_swipe(quip, SYNAPTICS_WAKEUP_PASSED, criteria, data);

	return 1;
}

#endif /* SYNAPTICS_DSX_WAKEUP_C_ */
