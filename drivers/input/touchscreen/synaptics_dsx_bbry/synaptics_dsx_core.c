/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2014 BlackBerry Limited
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#ifdef CONFIG_MSM_GPIOMUX
#include <mach/gpiomux.h>
#endif
#include <synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#include <linux/i2c.h>
#include <linux/of_i2c.h>
#include <linux/sensors.h>
#include <linux/completion.h>

#include "synaptics_dsx_core.h"
#include "synaptics_dsx_fw_update.h"
#include "synaptics_dsx_rmi_dev.h"
#include "synaptics_dsx_test_reporting.h"
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
#include "synaptics_dsx_ddt.h"
#endif
#include "synaptics_dsx_slide.h"

#define INPUT_PHYS_NAME "synaptics_dsx/touch_input"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define NO_0D_WHILE_2D
#define REPORT_2D_Z
#define REPORT_2D_W
#define IGNORE_FN_INIT_FAILURE

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | \
			RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */
#define MAX_F11_TOUCH_WIDTH 15
#define MAX_Z 255

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

#define F11_CONTINUOUS_MODE     (0x00)
#define F11_WAKEUP_GESTURE_MODE (0x04)
#define F12_CONTINUOUS_MODE     (0x00)
#define F12_WAKEUP_GESTURE_MODE (0x02)
#define F12_FACE_DETECTION	(0x01)
#define F12_CTRL8_SENSOR_REPORT_BEYOND_ACTIVE (0x01)
#define F12_CTRL8_SENSOR_INACTIVE_AREA (0x4A)
#define F12_CTRL8_SENSOR_INACTIVE_AREA_WAKE (0x05)

#define F12_FACE_DETECTION_DELAY_MS     32
#define F12_FACE_DETECTION_CLEAR_DELAY_MS     100

#define DDIC_POWER_ON_DELAY_MS	36
#define DDIC_PRE_RESET_DELAY_MS	51

#define SYNAPTICS_MAX_RESET_ATTEMPTS	20
#define SYNAPTICS_MAX_RESET_WAIT	10
#define SYNAPTICS_RESET_WAIT_MS		100
#define SYNAPTICS_MAX_FAILURE_RETRY	5
#define SYNAPTICS_POWER_DOWN_DELAY_US	5


/* Custom Function 51 specific values. */
#define SYNAPTICS_F51_CUSTOM_DATA04                 (0x400)
#define SYNAPTICS_F51_CUSTOM_DATA_MAX_PEAK_SIZE      (0x02)
#define SYNAPTICS_F51_CUSTOM_DATA_MAX_DIFF_SIZE      (0x02)
#define SYNAPTICS_F51_CUSTOM_DATA_MAX_ENERGY_SIZE    (0x02)

#define SYNAPTICS_F51_CUSTOM_CTRL06                 (0x406)

#define SYNAPTICS_F51_CUSTOM_DATA06                 (0x456)
#define SYNAPTICS_LPWG_STATE_NOT_LPWG                (0x00)

#define SYNAPTICS_WAKEUP_DELAY_MS         (36)
#define SYNAPTICS_LPWG_DELAY_MS           (8)
#define SYNAPTICS_LPWG_MAX_RETRIES        (1000/SYNAPTICS_LPWG_DELAY_MS)

#define SYNAPTICS_MAX_REGLEN				(256)

#define SYNAPTICS_MAGIC_MULTIPLIER			60
#define SYNAPTICS_MAGIC_MAX_Y				2639
#define SYNAPTICS_MAGIC_FRACTION			(62/1000)

#define PROXI_SENSOR_POCKET_DETECT_RANGE		25
#define PROXI_TIMEOUT					100

#define POSITION_BUFFER_MIN				2

/* inadvertent tap detect timer timeout period */
#define SYNAPTICS_MTOUCH_INADV_TAP_DETECT_TIMEOUT_VAL	(1*60*1000)    /* in ms */
#define MS_TO_KTIME(x)        ktime_set((((uint32_t)x)/1000), ((x%1000)*1000*1000))

struct synaptics_rmi4_data *glass_on_data;

enum {
	POWER_STATE_UNKNOWN,
	POWER_STATE_AWAKE,
	POWER_STATE_ASLEEP,
	POWER_STATE_DEEP_SLEEP,
	POWER_STATE_OFF
};

enum {
	SLIDER_STATE_CLOSED,
	SLIDER_STATE_OPENING,
	SLIDER_STATE_OPENED,
	SLIDER_STATE_CLOSING,
	SLIDER_STATE_MAX,
};


enum {
	WAKEUP_GESTURE_DETECTED = 0,
	WAKEUP_UP,
	FACE_DETECTED,
	SUSPEND,
	SLIDER,
	PROXI_DETECTED,
	EVENT_MAX,
};

const char *touch_event_str[] = {
	"dtap",
	"wake",
	"face",
	"sleep",
	"slider",
	"proxi",
};

enum {
	WAKEUP_BY_OTHER,
	WAKEUP_BY_DOUBLE_TAP,
	WAKEUP_BY_DISPLAY,
	WAKEUP_SOURCE_MAX,
};

const char *wakeup_source_string[] = {
	"other",
	"double tap",
	"display",
};

const char *power_state_str[] = {
	"unknown",
	"awake",
	"asleep",
	"deep_sleep",
	"off",
};

static char *tap_failure_string[] = {
	"first tap moved too much",
	"first tap too long",
	"second tap moved too much",
	"second tap too long",
	"second tap too far",
	"time between taps too long",
};

const char *slider_state_str[] = {
	"closed",
	"opening",
	"opened",
	"closing",
};

enum {
	F12_OBJECT_TYPE_NONE            = 0x00,
	F12_OBJECT_TYPE_FINGER          = 0x01,
	F12_OBJECT_TYPE_STYLUS          = 0x02,
	F12_OBJECT_TYPE_PALM            = 0x03,
	F12_OBJECT_TYPE_UNCLASSIFIED    = 0x04,
	F12_OBJECT_TYPE_RESERVED        = 0x05,
	F12_OBJECT_TYPE_GLOVED_FINGER   = 0x06
};


#define WAKEUP_CLEAR_MASK		0x00
#define WAKEUP_CHECK_RESUME		0x01
#define WAKEUP_CHECK_GLASS_ON		0x02

#define	RMI4_INIT_MASK			0x00
#define	RMI4_FWUPG_COMPLETE_MASK	0x01
#define	RMI4_INIT_COMPLETE_MASK		(RMI4_FWUPG_COMPLETE_MASK)


#define SLIDER_OPENED_KEY_MASK          (0x01)
#define SLIDER_TRANSITION_KEY_MASK      (0x02)

/* This define is not available in 8974 branch yet   */
/* we don't use it, but to pass the compiler for now */
#ifndef SW_KEYPAD_TRANSITION
#define SW_KEYPAD_TRANSITION		0x13
#endif

struct synaptics_slider_fsm_info {
	uint8_t    mask;
	uint8_t    slider_state;
};

const struct synaptics_slider_fsm_info  slider_fsm_table_single_sensor[] = {
	{0x00, SLIDER_STATE_CLOSED  },
	{0x01, SLIDER_STATE_OPENED  },
};

/* for the regular devices with both sensors */
const struct synaptics_slider_fsm_info  slider_fsm_table[] = {
	{0x00, SLIDER_STATE_CLOSED  },
	{0x01, SLIDER_STATE_OPENED  },
	{0x02, SLIDER_STATE_OPENING },
	{0x03, SLIDER_STATE_CLOSING },
};

#ifdef CONFIG_INPUT_EVENTS_BUFFER
static const struct input_event_filter event_filter[] = {
	{{0, ABS_MT_POSITION_X, 0}, FILTER_WHOLE_EVENT},
	{{0, ABS_MT_POSITION_Y, 0}, FILTER_WHOLE_EVENT},
	{{0, ABS_MT_TOUCH_MINOR, 0}, FILTER_WHOLE_EVENT},
	{{0, ABS_MT_TOUCH_MAJOR, 0}, FILTER_WHOLE_EVENT},
	{{0, ABS_MT_TOUCH_MAJOR, 0}, FILTER_WHOLE_EVENT},
	{{0, ABS_MT_TOOL_TYPE, 0}, FILTER_WHOLE_EVENT},
	{{0, BTN_TOOL_FINGER, 0}, FILTER_WHOLE_EVENT},
	{{0, 0, 0}, FILTER_NULL},
};
#endif /*CONFIG_INPUT_EVENTS_BUFFER*/

static int synaptics_rmi4_f12_set_enables(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(
		struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(
		struct synaptics_rmi4_data *rmi4_data);

static void synaptics_rmi4_power_state_handler(
		struct synaptics_rmi4_data *rmi4_data);

static void synaptics_rmi4_glass_on_work(struct work_struct *work);

static void synaptics_rmi4_reset_work(struct work_struct *work);

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
					bool is_hw_reset);
static int synaptics_rmi4_power_enable(struct synaptics_rmi4_data *rmi4_data,
					bool enable);

static int go_sleep(struct device *dev);
static int do_wakeup(struct device *dev, bool reset);

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_runtime_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_i2cerror_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
static ssize_t synaptics_rmi4_ddt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_proxi_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_stats_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_events_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_info_show(struct device *dev,
		struct device_attribute *attr, char *buf);
#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_stop_timer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif /*CONFIG_BBRY_DEBUG*/

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static ssize_t synaptics_rmi4_max_touchpoints_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_product_id_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_serial_id_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_firmware_id_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_config_id_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_BIST_show(struct device *dev,
		struct device_attribute *attr, char *buf);
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_touch_ready_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static int synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler, bool report);
static void synaptics_lockup_poll(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_f12_lockup_check(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler, bool *locked_up);
#ifdef CONFIG_PM
static void synaptics_rmi4_power_state_work(struct work_struct *work);
#ifdef CONFIG_FB
static int synaptics_rmi4_fb_notif(struct notifier_block *self,
				unsigned long event, void *data);
#endif
#endif

static void synaptics_rmi4_slider_state_work(struct work_struct *work);

static ssize_t synaptics_rmi4_turn_off_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_turn_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_i2c_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_i2c_addr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_i2c_read_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_i2c_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static int synaptics_regulator_enable(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator);

static int synaptics_regulator_disable(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator);

static void synaptics_regulator_put(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator);

static int synaptics_rmi4_hw_reset(struct synaptics_rmi4_data *rmi4_data);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
static ssize_t synaptics_rmi4_mtouch_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static void synaptics_rmi4_mtouch_counter_init(
				struct synaptics_rmi4_data *rmi4_data);
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/

static void record_wakeup_failures_in_proxi(bool in_proxi, struct synaptics_rmi4_data  *rmi4_data);
static void proxi_complete(int object_detected, void *data);

static void synaptics_record_events(struct synaptics_rmi4_data  *rmi4_data,
	struct synaptics_rmi4_event_entry *event);
static int synaptics_rmi4_cancel_touch(
	struct synaptics_rmi4_data *rmi4_data,
	unsigned char   finger);

struct synaptics_rmi4_f11_query_0_5 {
	union {
		struct {
			/* query 0 */
			unsigned char f11_query0_b0__2:3;
			unsigned char has_query_9:1;
			unsigned char has_query_11:1;
			unsigned char has_query_12:1;
			unsigned char has_query_27:1;
			unsigned char has_query_28:1;

			/* query 1 */
			unsigned char num_of_fingers:3;
			unsigned char has_rel:1;
			unsigned char has_abs:1;
			unsigned char has_gestures:1;
			unsigned char has_sensitibity_adjust:1;
			unsigned char f11_query1_b7:1;

			/* query 2 */
			unsigned char num_of_x_electrodes;

			/* query 3 */
			unsigned char num_of_y_electrodes;

			/* query 4 */
			unsigned char max_electrodes:7;
			unsigned char f11_query4_b7:1;

			/* query 5 */
			unsigned char abs_data_size:2;
			unsigned char has_anchored_finger:1;
			unsigned char has_adj_hyst:1;
			unsigned char has_dribble:1;
			unsigned char has_bending_correction:1;
			unsigned char has_large_object_suppression:1;
			unsigned char has_jitter_filter:1;
		} __packed;
		unsigned char data[6];
	};
};

struct synaptics_rmi4_f11_query_7_8 {
	union {
		struct {
			/* query 7 */
			unsigned char has_single_tap:1;
			unsigned char has_tap_and_hold:1;
			unsigned char has_double_tap:1;
			unsigned char has_early_tap:1;
			unsigned char has_flick:1;
			unsigned char has_press:1;
			unsigned char has_pinch:1;
			unsigned char has_chiral_scroll:1;

			/* query 8 */
			unsigned char has_palm_detect:1;
			unsigned char has_rotate:1;
			unsigned char has_touch_shapes:1;
			unsigned char has_scroll_zones:1;
			unsigned char individual_scroll_zones:1;
			unsigned char has_multi_finger_scroll:1;
			unsigned char has_multi_finger_scroll_edge_motion:1;
			unsigned char has_multi_finger_scroll_inertia:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f11_query_9 {
	union {
		struct {
			unsigned char has_pen:1;
			unsigned char has_proximity:1;
			unsigned char has_large_object_sensitivity:1;
			unsigned char has_suppress_on_large_object_detect:1;
			unsigned char has_two_pen_thresholds:1;
			unsigned char has_contact_geometry:1;
			unsigned char has_pen_hover_discrimination:1;
			unsigned char has_pen_hover_and_edge_filters:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_12 {
	union {
		struct {
			unsigned char has_small_object_detection:1;
			unsigned char has_small_object_detection_tuning:1;
			unsigned char has_8bit_w:1;
			unsigned char has_2d_adjustable_mapping:1;
			unsigned char has_general_information_2:1;
			unsigned char has_physical_properties:1;
			unsigned char has_finger_limit:1;
			unsigned char has_linear_cofficient_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_query_27 {
	union {
		struct {
			unsigned char f11_query27_b0:1;
			unsigned char has_pen_position_correction:1;
			unsigned char has_pen_jitter_filter_coefficient:1;
			unsigned char has_group_decomposition:1;
			unsigned char has_wakeup_gesture:1;
			unsigned char has_small_finger_correction:1;
			unsigned char has_data_37:1;
			unsigned char f11_query27_b7:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f11_ctrl_6_9 {
	union {
		struct {
			unsigned char sensor_max_x_pos_7_0;
			unsigned char sensor_max_x_pos_11_8:4;
			unsigned char f11_ctrl7_b4__7:4;
			unsigned char sensor_max_y_pos_7_0;
			unsigned char sensor_max_y_pos_11_8:4;
			unsigned char f11_ctrl9_b4__7:4;
		} __packed;
		unsigned char data[4];
	};
};

struct synaptics_rmi4_f11_data_1_5 {
	union {
		struct {
			unsigned char x_position_11_4;
			unsigned char y_position_11_4;
			unsigned char x_position_3_0:4;
			unsigned char y_position_3_0:4;
			unsigned char wx:4;
			unsigned char wy:4;
			unsigned char z;
		} __packed;
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl32_is_present:1;
				unsigned char ctrl33_is_present:1;
				unsigned char ctrl34_is_present:1;
				unsigned char ctrl35_is_present:1;
				unsigned char ctrl36_is_present:1;
				unsigned char ctrl37_is_present:1;
				unsigned char ctrl38_is_present:1;
				unsigned char ctrl39_is_present:1;
			} __packed;
		};
		unsigned char data[6];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
			unsigned char sensor_flags;
		};
		unsigned char data[15];
	};
};

struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
	unsigned char z;
	unsigned char wx;
	unsigned char wy;
};

struct synaptics_rmi4_f12_gesture_data {
	unsigned char gesture_type;
	unsigned char gesture_prop0;
	unsigned char gesture_prop1;
	unsigned char gesture_prop2;
	unsigned char gesture_prop3;
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

static struct device_attribute attrs[] = {
#ifdef CONFIG_BBRY_DEBUG
	__ATTR(reset, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(hw_reset, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_hw_reset_store),
#endif /*CONFIG_BBRY_DEBUG*/
	__ATTR(productinfo, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(wakeup_gesture, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_wakeup_gesture_show,
			synaptics_rmi4_wakeup_gesture_store),
	__ATTR(proxi_check, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_proxi_check_store),
	__ATTR(stats, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_stats_show,
			synaptics_rmi4_store_error),
	__ATTR(events, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_events_show,
			synaptics_rmi4_store_error),
	__ATTR(info, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_info_show,
			synaptics_rmi4_store_error),
	__ATTR(turn_off, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_turn_off_show,
			synaptics_rmi4_turn_off_store),
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
	__ATTR(mtouch_counter, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_mtouch_counter_show,
			synaptics_rmi4_store_error),
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/

#ifdef CONFIG_BBRY_DEBUG
	__ATTR(0dbutton, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_runtime_suspend_store),
	__ATTR(i2cerror, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_i2cerror_store),
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
	__ATTR(ddt, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_ddt_store),
#endif
#endif /*CONFIG_BBRY_DEBUG*/

#ifdef CONFIG_BBRY_DEBUG
	__ATTR(stop_timer, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_stop_timer_store),
#endif /*CONFIG_BBRY_DEBUG*/
#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
	__ATTR(max_touchpoints, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_max_touchpoints_show,
			synaptics_rmi4_store_error),
	__ATTR(vendor, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_vendor_show,
			synaptics_rmi4_store_error),
	__ATTR(product_id, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_product_id_show,
			synaptics_rmi4_store_error),
	__ATTR(serial_id, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_serial_id_show,
			synaptics_rmi4_store_error),
	__ATTR(firmware_id, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_firmware_id_show,
			synaptics_rmi4_store_error),
	__ATTR(config_id, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_config_id_show,
			synaptics_rmi4_store_error),
	__ATTR(BIST, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_BIST_show,
			synaptics_rmi4_store_error),
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/
__ATTR(i2c_addr, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_i2c_addr_show,
			synaptics_rmi4_i2c_addr_store),
__ATTR(i2c_read, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_i2c_read_show,
			synaptics_rmi4_store_error),
__ATTR(i2c_write, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			synaptics_rmi4_i2c_write_store),
__ATTR(touch_ready, (S_IRUSR | S_IRGRP),
			synaptics_rmi4_touch_ready_show,
			synaptics_rmi4_store_error),
};

#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	1

#define HEIGHT_SCALE_NUM 8
#define HEIGHT_SCALE_DENOM 10

/* numerator and denomenator for border equations */
#define BORDER_ADJUST_NUM 3
#define BORDER_ADJUST_DENOM 4

static struct kobject *vkey_kobj = NULL;
static struct synaptics_rmi4_data *vkey_rmi4_data = NULL;

static ssize_t vkey_show(struct kobject  *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct synaptics_dsx_vkey_data *keydata;
	struct synaptics_rmi4_data *rmi4_data = NULL;
	struct synaptics_dsx_board_data	*bdata = NULL;
	uint8_t num_vkeys;
	int idx;

	rmi4_data = vkey_rmi4_data;
	bdata = rmi4_data->hw_if->board_data;

	num_vkeys = bdata->vkeymap_info.nvkeys;
	keydata = bdata->vkeymap_info.data;

	if ((num_vkeys == 0) || (keydata == NULL)) {
		dev_err(rmi4_data->pdev->dev.parent,
			"No virtual key available\n");
		return 0;
	}

	/* syntax of virtual key map file :
	 * Each virtual key is described by 6 colon-delimited components:
	 *    0x01: A version code. Must always be 0x01.
	 *    <Linux key code>: The Linux key code of the virtual key.
	 *    <centerX>: The X pixel coordinate of the center of the virtual key.
	 *    <centerY>: The Y pixel coordinate of the center of the virtual key.
	 *    <width>: The width of the virtual key in pixels.
	 *    <height>: The height of the virtual key in pixels.
	 */
	buf[0] = '\0';
	for (idx = 0; idx < num_vkeys; idx++) {
		snprintf(buf + strlen(buf), MAX_BUF_SIZE - strlen(buf),
			"0x%02x:%d:%d:%d:%d:%d\n", VKEY_VER_CODE,
			keydata[idx].code, keydata[idx].center_x,
			keydata[idx].center_y, keydata[idx].width, keydata[idx].height);
	}

	return strnlen(buf, MAX_BUF_SIZE);
}

static struct kobj_attribute vkey_obj_attr = {
	.attr = {
		.mode = S_IRUGO,
		.name = "virtualkeys.touch_keypad", //PLATFORM_DRIVER_NAME,
	},
	.show = vkey_show,
};

static struct attribute *vkey_attr[] = {
	&vkey_obj_attr.attr,
	NULL,
};

static struct attribute_group vkey_grp = {
	.attrs = vkey_attr,
};


static int synaptics_regulator_enable(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator)
{
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: regulator %p\n",
		__func__, regulator->regulator);
	if (regulator->enabled) {
		dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: regulator already enabled\n",
		__func__);
		return 0;
	}
	regulator->enabled = true;

	return regulator_enable(regulator->regulator);
}

static int synaptics_regulator_disable(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator)
{
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: regulator %p\n",
		__func__, regulator->regulator);
	if (!regulator->enabled) {
		dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: regulator already disabled\n",
		__func__);
		return 0;
	}

	regulator->enabled = false;
	return regulator_disable(regulator->regulator);
}

static void synaptics_regulator_put(
		struct synaptics_rmi4_data *rmi4_data,
		struct synpatics_regulator *regulator)
{
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: regulator %p\n",
		__func__, regulator->regulator);

	regulator_put(regulator->regulator);
}

/* handler to listen for input event and enable/disable keypad. */
static void synaptics_input_event(struct input_handle *handle,
				unsigned int type,
				unsigned int code, int value)
{
	struct synaptics_rmi4_data *rmi4_data = handle->private;
	const struct synaptics_dsx_board_data *bdata =
					rmi4_data->hw_if->board_data;

	/* check if we need to handle these keys */
	if ((NULL == rmi4_data) || (EV_SW != type) ||
		((SW_LID != code) && (SW_KEYPAD_SLIDE != code) &&
			(SW_KEYPAD_TRANSITION != code) &&
			(SW_SMART_FLIP != code)))

		return;

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: type=%d, code=%d, value=%d\n",
		__func__, type, code, value);

	if (code == SW_LID) {
		rmi4_data->lid_state = value;
		if (bdata->dis_in_holster) {
			queue_work(rmi4_data->workqueue,
				&rmi4_data->power_state_work);
		}
		return;
	}

	if (code == SW_SMART_FLIP) {
		rmi4_data->smart_flip_state = value;
		return;
	}


	if ((0 == bdata->dis_in_slider) && (0 == bdata->dis_while_sliding))
		return; /* we are not interested in slider input keys */

	if ((SW_KEYPAD_SLIDE  == code) && (value != 0))
		rmi4_data->slider_keys_values |= SLIDER_OPENED_KEY_MASK;
	else if ((SW_KEYPAD_SLIDE  == code) && (value == 0))
		rmi4_data->slider_keys_values &= ~(SLIDER_OPENED_KEY_MASK);
	else if ((SW_KEYPAD_TRANSITION  == code) && (value != 0))
		rmi4_data->slider_keys_values |= SLIDER_TRANSITION_KEY_MASK;
	else if ((SW_KEYPAD_TRANSITION  == code) && (value == 0))
		rmi4_data->slider_keys_values &= ~(SLIDER_TRANSITION_KEY_MASK);
	else {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Invalid input. code=0x%02x, value=%d\n",
			__func__, code, value);
		return;
	}

	if (((bdata->num_of_slider_hall_sensors == 1) &&
	     (code == SW_KEYPAD_SLIDE)) ||
	    ((bdata->num_of_slider_hall_sensors == 2) &&
	     (code == SW_KEYPAD_TRANSITION))) {

		queue_work(rmi4_data->workqueue,
				&rmi4_data->slider_work);
	}
}

static int synaptics_input_event_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	int    ret;
	struct input_handle *handle;
	struct synaptics_rmi4_data *rmi4_data = handler->private;
	const struct synaptics_dsx_board_data *bdata =
					rmi4_data->hw_if->board_data;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	if (test_bit(SW_KEYPAD_SLIDE, dev->swbit)) {
		if ((bdata->dis_in_holster) && (test_bit(SW_LID, dev->sw))) {
			rmi4_data->lid_state = true;
			queue_work(rmi4_data->workqueue,
				&rmi4_data->power_state_work);
		}

		if (test_bit(SW_SMART_FLIP, dev->sw))
			rmi4_data->smart_flip_state = true;

		if (test_bit(SW_KEYPAD_SLIDE, dev->sw))
			rmi4_data->slider_keys_values |= SLIDER_OPENED_KEY_MASK;

		if ((bdata->num_of_slider_hall_sensors == 2) &&
		    (test_bit(SW_KEYPAD_TRANSITION, dev->sw)))
			rmi4_data->slider_keys_values |= SLIDER_TRANSITION_KEY_MASK;

		queue_work(rmi4_data->workqueue,
			&rmi4_data->slider_work);
	}

	handle->private = handler->private;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "synaptics_dsx";

	dev_info(rmi4_data->pdev->dev.parent,
		"slider_state=%d, lid_state=%d, smart_flip_state = %d\n",
		rmi4_data->slider_state,
		rmi4_data->lid_state, rmi4_data->smart_flip_state);
	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;


	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void synaptics_input_event_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

void synaptics_rmi4_monitor_timer_start(
			struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_timer *timer = &rmi4_data->monitor_timer;

	if (!rmi4_data->monitor_timer.interval_ms)
		return;

	mutex_lock(&timer->mutex);
	if (!timer->active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Start %s", timer->name);
		timer->timer.expires =
			jiffies + msecs_to_jiffies(timer->interval_ms);
		add_timer(&timer->timer);
		timer->active = true;
	}
	mutex_unlock(&timer->mutex);
}

void synaptics_rmi4_monitor_timer_stop(
			struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_timer *timer = &rmi4_data->monitor_timer;

	if (!rmi4_data->monitor_timer.interval_ms)
		return;

	mutex_lock(&timer->mutex);
	if (timer->active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Stop %s", timer->name);
		del_timer_sync(&timer->timer);
		timer->active = false;
	}
	mutex_unlock(&timer->mutex);
}

static void synaptics_rmi4_monitor_timer_restart(
			struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_timer *timer = &rmi4_data->monitor_timer;

	if (!rmi4_data->monitor_timer.interval_ms)
		return;

	mutex_lock(&timer->mutex);
	if (timer->active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Restart %s\n", timer->name);
		del_timer_sync(&timer->timer);
		timer->timer.expires =
			jiffies + msecs_to_jiffies(timer->interval_ms);
		add_timer(&timer->timer);
	}
	mutex_unlock(&timer->mutex);
}

static void watchdog_timeout_check(struct work_struct *work)
{
	int retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_timer *watchdog =
		container_of(work,
			struct synaptics_rmi4_timer,
			timeout_work);
	struct synaptics_rmi4_data  *rmi4_data =
		container_of(watchdog,
			struct synaptics_rmi4_data,
			monitor_timer);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (rmi4_data->sensor_sleep || rmi4_data->suspend) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Abort watchdog timeout during sleep\n",
				__func__);
		return;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s:watchdog timeout, suspend=%d, "
				"irq_gpio = %d\n",
				__func__, rmi4_data->suspend,
				gpio_get_value(bdata->irq_gpio));

	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: enable_irq_counter irq=%d, "
				"irq_nosync = %d, irq_wake = %d\n",
				__func__, rmi4_data->en_irq_counter.irq,
				rmi4_data->en_irq_counter.irq_nosync,
				rmi4_data->en_irq_counter.irq_wake);

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: slider=%s, touch_obj=%d, touch=%s\n",
		__func__, slider_state_str[rmi4_data->slider_state],
		rmi4_data->touch_obj_cnt,
		(rmi4_data->ignore_touch | rmi4_data->resume_ignore_touch) ? "ignored" : "normal");

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Traverse the function handler list and polling
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				retval = synaptics_rmi4_report_touch(
						rmi4_data,
						fhandler, false);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to report touch\n",
						__func__);
					break;
				}
			}
		}
	}

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
		"%s: Failed to reporting finger data, reset controller\n",
			__func__);
		rmi4_data->reset_device(rmi4_data, true);
	} else {
		mutex_lock(&(rmi4_data->rmi4_reset_mutex));
		rmi4_data->num_failures = 0;
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	}

	synaptics_rmi4_monitor_timer_restart(rmi4_data);

	return;
}

static void synaptics_rmi4_timer_handler(unsigned long arg)
{

	struct synaptics_rmi4_data  *rmi4_data =
			(struct synaptics_rmi4_data *) arg;

	queue_work(rmi4_data->workqueue,
		&rmi4_data->monitor_timer.timeout_work);
}

static void synaptics_rmi4_timer_init(struct synaptics_rmi4_timer *timer,
			char *name, int interval_ms,
			void (*timeout_work)(struct work_struct *work))
{
	struct synaptics_rmi4_data *rmi4_data =
		container_of(timer,
		struct synaptics_rmi4_data, monitor_timer);

	mutex_init(&timer->mutex);

	/* initialize timer */
	init_timer(&timer->timer);
	timer->active = 0;
	timer->name = name;
	timer->interval_ms = interval_ms;
	timer->timer.data = (unsigned long)rmi4_data;
	timer->timer.function = synaptics_rmi4_timer_handler;

	INIT_WORK(&timer->timeout_work, timeout_work);
}

static int synaptics_f12_lockup_check(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler, bool *locked_up)
{
	int retval = 0;
	struct synaptics_rmi4_f54_handle *f54;
	unsigned char data;
	unsigned char command;
	int i, get_report_retry = 1;
	unsigned char report_index[2];

	unsigned cap[2];
	int16_t capval;
	int16_t lock_min_cap = 0x7FFF;
	int16_t lock_max_cap = 0x8000;

	*locked_up = false;

	f54 = rmi4_data->f54;
	if (!f54) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: No F54, abort\n",
			__func__);
		return 0;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"electrodes = %dx%d", rmi4_data->num_of_rx,
		rmi4_data->num_of_tx);

	/* Set the report type */
	mutex_lock(&f54->status_mutex);
	if (f54->status != STATUS_BUSY) {
		f54->report_type = F54_16BIT_IMAGE;
		data = (unsigned char)F54_16BIT_IMAGE;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->data_base_addr + f54->data_offsets[0],
				&data,
				sizeof(data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write data register\n",
				__func__);
			goto error_exit;
		}
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Previous get report still ongoing\n",
			__func__);
		goto error_exit;
	}

	f54->status = STATUS_BUSY;

	/* Request the report and poll until ready;
	 * we may timeout if the controller happens to enter noise
	 * mitigation during the report, so retry as needed.
	 */

	do {
		if (rmi4_data->suspend) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Suspending, abort\n",
				__func__);
			goto error_exit;
		}

		command = 0;

		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to clear F54 command 0 register\n",
				__func__);
			goto error_exit;
		}

		command = (unsigned char)COMMAND_GET_REPORT;

		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write get report command\n",
				__func__);
			goto error_exit;
		}

		for (i = 0; i < SYNAPTICS_F54_GET_REPORT_MAX_RETRY; i++) {
			usleep(SYNAPTICS_SAMPLE_PERIOD_MS * 1000);
			retval = synaptics_rmi4_reg_read(rmi4_data,
					f54->command_base_addr,
					&command,
					sizeof(command));
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read command register\n",
					__func__);
				goto error_exit;
			}

			if ((command & COMMAND_GET_REPORT) == 0) {
				get_report_retry = 0;
				break;
			}
		}
		if (get_report_retry > 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: timed out waiting for report - retrying\n",
				__func__);
		}
	} while (get_report_retry--);

	if (command & COMMAND_GET_REPORT) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: timed out waiting for report\n",
			__func__);
		/* a timeout is consistent with the lockup state */
		*locked_up = true;

		retval = 0;
		goto error_exit;
	}

	f54->report_size = 2 * rmi4_data->num_of_rx * rmi4_data->num_of_tx;
	if (f54->report_size == 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Report data size = 0\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}



	if (f54->data_buffer_size < f54->report_size) {
		mutex_lock(&f54->data_mutex);
		if (f54->data_buffer_size)
			kfree(f54->report_data);
		f54->report_data = kzalloc(f54->report_size, GFP_KERNEL);
		if (!f54->report_data) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for data buffer\n",
					__func__);
			f54->data_buffer_size = 0;
			mutex_unlock(&f54->data_mutex);
			retval = -ENOMEM;
			goto error_exit;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	/* reset the read offset */
	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write report data index\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read report data\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}

	/* Analyze and print out the data */
	/* compare if Delta response is abnormally high and flat */

	for (i = 0; i < (f54->report_size / 2) ; i++) {
		cap[0] = f54->report_data[i * 2];
		cap[1] = f54->report_data[i * 2 + 1];
		capval = ((uint16_t)cap[1] << 8) + cap[0];
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: data[%d] = %d, data[%d] = %d, capval = %d\n",
			__func__, i * 2, f54->report_data[i * 2],
			i * 2 + 1, f54->report_data[i * 2 + 1], capval);
		if (capval > lock_max_cap)
			lock_max_cap = capval;
		if (capval < lock_min_cap)
			lock_min_cap = capval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: lock_max_cap = %d, lock_min_cap = %d\n",
		__func__, lock_max_cap, lock_min_cap);

	if (lock_min_cap > 150 && (lock_max_cap-lock_min_cap) < 110)
		*locked_up = true;

error_exit:
	f54->status = STATUS_IDLE;

	mutex_unlock(&f54->status_mutex);

	return retval;
}

static int synaptics_lockup_check(
		struct synaptics_rmi4_data *rmi4_data,
		bool *locked_up)
{
	int retval = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	if (rmi4_data->sensor_sleep || rmi4_data->suspend)
		return -ENODATA;

	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s:Lockup check start\n",
				__func__);

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Traverse the function handler list and check
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->fn_number ==
					SYNAPTICS_RMI4_F12) {
					retval = synaptics_f12_lockup_check(
								rmi4_data,
								fhandler,
								locked_up);
					break;
				}
			}
		}
	}


	return retval;
}

static void synaptics_lockup_poll(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;
	struct synaptics_rmi4_f54_handle *f54;
	unsigned char image_metric[SYNAPTICS_F54_IMAGE_METRIC_LEN];
	unsigned int poll_value;
	bool locked_up;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	f54 = rmi4_data->f54;
	if (!f54) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: No F54, abort\n",
			__func__);
		return;
	}

	if (rmi4_data->suspend) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Suspending, abort\n",
			__func__);
		return;
	}
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s:Lockup poll, poll_count = %d, pre_value = %d\n",
		__func__,
		f54->current_lockup_poll_num,
		f54->prev_lockup_poll_value);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + f54->data_offsets[20],
			image_metric,
			sizeof(image_metric));

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to read report data\n",
			__func__);
		retval = -EINVAL;
		goto error_exit;
	}

	poll_value = image_metric[1] << 8 | image_metric[0];

	if (poll_value == f54->prev_lockup_poll_value) {
		locked_up = false;
		f54->current_lockup_poll_num++;
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Image metric unchanged %d (%d)\n",
				__func__,
				f54->current_lockup_poll_num,
				poll_value);

		if (f54->current_lockup_poll_num >=
					bdata->lockup_poll_count) {
			retval = synaptics_lockup_check(
					rmi4_data, &locked_up);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Error during lockup check",
					__func__);
				goto error_exit;
			}

			f54->current_lockup_poll_num = 0;

			if (locked_up) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
				synaptics_dsx_ddt_send(
					rmi4_data, CONTROLLER_LOCKUP, 0);
#endif
				goto error_exit;
			}
		}
	} else {
		f54->prev_lockup_poll_value = poll_value;
		f54->current_lockup_poll_num = 0;
	}

	return;

error_exit:
	dev_err(rmi4_data->pdev->dev.parent,
			"%s: Reset controller",
			__func__);
	rmi4_data->reset_device(rmi4_data, true);
	return;
}

static void synaptics_rmi4_stats_start(struct synaptics_rmi4_data *rmi4_data)
{
	mutex_lock(&rmi4_data->stats.mutex);
	if (!rmi4_data->stats.active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Start stats timer\n");
		rmi4_data->stats.timer.expires = jiffies +
				msecs_to_jiffies(SYNAPTICS_STATS_PERIOD);
		add_timer(&rmi4_data->stats.timer);
		rmi4_data->stats.active = true;
	}
	mutex_unlock(&rmi4_data->stats.mutex);
}

static void synaptics_rmi4_stats_stop(struct synaptics_rmi4_data *rmi4_data)
{
	mutex_lock(&rmi4_data->stats.mutex);
	if (rmi4_data->stats.active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Stop stats timer\n");
		del_timer_sync(&rmi4_data->stats.timer);
		rmi4_data->stats.active = false;
	}
	mutex_unlock(&rmi4_data->stats.mutex);
}

static void synaptics_rmi4_stats_restart(
			struct synaptics_rmi4_data *rmi4_data)
{
	mutex_lock(&rmi4_data->stats.mutex);
	if (rmi4_data->stats.active) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Restart stats timer\n");
		del_timer_sync(&rmi4_data->stats.timer);
		rmi4_data->stats.timer.expires =
			jiffies + msecs_to_jiffies(SYNAPTICS_STATS_PERIOD);
		add_timer(&rmi4_data->stats.timer);
	}
	mutex_unlock(&rmi4_data->stats.mutex);
}

static void synaptics_rmi4_stats_handler(unsigned long arg)
{
	struct synaptics_rmi4_data  *rmi4_data =
		(struct synaptics_rmi4_data *) arg;

	queue_work(rmi4_data->workqueue,
		&rmi4_data->stats.timeout_work);
}

static inline u32 get_timestamp(void)
{
	return ktime_to_ms(ktime_get());
}

static void stats_timeout_work(struct work_struct *work)
{
	struct synaptics_rmi4_stats *stats =
		container_of(work,
			struct synaptics_rmi4_stats,
			timeout_work);
	struct synaptics_rmi4_data  *rmi4_data =
		container_of(stats,
			struct synaptics_rmi4_data,
			stats);

	int total, total_served, touch, report_touch;
	int large_object, finger_release, touch_release;
	int status, reset, error;
	int entry;
	unsigned long msecs;
	unsigned int secs, usecs;

	mutex_lock(&rmi4_data->stats.mutex);
	/* read counts */
	total  = stats->total_int_cnt;
	total_served = stats->total_int_served_cnt;
	report_touch = stats->touch_report_cnt;
	large_object = stats->touch_large_object_cnt;
	finger_release = stats->touch_frelease_cnt;
	touch_release = stats->touch_trelease_cnt;
	touch  = stats->touch_int_cnt;
	status = stats->status_int_cnt;
	reset  = stats->unexpected_reset;
	error  = stats->status_error;

	/* clear counts */
	stats->total_int_cnt = 0;
	stats->total_int_served_cnt = 0;
	stats->touch_int_cnt = 0;
	stats->touch_report_cnt = 0;
	stats->touch_large_object_cnt = 0;
	stats->touch_frelease_cnt = 0;
	stats->touch_trelease_cnt = 0;
	stats->status_int_cnt = 0;
	stats->unexpected_reset = 0;
	stats->status_error = 0;

	/* there is nothing in the stats */
	if (total == 0)
		goto end;

	/* update stats entry */
	entry = stats->next_stats_entry;

	stats->list[entry].total_int_cnt = total;
	stats->list[entry].total_int_served_cnt = total_served;
	stats->list[entry].touch_int_cnt = touch;
	stats->list[entry].touch_report_cnt = report_touch;
	stats->list[entry].touch_large_object_cnt = large_object;
	stats->list[entry].touch_frelease_cnt = finger_release;
	stats->list[entry].touch_trelease_cnt = touch_release;
	stats->list[entry].status_int_cnt = status;
	stats->list[entry].unexpected_reset = reset;
	stats->list[entry].status_error = error;

	/* update stats list */
	stats->next_stats_entry++;
	if (stats->next_stats_entry >= SYNAPTICS_STATS_SIZE)
		stats->next_stats_entry = 0;

	if (stats->num_stats_entries < SYNAPTICS_STATS_SIZE)
		stats->num_stats_entries++;

	msecs = get_timestamp();
	secs  = msecs/1000;
	usecs = (msecs - (secs*1000)) * 1000;

	stats->list[entry].msecs = msecs;

	if (total != 0 || touch != 0 ||
		status != 0 || reset != 0 || error != 0) {
		dev_info(rmi4_data->pdev->dev.parent,
			"[%6u.%6u] INT=%d INT_S=%d T=%d REPORT_T=%d FR=%d TR=%d LO=%d STAT=%d RST=%d ERR=%d\n",
			secs, usecs, total, total_served, touch, report_touch, finger_release, touch_release, large_object, status, reset, error);
	}
end:
	mutex_unlock(&rmi4_data->stats.mutex);
	synaptics_rmi4_stats_restart(rmi4_data);
}

static void synaptics_rmi4_stats_init(struct synaptics_rmi4_data *rmi4_data)
{
	/* initialize stats */
	rmi4_data->stats.total_int_cnt = 0;
	rmi4_data->stats.total_int_served_cnt = 0;
	rmi4_data->stats.status_int_cnt = 0;
	rmi4_data->stats.touch_int_cnt = 0;
	rmi4_data->stats.touch_report_cnt = 0;
	rmi4_data->stats.touch_large_object_cnt = 0;
	rmi4_data->stats.touch_frelease_cnt = 0;
	rmi4_data->stats.touch_trelease_cnt = 0;
	rmi4_data->stats.unexpected_reset = 0;
	rmi4_data->stats.status_error = 0;
	rmi4_data->stats.active = false;
	rmi4_data->stats.update = false;
	rmi4_data->stats.num_stats_entries = 0;
	rmi4_data->stats.next_stats_entry = 0;

	mutex_init(&rmi4_data->stats.mutex);

	/* initialize timer */
	init_timer(&rmi4_data->stats.timer);
	rmi4_data->stats.timer.data = (unsigned long)rmi4_data;
	rmi4_data->stats.timer.function = synaptics_rmi4_stats_handler;

	INIT_WORK(&rmi4_data->stats.timeout_work, stats_timeout_work);
}

static void synaptics_record_events(struct synaptics_rmi4_data  *rmi4_data,
	struct synaptics_rmi4_event_entry *event)
{
	int entry;
	unsigned long msecs;
	struct timespec ts;
	struct synaptics_rmi4_events *events = &rmi4_data->events;

	mutex_lock(&rmi4_data->events.mutex);

	/* update event entry */
	entry = events->next_events_entry;

	memcpy(&events->list[entry], event, sizeof(struct synaptics_rmi4_event_entry));

	/* update event list */
	events->next_events_entry++;
	if (events->next_events_entry >= SYNAPTICS_EVENTS_SIZE)
		events->next_events_entry = 0;

	if (events->num_events_entries < SYNAPTICS_EVENTS_SIZE)
		events->num_events_entries++;

	msecs = get_timestamp();

	if (events->list[entry].event_id == WAKEUP_UP) {
		getnstimeofday(&ts);
		dev_info(rmi4_data->pdev->dev.parent,
		"Wake up at %.2lu:%.2lu:%.2lu:%.6lu\n",
		(ts.tv_sec / 3600) % (24),
		(ts.tv_sec / 60) % (60),
		ts.tv_sec % 60,
		ts.tv_nsec / 1000);
	}

	events->list[entry].msecs = msecs;

	mutex_unlock(&rmi4_data->events.mutex);
}

static void synaptics_rmi4_events_init(struct synaptics_rmi4_data *rmi4_data)
{
	/* initialize events */
	rmi4_data->events.num_events_entries = 0;
	rmi4_data->events.next_events_entry = 0;

	mutex_init(&rmi4_data->events.mutex);
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
static void synaptics_rmi4_set_inadvertent_tapdetect_timer(
		struct synaptics_rmi4_data *rmi4_data, bool set)
{
	uint32_t  timeout_ms = rmi4_data->mtouch_counter.inadv_tap_det_tmout_ms;
	uint32_t  rc = -1;

	if (set != 0) {
		rc = hrtimer_start(&rmi4_data->mtouch_counter.inadv_tapdet_hrtimer,
				MS_TO_KTIME(timeout_ms), HRTIMER_MODE_REL);
	} else {
		rc = (uint32_t)hrtimer_cancel(&rmi4_data->mtouch_counter.inadv_tapdet_hrtimer);
	}
}

static void synaptics_rmi4_inadvertent_tapdetect_timer_restart(
				struct synaptics_rmi4_data *rmi4_data)
{
	uint32_t  timeout_ms = rmi4_data->mtouch_counter.inadv_tap_det_tmout_ms;
	int       rc = -1;

	rc = hrtimer_start(&rmi4_data->mtouch_counter.inadv_tapdet_hrtimer,
			MS_TO_KTIME(timeout_ms), HRTIMER_MODE_REL);

}

static enum hrtimer_restart synaptics_rmi4_mtouch_inadv_detect_timer_handler(struct hrtimer *timer)
{
	struct synaptics_rmi4_mtouch_counter *mc =
		container_of(timer, struct synaptics_rmi4_mtouch_counter, inadv_tapdet_hrtimer);

	struct synaptics_rmi4_data  *rmi4_data =
		container_of(mc, struct synaptics_rmi4_data, mtouch_counter);
	pm_stay_awake(&rmi4_data->pdev->dev);
	queue_work(rmi4_data->workqueue,
		&rmi4_data->mtouch_counter.inadv_tapdet_tmout_work);
	return HRTIMER_NORESTART;
}


static void synaptics_rmi4_mtouch_counter_reset(struct synaptics_rmi4_mtouch_counter *mtouch_counter)
{
	int i;
	mtouch_counter->swipe_wakeup = 0;
	mtouch_counter->double_tap = 0;
	mtouch_counter->successful_double_tap = 0;
	mtouch_counter->glass_on_delay_300ms = 0;
	mtouch_counter->glass_on_delay_500ms = 0;
	mtouch_counter->glass_on_delay_750ms = 0;
	mtouch_counter->glass_on_delay_1000ms = 0;
	mtouch_counter->dtwakeup_delay_200ms = 0;
	mtouch_counter->dtwakeup_delay_400ms = 0;
	mtouch_counter->dtwakeup_delay_1sec = 0;
	mtouch_counter->upgrade_failure = 0;
	mtouch_counter->face_detection = 0;
	for (i = 0; i < (sizeof(mtouch_counter->tap_failure)/sizeof(int)); i++)
		mtouch_counter->tap_failure[i] = 0;
	mtouch_counter->failed_taps_in_proxi = 0;
	mtouch_counter->inadv_tap_failures = 0;
	mtouch_counter->proxi_io_error = 0;
	mtouch_counter->proxi_timeout = 0;
	mtouch_counter->proxi_not_ready = 0;
	mtouch_counter->proxi_detected = 0;
	mtouch_counter->few_pos_buff = 0;
	mtouch_counter->i2c_rw_error = 0;
	mtouch_counter->t_up_without_down = 0;
}

static void synpatics_rmi4_inadvertent_tapdetect_timeout_work(
				struct work_struct *work)
{
	struct synaptics_rmi4_mtouch_counter *mtouch_counter =
		container_of(work,
			struct synaptics_rmi4_mtouch_counter,
			inadv_tapdet_tmout_work);
	struct synaptics_rmi4_data  *rmi4_data =
		container_of(mtouch_counter,
			struct synaptics_rmi4_data,
			mtouch_counter);
	unsigned long start_msec;
	int  retval;

	start_msec = jiffies_to_msecs(jiffies);
	rmi4_data->proxi_pocket = -ETIMEDOUT;

	INIT_COMPLETION(rmi4_data->proxi_completion);
	get_proxi_value_async(proxi_complete, rmi4_data);
	retval = wait_for_completion_timeout(&rmi4_data->proxi_completion,
					msecs_to_jiffies(PROXI_TIMEOUT));

	record_wakeup_failures_in_proxi((rmi4_data->proxi_pocket == 1), rmi4_data);
	synaptics_rmi4_inadvertent_tapdetect_timer_restart(rmi4_data);
	pm_relax(&rmi4_data->pdev->dev);
}

static void synaptics_rmi4_mtouch_counter_init(
				struct synaptics_rmi4_data *rmi4_data)
{
	/* initilize the counters */
	synaptics_rmi4_mtouch_counter_reset(&rmi4_data->mtouch_counter);
	mutex_init(&rmi4_data->mtouch_counter.mutex);

	hrtimer_init(&rmi4_data->mtouch_counter.inadv_tapdet_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rmi4_data->mtouch_counter.inadv_tapdet_hrtimer.function =
				synaptics_rmi4_mtouch_inadv_detect_timer_handler;
	rmi4_data->mtouch_counter.inadv_tap_det_tmout_ms =
				SYNAPTICS_MTOUCH_INADV_TAP_DETECT_TIMEOUT_VAL;
	INIT_WORK(&rmi4_data->mtouch_counter.inadv_tapdet_tmout_work,
		synpatics_rmi4_inadvertent_tapdetect_timeout_work);


}
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data, false);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data, true);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_reg_write(rmi4_data,
				rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_runtime_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_runtime_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_runtime_resume(dev);
	else
		return -EINVAL;

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	ssize_t  size = 0;
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"power_state=%d, next=%d, suspend=%d\n",
		rmi4_data->power_state,
		rmi4_data->next_power_state, rmi4_data->suspend);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"irq=%d, irq_state=%d, irq_enabled=%d, irq_gpio=%d\n",
		rmi4_data->irq,
		irq_read_line(rmi4_data->irq), rmi4_data->irq_enabled,
		gpio_get_value(bdata->irq_gpio));
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"Enable irq counters:\nirq=%d, irq_nosync=%d, irq_wake=%d\n",
		rmi4_data->en_irq_counter.irq,
		rmi4_data->en_irq_counter.irq_nosync,
		rmi4_data->en_irq_counter.irq_wake);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"ignore_touch=%d, resume_ignore_touch=%d, slider_state=%d, lid_state=%d, smart_flip_state = %d\n",
		rmi4_data->ignore_touch,
		rmi4_data->resume_ignore_touch,
		rmi4_data->slider_state,
		rmi4_data->lid_state,
		rmi4_data->smart_flip_state);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"proxi_check=%d\n",
		rmi4_data->proxi_check);
	if (bdata->tap_status_addr > 0) {
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"last_tap_status=%d\n",
			rmi4_data->last_tap_status[0]);
		for (i = 1; i < sizeof(rmi4_data->last_tap_status); i++) {
			size += snprintf(buf+strlen(buf),
				PAGE_SIZE - strlen(buf),
				"%s : %d\n",
				tap_failure_string[i - 1],
				rmi4_data->last_tap_status[i]);
		}
	}

	if (bdata->pos_buf_addr > 0) {
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"[Position Buffer]\n");
		for (i = 0; i < SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT; i++) {
			if (rmi4_data->extra_wakeup_info.x[i]
				|| rmi4_data->extra_wakeup_info.y[i])
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
				"X,Y[%d] = (%d,%d)\n",
				i,
				rmi4_data->extra_wakeup_info.x[i],
				rmi4_data->extra_wakeup_info.y[i]);
		}
	}
	return size;
}

static ssize_t synaptics_rmi4_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	ssize_t  size = 0;

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"bdata->wg_enabled=%d\n", bdata->wg_enabled);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"wakeup_gesture.swipe=%d\n", rmi4_data->wakeup_gesture.swipe);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"wakeup_gesture.double_tap=%d\n", rmi4_data->wakeup_gesture.double_tap);

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"\n");

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"power_state=%d, next=%d, suspend=%d\n",
		rmi4_data->power_state,
		rmi4_data->next_power_state, rmi4_data->suspend);
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
		"stay_awake=%d, f11_wake=%d, "
			"f12_wake=%d\n",
		rmi4_data->stay_awake,
		rmi4_data->f11_wakeup_gesture, rmi4_data->f12_wakeup_gesture);

	return size;
}

static ssize_t synaptics_rmi4_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture) {
		if (input) {
			/* swipe to wakeup is disabled */
			/* rmi4_data->wakeup_gesture.swipe = (input & F12_SWIPE_WAKEUP) ? 1 : 0; */

			/* The position buffer is only for swip, enable swipe for position buffer*/
			/* The firmware has the trick which has the swipe box matching the double*/
			/* tap box and makes the required swipe distance very large.             */
			/* So the finger could not actually generate swipe interrupts, but the   */
			/* buffer would be filled.*/
			if (bdata->pos_buf_addr > 0)
				rmi4_data->wakeup_gesture.swipe = 1;
			rmi4_data->wakeup_gesture.double_tap = (input & F12_DOUBLE_TAP_WAKEUP) ? 1 : 0;
		} else {
			rmi4_data->wakeup_gesture.data = 0;
		}
	}
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: input = %d, swipe = %d, double tap = %d\n",
			__func__, input, rmi4_data->wakeup_gesture.swipe,
			rmi4_data->wakeup_gesture.double_tap);

	return count;
}

static ssize_t synaptics_rmi4_proxi_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input)
		rmi4_data->proxi_check = true;
	else
		rmi4_data->proxi_check = false;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: input = %d, proxi_check = %d\n",
			__func__, input, rmi4_data->proxi_check);

	return count;
}

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_i2cerror_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->sm_err.i2c = input;

	return count;
}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
static ssize_t synaptics_rmi4_ddt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	synaptics_dsx_ddt_send(rmi4_data, input, 0);

	return count;
}
#endif
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_turn_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", rmi4_data->turn_off);
}

static ssize_t synaptics_rmi4_turn_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;
	rmi4_data->turn_off = (bool)input;
	queue_work(rmi4_data->workqueue, &rmi4_data->power_state_work);
	return count;
}

static ssize_t synaptics_rmi4_stats_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i, n, len, size, entry, num;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_stats *stats = &rmi4_data->stats;
	unsigned int secs, msecs;

	len = 0;
	size = PAGE_SIZE;

	mutex_lock(&rmi4_data->stats.mutex);
	num = stats->num_stats_entries;
	entry = stats->next_stats_entry;

	n = snprintf(&buf[len], size,
			"TIME INT,INT_SERVED,TOUCH,REPORTED_TOUCH,FINGER_RELEASE,TOUCH_RELEASE,LARGE_OBJECT,STATUS,RESET,EERROR\n");
	size -= n;
	len  += n;

	for (i = 0; i < num; i++) {
		entry = (entry > 0) ? (entry-1) : (SYNAPTICS_STATS_SIZE-1);
		secs  = stats->list[entry].msecs/1000;
		msecs = (stats->list[entry].msecs - (secs*1000));

		n = snprintf(&buf[len], size,
			"%6u.%03u %d,%d,%d,%d,%d,%d",
			secs, msecs,
			stats->list[entry].total_int_cnt,
			stats->list[entry].total_int_served_cnt,
			stats->list[entry].touch_int_cnt,
			stats->list[entry].touch_report_cnt,
			stats->list[entry].touch_frelease_cnt,
			stats->list[entry].touch_trelease_cnt);
		size -= n;
		len  += n;

		if ((stats->list[entry].touch_large_object_cnt != 0)
			|| (stats->list[entry].status_int_cnt != 0)
			|| (stats->list[entry].unexpected_reset != 0)
			|| (stats->list[entry].status_error != 0)) {
			n = snprintf(&buf[len], size,
			",%d,%d,%d,%d",
			stats->list[entry].touch_large_object_cnt,
			stats->list[entry].status_int_cnt,
			stats->list[entry].unexpected_reset,
			stats->list[entry].status_error);
			size -= n;
			len  += n;
		}

		n = snprintf(&buf[len], size,
			"\n");

		size -= n;
		len  += n;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: size = %d, PAGE_SIZE = %lu\n",
		__func__, size,
		PAGE_SIZE);
	mutex_unlock(&rmi4_data->stats.mutex);

	return len;
}

static ssize_t synaptics_rmi4_events_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i, size, num, entry;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_events *events = &rmi4_data->events;
	unsigned int secs, msecs;
	const struct synaptics_dsx_board_data *bdata;

	bdata = rmi4_data->hw_if->board_data;
	mutex_lock(&events->mutex);
	num = events->num_events_entries;
	entry = events->next_events_entry;

	size = 0;
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"Wakeup source:\n");

	for (i = 0; i < WAKEUP_SOURCE_MAX; i++) {
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
				"%d: %s\n",
				i,
				wakeup_source_string[i]);
	}

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"\nSlider state:\n");

	for (i = 0; i < SLIDER_STATE_MAX; i++) {
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"%d: %s\n", i, slider_state_str[i]);
	}

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"\nTIME EVENT:EVENT_DATA\n");

	if (bdata->tap_status_addr < 0)
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"wakeup - EVENT_DATA format: source\n");
	else
		size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"wakeup - EVENT_DATA format: source,failure1,failure2,failure3,failure4,failure5,failure6\n");

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"slider - EVENT_DATA format: slider state\n");
	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"face - EVENT_DATA format: detect time(ms)\n");

	size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
			"proxi - EVENT_DATA format: range,detect time(ms)\n");

	for (i = 0; i < num; i++) {
		entry = (entry > 0) ? (entry-1) : (SYNAPTICS_EVENTS_SIZE-1);
		secs  = events->list[entry].msecs/1000;
		msecs = (events->list[entry].msecs - (secs*1000));

		switch (events->list[entry].event_id) {
		case WAKEUP_UP:
			if (bdata->tap_status_addr < 0)
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
					"%6u.%3u %s:%d\n",
					secs, msecs,
					touch_event_str[events->list[entry].event_id],
					events->list[entry].event_data[0]);
			else
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
					"%6u.%3u %s:%d,%x,%x,%x,%x,%x,%x\n",
					secs, msecs,
					touch_event_str[events->list[entry].event_id],
					events->list[entry].event_data[0],
					events->list[entry].event_data[1],
					events->list[entry].event_data[2],
					events->list[entry].event_data[3],
					events->list[entry].event_data[4],
					events->list[entry].event_data[5],
					events->list[entry].event_data[6]);
			break;
		case SLIDER:
		case FACE_DETECTED:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
				"%6u.%3u %s:%d\n",
				secs, msecs,
				touch_event_str[events->list[entry].event_id],
				events->list[entry].event_data[0]);
			break;
		case PROXI_DETECTED:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
				"%6u.%3u %s:%d,%d\n",
				secs, msecs,
				touch_event_str[events->list[entry].event_id],
				events->list[entry].event_data[0],
				events->list[entry].event_data[1]);
			break;
		default:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),
				"%6u.%3u %s\n",
				secs, msecs,
				touch_event_str[events->list[entry].event_id]);
			break;
		}
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: size = %d, PAGE_SIZE = %lu\n",
		__func__, size,
		PAGE_SIZE);

	mutex_unlock(&events->mutex);

	return size;
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
static ssize_t synaptics_rmi4_mtouch_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_mtouch_counter *mtouch_counter  = &rmi4_data->mtouch_counter;
	ssize_t  size = 0;

	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"mtouch counter for %s\n", rmi4_data->input_dev->name);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"double tab enabled = %d\n", rmi4_data->wakeup_gesture.double_tap);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"swipe wake = %d\n", mtouch_counter->swipe_wakeup);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Double tap wakeups with delay 200ms to 400ms = %d\n",
			mtouch_counter->dtwakeup_delay_200ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Double tap wakeups with delay 400ms to 1000ms= %d\n",
			mtouch_counter->dtwakeup_delay_400ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Double tap wakeups with delay greater than 1000ms = %d\n",
			mtouch_counter->dtwakeup_delay_1sec);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Glass on delay 300ms to 500ms = %d\n",
			mtouch_counter->glass_on_delay_300ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Glass on delay 500ms to 750ms = %d\n",
			mtouch_counter->glass_on_delay_500ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Glass on delay 750ms to 1000ms = %d\n",
			mtouch_counter->glass_on_delay_750ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Glass on delay greater than 1000ms = %d\n",
			mtouch_counter->glass_on_delay_1000ms);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"double tap interrupt = %d\n", mtouch_counter->double_tap);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Total number of successful wakeups due to double taps = %d\n",
			mtouch_counter->successful_double_tap);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"upgrade failure = %d\n",
			mtouch_counter->upgrade_failure);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"face detected = %d\n",
			mtouch_counter->face_detection);

	for (i = 0; i < (sizeof(mtouch_counter->tap_failure)/sizeof(int)); i++)
		size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"%s = %d\n",
			tap_failure_string[i],
			mtouch_counter->tap_failure[i]);

	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"proxi detected = %d\n",
			mtouch_counter->proxi_detected);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"proxi timeout = %d\n",
			mtouch_counter->proxi_timeout);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"proxi not ready = %d\n",
			mtouch_counter->proxi_not_ready);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"proxi IO error = %d\n",
			mtouch_counter->proxi_io_error);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"not enough position buffer = %d\n",
			mtouch_counter->few_pos_buff);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"I2C RW error = %d\n",
			mtouch_counter->i2c_rw_error);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Number of accidental taps reported by touch FW while the device was in pocket(1 minute intervals) = %d\n",
			mtouch_counter->failed_taps_in_proxi);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Number of accidental taps reported by touch FW while the device was NOT in pocket(1 minute intervals) = %d\n",
			mtouch_counter->inadv_tap_failures);
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"remaining time = %lld\n",
			div_s64(ktime_to_us(hrtimer_get_remaining(
				&rmi4_data->mtouch_counter.inadv_tapdet_hrtimer)), 1000));
	size += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf),
			"Touch up without down = %d\n",
			mtouch_counter->t_up_without_down);
	/* reset the mtouch_counter */
	synaptics_rmi4_mtouch_counter_reset(&rmi4_data->mtouch_counter);

	return size;
}
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_stop_timer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (input == 1) {
		synaptics_rmi4_monitor_timer_stop(rmi4_data);

		synaptics_rmi4_stats_stop(rmi4_data);
	} else {
		synaptics_rmi4_monitor_timer_start(rmi4_data);

		synaptics_rmi4_stats_start(rmi4_data);
	}

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static ssize_t synaptics_rmi4_BIST_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	bool  passfail;
	uint16_t bist_max_cap;
	uint16_t bist_min_cap;
	int retval;
	const struct synaptics_dsx_board_data *bdata;

	passfail = false;
	bist_max_cap = 0;
	bist_min_cap = 0;
	bdata = rmi4_data->hw_if->board_data;

	retval = synaptics_rmi4_f54_do_BIST(rmi4_data, &passfail,
				&bist_max_cap, &bist_min_cap);

	if (retval)
		return snprintf(buf, PAGE_SIZE,
			"BIST TEST FAILED: %d\n", retval);
	else
		return snprintf(buf, PAGE_SIZE, "BIST TEST COMPLETE\n"
			"passfail: %s\n"
			"normal range(%u, %u)\n"
			"test result(%u, %u)\n",
			passfail ? "pass" : "fail",
			bdata->bist_min, bdata->bist_max,
			bist_min_cap, bist_max_cap);
}

static ssize_t synaptics_rmi4_max_touchpoints_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", rmi4_data->num_of_fingers);
}

static ssize_t synaptics_rmi4_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "Synaptics\n");
}

static ssize_t synaptics_rmi4_product_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n",
		rmi4_data->rmi4_mod_info.product_id_string);
}

static ssize_t synaptics_rmi4_serial_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		rmi4_data->rmi4_mod_info.serial_number);
}

static ssize_t synaptics_rmi4_firmware_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n",
		rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_config_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x%02x%02x%02x\n",
		rmi4_data->config_id[0],
		rmi4_data->config_id[1],
		rmi4_data->config_id[2],
		rmi4_data->config_id[3]);
}
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_i2c_addr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->test_i2c_addr = input;
	return count;
}

static ssize_t synaptics_rmi4_i2c_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "Address %u\n",
			rmi4_data->test_i2c_addr);
}

static ssize_t synaptics_rmi4_i2c_read_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned char data;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->test_i2c_addr,
			&data,
			sizeof(data));

	return snprintf(buf, PAGE_SIZE, "Address %u : 0x%x\n",
			rmi4_data->test_i2c_addr,
			data);
}

static ssize_t synaptics_rmi4_i2c_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	if (sscanf(buf, "%hhu", &data) != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reg_write(rmi4_data,
		rmi4_data->test_i2c_addr,
		&data,
		sizeof(data));
	return count;
}

static ssize_t synaptics_rmi4_touch_ready_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			rmi4_data->touch_ready);
}

static int synaptics_rmi4_request_recovery(
	struct synaptics_rmi4_data *rmi4_data)
{
	char *envp[2] = {"PANEL_ALIVE=0", NULL};

	dev_info(rmi4_data->pdev->dev.parent,
			"Requesting display/touch recovery\n");
	queue_delayed_work(rmi4_data->workqueue,
		&rmi4_data->recovery_work, msecs_to_jiffies(10*1000));
	kobject_uevent_env(&rmi4_data->pdev->dev.kobj, KOBJ_CHANGE, envp);

	return 0;
}

static void synaptics_rmi4_recovery_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(
			work, struct delayed_work, work);
	struct synaptics_rmi4_data *rmi4_data =
		container_of(dw, struct synaptics_rmi4_data, recovery_work);

	dev_info(rmi4_data->pdev->dev.parent,
		"Timeout waiting for recovery - requesting again\n");
	synaptics_rmi4_request_recovery(rmi4_data);
}

static int synaptics_rmi4_f12_get_wakeup_delta(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	int16_t *dx, int16_t *dy)
{
	unsigned short data_addr;
	unsigned short data_offset;
	struct synaptics_rmi4_f12_gesture_data gesture;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	int retval;

	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	data_offset = extra_data->data4_offset;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + data_offset,  (uint8_t *)&gesture,
			sizeof(struct synaptics_rmi4_f12_gesture_data));

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: failed to read wakeup delta data, rc=%d\n",
			__func__, retval);
		return retval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"gesture type=%d, prop=(%d, %d, %d %d)",
			gesture.gesture_type,
			gesture.gesture_prop0, gesture.gesture_prop1,
			gesture.gesture_prop2, gesture.gesture_prop3);
	*dx = (int8_t)gesture.gesture_prop0;
	*dy = (int8_t)gesture.gesture_prop1;
	return 0;
} /* synaptics_rmi4_f12_get_wakeup_delta */

static int synaptics_rmi4_capture_wakeup_buffer(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_f51_extra_wakeup_info *info)
{
	int i;
	uint8_t buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT*
				SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE];
	uint8_t size;
	unsigned short data_addr;
	int retval;
	const struct synaptics_dsx_board_data *bdata
		= rmi4_data->hw_if->board_data;

	if (bdata->pos_buf_addr < 0)
		return -EINVAL;
	data_addr = bdata->pos_buf_addr;
	size = SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT*
			SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr, buffer, size);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: failed to read swipe buffe, rc=%d\n",
			__func__, retval);
		return retval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent, "F51 position buffer:");
	info->first_x = 0;
	info->pos_count = 0;
	for (i = 0; i < SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT; i++) {
		uint16_t x = buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i] |
			(buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+1] << 8);
		uint16_t y = buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+2] |
			 (buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+3] << 8);

		if (x || y) {
			info->pos_count++;
			if (x != info->x[i] || y != info->y[i])
				dev_info(rmi4_data->pdev->dev.parent,
					" pos_buf[%d] = %d, %d", i, x, y);
			if (x != 0 && info->first_x == 0)
				info->first_x = x;
		}

		if (i < (sizeof(info->y)/sizeof(int)))
			info->y[i] = y;

		if (i < (sizeof(info->x)/sizeof(int)))
			info->x[i] = x;
	}
	return 0;
}

static int synaptics_rmi4_capture_wakeup_buffer_legacy(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	struct synaptics_f51_extra_wakeup_info *info)
{
	int i;
	uint8_t buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT*
				SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE];
	uint8_t size;
	unsigned short data_addr;
	unsigned short data_offset;
	int retval;

	data_addr = SYNAPTICS_F51_CUSTOM_CTRL06;
	data_offset = 0;
	size = SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT*
			SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + data_offset, buffer, size);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: failed to read swipe buffe, rc=%d\n",
			__func__, retval);
		return retval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent, "F51 position buffer:");
	info->first_x = 0;
	for (i = 0; i < SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT; i++) {
		uint16_t x = buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i] |
			(buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+1] << 8);
		uint16_t y = buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+2] |
			 (buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE*i+3] << 8);
		if (i < (sizeof(info->y)/sizeof(int)))
			info->y[i] = y;

		if (x || y) {
			dev_info(rmi4_data->pdev->dev.parent,
				" pos_buf[%d] = %d, %d", i, x, y);
			if (x != 0 && info->first_x == 0)
				info->first_x = x;
		}
	}

	data_addr = SYNAPTICS_F51_CUSTOM_CTRL06;
	data_offset = 0;
	size = SYNAPTICS_F51_CUSTOM_DATA_MAX_PEAK_SIZE +
		SYNAPTICS_F51_CUSTOM_DATA_MAX_DIFF_SIZE +
		SYNAPTICS_F51_CUSTOM_DATA_MAX_ENERGY_SIZE;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + data_offset, buffer, size);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			 "Error reading energy data");
		return retval;
	}

	info->max_peak = buffer[0] | (buffer[1] << 8);
	info->max_diff = buffer[2] | (buffer[3] << 8);
	info->max_energy = buffer[4] | (buffer[5] << 8);

	dev_info(rmi4_data->pdev->dev.parent,
		"Max Peak=%d, Max Diff=%d, Max Energy=%d",
		info->max_peak, info->max_diff, info->max_energy);


	return 0;
} /* synaptics_rmi4_capture_wakeup_buffer_legacy */

static int synaptics_rmi4_f12_get_fingerdata(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	uint8_t  *buffer,
	uint8_t  size)
{
	int retval;
	uint16_t data_addr;
	uint16_t data_offset;
	uint16_t finger_data_size =
			((sizeof(struct synaptics_rmi4_f12_finger_data)) *
					fhandler->num_of_data_points);

	data_addr = fhandler->full_addr.data_base;
	data_offset = 0;
	size = (finger_data_size > size) ? size : finger_data_size;
	retval = synaptics_rmi4_reg_read(rmi4_data, data_addr + data_offset,
						buffer, size);
	return retval;
}

static void synatptics_f12_is_contact_down(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	uint8_t  *buffer,
	uint8_t  size,
	uint8_t  finger_idx,
	uint8_t  *touch_down)
{
	struct synaptics_rmi4_f12_finger_data *finger_data = NULL;

	*touch_down = false;
	if (size >= ((finger_idx + 1) *
			sizeof(struct synaptics_rmi4_f12_finger_data)))
		finger_data = (struct synaptics_rmi4_f12_finger_data *)
			(buffer + finger_idx * sizeof(
			struct synaptics_rmi4_f12_finger_data));

	if (finger_data != NULL) {
		*touch_down =
			(finger_data->object_type_and_status ==
					F12_OBJECT_TYPE_FINGER) ||
			(finger_data->object_type_and_status ==
					F12_OBJECT_TYPE_STYLUS) ||
			(finger_data->object_type_and_status ==
					F12_OBJECT_TYPE_PALM) ||
			(finger_data->object_type_and_status ==
					F12_OBJECT_TYPE_GLOVED_FINGER);
	} else
		dev_err(rmi4_data->pdev->dev.parent,
		"Invalid ptr, buffer=%p, size=%d, idx=%d, offset=%d\n",
		buffer, size, finger_idx, (unsigned int)((finger_idx + 1) *
			sizeof(struct synaptics_rmi4_f12_finger_data)));

	dev_dbg(rmi4_data->pdev->dev.parent,
		"touch=%d, type_staus=0x%02x\n",
		*touch_down, finger_data->object_type_and_status);
	return;
}

static void synaptics_f12_finger_data_info(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	uint8_t  *buffer,
	uint8_t  size,
	uint8_t  finger_idx,
	uint8_t  *touch_down,
	int16_t  *x,
	int16_t  *y,
	int16_t  *w,
	int16_t  *h,
	int16_t  *z)
{
	struct synaptics_rmi4_f12_finger_data *finger_data = NULL;

	if (size >= ((finger_idx + 1) *
			sizeof(struct synaptics_rmi4_f12_finger_data)))
		finger_data = (struct synaptics_rmi4_f12_finger_data *)
				(buffer + finger_idx *
			sizeof(struct synaptics_rmi4_f12_finger_data));

	if (finger_data != NULL) {
		*touch_down =
			(finger_data->object_type_and_status = 0x01) ||
			(finger_data->object_type_and_status = 0x02) ||
			(finger_data->object_type_and_status = 0x03) ||
			(finger_data->object_type_and_status = 0x06);
		*x = (finger_data->x_msb << 8) + finger_data->x_lsb;
		*y = (finger_data->y_msb << 8) + finger_data->y_lsb;
		*w = finger_data->wx;
		*h = finger_data->wy;
		*z = finger_data->z;
	} else {
		touch_down = false;
		*x = 0;
		*y = 0;
		*w = 0;
		*h = 0;
		*z = 0;
	}
	return;
}

static bool synpatics_rmi4_face_detection(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char org_state_detection;
	unsigned char state_detection;
	unsigned char state;
	unsigned short data_addr;
	unsigned short ctrl_base;
	bool face_detected = false;
	struct synaptics_rmi4_f12_extra_data *extra_data;

	ctrl_base = fhandler->full_addr.ctrl_base;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;

	if (extra_data->ctrl36_offset) {

		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data13_offset,
				&state,
				sizeof(state));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read detected state\n",
				__func__);
			return false;
		}

		retval = synaptics_rmi4_reg_read(rmi4_data,
			ctrl_base + extra_data->ctrl36_offset,
			&org_state_detection,
			sizeof(org_state_detection));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				 "%s: Failed to read state detection\n",
				 __func__);
			return false;
		}

		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: F12 ctrl36 (0x%x) = %d, face detection = %d\n",
			__func__,
			ctrl_base + extra_data->ctrl36_offset,
			org_state_detection,
			state);

		state_detection = org_state_detection | F12_FACE_DETECTION;

		retval = synaptics_rmi4_reg_write(rmi4_data,
			 ctrl_base + extra_data->ctrl36_offset,
			 &state_detection,
			sizeof(state_detection));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to change state detection\n",
			__func__);
			 return false;
		}

		if (state)
			usleep(F12_FACE_DETECTION_CLEAR_DELAY_MS * 1000);
		else
			usleep(F12_FACE_DETECTION_DELAY_MS * 1000);

		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data13_offset,
				&state,
				sizeof(state));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read detected state\n",
				__func__);
			return false;
		}

		retval = synaptics_rmi4_reg_write(rmi4_data,
			 ctrl_base + extra_data->ctrl36_offset,
			 &org_state_detection,
			sizeof(org_state_detection));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to restore state detection\n",
			__func__);
		}

		if (state)
			face_detected = true;
	}

	dev_info(rmi4_data->pdev->dev.parent,
			"Face detection (0x%x) = %d\n",
			data_addr + extra_data->data13_offset,
			state);
	return face_detected;
}


static void proxi_complete(int object_detected, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	rmi4_data->proxi_pocket = object_detected;
	dev_dbg(rmi4_data->pdev->dev.parent,
			"proxi complete, object_detected = %d\n",
			rmi4_data->proxi_pocket);
	complete(&rmi4_data->proxi_completion);
}

static void synaptics_rmi4_handle_gesture_wakeup(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler)
{
	int16_t    dx, dy = 0;
	int16_t    y0, y1, y2 = 0;
	int16_t    x, y, z, w, h = 0;
	uint8_t    fingers = 0, finger_id = 0;
	uint8_t    touch_down = 0;
	uint8_t    i;
	unsigned short data_addr;
	unsigned short data_offset;
	struct synaptics_rmi4_f12_gesture_data gesture;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_event_entry event;
	struct synaptics_rmi4_mtouch_counter *mtouch_counter  = &rmi4_data->mtouch_counter;

	uint8_t    buffer[SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT*
				SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE];
	struct     synaptics_wakeup_data_t wakeup_data = {0};
	int        retval;
	int16_t    height = rmi4_data->hw_if->board_data->resolution_y;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	unsigned long start_msec;

	dev_info(rmi4_data->pdev->dev.parent,
		"%s:possible wakeup gesture detected, suspend=%d\n",
		__func__, rmi4_data->suspend);

	if (bdata->wg_no_ct) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s:No wakeup criteria", __func__);

		/* READ WAKEUP GESTURE TYPE and DATA*/
		data_addr = fhandler->full_addr.data_base;
		extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
		data_offset = extra_data->data4_offset;
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + data_offset,  (uint8_t *)&gesture,
				sizeof(struct synaptics_rmi4_f12_gesture_data));

		dev_dbg(rmi4_data->pdev->dev.parent,
			"read wakeup gesture type, data_addr = %d, data_offset = %d",
			data_addr, data_offset);

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: failed to read wakeup gesture data, rc=%d\n",
				__func__, retval);
			return;
		}

		dev_dbg(rmi4_data->pdev->dev.parent,
			"gesture type=%d, prop=(%d, %d, %d %d)",
			gesture.gesture_type,
			gesture.gesture_prop0, gesture.gesture_prop1,
			gesture.gesture_prop2, gesture.gesture_prop3);

		/* Check What Gesture is Completed and extra Criteria and Take Action*/
		switch (gesture.gesture_type) {
		case 0x07:
			dx = (int8_t)gesture.gesture_prop0;
			dy = (int8_t)gesture.gesture_prop1;

			dev_dbg(rmi4_data->pdev->dev.parent,
			"Wakeup Swipe dx %d dy %d ", dx, dy);

			rmi4_data->mtouch_counter.swipe_wakeup++;

			if (synpatics_rmi4_face_detection(rmi4_data, fhandler))
				return;

			synaptics_rmi4_capture_wakeup_buffer(
				rmi4_data, &rmi4_data->extra_wakeup_info);

			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 0);
			input_sync(rmi4_data->input_dev);
			break;
		case 0x03:
			x = (gesture.gesture_prop1 << 4) |
					gesture.gesture_prop0;
			y = (gesture.gesture_prop3 << 4) |
					gesture.gesture_prop2;

			dev_dbg(rmi4_data->pdev->dev.parent,
			"Wakeup Double Tap occurred at position x %d y %d ", x, y);

			rmi4_data->mtouch_counter.double_tap++;
			event.event_id = WAKEUP_GESTURE_DETECTED;
			synaptics_record_events(rmi4_data, &event);

			if (rmi4_data->smart_flip_state)
				dev_info(rmi4_data->pdev->dev.parent,
					"smart_flip_state = %d, by pass proxi check",
					rmi4_data->smart_flip_state);
			else if (rmi4_data->proxi_check) {
				start_msec = jiffies_to_msecs(jiffies);
				rmi4_data->proxi_pocket = -ETIMEDOUT;

				INIT_COMPLETION(rmi4_data->proxi_completion);
				get_proxi_value_async(proxi_complete, rmi4_data);
				retval = wait_for_completion_timeout(&rmi4_data->proxi_completion,
						msecs_to_jiffies(PROXI_TIMEOUT));

				event.event_id = PROXI_DETECTED;
				event.event_data[1] = jiffies_to_msecs(jiffies) - start_msec;

				if (retval > 0)
					event.event_data[0] = rmi4_data->proxi_pocket;
				else
					event.event_data[0] = -ETIMEDOUT;

				switch (event.event_data[0]) {
				case 1:
					mtouch_counter->proxi_detected++;
					break;
				case -ETIMEDOUT:
					mtouch_counter->proxi_timeout++;
					break;
				case -EIO:
					mtouch_counter->proxi_io_error++;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
					synaptics_dsx_ddt_send(
					rmi4_data, MTOUCH_DEBUG, MTOUCH_DBG_PROXI_IO_ERROR);
#endif
					break;
				case -EAGAIN:
					mtouch_counter->proxi_not_ready++;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
					synaptics_dsx_ddt_send(rmi4_data, MTOUCH_PROXI_NOT_READY, 0);
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/
					break;
				default:
					break;
				}

				synaptics_record_events(rmi4_data, &event);

				if (mtouch_counter->proxi_detected ==
						mtouch_counter->double_tap) {
					dev_info(rmi4_data->pdev->dev.parent,
						"Possible dirt in the housing of the proxi sensor that is preventing the wakeup. Get it cleaned");
				}

				if (rmi4_data->proxi_pocket == 1) {
					record_wakeup_failures_in_proxi(true, rmi4_data);
					dev_info(rmi4_data->pdev->dev.parent,
						"Proxi pocket detected, refuse to wakeup");
					return;
				}
			}

			if (rmi4_data->face_detection_check) {
				start_msec = jiffies_to_msecs(jiffies);
				if (synpatics_rmi4_face_detection(rmi4_data, fhandler)) {
					rmi4_data->mtouch_counter.face_detection++;
					event.event_id = FACE_DETECTED;
					event.event_data[0] = jiffies_to_msecs(jiffies) - start_msec;
					synaptics_record_events(rmi4_data, &event);
					return;
				}
			}

			synaptics_rmi4_capture_wakeup_buffer(
				rmi4_data, &rmi4_data->extra_wakeup_info);

			if (rmi4_data->extra_wakeup_info.pos_count < POSITION_BUFFER_MIN) {
				mtouch_counter->few_pos_buff++;
				dev_info(rmi4_data->pdev->dev.parent,
					"position buffer count < %d, refuse to wakeup",
					POSITION_BUFFER_MIN);
				return;
			}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
			synaptics_rmi4_inadvertent_tapdetect_timer_restart(rmi4_data);
			dev_info(rmi4_data->pdev->dev.parent,
				"report wake up event");
#endif

			rmi4_data->wakeup_source = WAKEUP_BY_DOUBLE_TAP;
			rmi4_data->wakeup_check_mask = WAKEUP_CHECK_RESUME | WAKEUP_CHECK_GLASS_ON;
			rmi4_data->mtouch_counter.successful_double_tap++;
			rmi4_data->dtwakeup_time_ms = get_timestamp();
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 0);
			input_sync(rmi4_data->input_dev);

			break;
		case 0x08: /*One-Finger Circle*/
			/*launch camera and power key*/
		default:
			dev_info(rmi4_data->pdev->dev.parent,
			"Gesture Type Not supported");
			break;
		}

		return;
	}

	/* Not supported on f12? */
	wakeup_data.large_object = false;


	synaptics_rmi4_f12_get_wakeup_delta(rmi4_data, fhandler, &dx, &dy);

	synaptics_rmi4_capture_wakeup_buffer_legacy(rmi4_data, fhandler,
			&rmi4_data->extra_wakeup_info);

	y0 = rmi4_data->extra_wakeup_info.y[0];
	y1 = rmi4_data->extra_wakeup_info.y[1];
	y2 = rmi4_data->extra_wakeup_info.y[2];

	if (0 == y0 || 0 == y1 || 0 == y2) {
		dev_info(rmi4_data->pdev->dev.parent,
			"Position data missing at start of swipe - skipping"
			" initial position check\n");
	} else {
		int y_avg_velocity = ((y0-y1) +
				(y1-y2)) * SYNAPTICS_MAGIC_MULTIPLIER;
		int magic_y_threshold = height -
				(y_avg_velocity * SYNAPTICS_MAGIC_FRACTION);
		dev_info(rmi4_data->pdev->dev.parent,
			" y_avg_vel: %d, magic: %d",
			y_avg_velocity, magic_y_threshold);
		if (y0 < SYNAPTICS_MAGIC_MAX_Y && y0 < magic_y_threshold) {
			dev_info(rmi4_data->pdev->dev.parent,
				"swipe rejected - started too high");
			return;
		}


		if (x > 0) {
			int first_x = rmi4_data->extra_wakeup_info.first_x;
			if (x < bdata->fw_wake_zone_top_left_x1 &&
							x < first_x) {
				dev_info(rmi4_data->pdev->dev.parent,
				"swipe rejected - left edge "
				"swipe first_x=%d left_x1=%d",
				first_x, bdata->fw_wake_zone_top_left_x1);
				return;
			}

			if (x > bdata->fw_wake_zone_bottom_right_x1 &&
							 x > first_x) {
				dev_info(rmi4_data->pdev->dev.parent,
					"swipe rejected - right "
					"edge swipe first_x=%d right_x1=%d",
					first_x,
					bdata->fw_wake_zone_bottom_right_x1);
				return;
			}
		}
	}


	retval = synaptics_rmi4_f12_get_fingerdata(
			rmi4_data, fhandler, buffer, sizeof(buffer));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
		"Failed to read sample data from device");
		return;
	}

	for (i = 0; i < fhandler->num_of_data_points; ++i) {
		synatptics_f12_is_contact_down(rmi4_data,
			fhandler, buffer, sizeof(buffer), i, &touch_down);
		if (touch_down) {
			fingers++;
			finger_id = i;
		}
	}

	if ((fingers == 1) && (finger_id == 0)) {
		dev_info(rmi4_data->pdev->dev.parent,
			"finger contact %d", finger_id);
		synaptics_f12_finger_data_info(rmi4_data, fhandler,
			buffer, sizeof(buffer), finger_id,
			&touch_down, &x, &y, &w, &h, &z);
	} else if ((fingers == 1) && (finger_id > 0)) {
		dev_info(rmi4_data->pdev->dev.parent,
			 "Not a wakeup swipe - invalid finger id");
		return;
	}

	wakeup_data.fingers = fingers;
	wakeup_data.dy = dy;
	wakeup_data.dx = dx;
	wakeup_data.x = x;
	wakeup_data.y = y;
	wakeup_data.z = z;
	wakeup_data.w = w;
	wakeup_data.h = h;
	wakeup_data.vel = ((dx * dx) + (dy * dy));

	dev_dbg(rmi4_data->pdev->dev.parent,
	    "wakeup point dx %d dy %d first point x %d, y %d w %d h %d z %d"
	    " fingers=%d\n", dx, dy, x, y, w, h, z, fingers);
	if (synaptics_is_wakeup_swipe(&bdata->wakeup_criteria,
					&wakeup_data, NULL) == 1) {

		input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 1);
		input_sync(rmi4_data->input_dev);
		input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 0);
		input_sync(rmi4_data->input_dev);
	}

	return;
}

 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(
		struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		bool report)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char finger_status_reg[3];
	unsigned char detected_gestures;
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
#ifdef REPORT_2D_Z
	int z;
#endif
#ifdef REPORT_2D_W
	int wx;
	int wy;
#endif
	int temp;
	struct synaptics_rmi4_f11_data_1_5 data;
	struct synaptics_rmi4_f11_extra_data *extra_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;

	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	if (rmi4_data->suspend && rmi4_data->wakeup_gesture.data) {
		if (!bdata->wg_no_ct) {
			retval = synaptics_rmi4_reg_read(rmi4_data,
					data_addr + extra_data->data38_offset,
					&detected_gestures,
					sizeof(detected_gestures));
			if (retval < 0)
				return retval;

			if (detected_gestures)
				synaptics_rmi4_handle_gesture_wakeup(
							rmi4_data, fhandler);
		} else
			synaptics_rmi4_handle_gesture_wakeup(
						rmi4_data, fhandler);

		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return retval;

	if (!report) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: No need to report\n", __func__);
		return 0;
	}

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >>
				 finger_shift) & MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * sizeof(data.data));
			retval = synaptics_rmi4_reg_read(rmi4_data,
					data_offset,
					data.data,
					sizeof(data.data));
			if (retval < 0) {
				mutex_unlock(&(
					rmi4_data->rmi4_report_mutex));
				return retval;
			}

			x = (data.x_position_11_4 << 4) |
						data.x_position_3_0;
			y = (data.y_position_11_4 << 4) |
						data.y_position_3_0;
#ifdef REPORT_2D_Z
			z = data.z;
#endif
#ifdef REPORT_2D_W
			wx = data.wx;
			wy = data.wy;
#endif

			if (rmi4_data->hw_if->board_data->swap_axes) {
				temp = x;
				x = y;
				y = temp;
#ifdef REPORT_2D_W
				temp = wx;
				wx = wy;
				wy = temp;
#endif
			}

			if (rmi4_data->hw_if->board_data->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->hw_if->board_data->y_flip)
				y = rmi4_data->sensor_max_y - y;

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_Z
			/* report 0xf9 (MAX_Z-6) for edge and 0x00 - 0xf6 (MAX_Z-9) for the reset */
			input_report_abs(rmi4_data->input_dev, ABS_MT_PRESSURE,
				(rmi4_data->touch_edge[finger] == 0) ? min(z, MAX_Z-9) : (MAX_Z-6));
#endif
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif

			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Finger I=%d,S=%d,X=%d,Y=%d,W=%d,H=%d\n",
				__func__, finger,
				finger_status, x, y, wx, wy);

			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

 /**
 * synaptics_rmi4_check_edge_touches
 *
 * This helper function check for touches along the edges.
 *
 */
static int synaptics_rmi4_check_edge_touches(
	struct synaptics_rmi4_data *rmi4_data,
	unsigned char   finger,
	int		x,
	int		y,
	int		z)
{
	int          edge_val = -1;

	if ((false == rmi4_data->touch_edge[finger]) &&
	    ((x == 0) || (x == rmi4_data->sensor_max_x)))
		edge_val = 1;
	else if ((true == rmi4_data->touch_edge[finger]) &&
		  (x != 0) && (x != rmi4_data->sensor_max_x))
		edge_val = 0;

	if (-1 != edge_val) {
		rmi4_data->touch_edge[finger] = (edge_val == 1);
	}

	return 0;
}

 /**
 * synaptics_rmi4_cancel_touch(
 *
 * This helper function inject a touch cancel event to the upper layer.
 */
static int synaptics_rmi4_cancel_touch(
	struct synaptics_rmi4_data *rmi4_data,
	unsigned char   finger)
{
	input_mt_slot(rmi4_data->input_dev, finger);
	input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 1);

	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 1);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 1);

	rmi4_data->touch_state = true;

#ifdef REPORT_2D_Z
	/* report 0xfd for cancel */
	input_report_abs(rmi4_data->input_dev, ABS_MT_PRESSURE, MAX_Z-2);
#endif

	input_sync(rmi4_data->input_dev);
	return 0;
}

 /**
 * synaptics_rmi4_f12_abs_report
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(
	struct synaptics_rmi4_data *rmi4_data,
	struct synaptics_rmi4_fn *fhandler,
	bool report)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	bool slider_in_transition;
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned char detected_gestures;
	unsigned short data_addr;
	int x;
	int y;
#ifdef REPORT_2D_Z
	int z;
#endif
#ifdef REPORT_2D_W
	int wx;
	int wy;
#endif
	int temp;
	unsigned long no_touch_msec;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: suspend=%d\n",
			__func__, rmi4_data->suspend);
	if (rmi4_data->suspend && rmi4_data->wakeup_gesture.data) {
		if (!bdata->wg_no_ct) {
			retval = synaptics_rmi4_reg_read(rmi4_data,
					data_addr + extra_data->data4_offset,
					&detected_gestures,
					sizeof(detected_gestures));
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: failed to read detected gestures. rc %d\n",
				__func__, retval);
				return retval;
			}

			if (detected_gestures)
				synaptics_rmi4_handle_gesture_wakeup(
							rmi4_data, fhandler);
		} else
			synaptics_rmi4_handle_gesture_wakeup(
						rmi4_data, fhandler);

		return 0;
	}

	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return retval;

		/* Start checking from the highest bit */
		temp = extra_data->data15_size - 1; /* Highest byte */
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[temp] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else {
				temp--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

#ifndef F12_DATA_15_WORKAROUND
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
#endif
	}

	if (!report) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: No need to report %d fingers\n",
			__func__,
			fingers_to_process);
		return 0;
	}

#ifdef F12_DATA_15_WORKAROUND
	fingers_to_process = max(fingers_to_process,
				rmi4_data->fingers_already_present);
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of fingers to process = (%d, %d)\n",
			__func__, fingers_to_process,
				rmi4_data->fingers_already_present);
#endif

	if (!fingers_to_process) {
		if (false != rmi4_data->ignore_touch) {
			dev_info(rmi4_data->pdev->dev.parent,
				"%s: clear ignore_touch\n", __func__);
			rmi4_data->ignore_touch = false;
		}
		rmi4_data->touch_obj_cnt = 0;
		if (!rmi4_data->resume_notouch_jiffies) {
			rmi4_data->resume_notouch_jiffies = jiffies;
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: no touch, rmi4_data->resume_notouch_jiffies=%lu\n",
				__func__, rmi4_data->resume_notouch_jiffies);
		}
		synaptics_rmi4_free_fingers(rmi4_data);
		return 0;
	}

	if (rmi4_data->resume_ignore_touch && rmi4_data->resume_notouch_jiffies) {
		no_touch_msec = jiffies_to_msecs(jiffies - rmi4_data->resume_notouch_jiffies);

		if (no_touch_msec > RESUME_IGNORE_TOUCH_DELAY) {
			dev_info(rmi4_data->pdev->dev.parent,
			"Resume free from touch for %lu ms, start to report touch\n",
			no_touch_msec);
			rmi4_data->resume_ignore_touch = false;
		} else {
			dev_info(rmi4_data->pdev->dev.parent,
				"Resume free from touch for %lu ms, ignore touch\n",
				no_touch_msec);
		}
	}

	if (rmi4_data->resume_ignore_touch) {
		rmi4_data->resume_notouch_jiffies = 0;
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: touch happened, reset rmi4_data->resume_notouch_jiffies=%lu\n",
			__func__, rmi4_data->resume_notouch_jiffies);
		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: failed to read finger data, return %d\n",
			__func__, retval);
		return retval;
	}

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

	/* if we are currently ignoring touch check if all fingers */
	/* are released */
	touch_count = 0;
	slider_in_transition = ((rmi4_data->slider_state == SLIDER_STATE_OPENING) ||
				  (rmi4_data->slider_state == SLIDER_STATE_CLOSING));
	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status;

		switch (finger_status) {
		case F12_NO_OBJECT_STATUS:
			break;
		default:
			/* something is on */
			touch_count++;
			if ((true == bdata->dis_while_sliding) &&
			    (true == slider_in_transition) &&
			    (false == rmi4_data->ignore_touch)) {
				synaptics_rmi4_cancel_touch(rmi4_data, finger);
				rmi4_data->ignore_touch = true;
				dev_info(rmi4_data->pdev->dev.parent,
					"%s: set ignore_touch to true\n", __func__);
			}
			break;
		}
	}

	if ((false != rmi4_data->ignore_touch) && (touch_count == 0) &&
	    (false == slider_in_transition)) {
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: reset ignore_touch to false\n", __func__);
		rmi4_data->ignore_touch = false;
	} else if (false != rmi4_data->ignore_touch) {
		/* simply ignore the touches while ignore_touch is set */
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: ignore_touch=%d, ignoring touches\n",
			__func__, rmi4_data->ignore_touch);
		return 0;
	}

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	touch_count = 0;
	rmi4_data->touch_obj_cnt = 0;
	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status;

		switch (finger_status) {
		case F12_FINGER_STATUS:
		case F12_GLOVED_FINGER_STATUS:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 1);
#endif


#ifdef F12_DATA_15_WORKAROUND
			rmi4_data->fingers_already_present = finger + 1;
#endif

			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_Z
			z = finger_data->z;
#endif
#ifdef REPORT_2D_W
			wx = finger_data->wx;
			wy = finger_data->wy;
#endif

			if (rmi4_data->hw_if->board_data->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->hw_if->board_data->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->hw_if->board_data->y_flip)
				y = rmi4_data->sensor_max_y - y;

			if (bdata->enable_abs_edge) {
				synaptics_rmi4_check_edge_touches(
					rmi4_data, finger, x, y, z);
			}

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);

			rmi4_data->touch_state = true;

			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_Z
			/* report 0xf9 (MAX_Z-6) for edge and 0x00 - 0xf6 (MAX_Z-9) for the reset */
			input_report_abs(rmi4_data->input_dev, ABS_MT_PRESSURE,
				(rmi4_data->touch_edge[finger] == 0) ? min(z, MAX_Z-9) : (MAX_Z-6));
#endif
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			rmi4_data->stats.touch_report_cnt++;
#ifdef F12_DATA_15_WORKAROUND
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Finger I=%d,S=%d,X=0x%x,"
				"Y=0x%x,W=%d,H=%d,FP=%d\n",
				__func__, finger,
				finger_status, x, y, wx, wy,
				rmi4_data->fingers_already_present);
#else
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Finger I=%d,S=%d,X=0x%x,"
				"Y=0x%x,W=%d,H=%d\n",
				__func__, finger,
				finger_status, x, y, wx, wy);
#endif

			touch_count++;
			break;
		case F12_PALM_STATUS:
#ifdef F12_DATA_15_WORKAROUND
			rmi4_data->fingers_already_present = finger + 1;
#endif
			synaptics_rmi4_cancel_touch(rmi4_data, finger);
			touch_count++;
			rmi4_data->ignore_touch = true;
			rmi4_data->stats.touch_large_object_cnt++;
			dev_info(rmi4_data->pdev->dev.parent,
				"Large object detected\n");
			break;
		default:
			if (bdata->enable_abs_edge) {
				synaptics_rmi4_check_edge_touches(
					rmi4_data, finger, -1, -1, -1);
			}
#ifdef TYPE_B_PROTOCOL
			rmi4_data->stats.touch_frelease_cnt++;
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 0);

			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Finger release f=%d, I=%d,S=%d\n",
				__func__, fingers_to_process, finger, finger_status);
#else
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: other finger status %d\n",
				__func__, finger_status);
#endif
			break;
		}

		if (F12_NO_OBJECT_STATUS != finger_status)
			rmi4_data->touch_obj_cnt++;
	}

	if (touch_count == 0) {
#ifdef F12_DATA_15_WORKAROUND
		rmi4_data->fingers_already_present = 0;
#endif

		rmi4_data->stats.touch_trelease_cnt++;
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);

		if (!rmi4_data->touch_state) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: a touch up without down!\n",
				__func__);
			rmi4_data->mtouch_counter.t_up_without_down++;
		}

		rmi4_data->touch_state = false;

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Touch UP\n",
				__func__);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

static int synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		bool report)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (rmi4_data->do_once) {
		memset(rmi4_data->current_status, 0,
				 sizeof(rmi4_data->current_status));
#ifdef NO_0D_WHILE_2D
		memset(rmi4_data->before_2d_status, 0,
				 sizeof(rmi4_data->before_2d_status));
		memset(rmi4_data->while_2d_status, 0,
				 sizeof(rmi4_data->while_2d_status));
#endif
		rmi4_data->do_once = 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read button data registers\n",
				__func__);
		return retval;
	}

	if (!report) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: No need to report\n", __func__);
		return 0;
	}

	data = f1a->button_data_buffer;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (rmi4_data->current_status[button] == status)
			continue;
		else
			rmi4_data->current_status[button] = status;

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				rmi4_data->before_2d_status[button] = 1;
			} else {
				if (rmi4_data->while_2d_status[button] == 1) {
					rmi4_data->while_2d_status[button] = 0;
					continue;
				} else {
					rmi4_data->before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (rmi4_data->before_2d_status[button] == 1) {
				rmi4_data->before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					rmi4_data->while_2d_status[button] = 1;
				else
					rmi4_data->while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	if (touch_count)
		input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return 0;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static int synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler, bool report)
{
	int retval = 0;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		retval = synaptics_rmi4_f11_abs_report(rmi4_data, fhandler, report);

		if (retval < 0)
			return retval;
		else if (retval)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		retval = synaptics_rmi4_f12_abs_report(rmi4_data, fhandler, report);

		if (retval < 0)
			return retval;
		else if (retval)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		retval = synaptics_rmi4_f1a_report(rmi4_data, fhandler, report);
		break;
	default:
		break;
	}

	return retval;
}

static int synaptics_rmi4_poll_sensor_report(
	struct synaptics_rmi4_data *rmi4_data,
	bool			   report)
{
	int 	retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	if (rmi4_data->suspend)
		return 0;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Traverse the function handler list and polling
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				retval = synaptics_rmi4_report_touch(
						rmi4_data,
						fhandler, report);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to report touch\n",
						__func__);
					break;
				}
			}
		}
	}
	return retval;
}

 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static int synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;
	struct device *dev = rmi4_data->pdev->dev.parent;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	status.data[0] = data[0];
	if (status.unconfigured && !status.flash_prog) {
		dev_info(dev,
				"%s: Reconfiguring controller\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(dev,
					"%s: Failed to reinit device\n",
					__func__);
			return retval;
		}
	}


	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					retval = synaptics_rmi4_report_touch(
							rmi4_data,
							fhandler, true);
					if (retval < 0) {
						dev_err(dev,
						"%s: Failed to report "
						"touch\n", __func__);
						return retval;
					}

					switch (fhandler->fn_number) {
					case SYNAPTICS_RMI4_F11:
					case SYNAPTICS_RMI4_F12:
						rmi4_data->stats.touch_int_cnt++;
						break;
					case SYNAPTICS_RMI4_F01:
						rmi4_data->stats.status_int_cnt++;
						if (status.status_code == STATUS_RESET_OCCURRED) {
							rmi4_data->stats.unexpected_reset++;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
							synaptics_dsx_ddt_send(rmi4_data,
								UNEXPECTED_RESET, 0);
#endif
							return -EAGAIN;
						} else if (status.status_code != STATUS_NO_ERROR) {
							rmi4_data->stats.status_error++;
							dev_err(dev,
								"%s: Status error %d\n",
								__func__, status.status_code);
							return -EAGAIN;
						}
						break;
					default:
						break;
					}
				}
			}
		}
	}

	mutex_lock(&rmi4_data->exp_data.mutex);
	if (!list_empty(&rmi4_data->exp_data.list)) {
		list_for_each_entry(exp_fhandler,
				&rmi4_data->exp_data.list,
				link) {
			if (!exp_fhandler->insert &&
					!exp_fhandler->remove &&
					(exp_fhandler->exp_fn->attn != NULL))
				retval = exp_fhandler->exp_fn->
						     attn(rmi4_data, intr[0]);
			if (retval)
				break;
		}
	}
	mutex_unlock(&rmi4_data->exp_data.mutex);

	return retval;
}

static void synaptics_rmi4_diable_irq_nosync(
	int irq, struct synaptics_rmi4_data *rmi4_data, bool disable)
{
	if (disable) {
		disable_irq_nosync(irq);
		rmi4_data->en_irq_counter.irq_nosync--;
	} else {
		enable_irq(irq);
		rmi4_data->en_irq_counter.irq_nosync++;
	}
}

 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;

	synaptics_rmi4_diable_irq_nosync(irq, rmi4_data, true);

	pm_stay_awake(&rmi4_data->pdev->dev);

	queue_work(rmi4_data->workqueue,
			&rmi4_data->irq_work);

	return IRQ_HANDLED;
}

/*
 * synaptics_rmi4_irq_work()
 *
 * This function is the ISR worker and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static void synaptics_rmi4_irq_work(struct work_struct *work)
{
	int retval = 0;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					irq_work);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	rmi4_data->stats.update = true;
	rmi4_data->stats.total_int_cnt++;

	mutex_lock(&rmi4_data->rmi4_irq_mutex);
	if (true != rmi4_data->irq_enabled) {
		/* IRQ is disabled. Must be a race conditon */
		dev_dbg(rmi4_data->pdev->dev.parent,
			"IRQ is disabled. exit.\n");
		mutex_unlock(&rmi4_data->rmi4_irq_mutex);
		goto exit;
	}
	mutex_unlock(&rmi4_data->rmi4_irq_mutex);

	if (gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state) {
		rmi4_data->stats.update = false;
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s:No interrupt (irq_gpio = %d), abandon\n",
				__func__,
				gpio_get_value(bdata->irq_gpio));
		synaptics_rmi4_diable_irq_nosync(rmi4_data->irq, rmi4_data, false);
		goto exit;
	}

	rmi4_data->stats.total_int_served_cnt++;
	retval = synaptics_rmi4_sensor_report(rmi4_data);
	synaptics_rmi4_diable_irq_nosync(rmi4_data->irq, rmi4_data, false);

	rmi4_data->stats.update = false;

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed sensor report, reset controller\n",
			__func__);
		rmi4_data->reset_device(rmi4_data, true);
	} else {
		mutex_lock(&(rmi4_data->rmi4_reset_mutex));
		rmi4_data->num_failures = 0;
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		synaptics_rmi4_monitor_timer_restart(rmi4_data);
	}

exit:
	pm_relax(&rmi4_data->pdev->dev);
	return;
}

static int synaptics_rmi4_int_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short intr_addr;

	intr_mask = rmi4_data->intr_mask;

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (enable) {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to write "
						"interrupt mask\n",
						__func__);
					return retval;
				}
			} else {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&zero,
						sizeof(zero));
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to zero "
						"interrupt mask\n",
						__func__);
					return retval;
				}
			}
		}
	}

	if (retval > 0)
		retval = 0;
	return retval;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable, bool attn_only)
{
	int retval = 0;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (attn_only) {
		retval = synaptics_rmi4_int_enable(rmi4_data, enable);
		return retval;
	}

	mutex_lock(&rmi4_data->rmi4_irq_mutex);
	if (enable) {
		if (rmi4_data->irq_enabled) {
			mutex_unlock(&rmi4_data->rmi4_irq_mutex);
			return retval;
		}

		retval = synaptics_rmi4_int_enable(rmi4_data, false);
		if (retval < 0) {
			mutex_unlock(&rmi4_data->rmi4_irq_mutex);
			return retval;
		}

		retval = request_irq(rmi4_data->irq,
				synaptics_rmi4_irq, bdata->irq_flags,
				PLATFORM_DRIVER_NAME, rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to create irq thread(%d)\n",
					__func__, retval);
			mutex_unlock(&rmi4_data->rmi4_irq_mutex);
			return retval;
		}
		rmi4_data->irq_enabled = true;


		retval = synaptics_rmi4_int_enable(rmi4_data, true);
		if (retval < 0) {
			rmi4_data->irq_enabled = false;
			mutex_unlock(&rmi4_data->rmi4_irq_mutex);
			return retval;
		}

		rmi4_data->en_irq_counter.irq++;

	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq(rmi4_data->irq);
			free_irq(rmi4_data->irq, rmi4_data);
			rmi4_data->irq_enabled = false;
			rmi4_data->en_irq_counter.irq--;
		}
	}
	mutex_unlock(&rmi4_data->rmi4_irq_mutex);

	return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	unsigned char ii;
	unsigned char intr_offset;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f01_query_base_addr = fd->query_base_addr;
	rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
	rmi4_data->f01_data_base_addr = fd->data_base_addr;
	rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

	return 0;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char offset;
	unsigned char fingers_supported;
	struct synaptics_rmi4_f11_extra_data *extra_data;
	struct synaptics_rmi4_f11_query_0_5 query_0_5;
	struct synaptics_rmi4_f11_query_7_8 query_7_8;
	struct synaptics_rmi4_f11_query_9 query_9;
	struct synaptics_rmi4_f11_query_12 query_12;
	struct synaptics_rmi4_f11_query_27 query_27;
	struct synaptics_rmi4_f11_ctrl_6_9 control_6_9;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			query_0_5.data,
			sizeof(query_0_5.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if (query_0_5.num_of_fingers <= 4)
		fhandler->num_of_data_points = query_0_5.num_of_fingers + 1;
	else if (query_0_5.num_of_fingers == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + 6,
			control_6_9.data,
			sizeof(control_6_9.data));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = control_6_9.sensor_max_x_pos_7_0 |
			(control_6_9.sensor_max_x_pos_11_8 << 8);
	rmi4_data->sensor_max_y = control_6_9.sensor_max_y_pos_7_0 |
			(control_6_9.sensor_max_y_pos_11_8 << 8);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	fhandler->data = NULL;

	offset = sizeof(query_0_5.data);

	/* query 6 */
	if (query_0_5.has_rel)
		offset += 1;

	/* queries 7 8 */
	if (query_0_5.has_gestures) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_7_8.data,
				sizeof(query_7_8.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_7_8.data);
	}

	/* query 9 */
	if (query_0_5.has_query_9) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_9.data,
				sizeof(query_9.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_9.data);
	}

	/* query 10 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += 1;

	/* query 11 */
	if (query_0_5.has_query_11)
		offset += 1;

	/* query 12 */
	if (query_0_5.has_query_12) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_12.data,
				sizeof(query_12.data));
		if (retval < 0)
			return retval;

		offset += sizeof(query_12.data);
	}

	/* query 13 */
	if (query_0_5.has_jitter_filter)
		offset += 1;

	/* query 14 */
	if (query_0_5.has_query_12 && query_12.has_general_information_2)
		offset += 1;

	/* queries 15 16 17 18 19 20 21 22 23 24 25 26*/
	if (query_0_5.has_query_12 && query_12.has_physical_properties)
		offset += 12;

	/* query 27 */
	if (query_0_5.has_query_27) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.query_base + offset,
				query_27.data,
				sizeof(query_27.data));
		if (retval < 0)
			return retval;

		rmi4_data->f11_wakeup_gesture = query_27.has_wakeup_gesture;
	}

	if (!rmi4_data->f11_wakeup_gesture)
		return retval;

	/* data 0 */
	fingers_supported = fhandler->num_of_data_points;
	offset = (fingers_supported + 3) / 4;

	/* data 1 2 3 4 5 */
	offset += 5 * fingers_supported;

	/* data 6 7 */
	if (query_0_5.has_rel)
		offset += 2 * fingers_supported;

	/* data 8 */
	if (query_0_5.has_gestures && query_7_8.data[0])
		offset += 1;

	/* data 9 */
	if (query_0_5.has_gestures && (query_7_8.data[0] || query_7_8.data[1]))
		offset += 1;

	/* data 10 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_pinch || query_7_8.has_flick))
		offset += 1;

	/* data 11 12 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_flick || query_7_8.has_rotate))
		offset += 2;

	/* data 13 */
	if (query_0_5.has_gestures && query_7_8.has_touch_shapes)
		offset += (fingers_supported + 3) / 4;

	/* data 14 15 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones ||
			query_7_8.has_multi_finger_scroll ||
			query_7_8.has_chiral_scroll))
		offset += 2;

	/* data 16 17 */
	if (query_0_5.has_gestures &&
			(query_7_8.has_scroll_zones &&
			query_7_8.individual_scroll_zones))
		offset += 2;

	/* data 18 19 20 21 22 23 24 25 26 27 */
	if (query_0_5.has_query_9 && query_9.has_contact_geometry)
		offset += 10 * fingers_supported;

	/* data 28 */
	if (query_0_5.has_bending_correction ||
			query_0_5.has_large_object_suppression)
		offset += 1;

	/* data 29 30 31 */
	if (query_0_5.has_query_9 && query_9.has_pen_hover_discrimination)
		offset += 3;

	/* data 32 */
	if (query_0_5.has_query_12 &&
			query_12.has_small_object_detection_tuning)
		offset += 1;

	/* data 33 34 */
	if (query_0_5.has_query_27 && query_27.f11_query27_b0)
		offset += 2;

	/* data 35 */
	if (query_0_5.has_query_12 && query_12.has_8bit_w)
		offset += fingers_supported;

	/* data 36 */
	if (query_0_5.has_bending_correction)
		offset += 1;

	/* data 37 */
	if (query_0_5.has_query_27 && query_27.has_data_37)
		offset += 1;

	/* data 38 */
	if (query_0_5.has_query_27 && query_27.has_wakeup_gesture)
		extra_data->data38_offset = offset;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;

	if (ctrl28)
		rmi4_data->ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

 /**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_11_offset;
	unsigned char ctrl_20_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_27_offset;
	unsigned char ctrl_28_offset;
	unsigned char ctrl_36_offset = 0;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	unsigned char size_of_query_5;
	struct synaptics_rmi4_f12_query_5 query_5 = { { {0} } };
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;
	uint16_t  pitch_of_rx_q4_12, pitch_of_tx_q4_12;

	unsigned char ctrl_20_data[3] = {1, 1, 0};

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 4,
			&size_of_query_5,
			sizeof(size_of_query_5));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			size_of_query_5);
	if (retval < 0)
		return retval;

	ctrl_8_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present;

	ctrl_11_offset = ctrl_8_offset +
			query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present;

	ctrl_20_offset = ctrl_8_offset +
			query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present;

	ctrl_23_offset = ctrl_20_offset +
			query_5.ctrl20_is_present +
			query_5.ctrl21_is_present +
			query_5.ctrl22_is_present;

	ctrl_27_offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present;

	ctrl_28_offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present +
			query_5.ctrl27_is_present;

	if ((size_of_query_5 > 5) && query_5.ctrl36_is_present) {
		ctrl_36_offset = ctrl_28_offset +
				query_5.ctrl28_is_present +
				query_5.ctrl29_is_present +
				query_5.ctrl30_is_present +
				query_5.ctrl31_is_present +
				query_5.ctrl32_is_present +
				query_5.ctrl33_is_present +
				query_5.ctrl34_is_present +
				query_5.ctrl35_is_present;
	}
	extra_data->ctrl36_offset = ctrl_36_offset;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			size_of_query8);
	if (retval < 0)
		return retval;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;

	if ((size_of_query8 >= 3) && (query_8.data13_is_present)) {
		extra_data->data13_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present;
	}

	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present +
				query_8.data13_is_present +
				query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Writing to F12_CTRL_20\n",
		__func__);
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_20_offset,
			ctrl_20_data,
			sizeof(ctrl_20_data));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));

	extra_data->ctrl8_offset = ctrl_8_offset;

	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x =
			((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
			((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_y_coord_msb << 8);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	/* pitch x and y  - in Q4.12 fixed-point format */
	pitch_of_rx_q4_12 = (((unsigned short)ctrl_8.rx_pitch_lsb << 0) |
				  ((unsigned short)ctrl_8.rx_pitch_msb << 8));
	pitch_of_tx_q4_12 = (((unsigned short)ctrl_8.tx_pitch_lsb << 0) |
				  ((unsigned short)ctrl_8.tx_pitch_msb << 8));

	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;

	/* pitch x and y  - decode Q4.12 fixed-point format */
	rmi4_data->max_touch_width = max((rmi4_data->num_of_rx * pitch_of_rx_q4_12)/4096,
					 (rmi4_data->num_of_tx * pitch_of_tx_q4_12)/4096);

	rmi4_data->f12_wakeup_gesture = query_5.ctrl27_is_present;
	if (rmi4_data->f12_wakeup_gesture) {
		extra_data->ctrl20_offset = ctrl_20_offset;
		extra_data->ctrl27_offset = ctrl_27_offset;
		extra_data->data4_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present;
	}

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				sizeof(f1a->button_control.txrx_map));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (!bdata->cap_button_map) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: cap_button_map is NULL in board file\n",
				__func__);
		return -ENODEV;
	} else if (!bdata->cap_button_map->map) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Button map is missing in board file\n",
				__func__);
		return -ENODEV;
	} else {
		if (bdata->cap_button_map->nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					bdata->cap_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = bdata->cap_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);

	return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	dev_dbg(rmi4_data->pdev->dev.parent, "%s: Checking status\n", __func__);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			usleep(20 * 1000);
		else
			return -EBUSY;

		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: In flash prog mode, status = 0x%02x\n",
			__func__,
			status.status_code);
		return -EBUSY;
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	return 0;
}

static int synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
		return retval;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
	}

	return retval;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	bool f01found;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	rmi = &(rmi4_data->rmi4_mod_info);

	f01found = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);
			retval = synaptics_rmi4_reg_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			pdt_entry_addr &= ~(MASK_8BIT << 8);

			fhandler = NULL;

			if (rmi_fd.fn_number == 0)
				break;

			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				if (rmi_fd.intr_src_count == 0)
					break;

				f01found = true;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to alloc for F%d\n",
					__func__, rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f01_init(rmi4_data,
					fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to alloc for F%d\n",
					__func__, rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
					fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to alloc for F%d\n",
					__func__, rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
					fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to alloc for F%d\n",
					__func__, rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
					fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

	if (!f01found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to find F01\n",
				__func__);
		return -EINVAL;
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(rmi4_data->pdev->dev.parent,
		"%s: Non-Synaptics device found, manufacturer ID = %d\n",
		__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	if ((rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		&& (bdata->wg_enabled))
		dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Wakeup gesture is supported by device\n",
		__func__);
	else
		rmi4_data->wakeup_gesture.data = 0;

	retval = synaptics_rmi4_set_configured(rmi4_data);
	if (retval < 0)
		return retval;

	return 0;
}

static int synaptics_rmi4_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_err("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_Z
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_PRESSURE, 0,
			MAX_Z, 0, 0);
#endif
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif
	input_abs_set_res(rmi4_data->input_dev, ABS_MT_POSITION_X,
			rmi4_data->hw_if->board_data->resolution_x);
	input_abs_set_res(rmi4_data->input_dev, ABS_MT_POSITION_Y,
			rmi4_data->hw_if->board_data->resolution_y);

#ifdef TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers, 0);
#else
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture) {
		set_bit(KEY_WAKEUP, rmi4_data->input_dev->keybit);
		input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_WAKEUP);
	}

	return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	retval = synaptics_rmi4_query_device(rmi4_data);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}

	if (bdata->bl_product_id) {
		if (strncmp(bdata->bl_product_id,
			rmi4_data->rmi4_mod_info.product_id_string,
			sizeof(rmi4_data->
				rmi4_mod_info.product_id_string)) == 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Blacklisted product ID = %s, return.\n",
				__func__, bdata->bl_product_id);
			retval = -EINVAL;
			goto err_query_device;
		}
	}

	rmi4_data->input_dev->name =
		rmi4_data->hw_if->board_data->input_dev_name;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->dev.parent = rmi4_data->pdev->dev.parent;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	if (rmi4_data->hw_if->board_data->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
		temp = rmi4_data->hw_if->board_data->resolution_x;
		rmi4_data->hw_if->board_data->resolution_x =
				rmi4_data->hw_if->board_data->resolution_y;
		rmi4_data->hw_if->board_data->resolution_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

#ifdef CONFIG_INPUT_EVENTS_BUFFER
	input_enable_events_log(rmi4_data->input_dev, event_filter);
#endif /*CONFIG_INPUT_EVENTS_BUFFER*/

	return 0;

err_register_input:
err_query_device:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_free_device(rmi4_data->input_dev);

err_input_device:
	return retval;
}

static int synaptics_rmi4_set_gpio(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int power_on;
	int reset_on;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	power_on = bdata->power_on_state;
	reset_on = bdata->reset_on_state;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"Configuring GPIO\n");

	retval = synaptics_rmi4_gpio_setup(
			bdata->irq_gpio,
			true, 0, 0);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to configure attention GPIO\n",
				__func__);
		goto err_gpio_irq;
	}

	if (bdata->power_gpio >= 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Configuring power GPIO\n");
		retval = synaptics_rmi4_gpio_setup(
				bdata->power_gpio,
				true, 1, power_on);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to configure power GPIO\n",
					__func__);
			goto err_gpio_power;
		}
		usleep(bdata->power_delay_ms * 1000);
	}

	if (bdata->reset_gpio >= 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Configuring reset GPIO\n");
		retval = synaptics_rmi4_gpio_setup(
				bdata->reset_gpio,
				true, 1, !reset_on);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to configure reset GPIO\n",
					__func__);
			goto err_gpio_reset;
		}
	}


	if (bdata->power_gpio >= 0 || bdata->reset_gpio >= 0)
		usleep(bdata->reset_delay_ms * 1000);

	return 0;

err_gpio_reset:
	if (bdata->power_gpio >= 0) {
		synaptics_rmi4_gpio_setup(
				bdata->power_gpio,
				false, 0, 0);
	}

err_gpio_power:
	synaptics_rmi4_gpio_setup(
			bdata->irq_gpio,
			false, 0, 0);

err_gpio_irq:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif

#ifdef F12_DATA_15_WORKAROUND
	rmi4_data->fingers_already_present = 0;
#endif
	rmi4_data->stats.touch_trelease_cnt++;
	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	rmi4_data->fingers_on_2d = false;

	return 0;
}

static int synaptics_rmi4_sw_reset(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char command = 0x01;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	usleep(rmi4_data->hw_if->board_data->reset_delay_ms * 1000);

	if (rmi4_data->hw_if->ui_hw_init) {
		retval = rmi4_data->hw_if->ui_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int synaptics_rmi4_hw_reset(struct synaptics_rmi4_data *rmi4_data)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Hard reset controller\n",
				__func__);

	retval = synaptics_rmi4_power_enable(rmi4_data, false);
	if (retval < 0)
		dev_err(rmi4_data->pdev->dev.parent,
		"%s: Failed to disable touch power during hard reset\n",
			__func__);

	usleep(bdata->reset_delay_ms * 1000);

	retval = synaptics_rmi4_power_enable(rmi4_data, true);
	if (retval < 0)
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to enable touch "
			"power during hard reset\n", __func__);

	if (rmi4_data->hw_if->ui_hw_init) {
		retval = rmi4_data->hw_if->ui_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int synaptics_rmi4_reinit_device(
			struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	rmi4_data->num_failures++;

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				retval = synaptics_rmi4_f12_set_enables(
						rmi4_data, 0);
				if (retval < 0)
					goto exit;
				break;
			}
		}
	}

	retval = synaptics_rmi4_int_enable(rmi4_data, true);
	if (retval < 0)
		goto exit;

	mutex_lock(&rmi4_data->exp_data.mutex);
	if (!list_empty(&rmi4_data->exp_data.list)) {
		list_for_each_entry(exp_fhandler,
				&rmi4_data->exp_data.list, link)
			if (exp_fhandler->exp_fn->reinit != NULL)
				exp_fhandler->exp_fn->reinit(rmi4_data);
	}
	mutex_unlock(&rmi4_data->exp_data.mutex);

	retval = synaptics_rmi4_set_configured(rmi4_data);
exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static void synaptics_rmi4_reset_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					reset_work);

	synaptics_rmi4_reset_device(rmi4_data, rmi4_data->hw_reset);
}

static void synaptics_rmi4_fwudone_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					fwu_done);

	rmi4_data->init_complete = RMI4_FWUPG_COMPLETE_MASK;
	synaptics_rmi4_power_state_handler(rmi4_data);
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
					bool is_hw_reset)
{
	int retval = 0;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	int retry = 0;
	bool reset_complete;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!rmi4_data->suspend
			&& rmi4_data->hw_if->board_data->ddic_power_control) {
		dev_info(rmi4_data->pdev->dev.parent,
			"Need to reset controller - requesting recovery\n");
		return synaptics_rmi4_request_recovery(rmi4_data);
	}

	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	mutex_lock(&rmi4_data->rmi4_reset_mutex);

	reset_complete = false;

	while (!reset_complete && retry < SYNAPTICS_MAX_RESET_WAIT) {
		if (retry || rmi4_data->num_failures)
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: %s reset start, retry = %d, failure number = %d\n",
			__func__, is_hw_reset == true ? "Hard" : "Soft",
		retry, rmi4_data->num_failures);

		if (rmi4_data->num_failures > SYNAPTICS_MAX_RESET_ATTEMPTS) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to to configure TP "
			"after %d attempts, giving up\n",
			__func__, SYNAPTICS_MAX_RESET_ATTEMPTS);

			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Restart system to reset device properly\n",
				__func__);
			kernel_restart("oem-0x03");
		}

		if (is_hw_reset) {
			if (rmi4_data->suspend) {
				/* In normal case, reset happened during suspend, */
				/* power should in off state.                     */
				/* But when wakeup gesture is enabled,            */
				/* power is on during suspend.                    */
				/* Or when reset_device is called by suspend,     */
				/* power is not turned off yet.                   */
				if (!bdata->pm_disabled
					&& !rmi4_data->wakeup_gesture.data
					&& !rmi4_data->pre_suspend_reset) {
					retval = synaptics_rmi4_power_enable(rmi4_data, true);
					if (retval < 0) {
						dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to enable touch "
						"power during hard reset\n", __func__);
						rmi4_data->num_failures++;
						goto fail;
					}
				} else {
					retval = synaptics_rmi4_hw_reset(rmi4_data);
				}
			}  else {
				retval = synaptics_rmi4_hw_reset(rmi4_data);
			}
		 } else
			retval = synaptics_rmi4_sw_reset(rmi4_data);

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
			rmi4_data->num_failures++;
			goto fail;
		}

		synaptics_rmi4_free_fingers(rmi4_data);

		if (rmi4_data->suspend == true)
			retval = go_sleep(&rmi4_data->input_dev->dev);
		else {
			mutex_lock(&rmi4_data->exp_data.mutex);
			if (!list_empty(&rmi4_data->exp_data.list)) {
				list_for_each_entry(exp_fhandler,
					&rmi4_data->exp_data.list, link)
				if (exp_fhandler->exp_fn->reset != NULL)
					exp_fhandler->exp_fn->reset(rmi4_data);
			}
			mutex_unlock(&rmi4_data->exp_data.mutex);

			retval = do_wakeup(&rmi4_data->input_dev->dev, true);
		}

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set power state\n",
				__func__);
			rmi4_data->num_failures++;
			goto fail;
		}
		reset_complete = true;
		break;
fail:
		retry++;
		rmi4_data->pre_suspend_reset = false;
	}

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	dev_err(rmi4_data->pdev->dev.parent,
			"%s:reset_complete, is_hw=%d, retval=%d\n",
			__func__, is_hw_reset, retval);

	if (reset_complete) {
		dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: reset controller complete, "
		"is_hw=%d, suspend=%d, retval=%d\n",
		__func__, is_hw_reset, rmi4_data->suspend, retval);
		rmi4_data->num_failures = 0;
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
		"%s: Failed to reset touch\n",
		__func__);
	}

	return retval;
}

/**
* synaptics_rmi4_exp_fn_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks for the insertion and
* removal of other expansion Function modules such as rmi_dev and calls
* their initialization and removal callback functions accordingly.
*/
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
	struct synaptics_rmi4_exp_fn_data *exp_data =
		container_of((struct delayed_work *) work,
		struct synaptics_rmi4_exp_fn_data, work);
	struct synaptics_rmi4_data *rmi4_data = exp_data->rmi4_data;

	mutex_lock(&rmi4_data->exp_data.mutex);
	if (!list_empty(&rmi4_data->exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&rmi4_data->exp_data.list,
				link) {
			if ((exp_fhandler->exp_fn->init != NULL) &&
					exp_fhandler->insert) {
				exp_fhandler->exp_fn->init(rmi4_data);
				exp_fhandler->insert = false;
			} else if ((exp_fhandler->exp_fn->remove != NULL) &&
					exp_fhandler->remove) {
				exp_fhandler->exp_fn->remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&rmi4_data->exp_data.mutex);

	return;
}

/**
* synaptics_rmi4_lockup_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks lockup.
*/
static void synaptics_rmi4_lockup_work(struct work_struct *work)
{
	const struct synaptics_dsx_board_data *bdata;
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct synaptics_rmi4_data *rmi4_data =
		container_of(dw, struct synaptics_rmi4_data, lockup_work);

	bdata = rmi4_data->hw_if->board_data;
	synaptics_lockup_poll(rmi4_data);
	if (bdata->lockup_poll_interval_ms)
		queue_delayed_work(rmi4_data->workqueue,
			&rmi4_data->lockup_work,
			msecs_to_jiffies(bdata->lockup_poll_interval_ms));
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_exp_fn *exp_fn,
		bool insert)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (!rmi4_data->exp_data.initialized) {
		mutex_init(&rmi4_data->exp_data.mutex);
		INIT_LIST_HEAD(&rmi4_data->exp_data.list);
		rmi4_data->exp_data.initialized = true;
	}

	mutex_lock(&rmi4_data->exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for "
				"expansion function\n",
				__func__);
			mutex_unlock(&rmi4_data->exp_data.mutex);
			return;
		}
		exp_fhandler->exp_fn = exp_fn;
		exp_fhandler->insert = true;
		exp_fhandler->remove = false;
		list_add_tail(&exp_fhandler->link, &rmi4_data->exp_data.list);
	} else if (!list_empty(&rmi4_data->exp_data.list)) {
		list_for_each_entry(exp_fhandler,
				&rmi4_data->exp_data.list, link) {
			if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
				exp_fhandler->insert = false;
				exp_fhandler->remove = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&rmi4_data->exp_data.mutex);

	if (rmi4_data->exp_data.queue_work) {
		queue_delayed_work(rmi4_data->exp_data.workqueue,
				&rmi4_data->exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);


static int synaptics_dsx_virtual_keys_init(
		struct synaptics_rmi4_data *rmi4_data)
{
	int rc;
	struct synaptics_dsx_board_data	*bdata =
			rmi4_data->hw_if->board_data;

	if ((0 == bdata->vkeymap_info.nvkeys)  ||
	    (NULL == bdata->vkeymap_info.data)) {
		dev_err(rmi4_data->pdev->dev.parent,
			"No virtual keys supported\n");
		return -ENOMEM;
	}

	vkey_kobj = kobject_create_and_add("board_properties", NULL);
	if (!vkey_kobj) {
		dev_err(rmi4_data->pdev->dev.parent,
			"unable to create kobject\n");
		return -ENOMEM;
	}
	/* TODO: Currently no reference to the driver data. Need to */
	/*       find a better way to do this */
	vkey_rmi4_data = rmi4_data;

	rc = sysfs_create_group(vkey_kobj, &vkey_grp);
	if (rc) {
		dev_err(rmi4_data->pdev->dev.parent,
			"failed to create attributes\n");
		kobject_put(vkey_kobj);
	}

	return rc;
}

 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, and
 * creates a work queue for detection of other expansion Function
 * modules.
 */
static int synaptics_rmi4_probe(struct platform_device *pdev)
{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data;
	const struct synaptics_dsx_hw_interface *hw_if;
	const struct synaptics_dsx_board_data *bdata;

	dev_dbg(&pdev->dev, "%s:ENTRY\n", __func__);

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		dev_err(&pdev->dev,
				"%s: No hardware interface found\n",
				__func__);
		return -EINVAL;
	}

	bdata = hw_if->board_data;
	if (!bdata) {
		dev_err(&pdev->dev,
				"%s: No board data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&pdev->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->do_once = 1;
	if (bdata->wg_enabled) {
		glass_on_data = rmi4_data;
	}
#ifndef CONFIG_OF
	if (*bdata->regulator_name != 0x00) {
		rmi4_data->regulator.regulator = regulator_get(&pdev->dev,
				bdata->regulator_name);
		if (IS_ERR(rmi4_data->regulator.regulator)) {
			dev_err(&pdev->dev,
					"%s: Failed to get regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->regulator.regulator);
			goto err_regulator;
		}
		retval = synaptics_regulator_enable(rmi4_data, &rmi4_data->regulator);
		if (retval) {
			dev_err(&pdev->dev,
				"%s: Error with %s\n",
				__func__, bdata->regulator_name);
			goto err_regulator;
		} else
			dev_dbg(&pdev->dev,
				"%s: Regulator %s enabled\n",
				__func__, bdata->regulator_name);
	}
#else
	if (bdata->reg_en) {
		rmi4_data->regulator.regulator = bdata->vcc;

		dev_dbg(&pdev->dev,
				"%s: Enabling vcc\n",
				__func__);
		retval = synaptics_regulator_enable(rmi4_data, &rmi4_data->regulator);
		if (retval) {
			dev_err(&pdev->dev,
				"%s: Error with vcc\n",
				__func__);
			goto err_regulator;
		}
	}
	if (bdata->main_power_gpio >= 0) {
		dev_dbg(&pdev->dev,
			"%s: Configuring main power GPIO %d\n",
			__func__, bdata->main_power_gpio);
		retval = synaptics_rmi4_gpio_setup(
				bdata->main_power_gpio,
				true, 1, bdata->main_power_on_state);
		if (retval < 0) {
			dev_err(&pdev->dev,
			"%s: Failed to configure main power GPIO %d\n",
			__func__, retval);
			goto err_gpio_main_power;
		}
		usleep(bdata->power_delay_ms * 1000);
	}

	if (bdata->i2c_pull_up) {
		rmi4_data->vcc_i2c.regulator = bdata->vcc_i2c;
		dev_dbg(&pdev->dev,
				"%s: (%d) Enabling i2c_pull_up\n",
				__func__, bdata->device_id);
		retval = synaptics_regulator_enable(rmi4_data, &rmi4_data->vcc_i2c);
		if (retval) {
			dev_err(&pdev->dev,
				"%s: Error with i2c_pull_up\n",
				__func__);
			goto err_vcc_i2c;
		}
	}
#endif
	usleep(bdata->power_delay_ms * 1000);
	rmi4_data->pdev = pdev;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->hw_if = hw_if;
	rmi4_data->suspend = false;
	rmi4_data->turn_off = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;
	rmi4_data->pre_suspend_reset = false;
	rmi4_data->touch_ready = false;
	rmi4_data->power_state = POWER_STATE_AWAKE;
	rmi4_data->wakeup_source = WAKEUP_BY_OTHER;
	rmi4_data->proxi_check = true;
	rmi4_data->face_detection_check = false;
	rmi4_data->touch_state = false;

	rmi4_data->reset_device = synaptics_rmi4_reset_device;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->power_enable = synaptics_rmi4_power_enable;

	mutex_init(&(rmi4_data->rmi4_reset_mutex));
	mutex_init(&(rmi4_data->rmi4_report_mutex));
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_power_mutex));
	mutex_init(&(rmi4_data->rmi4_irq_mutex));

	synaptics_rmi4_stats_init(rmi4_data);
	synaptics_rmi4_events_init(rmi4_data);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
	synaptics_rmi4_mtouch_counter_init(rmi4_data);
#endif /*CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT*/

	platform_set_drvdata(pdev, rmi4_data);

	retval = synaptics_rmi4_set_gpio(rmi4_data);
	if (retval < 0) {
		dev_err(&pdev->dev,
			"%s: Failed to set up GPIO's\n",
			__func__);
		goto err_set_gpio;
	}

	if (hw_if->ui_hw_init) {
		retval = hw_if->ui_hw_init(rmi4_data);
		if (retval < 0) {
			dev_err(&pdev->dev,
				"%s: Failed to initialize hardware interface\n",
				__func__);
			goto err_ui_hw_init;
		}
	}

	retval = synaptics_rmi4_set_input_dev(rmi4_data);

	if (retval < 0) {
		dev_err(&pdev->dev,
			"%s: Failed to set up input device\n",
			__func__);
		goto err_set_input_dev;
	}

	if (bdata->vkeymap_info.nvkeys != 0) {
		synaptics_dsx_virtual_keys_init(rmi4_data);
	}

	if (!rmi4_data->exp_data.initialized) {
		mutex_init(&rmi4_data->exp_data.mutex);
		INIT_LIST_HEAD(&rmi4_data->exp_data.list);
		rmi4_data->exp_data.initialized = true;
	}

	rmi4_data->irq = gpio_to_irq(bdata->irq_gpio);


	rmi4_data->workqueue = alloc_workqueue("background_workqueue",
				(WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM), 1);

	INIT_WORK(&rmi4_data->irq_work,
			synaptics_rmi4_irq_work);

	init_completion(&rmi4_data->proxi_completion);

	/* Create link to the touch_keypad in sysfs */
	if (strcmp(rmi4_data->input_dev->name, "touch_keypad") == 0) {
		retval = sysfs_create_link(
			rmi4_data->input_dev->dev.kobj.parent->parent->parent->parent->parent->parent,
			&rmi4_data->input_dev->dev.kobj, "touch_keypad");
		if (retval < 0)
			dev_err(&pdev->dev,
					"%s: Failed to create link to the touch_keypad\n",
					__func__);
	}

	retval = synaptics_rmi4_irq_enable(rmi4_data, true, false);
	if (retval < 0) {
		dev_err(&pdev->dev,
			"%s: Failed to enable attention interrupt\n",
			__func__);
		goto err_enable_irq;
	}

	retval = device_init_wakeup(&pdev->dev, 1);

	/* register a handler to listen about interested input events */
	rmi4_data->lid_state = 0; /* set the initial value */
	rmi4_data->smart_flip_state = false;
	rmi4_data->slider_state = SLIDER_STATE_CLOSED;
	rmi4_data->ignore_touch = false;
	rmi4_data->init_complete = RMI4_INIT_MASK;
	memset(rmi4_data->touch_edge, 0, sizeof(rmi4_data->touch_edge));

	dev_info(pdev->dev.parent,
		"%s: dis_in_holster=%d, dis_in_slider=%d, dis_while_sliding=%d,"
		" num_of_slider_hall_sensors=%d\n",
		__func__, bdata->dis_in_holster,
		bdata->dis_in_slider, bdata->dis_while_sliding,
		bdata->num_of_slider_hall_sensors);

	if ((bdata->dis_in_holster) ||
	    (bdata->dis_in_slider) || (bdata->dis_while_sliding)) {
		struct input_handler *inputevt_handler = NULL;
		struct input_device_id *inputid_table = NULL;
		unsigned char num_input_evts = 0;
		unsigned char input_evt_idx = 0;

		inputevt_handler = kzalloc(sizeof(*inputevt_handler), GFP_KERNEL);
		if (!inputevt_handler) {
			dev_err(&pdev->dev,
				"%s: Failed to alloc mem input_handler\n",
				__func__);
			return -ENOMEM;
		}

		if (bdata->dis_in_holster)
			num_input_evts += 1; /* holster event */

		if ((bdata->dis_in_slider) || (bdata->dis_while_sliding))
			num_input_evts += 2;  /* slider open & close */

		/* add one for the last null entry */
		inputid_table = kzalloc(((num_input_evts + 1) *
					 (sizeof(*inputid_table))), GFP_KERNEL);
		if (!inputid_table) {
			dev_err(&pdev->dev,
				"%s: Failed to alloc mem inputid_table\n",
				__func__);
			return -ENOMEM;
		}
		input_evt_idx = 0;
		if (bdata->dis_in_holster) {
			inputid_table[input_evt_idx].flags =
				INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT;
			__set_bit(EV_SW, inputid_table[input_evt_idx].evbit);
			__set_bit(SW_LID, inputid_table[input_evt_idx].swbit);
			input_evt_idx++;
		}

		if (bdata->wg_enabled) {
			inputid_table[input_evt_idx].flags =
				INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT;
			__set_bit(EV_SW, inputid_table[input_evt_idx].evbit);
			__set_bit(SW_SMART_FLIP, inputid_table[input_evt_idx].swbit);
			input_evt_idx++;
		}

		if ((bdata->dis_in_slider) || (bdata->dis_while_sliding)) {
			inputid_table[input_evt_idx].flags =
				INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT;
			__set_bit(EV_SW, inputid_table[input_evt_idx].evbit);
			__set_bit(SW_KEYPAD_SLIDE, inputid_table[input_evt_idx].swbit);
			input_evt_idx++;

			inputid_table[input_evt_idx].flags =
				INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT;
			__set_bit(EV_SW, inputid_table[input_evt_idx].evbit);
			__set_bit(SW_KEYPAD_TRANSITION, inputid_table[input_evt_idx].swbit);
			input_evt_idx++;
		}
		/* terminate with null entry */
		memset(&inputid_table[input_evt_idx], 0x00, sizeof(*inputid_table));
		memset(inputevt_handler, 0x00, sizeof(*inputevt_handler));
		inputevt_handler->event	     = synaptics_input_event;
		inputevt_handler->connect    = synaptics_input_event_connect;
		inputevt_handler->disconnect = synaptics_input_event_disconnect;
		inputevt_handler->minor	     = 0;
		inputevt_handler->name	     = "synaptics_dsx";
		inputevt_handler->id_table   = inputid_table;
		inputevt_handler->private     = rmi4_data;
		retval = input_register_handler(inputevt_handler);
		if (retval)
			dev_err(&pdev->dev,
				"input_register_handler failed,"
				" retval=%d\n", retval);
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = device_create_file(&rmi4_data->input_dev->dev,
				&attrs[attr_count]);
		if (retval < 0) {
			dev_err(&pdev->dev,
				"%s: Failed to create sysfs attributes\n",
				__func__);
			goto err_sysfs;
		}
	}

#if defined(CONFIG_PM) && defined(CONFIG_FB)
	if (!rmi4_data->hw_if->board_data->ddic_power_control) {
		rmi4_data->fb_notif.notifier_call = synaptics_rmi4_fb_notif;
		retval = fb_register_client(&rmi4_data->fb_notif);
		if (retval < 0) {
			dev_err(&pdev->dev,
				"%s: Failed to register fb client callback\n",
				__func__);
			goto err_sysfs;
		}
		INIT_WORK(&rmi4_data->power_state_work,
				synaptics_rmi4_power_state_work);
	}
#endif
	if ((0 != bdata->dis_in_slider) || (0 != bdata->dis_while_sliding)) {
		INIT_WORK(&rmi4_data->slider_work,
				synaptics_rmi4_slider_state_work);
	}

	rmi4_data->exp_data.workqueue =
			create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&rmi4_data->exp_data.work,
			synaptics_rmi4_exp_fn_work);
	rmi4_data->exp_data.rmi4_data = rmi4_data;
	rmi4_data->exp_data.queue_work = true;
	queue_delayed_work(rmi4_data->exp_data.workqueue,
			&rmi4_data->exp_data.work,
			0);

	if (bdata->watchdog_timeout_s) {
		synaptics_rmi4_timer_init(&rmi4_data->monitor_timer,
				"watchdog",
				bdata->watchdog_timeout_s * 1000,
				watchdog_timeout_check);
	}
	synaptics_rmi4_stats_start(rmi4_data);

	synaptics_rmi4_monitor_timer_start(rmi4_data);


	if (rmi4_data->hw_if->board_data->ddic_power_control)
		INIT_DELAYED_WORK(&rmi4_data->recovery_work,
				synaptics_rmi4_recovery_work);

	if (rmi4_data->hw_if->board_data->wg_enabled) {
		rmi4_data->wakeup_gesture.double_tap = 1;
		rmi4_data->wakeup_gesture.swipe = 1;
		retval = sysfs_create_link(
			rmi4_data->input_dev->dev.kobj.parent->parent->parent->parent->parent->parent,
			&rmi4_data->input_dev->dev.kobj, "touch");

		if (retval)
			dev_err(&pdev->dev, "Unable to create link, rc=%d\n", retval);
	}

	if (bdata->lockup_poll_interval_ms) {
		INIT_DELAYED_WORK(&rmi4_data->lockup_work,
			synaptics_rmi4_lockup_work);

		queue_delayed_work(rmi4_data->workqueue,
			&rmi4_data->lockup_work,
			msecs_to_jiffies(bdata->lockup_poll_interval_ms));
	}

	INIT_WORK(&rmi4_data->fwu_done,
				synaptics_rmi4_fwudone_work);

	INIT_WORK(&rmi4_data->reset_work,
				synaptics_rmi4_reset_work);

	INIT_WORK(&rmi4_data->glass_on_work,
				synaptics_rmi4_glass_on_work);
	synaptics_rmi4_fw_update_module_init(rmi4_data);
	synaptics_rmi4_f54_module_init(rmi4_data);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV
	synaptics_rmidev_module_init(rmi4_data);
#endif
#ifdef CONFIG_BBRY_DEBUG
	if (!rmi4_data->hw_if->board_data->ddic_power_control)
		synaptics_rmi4_slide_module_init(rmi4_data);
#endif

	dev_dbg(&pdev->dev,
		"%s:EXIT\n",
		__func__);

	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	cancel_delayed_work_sync(&rmi4_data->exp_data.work);
	flush_workqueue(rmi4_data->exp_data.workqueue);
	destroy_workqueue(rmi4_data->exp_data.workqueue);

	synaptics_rmi4_irq_enable(rmi4_data, false, false);

err_enable_irq:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

err_set_input_dev:
	synaptics_rmi4_gpio_setup(
			bdata->irq_gpio,
			false, 0, 0);

	if (bdata->reset_gpio >= 0) {
		synaptics_rmi4_gpio_setup(
				bdata->reset_gpio,
				false, 0, 0);
	}

	if (bdata->power_gpio >= 0) {
		dev_dbg(&pdev->dev,
				"%s: Disabling power gpio(%d)\n",
				__func__, !bdata->power_on_state);
		gpio_set_value(bdata->power_gpio,
					!bdata->power_on_state);
		synaptics_rmi4_gpio_setup(
				bdata->power_gpio,
				false, 0, 0);
	}

err_ui_hw_init:
#ifndef CONFIG_OF
err_set_gpio:
	if (rmi4_data->regulator.regulator) {
		dev_dbg(&pdev->dev,
				"%s: Disabling vcc\n",
				__func__);
		synaptics_regulator_disable(rmi4_data, &rmi4_data->regulator);
		synaptics_regulator_put(rmi4_data, &rmi4_data->regulator);
	}

#else
err_set_gpio:
	if (rmi4_data->vcc_i2c.regulator) {
		dev_dbg(&pdev->dev,
				"%s: Disabling vcc_i2c\n",
				__func__);
		synaptics_regulator_disable(rmi4_data, &rmi4_data->vcc_i2c);
		synaptics_regulator_put(rmi4_data, &rmi4_data->vcc_i2c);
	}

err_vcc_i2c:
	if (bdata->main_power_gpio >= 0) {
		dev_dbg(&pdev->dev,
				"%s: Disabling main power gpio(%d)\n",
				__func__, !bdata->main_power_on_state);
		gpio_set_value(bdata->main_power_gpio,
					!bdata->main_power_on_state);
		synaptics_rmi4_gpio_setup(
				bdata->main_power_gpio,
				false, 0, 0);
	}

err_gpio_main_power:
	if (rmi4_data->regulator.regulator) {
		dev_dbg(&pdev->dev,
				"%s: Disabling vcc\n",
				__func__);
		synaptics_regulator_disable(rmi4_data, &rmi4_data->regulator);
		synaptics_regulator_put(rmi4_data, &rmi4_data->regulator);
	}
#endif
err_regulator:
	kfree(rmi4_data);

	return retval;

}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int synaptics_rmi4_remove(struct platform_device *pdev)
{
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data = platform_get_drvdata(pdev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (rmi4_data->hw_if->board_data->ddic_power_control)
		cancel_delayed_work_sync(&rmi4_data->recovery_work);
	if (bdata->lockup_poll_interval_ms)
		cancel_delayed_work_sync(&rmi4_data->lockup_work);
	synaptics_rmi4_monitor_timer_stop(rmi4_data);
	synaptics_rmi4_stats_stop(rmi4_data);

	cancel_delayed_work_sync(&rmi4_data->exp_data.work);
	flush_workqueue(rmi4_data->exp_data.workqueue);
	destroy_workqueue(rmi4_data->exp_data.workqueue);

#if defined(CONFIG_PM) && defined(CONFIG_FB)
	fb_unregister_client(&rmi4_data->fb_notif);
#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	synaptics_rmi4_empty_fn_list(rmi4_data);
#ifdef CONFIG_INPUT_EVENTS_BUFFER
	input_disable_events_log(rmi4_data->input_dev);
#endif /*CONFIG_INPUT_EVENTS_BUFFER*/
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

#ifdef CONFIG_OF
	if (rmi4_data->vcc_i2c.regulator) {
		dev_dbg(&pdev->dev,
				"%s: Disabling vcc_i2c\n",
				__func__);
		synaptics_regulator_disable(rmi4_data, &rmi4_data->vcc_i2c);
		synaptics_regulator_put(rmi4_data, &rmi4_data->vcc_i2c);
	}

	if (bdata->main_power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->main_power_gpio, false, 0, 0);
#endif

	if (rmi4_data->regulator.regulator) {
		dev_dbg(&pdev->dev,
			"%s: Disabling vcc\n",
			__func__);
		synaptics_regulator_disable(rmi4_data, &rmi4_data->regulator);
		synaptics_regulator_put(rmi4_data, &rmi4_data->regulator);
	}

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
static int synaptics_rmi4_set_power_state(struct synaptics_rmi4_data *rmi4_data,
		int desired_power_state)
{
	int rc = 0;

	if (rmi4_data->power_state != desired_power_state) {
		switch (desired_power_state) {
		case POWER_STATE_AWAKE:
			rc = synaptics_rmi4_runtime_resume(
					&rmi4_data->pdev->dev);
			break;
		case POWER_STATE_ASLEEP:
		case POWER_STATE_DEEP_SLEEP:
		case POWER_STATE_OFF:
			rc = synaptics_rmi4_runtime_suspend(
					&rmi4_data->pdev->dev);

			break;
		default:
			dev_err(rmi4_data->pdev->dev.parent,
				"Invalid power state requested: %d\n",
					desired_power_state);
			rc = -1;
			break;
		}

		if (rc)
			dev_err(rmi4_data->pdev->dev.parent,
			"Failed to set power state to state %d, rc=%d\n",
				desired_power_state, rc);
		else
			rmi4_data->power_state = desired_power_state;
	}

	return rc;
}

static void synaptics_rmi4_power_state_handler(
		struct synaptics_rmi4_data *rmi4_data)
{
	bool		lid_state = 0;
	uint8_t		slider_state = 0;
	unsigned char	fb_blank = 0;
	bool		turn_off = 0;
	int		next_power_state;
	const struct	synaptics_dsx_board_data *bdata =
				rmi4_data->hw_if->board_data;

	lid_state = rmi4_data->lid_state;
	slider_state = rmi4_data->slider_state;
	fb_blank = rmi4_data->fb_blank;
	turn_off = rmi4_data->turn_off;

	if (rmi4_data->init_complete != RMI4_INIT_COMPLETE_MASK)
		next_power_state = POWER_STATE_AWAKE;
	else if ((turn_off) ||
	    ((0 != bdata->dis_in_holster) && (lid_state != false)) ||
	    ((0 != bdata->dis_in_slider) &&
				(slider_state != SLIDER_STATE_OPENED)))
		next_power_state = POWER_STATE_OFF;
	else {
		/* fb_blank is failed, revert effects of
		 * the early blank event. */
		if ((FB_BLANK_POWERDOWN == fb_blank) &&
			(rmi4_data->fb_event == FB_R_EARLY_EVENT_BLANK))
			fb_blank = FB_BLANK_UNBLANK;

		next_power_state = (FB_BLANK_UNBLANK == fb_blank) ?
				POWER_STATE_AWAKE : POWER_STATE_OFF;
	}

	dev_info(rmi4_data->pdev->dev.parent,
		"init_complete=0x%02x, turn_off=%d, lid_state=%d, fb_blank=%d\n",
		rmi4_data->init_complete, turn_off, lid_state, fb_blank);
	dev_info(rmi4_data->pdev->dev.parent,
		"slider=%s,  power=%s, touch_obj=%d, touch=%s\n",
		slider_state_str[slider_state],
		power_state_str[next_power_state],
		rmi4_data->touch_obj_cnt,
		rmi4_data->ignore_touch ? "ignored" : "normal");
	synaptics_rmi4_set_power_state(rmi4_data, next_power_state);
	synaptics_rmi4_poll_sensor_report(rmi4_data, true);
}

static void synaptics_rmi4_slider_state_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					slider_work);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	const struct synaptics_slider_fsm_info  *slider_fsm_tbl = NULL;
	uint8_t      slider_fsm_tbl_size = 0;
	uint8_t      idx;
	struct synaptics_rmi4_event_entry event;

	if (1 == bdata->num_of_slider_hall_sensors) {
		slider_fsm_tbl = slider_fsm_table_single_sensor;
		slider_fsm_tbl_size = (sizeof(slider_fsm_table_single_sensor) /
					sizeof(struct synaptics_slider_fsm_info));
	} else {
		slider_fsm_tbl = slider_fsm_table;
		slider_fsm_tbl_size = (sizeof(slider_fsm_table) /
					sizeof(struct synaptics_slider_fsm_info));
	}

	for (idx = 0; idx < slider_fsm_tbl_size; idx++) {
		if (slider_fsm_tbl[idx].mask == rmi4_data->slider_keys_values)
			rmi4_data->slider_state = slider_fsm_tbl[idx].slider_state;
	}

	event.event_id = SLIDER;
	event.event_data[0] = rmi4_data->slider_state;
	synaptics_record_events(rmi4_data, &event);

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s:slider_key_values=0x%02x, slider_state=%s\n",
		__func__, rmi4_data->slider_keys_values, slider_state_str[rmi4_data->slider_state]);

	switch (rmi4_data->slider_state) {
	case SLIDER_STATE_OPENING:
		if (1 >= bdata->num_of_slider_hall_sensors) {
			/* There is no key find if the slider is fully */
			/* opened. Let's move to the opened state */
			rmi4_data->ignore_touch =
				((rmi4_data->touch_obj_cnt != 0) &&
						bdata->dis_while_sliding);
			rmi4_data->slider_state = SLIDER_STATE_OPENED;
		}
		break;
	case SLIDER_STATE_OPENED:
		break;
	case SLIDER_STATE_CLOSING:
		break;
	case SLIDER_STATE_CLOSED:
		break;
	default:
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: unexpected slider state =%d\n",
			__func__, rmi4_data->slider_state);
		break;
	}

	synaptics_rmi4_power_state_handler(rmi4_data);
}

static void synaptics_rmi4_power_state_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					power_state_work);

	synaptics_rmi4_power_state_handler(rmi4_data);
}


static void synaptics_rmi4_glass_on_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data,
					glass_on_work);
	unsigned long glass_on_time_ms;
	unsigned long total_glass_on_delay;

	if ((rmi4_data->wakeup_check_mask & WAKEUP_CHECK_GLASS_ON) != 0) {
		rmi4_data->wakeup_check_mask &= ~WAKEUP_CHECK_GLASS_ON;
		glass_on_time_ms = get_timestamp();
		total_glass_on_delay = (glass_on_time_ms) - (rmi4_data->dtwakeup_time_ms);
		if (total_glass_on_delay < 300)
			/*Do nothing */;
		else if (total_glass_on_delay < 500)
			rmi4_data->mtouch_counter.glass_on_delay_300ms++;
		else if (total_glass_on_delay < 750)
			rmi4_data->mtouch_counter.glass_on_delay_500ms++;
		else if (total_glass_on_delay < 1000)
			rmi4_data->mtouch_counter.glass_on_delay_750ms++;
		else if (total_glass_on_delay < 2000)
			rmi4_data->mtouch_counter.glass_on_delay_1000ms++;
		else {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
			synaptics_dsx_ddt_send(
			rmi4_data, MTOUCH_DEBUG, MTOUCH_DBG_GLASS_ON);
#endif
		}
	}
}

#ifdef CONFIG_FB
static int synaptics_rmi4_fb_notif(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct fb_event *evdata = data;
	int fb_blank;

	if (self == NULL || data == NULL)
		return 0;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EARLY_EVENT_BLANK && event != FB_R_EARLY_EVENT_BLANK && event != FB_EVENT_CONBLANK)
		return 0;

	rmi4_data = container_of(self, struct synaptics_rmi4_data, fb_notif);

	if (rmi4_data == NULL)
		return 0;

	fb_blank = *(int *)evdata->data;

	if (!(FB_BLANK_NORMAL == fb_blank)) {
		if (rmi4_data->wakeup_source == WAKEUP_BY_OTHER)
			rmi4_data->wakeup_source = WAKEUP_BY_DISPLAY;
		flush_workqueue(rmi4_data->workqueue);
		rmi4_data->fb_blank = fb_blank;
		rmi4_data->fb_event = event;
		queue_work(rmi4_data->workqueue,
				&rmi4_data->power_state_work);
	}
	return 0;
}
#endif

void synaptics_rmi4_glass_on_notif(bool glass_off)
{
	struct synaptics_rmi4_data *rmi4_data = glass_on_data;

	if (rmi4_data && rmi4_data->touch_ready) {
		if (1 == glass_off) {
			rmi4_data->wakeup_check_mask &= ~WAKEUP_CHECK_GLASS_ON;
		} else {
			queue_work(rmi4_data->workqueue,
				&rmi4_data->glass_on_work);
		}
	} else {
		pr_err("%s:Touch is not ready!\n", __func__);
	}
}
EXPORT_SYMBOL(synaptics_rmi4_glass_on_notif);

static int synaptics_check_lpwg(
		struct synaptics_rmi4_data *rmi4_data, bool wg_enable)
{
	int retval;
	unsigned char state;
	int tries;

	/*
	 * This check only applicable the main touch controller that
	 * controls the ddic power.
	 */
	if (0 == rmi4_data->hw_if->board_data->ddic_power_control)
		return 0;

	/* wait until LPWG state is NOT_LPWG or timeout occurs */
	for (tries = 0; tries < SYNAPTICS_LPWG_MAX_RETRIES; ++tries) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				SYNAPTICS_F51_CUSTOM_DATA06,
					 &state, sizeof(state));

		if (((true == wg_enable) &&
			(SYNAPTICS_LPWG_STATE_NOT_LPWG != state)) ||
		    ((false == wg_enable) &&
			(SYNAPTICS_LPWG_STATE_NOT_LPWG == state))) {
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: state become 0x%02x after %d tries\n",
				__func__, state, tries);
			break;
		} else
			dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: state is 0x%02x at %d tries",
					 __func__, state, tries);

		usleep(SYNAPTICS_LPWG_DELAY_MS * 1000);
	}

	if (tries == SYNAPTICS_LPWG_MAX_RETRIES)
		dev_err(rmi4_data->pdev->dev.parent,
		"%s: LPWG timeout, last LPWG State = 0x%02x\n",
					 __func__, state);
	else
		retval = 0;

	usleep(SYNAPTICS_WAKEUP_DELAY_MS * 1000);
	return retval;
}

static int synaptics_rmi4_f11_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char reporting_control;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F11)
			break;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return retval;
	}

	reporting_control = (reporting_control & ~MASK_3BIT);
	if (enable)
		reporting_control |= F11_WAKEUP_GESTURE_MODE;
	else
		reporting_control |= F11_CONTINUOUS_MODE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return retval;
	} else
		retval = 0;

	/* check for switching in and out of LPWG state */
	return synaptics_check_lpwg(rmi4_data, enable);
}

static int synaptics_rmi4_f12_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char offset;
	unsigned short ctrl_base;
	unsigned char reporting_control[3];
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler = NULL;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
			break;
	}

	if ((fhandler == NULL) || (fhandler->fn_number != SYNAPTICS_RMI4_F12)) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to find fhandler\n", __func__);
		return -EINVAL;
	}

	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	offset = extra_data->ctrl20_offset;
	ctrl_base = fhandler->full_addr.ctrl_base;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read reporting control\n",
				__func__);
		return retval;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
		ctrl_base + extra_data->ctrl8_offset,
		ctrl_8.data,
		sizeof(ctrl_8.data));

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read sensor flags\n",
				__func__);
		return retval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: ctrl_8.sensor_flags = 0x%x\n",
				__func__, ctrl_8.sensor_flags);

	if (enable) {
		reporting_control[2] = F12_WAKEUP_GESTURE_MODE;
		ctrl_8.sensor_flags &= ~(F12_CTRL8_SENSOR_REPORT_BEYOND_ACTIVE);
		ctrl_8.low_rx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA;
		ctrl_8.high_rx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA;
		ctrl_8.low_tx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA;
		ctrl_8.high_tx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA;
	} else {
		reporting_control[2] = F12_CONTINUOUS_MODE;
		ctrl_8.sensor_flags |= F12_CTRL8_SENSOR_REPORT_BEYOND_ACTIVE;
		ctrl_8.low_rx_clip = 0;
		ctrl_8.high_rx_clip = 0;
		ctrl_8.low_tx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA_WAKE;
		ctrl_8.high_tx_clip = F12_CTRL8_SENSOR_INACTIVE_AREA_WAKE;
	}

	retval = synaptics_rmi4_reg_write(rmi4_data,
			ctrl_base + extra_data->ctrl8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change sensor flags\n",
				__func__);
		return retval;
	}

	retval = synaptics_rmi4_reg_write(rmi4_data,
			ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return retval;
	}

	if (enable && rmi4_data->wakeup_gesture.data) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
			ctrl_base + extra_data->ctrl27_offset,
			&rmi4_data->wakeup_gesture.data,
			sizeof(rmi4_data->wakeup_gesture.data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change wakeup mode\n",
				__func__);
			return retval;
		}
	}

	/* check for switching out of LPWG state */
	return synaptics_check_lpwg(rmi4_data, enable);
}

static int synaptics_rmi4_wakeup_gesture(
		struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	if (rmi4_data->f11_wakeup_gesture)
		retval = synaptics_rmi4_f11_wg(rmi4_data, enable);
	else if (rmi4_data->f12_wakeup_gesture)
		retval = synaptics_rmi4_f12_wg(rmi4_data, enable);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to %s wakeup gesture\n",
			__func__, enable ? "enable" : "dislabe");
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
	} else {
		synaptics_rmi4_set_inadvertent_tapdetect_timer(rmi4_data, enable);
#endif
	}

	return retval;
}

 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static int synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data,
					bool gesture_mode)
{
	int retval;
	unsigned char device_ctrl;

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s\n",
		__func__);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return retval;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	if (0 == gesture_mode)
		device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);
	else {
		if (rmi4_data->hw_if->board_data->ddic_power_control)
			device_ctrl = device_ctrl |
					NO_SLEEP_ON | NORMAL_OPERATION;
		else
			device_ctrl = device_ctrl |
					NO_SLEEP_OFF | NORMAL_OPERATION;
	}

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
	} else {
		if (rmi4_data->hw_if->board_data->ddic_power_control)
			usleep(DDIC_PRE_RESET_DELAY_MS * 1000);
		rmi4_data->sensor_sleep = true;
	}

	return retval;
}

 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_runtime_resume()
 * and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static int synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;
	dev_info(rmi4_data->pdev->dev.parent,
		"%s: Waking sensor\n",
		__func__);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return retval;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return retval;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	if (rmi4_data->hw_if->board_data->ddic_power_control)
		usleep(DDIC_POWER_ON_DELAY_MS * 1000);

	return 0;
}

/*
 * power_enable()
 *
 * Enables or disables power to the touch controller hardware.
 */
static int synaptics_rmi4_power_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retval = 0;
#ifdef CONFIG_MSM_GPIOMUX
	struct gpiomux_setting gpiomux = {
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN
	};
#endif

	if (enable) {
		if (rmi4_data->regulator.regulator) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Enabling vcc\n",
					__func__);
			retval = synaptics_regulator_enable(rmi4_data, &rmi4_data->regulator);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable vcc\n",
					__func__);
				return -EAGAIN;
			}
			usleep(bdata->power_delay_ms * 1000);
		}

		if (bdata->main_power_gpio >= 0) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Enabling main power gpio\n",
					__func__);
			gpio_set_value(bdata->main_power_gpio,
					bdata->main_power_on_state);
			usleep(bdata->power_delay_ms * 1000);
		}

		if (rmi4_data->vcc_i2c.regulator) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Enabling vcc_i2c\n",
					__func__);
			retval = synaptics_regulator_enable(rmi4_data, &rmi4_data->vcc_i2c);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enable i2c regulator\n",
					__func__);
				return -EAGAIN;
			}
			usleep(bdata->power_delay_ms * 1000);
		}

		if (bdata->power_gpio >= 0) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Enabling power gpio\n",
					__func__);
			gpio_set_value(bdata->power_gpio,
					bdata->power_on_state);
			usleep(bdata->power_delay_ms * 1000);
		}

#ifdef CONFIG_MSM_GPIOMUX
		msm_gpiomux_write(bdata->irq_gpio, GPIOMUX_ACTIVE,
				&gpiomux, NULL);
#endif

		if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio,
					 !bdata->reset_on_state);
			usleep(bdata->reset_delay_ms * 1000);
		}
	} else {
		if (bdata->reset_gpio >= 0) {
			gpio_set_value(bdata->reset_gpio,
					 bdata->reset_on_state);
			usleep(bdata->reset_active_ms * 1000);
		}

#ifdef CONFIG_MSM_GPIOMUX
		gpiomux.pull = GPIOMUX_PULL_NONE;
		msm_gpiomux_write(bdata->irq_gpio, GPIOMUX_ACTIVE,
				&gpiomux, NULL);
#endif

		if (bdata->power_gpio >= 0) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Disabling power gpio\n",
					__func__);
			gpio_set_value(bdata->power_gpio,
					!bdata->power_on_state);
			usleep(SYNAPTICS_POWER_DOWN_DELAY_US);
		}

		if (rmi4_data->vcc_i2c.regulator) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Disabling vcc_i2c\n",
					__func__);
			retval = synaptics_regulator_disable(rmi4_data, &rmi4_data->vcc_i2c);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to disable i2c regulator\n",
					__func__);
				return -EAGAIN;
			}
		}

		if (bdata->main_power_gpio >= 0) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Disabling main power gpio\n",
					__func__);
			gpio_set_value(bdata->main_power_gpio,
					!bdata->main_power_on_state);
			usleep(SYNAPTICS_POWER_DOWN_DELAY_US);
		}

		if (rmi4_data->regulator.regulator) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Disabling vcc\n",
					__func__);
			retval = synaptics_regulator_disable(rmi4_data, &rmi4_data->regulator);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to disable vcc regulator\n",
					__func__);
				return -EAGAIN;
			}
		}
	}

	return 0;
}

 /**
 * go_sleep()
 *
 * Called when driver need to put device into sleep mode
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int go_sleep(struct device *dev)
{
	int retval = 0;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!bdata->pm_disabled && rmi4_data->wakeup_gesture.data) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Putting controller into wakeup gesture mode\n",
			__func__);
		synaptics_rmi4_empty_fn_list(rmi4_data);
		retval = synaptics_rmi4_query_device(rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
			return retval;
		}
		retval = synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		if (retval < 0) {
			dev_err(dev,
				"%s: Failed to put controller "
				"into wakeup gesture mode\n",
				__func__);
		}

		retval = synaptics_rmi4_sensor_sleep(rmi4_data, true);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake sensor\n",
				__func__);
		}
		retval = synaptics_rmi4_irq_enable(rmi4_data, true, false);
		if (retval < 0) {
			dev_err(dev,
				"%s: Failed to put controller into "
				"wakeup gesture mode\n",
				__func__);
		}

		return retval;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Putting controller into deep sleep\n", __func__);

	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	if (!bdata->pm_disabled) {
		retval = synaptics_rmi4_sensor_sleep(rmi4_data, false);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to put sensor into sleep mode\n",
			__func__);
			return retval;
		}
	}
	synaptics_rmi4_free_fingers(rmi4_data);

	if (!bdata->pm_disabled) {
		retval = synaptics_rmi4_power_enable(rmi4_data, false);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to disable touch power\n",
				__func__);
				return retval;
		}
	}
	return 0;
}

 /**
 * synaptics_rmi4_runtime_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
int synaptics_rmi4_runtime_suspend(struct device *dev)
{
	int retval = 0;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	struct synaptics_rmi4_event_entry event;

	if (rmi4_data->hw_if->board_data->ddic_power_control)
		cancel_delayed_work_sync(&rmi4_data->recovery_work);
	synaptics_rmi4_monitor_timer_stop(rmi4_data);
	synaptics_rmi4_stats_stop(rmi4_data);
	if (bdata->lockup_poll_interval_ms)
		cancel_delayed_work_sync(&rmi4_data->lockup_work);

	if (bdata->pm_disabled) {
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: PM has been disabled by configuration. "
			"Controller power state and associated power "
			"rails will not be changed\n",
			__func__);
	}

	if (rmi4_data->stay_awake) {
		dev_info(dev, "%s: Staying awake\n", __func__);
		rmi4_data->state_changed = true;
		return -EBUSY;
	}

	dev_dbg(dev, "%s: Suspending controller\n", __func__);

	event.event_id = SUSPEND;
	synaptics_record_events(rmi4_data, &event);

	mutex_lock(&rmi4_data->rmi4_power_mutex);
	if (!rmi4_data->suspend) {
		rmi4_data->suspend = true;
		if (!bdata->pm_disabled) {
			rmi4_data->pre_suspend_reset = true;
			retval = rmi4_data->reset_device(rmi4_data, true);
			rmi4_data->pre_suspend_reset = false;
			if (retval < 0)
				dev_err(dev, "%s: Failed to suspend\n",
						__func__);
		} else {
			retval = go_sleep(&rmi4_data->input_dev->dev);
			if (retval < 0)
				dev_err(dev, "%s: Failed to go to sleep\n",
						__func__);
		}

		mutex_lock(&rmi4_data->exp_data.mutex);
		if (!list_empty(&rmi4_data->exp_data.list)) {
			list_for_each_entry(exp_fhandler,
					&rmi4_data->exp_data.list, link)
				if (exp_fhandler->exp_fn->suspend != NULL)
					exp_fhandler->
						exp_fn->suspend(rmi4_data);
		}
		mutex_unlock(&rmi4_data->exp_data.mutex);
	}

	mutex_unlock(&rmi4_data->rmi4_power_mutex);

	return retval;
}

static void record_wakeup_failures_in_proxi(bool in_proxi, struct synaptics_rmi4_data  *rmi4_data)
{
	int i;
	int retval;
	int tap_failure_total = 0;
	int tap_failure_delta = 0;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (bdata->tap_status_addr < 0)
		return;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			bdata->tap_status_addr,
			rmi4_data->last_tap_status,
			sizeof(rmi4_data->last_tap_status));

	/* We are not interested in the Last Wekeup failure which is last_tab_status[0] */
	for (i = 1; i < sizeof(rmi4_data->last_tap_status); i++) {
		if (rmi4_data->last_tap_status[i])
			dev_info(rmi4_data->pdev->dev.parent,
			"%s (in proxi): %d\n",
			tap_failure_string[i - 1],
			rmi4_data->last_tap_status[i]);
		tap_failure_total += rmi4_data->last_tap_status[i];
	}

	tap_failure_delta = (tap_failure_total - rmi4_data->mtouch_counter.total_tap_failure_cnt);

	rmi4_data->mtouch_counter.total_tap_failure_cnt = tap_failure_total;
	if (true == in_proxi) {
		rmi4_data->mtouch_counter.tap_failure_in_proxi_delta += tap_failure_delta;
	} else {
		rmi4_data->mtouch_counter.inadv_tap_failure_delta1 =
				rmi4_data->mtouch_counter.inadv_tap_failure_delta0;
		rmi4_data->mtouch_counter.inadv_tap_failure_delta0 += tap_failure_delta;
	}
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s:tap_failure=%d, in_proxi=%d, inadv_tap_fail=[%d,%d]\n", __func__,
		rmi4_data->mtouch_counter.total_tap_failure_cnt,
		rmi4_data->mtouch_counter.tap_failure_in_proxi_delta,
		rmi4_data->mtouch_counter.inadv_tap_failure_delta0,
		rmi4_data->mtouch_counter.inadv_tap_failure_delta1);
}


static void wakeup_failure_check(struct device *dev)
{
	int i;
	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	struct synaptics_rmi4_event_entry event;

	if (bdata->tap_status_addr < 0)
		return;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			bdata->tap_status_addr,
			rmi4_data->last_tap_status,
			sizeof(rmi4_data->last_tap_status));
	if (rmi4_data->last_tap_status[0])
		dev_info(rmi4_data->pdev->dev.parent,
			"Last wakeup failure: %s\n",
			tap_failure_string[rmi4_data->last_tap_status[0] - 1]);

	event.event_id = WAKEUP_UP;
	event.event_data[0] = rmi4_data->wakeup_source;
	for (i = 1; i < sizeof(rmi4_data->last_tap_status); i++) {
		if (rmi4_data->last_tap_status[i])
			dev_info(rmi4_data->pdev->dev.parent,
			"%s : %d\n",
			tap_failure_string[i - 1],
			rmi4_data->last_tap_status[i]);
		rmi4_data->mtouch_counter.tap_failure[i - 1] +=
				rmi4_data->last_tap_status[i];
		event.event_data[i] = rmi4_data->last_tap_status[i];
	}

	rmi4_data->mtouch_counter.failed_taps_in_proxi +=
			rmi4_data->mtouch_counter.tap_failure_in_proxi_delta;
	rmi4_data->mtouch_counter.inadv_tap_failures +=
			rmi4_data->mtouch_counter.inadv_tap_failure_delta1;
	rmi4_data->mtouch_counter.total_tap_failure_cnt = 0;
	rmi4_data->mtouch_counter.tap_failure_in_proxi_delta = 0;
	rmi4_data->mtouch_counter.inadv_tap_failure_delta1 = 0;
	rmi4_data->mtouch_counter.inadv_tap_failure_delta0 = 0;

	synaptics_record_events(rmi4_data, &event);

	synaptics_rmi4_capture_wakeup_buffer(
		rmi4_data, &rmi4_data->extra_wakeup_info);
}

 /**
 * do_wakeup()
 *
 * Called when the system wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int do_wakeup(struct device *dev, bool reset)
{
	int retval = 0;
	bool was_in_bl_mode = false;
	int retry = 0;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	struct synaptics_rmi4_event_entry event;

	if (!bdata->pm_disabled) {
		if (!reset && rmi4_data->wakeup_gesture.data) {
			wakeup_failure_check(dev);

			rmi4_data->wakeup_source = WAKEUP_BY_OTHER;
			retval = synaptics_rmi4_wakeup_gesture(
						rmi4_data, false);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to get out of gesture mode\n",
					__func__);
				return retval;
			}
			retval = synaptics_rmi4_sensor_wake(rmi4_data);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to wake sensor\n",
					__func__);
			}
			return retval;
		}


		event.event_id = WAKEUP_UP;
		event.event_data[0] = rmi4_data->wakeup_source;
		synaptics_record_events(rmi4_data, &event);

		if (!reset) {
			retval = synaptics_rmi4_power_enable(rmi4_data, true);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable touch power\n",
					__func__);
				return retval;
			}
		}

		rmi4_data->current_page = MASK_8BIT;
		if (rmi4_data->hw_if->ui_hw_init)
			rmi4_data->hw_if->ui_hw_init(rmi4_data);

		do {
			if (was_in_bl_mode)
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: CRC in process, retry\n",
					__func__);

			synaptics_rmi4_empty_fn_list(rmi4_data);

			retval = synaptics_rmi4_query_device(rmi4_data);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to query device\n",
					__func__);
				return retval;
			}

			retval = synaptics_rmi4_check_status(rmi4_data,
					&was_in_bl_mode);

			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to check status\n",
					__func__);
					return retval;
			}
		} while (retry++ <
			SYNAPTICS_MAX_FAILURE_RETRY && was_in_bl_mode);

		if (was_in_bl_mode) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: CRC in process, timeout\n",
				__func__);
			return -EAGAIN;
		}

		synaptics_rmi4_set_params(rmi4_data);

		retval = synaptics_rmi4_sensor_wake(rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake sensor\n",
				__func__);
			return retval;
		}
	}
	retval = synaptics_rmi4_irq_enable(rmi4_data, true, false);
	if (retval < 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Failed to enable interrupt\n",
				__func__);
		return retval;
	}

	return 0;
}

 /**
 * synaptics_rmi4_runtime_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
int synaptics_rmi4_runtime_resume(struct device *dev)
{
	int retval = 0;
	unsigned long dtwakeup_complete_time_msecs;
	unsigned long total_dtwakeup_time_msecs;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retry = 0;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: dev=%p, rmi4_data=%p, bdata=%p\n",
		__func__, dev, rmi4_data, bdata);

	if (bdata->pm_disabled) {
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: PM has been disabled by configuration. "
			"Controller power state and associated power "
			"rails will not be changed\n",
			__func__);
	}

	if (rmi4_data->turn_off) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Staying in suspend mode, turn_off=%d\n",
			__func__, rmi4_data->turn_off);
		return 0;
	}

	rmi4_data->resume_ignore_touch = true;
	rmi4_data->resume_notouch_jiffies = 0;
	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: rmi4_data->resume_notouch_jiffies=%ld\n",
		__func__, rmi4_data->resume_notouch_jiffies);

	dev_dbg(dev,
		"%s: Putting controller to resume, suspend=%d\n",
		__func__, rmi4_data->suspend);
	dev_dbg(dev, "regulator=%d\n",
		regulator_is_enabled(rmi4_data->regulator.regulator));

	if (rmi4_data->stay_awake) {
		rmi4_data->state_changed = true;
		return -EBUSY;
	}

	mutex_lock(&rmi4_data->rmi4_power_mutex);

	while ((rmi4_data->suspend == true)
		&& (retry < SYNAPTICS_MAX_FAILURE_RETRY)) {
		retry++;

		mutex_lock(&rmi4_data->rmi4_reset_mutex);
		retval = do_wakeup(dev, false);
		mutex_unlock(&rmi4_data->rmi4_reset_mutex);

		if (retval < 0) {
			dev_err(dev,
				"%s: Failed to wakeup, retry = %d, "
				"reset controller\n", __func__, retry);
				retval = rmi4_data->reset_device(rmi4_data,
								true);
			continue;
		}
		rmi4_data->suspend = false;
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s:suspend=%d\n",
				__func__, rmi4_data->suspend);

		mutex_lock(&rmi4_data->exp_data.mutex);
		if (!list_empty(&rmi4_data->exp_data.list)) {
			list_for_each_entry(exp_fhandler,
					&rmi4_data->exp_data.list, link)
				if (exp_fhandler->exp_fn->resume != NULL)
					exp_fhandler->
						exp_fn->resume(rmi4_data);
		}
		mutex_unlock(&rmi4_data->exp_data.mutex);
	}

	if (rmi4_data->suspend == true)
		dev_err(dev,
			"%s: Failed to resume device\n",
			__func__);

	mutex_unlock(&rmi4_data->rmi4_power_mutex);
	synaptics_rmi4_monitor_timer_start(rmi4_data);
	/*
	 * Traverse the function handler list and polling
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				retval = synaptics_rmi4_report_touch(
						rmi4_data,
						fhandler, true);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to report touch\n",
						__func__);
					break;
				}
			}
		}
	}
	if ((rmi4_data->wakeup_check_mask & WAKEUP_CHECK_RESUME) != 0) {
		rmi4_data->wakeup_check_mask &= ~WAKEUP_CHECK_RESUME;
		dtwakeup_complete_time_msecs = get_timestamp();
		total_dtwakeup_time_msecs = (dtwakeup_complete_time_msecs) - (rmi4_data->dtwakeup_time_ms);
		if (total_dtwakeup_time_msecs < 200)
			/* Do nothing */;
		else if (total_dtwakeup_time_msecs < 400)
			rmi4_data->mtouch_counter.dtwakeup_delay_200ms++;
		else if (total_dtwakeup_time_msecs < 1000)
			rmi4_data->mtouch_counter.dtwakeup_delay_400ms++;
		else if (total_dtwakeup_time_msecs < 2000)
			rmi4_data->mtouch_counter.dtwakeup_delay_1sec++;
		else {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
				synaptics_dsx_ddt_send(
					rmi4_data, MTOUCH_DEBUG, MTOUCH_DBG_DT_WAKEUP);
#endif
		}
	}
	dev_dbg(dev, "%s: Resume done\n", __func__);

	synaptics_rmi4_stats_start(rmi4_data);

	if (bdata->lockup_poll_interval_ms)
		queue_delayed_work(rmi4_data->workqueue,
			&rmi4_data->lockup_work,
			msecs_to_jiffies(bdata->lockup_poll_interval_ms));

	return retval;
}
#endif

static void synpatics_rmi4_irq_wake_enable(
	struct synaptics_rmi4_data *rmi4_data, bool enable)
{
	if (enable) {
		enable_irq_wake(rmi4_data->irq);
		rmi4_data->en_irq_counter.irq_wake++;
	} else {
		disable_irq_wake(rmi4_data->irq);
		rmi4_data->en_irq_counter.irq_wake--;
	}
}

#ifdef CONFIG_PM_SLEEP
int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (NULL == rmi4_data)
		return 0;
	if (false == rmi4_data->suspend)
		return -EBUSY;


	if (rmi4_data->wakeup_gesture.data)
		synpatics_rmi4_irq_wake_enable(rmi4_data, true);
	return 0;
}

int synaptics_rmi4_resume(struct device *dev)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (NULL == rmi4_data)
		return 0;


	if (rmi4_data->wakeup_gesture.data)
		synpatics_rmi4_irq_wake_enable(rmi4_data, false);

	return 0;
}
#endif

static struct platform_driver synaptics_rmi4_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = synaptics_rmi4_probe,
	.remove = synaptics_rmi4_remove,
};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	int retval;

	retval = synaptics_rmi4_bus_init();
	if (retval)
		return retval;

	return platform_driver_register(&synaptics_rmi4_driver);
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	platform_driver_unregister(&synaptics_rmi4_driver);

	synaptics_rmi4_bus_exit();

	return;
}

fs_initcall_sync(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
