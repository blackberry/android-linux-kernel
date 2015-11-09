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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SYNAPTICS_DSX_RMI4_H_
#define _SYNAPTICS_DSX_RMI4_H_

#define SYNAPTICS_DS4 (1 << 0)
#define SYNAPTICS_DS5 (1 << 1)
#define SYNAPTICS_DSX_DRIVER_PRODUCT (SYNAPTICS_DS4 | SYNAPTICS_DS5)
#define SYNAPTICS_DSX_DRIVER_VERSION 0x2003

#include <linux/version.h>
#if defined(CONFIG_PM) && defined(CONFIG_FB)
#include <linux/fb.h>
#endif
#include <linux/completion.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif

#define PDT_PROPS (0X00EF)
#define PDT_START (0x00E9)
#define PDT_END (0x00D0)
#define PDT_ENTRY_SIZE (0x0006)
#define PAGES_TO_SERVICE (10)
#define PAGE_SELECT_LEN (2)
#define ADDRESS_WORD_LEN (2)

#define SYNAPTICS_RMI4_F01 (0x01)
#define SYNAPTICS_RMI4_F11 (0x11)
#define SYNAPTICS_RMI4_F12 (0x12)
#define SYNAPTICS_RMI4_F1A (0x1a)
#define SYNAPTICS_RMI4_F34 (0x34)
#define SYNAPTICS_RMI4_F51 (0x51)
#define SYNAPTICS_RMI4_F54 (0x54)
#define SYNAPTICS_RMI4_F55 (0x55)
#define SYNAPTICS_RMI4_FDB (0xdb)

#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2
#define SYNAPTICS_RMI4_DATE_CODE_SIZE 3
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_BUILD_ID_SIZE 3

/* F01 data registers - offsets from base address */
#define SYNAPTICS_F01_DATA_DEVICE_STATUS            0x00
  #define SYNAPTICS_F01_UNCONFIGURED                0x80
  #define SYNAPTICS_F01_FLASH_PROG                  0x40
  #define SYNAPTICS_F01_NO_ERROR                    0x00
  #define SYNAPTICS_F01_RESET_OCCURRED              0x01
  #define SYNAPTICS_F01_INV_CONFIG                  0x02
  #define SYNAPTICS_F01_DEV_FAIL                    0x03
  #define SYNAPTICS_F01_CONFIG_CRC_FAIL             0x04
  #define SYNAPTICS_F01_FIRMWARE_CRC_FAIL           0x05
  #define SYNAPTICS_F01_CRC_IN_PROGRESS             0x06

#define SYNAPTICS_CRC_DELAY_MS                      50

#define F12_FINGERS_TO_SUPPORT 10
#define F12_NO_OBJECT_STATUS 0x00
#define F12_FINGER_STATUS 0x01
#define F12_STYLUS_STATUS 0x02
#define F12_PALM_STATUS 0x03
#define F12_HOVERING_FINGER_STATUS 0x05
#define F12_GLOVED_FINGER_STATUS 0x06

#define MAX_NUMBER_OF_BUTTONS 4
#define MAX_INTR_REGISTERS 4

#define MASK_16BIT 0xFFFF
#define MASK_8BIT 0xFF
#define MASK_7BIT 0x7F
#define MASK_6BIT 0x3F
#define MASK_5BIT 0x1F
#define MASK_4BIT 0x0F
#define MASK_3BIT 0x07
#define MASK_2BIT 0x03
#define MASK_1BIT 0x01

/* stats period and size (i.e. number of entries in stat buffer)
 * total stats time is stats period * stats size
 */
#define SYNAPTICS_STATS_PERIOD  5000	/* 5000 ms period */
#define SYNAPTICS_STATS_SIZE	60	/* 60 entries in the buffer */
#define SYNAPTICS_STATS_WAIT    10	/* 10 ms wait */

#define SYNAPTICS_EVENT_DATA_SIZE 7
#define SYNAPTICS_EVENTS_SIZE	100	/* 100 entries in the buffer */

#define F12_DATA_15_WORKAROUND
#define NO_0D_WHILE_2D

#define F12_SWIPE_WAKEUP	(1 << 0)
#define F12_DOUBLE_TAP_WAKEUP	(1 << 1)

#define SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT             (20)
#define SYNAPTICS_SWIPE_BUFFER_EVENT_SIZE               (4)



#define RESUME_IGNORE_TOUCH_DELAY (100)	/*msec*/

/* mtouch counter error codes */
enum tap_failure {
	SYNAPTICS_FIRST_TAP_MOVED_TOO_MUCH = 1,
	SYNAPTICS_FIRST_TAP_TOO_LONG,
	SYNAPTICS_SECOND_TAP_MOVED_TOO_MUCH,
	SYNAPTICS_SECOND_TAP_TOO_LONG,
	SYNAPTICS_SECOND_TAP_TOO_FAR,
	SYNAPTICS_TIME_BETWEEN_TAPS_TOO_LONG,
/*	SYNAPTICS_THREE_TAPS_OBSERVED,*/
	SYNAPTICS_TAP_FAILURE_MAX,
};

enum exp_fn {
	RMI_DEV = 0,
	RMI_F54,
	RMI_FW_UPDATER,
	RMI_TEST_REPORTING,
	RMI_PROXIMITY,
	RMI_ACTIVE_PEN,
	RMI_DEBUG,
	RMI_SLIDE_POSITION,
	RMI_LAST,
};

enum {
	DEVICE_PROPERTY_MAX_TOUCHPOINTS = 1,
	DEVICE_PROPERTY_VENDOR,
	DEVICE_PROPERTY_PRODUCT_ID,
	DEVICE_PROPERTY_SERIAL_ID,
	DEVICE_PROPERTY_FIRMWARE_ID
};

struct synaptics_rmi4_data;

struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

/*
 * struct synaptics_rmi4_fn_desc - function descriptor fields in PDT entry
 * @query_base_addr: base address for query registers
 * @cmd_base_addr: base address for command registers
 * @ctrl_base_addr: base address for control registers
 * @data_base_addr: base address for data registers
 * @intr_src_count: number of interrupt sources
 * @fn_number: function number
 */
struct synaptics_rmi4_fn_desc {
	unsigned char query_base_addr;
	unsigned char cmd_base_addr;
	unsigned char ctrl_base_addr;
	unsigned char data_base_addr;
	unsigned char intr_src_count;
	unsigned char fn_number;
};

/*
 * synaptics_rmi4_fn_full_addr - full 16-bit base addresses
 * @query_base: 16-bit base address for query registers
 * @cmd_base: 16-bit base address for command registers
 * @ctrl_base: 16-bit base address for control registers
 * @data_base: 16-bit base address for data registers
 */
struct synaptics_rmi4_fn_full_addr {
	unsigned short query_base;
	unsigned short cmd_base;
	unsigned short ctrl_base;
	unsigned short data_base;
};

/*
 * struct synaptics_rmi4_f11_extra_data - extra data of F$11
 * @data38_offset: offset to F11_2D_DATA38 register
 */
struct synaptics_rmi4_f11_extra_data {
	unsigned char data38_offset;
};

/*
 * struct synaptics_rmi4_f12_extra_data - extra data of F$12
 * @data1_offset: offset to F12_2D_DATA01 register
 * @data4_offset: offset to F12_2D_DATA04 register
 * @data15_offset: offset to F12_2D_DATA15 register
 * @data15_size: size of F12_2D_DATA15 register
 * @data15_data: buffer for reading F12_2D_DATA15 register
 * @ctrl20_offset: offset to F12_2D_CTRL20 register
 */
struct synaptics_rmi4_f12_extra_data {
	unsigned char data1_offset;
	unsigned char data4_offset;
	unsigned char data13_offset;
	unsigned char data15_offset;
	unsigned char data15_size;
	unsigned char data15_data[(F12_FINGERS_TO_SUPPORT + 7) / 8];
	unsigned char ctrl8_offset;
	unsigned char ctrl20_offset;
	unsigned char ctrl27_offset;
	unsigned char ctrl36_offset;
};

/*
 * struct synaptics_rmi4_fn - RMI function handler
 * @fn_number: function number
 * @num_of_data_sources: number of data sources
 * @num_of_data_points: maximum number of fingers supported
 * @size_of_data_register_block: data register block size
 * @intr_reg_num: index to associated interrupt register
 * @intr_mask: interrupt mask
 * @full_addr: full 16-bit base addresses of function registers
 * @link: linked list for function handlers
 * @data_size: size of private data
 * @data: pointer to private data
 * @extra: pointer to extra data
 */
struct synaptics_rmi4_fn {
	unsigned char fn_number;
	unsigned char num_of_data_sources;
	unsigned char num_of_data_points;
	unsigned char size_of_data_register_block;
	unsigned char intr_reg_num;
	unsigned char intr_mask;
	struct synaptics_rmi4_fn_full_addr full_addr;
	struct list_head link;
	int data_size;
	void *data;
	void *extra;
};

/*
 * struct synaptics_rmi4_device_info - device information
 * @version_major: rmi protocol major version number
 * @version_minor: rmi protocol minor version number
 * @manufacturer_id: manufacturer id
 * @product_props: product properties information
 * @product_info: product info array
 * @date_code: device manufacture date
 * @tester_id: tester id array
 * @serial_number: device serial number
 * @product_id_string: device product id
 * @support_fn_list: linked list for function handlers
 */
struct synaptics_rmi4_device_info {
	unsigned int version_major;
	unsigned int version_minor;
	unsigned char manufacturer_id;
	unsigned char product_props;
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	unsigned char date_code[SYNAPTICS_RMI4_DATE_CODE_SIZE];
	unsigned short tester_id;
	unsigned short serial_number;
	unsigned char product_id_string[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char build_id[SYNAPTICS_RMI4_BUILD_ID_SIZE];
	struct list_head support_fn_list;
};

struct synaptics_rmi4_stats_entry {
	unsigned long msecs;
	int total_int_cnt;
	int total_int_served_cnt;
	int status_int_cnt;
	int touch_int_cnt;
	int touch_report_cnt;
	int touch_large_object_cnt;
	int touch_frelease_cnt;
	int touch_trelease_cnt;
	unsigned char unexpected_reset;
	unsigned char status_error;
};

struct synaptics_rmi4_event_entry {
	unsigned long msecs;
	unsigned char event_id;
	int16_t event_data[SYNAPTICS_EVENT_DATA_SIZE];
};

struct synaptics_rmi4_events {
	struct mutex mutex;
	int num_events_entries;
	int next_events_entry;
	struct synaptics_rmi4_event_entry list[SYNAPTICS_EVENTS_SIZE];
};

struct synaptics_rmi4_stats {
	int total_int_cnt;
	int total_int_served_cnt;
	int status_int_cnt;
	int touch_int_cnt;
	int touch_report_cnt;
	int touch_large_object_cnt;
	int touch_frelease_cnt;
	int touch_trelease_cnt;
	int unexpected_reset;
	int status_error;
	bool active;
	bool update;
	struct mutex mutex;
	struct timer_list timer;
	int num_stats_entries;
	int next_stats_entry;
	struct synaptics_rmi4_stats_entry list[SYNAPTICS_STATS_SIZE];
	struct work_struct timeout_work;
};

struct synaptics_rmi4_exp_fhandler {
	struct synaptics_rmi4_exp_fn *exp_fn;
	bool insert;
	bool remove;
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

struct sm_error {
	int i2c;
};

struct synaptics_rmi4_timer {
	bool active;
	char *name;
	int interval_ms;
	struct mutex mutex;
	struct timer_list timer;
	struct work_struct timeout_work;
};

struct synaptics_wakeup_gesture {
	union {
		struct {
			unsigned char double_tap:1;
			unsigned char swipe:1;
			unsigned char reserved:6;
		} __packed;
		unsigned char data;
	};
};

struct synaptics_rmi4_mtouch_counter {
	int swipe_wakeup;
	int double_tap;
	int upgrade_failure;
	int face_detection;
	int tap_failure[SYNAPTICS_TAP_FAILURE_MAX - 1];
	int total_tap_failure_cnt;
	int tap_failure_in_proxi_delta;
	int inadv_tap_failure_delta0;
	int inadv_tap_failure_delta1;
	int failed_taps_in_proxi;
	int inadv_tap_failures;
	int proxi_detected;
	int proxi_timeout;
	int proxi_not_ready;
	int proxi_io_error;
	int few_pos_buff;
	int i2c_rw_error;

	bool active;
	struct mutex mutex;
	struct timer_list timer;

	int inadv_tap_det_tmout_ms;
	struct timer_list inadv_tapdet_timer;
	struct work_struct inadv_tapdet_tmout_work;
};

/* struct synaptics_disable_irq_counter - enable irq counters
 * counters are decreased when irq is disabled
 * and are increased when irq is enabled
 * @irq: irq disabled/enabled by synaptics_rmi4_irq_enable
 * @irq_nosync: irq disabled/enabled by disable_irq_nosync/enable_irq
 * @irq_wake: irq disabled/enabled by disable_irq_wake/enable_irq_wake
 */
struct synaptics_enable_irq_counter {
	char irq;
	char irq_nosync;
	char irq_wake;
};

struct synpatics_regulator {
	struct regulator *regulator;
	bool enabled;
};

struct synaptics_f51_extra_wakeup_info {
	int x[SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT];
	int y[SYNAPTICS_SWIPE_BUFFER_EVENT_COUNT];
	int max_peak;
	int max_diff;
	int max_energy;
	int first_x;
	unsigned char pos_count;
};

/*
 * struct synaptics_rmi4_data - rmi4 device instance data
 * @pdev: pointer to platform device
 * @input_dev: pointer to associated input device
 * @hw_if: pointer to hardware interface data
 * @rmi4_mod_info: device information
 * @regulator: pointer to associated regulator
 * @rmi4_io_ctrl_mutex: mutex for i2c i/o control
 * @current_page: current page in sensor to acess
 * @button_0d_enabled: flag for 0d button support
 * @full_pm_cycle: flag for full power management cycle in early suspend stage
 * @num_of_intr_regs: number of interrupt registers
 * @f01_query_base_addr: query base address for f01
 * @f01_cmd_base_addr: command base address for f01
 * @f01_ctrl_base_addr: control base address for f01
 * @f01_data_base_addr: data base address for f01
 * @irq: attention interrupt
 * @sensor_max_x: sensor maximum x value
 * @sensor_max_y: sensor maximum y value
 * @irq_enabled: flag for indicating interrupt enable status
 * @fingers_on_2d: flag to indicate presence of fingers in 2d area
 * @sensor_sleep: flag to indicate sleep state of sensor
 * @wait: wait queue for touch data polling in interrupt thread
 * @irq_enable: pointer to irq enable function
 */
struct synaptics_rmi4_data {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	const struct synaptics_dsx_hw_interface *hw_if;
	struct synaptics_rmi4_device_info rmi4_mod_info;
	struct kobject *board_prop_dir;
	struct synpatics_regulator regulator;
	struct synpatics_regulator vcc_i2c;
	struct mutex rmi4_reset_mutex;
	struct mutex rmi4_report_mutex;
	struct mutex rmi4_io_ctrl_mutex;
	struct mutex rmi4_power_mutex;
	struct mutex rmi4_irq_mutex;
	unsigned char current_page;
	unsigned char button_0d_enabled;
	unsigned char full_pm_cycle;
	unsigned char num_of_tx;
	unsigned char num_of_rx;
	unsigned char num_of_fingers;
	unsigned char max_touch_width;
	unsigned char report_enable;
	unsigned char no_sleep_setting;
	unsigned char intr_mask[MAX_INTR_REGISTERS];
	unsigned char *button_txrx_mapping;
	unsigned short num_of_intr_regs;
	unsigned short f01_query_base_addr;
	unsigned short f01_cmd_base_addr;
	unsigned short f01_ctrl_base_addr;
	unsigned short f01_data_base_addr;
	unsigned int firmware_id;
	unsigned char config_id[4];
	int irq;
	int sensor_max_x;
	int sensor_max_y;
	int power_state;
	int next_power_state;
	bool flash_prog_mode;
	bool irq_enabled;
	bool fingers_on_2d;
	bool suspend;
	bool pre_suspend_reset;
	bool turn_off;
	bool sensor_sleep;
	bool stay_awake;
	bool state_changed;
	bool f11_wakeup_gesture;
	bool f12_wakeup_gesture;
	bool proxi_check;
	bool face_detection_check;
	struct synaptics_wakeup_gesture wakeup_gesture;
	struct synaptics_f51_extra_wakeup_info extra_wakeup_info;
#ifdef F12_DATA_15_WORKAROUND
	unsigned char fingers_already_present;
#endif
	unsigned char do_once;
	bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif
	unsigned short ctrl_28_address;
	struct sm_error sm_err;
	unsigned short test_i2c_addr;
	unsigned char num_failures;
	struct synaptics_rmi4_timer monitor_timer;
	struct workqueue_struct *workqueue;
	int (*reset_device)(struct synaptics_rmi4_data *rmi4_data,
			bool is_hw_reset);
	int (*irq_enable)(struct synaptics_rmi4_data *rmi4_data, bool enable,
			bool attn_only);
	int (*power_enable)(struct synaptics_rmi4_data *rmi4_data, bool enable);

	struct synaptics_rmi4_exp_fn_data exp_data;

	/* Holds opaque pointer to firmware module */
	void *fwu;

	/* Holds opaque pointers to test reporting module */
	void *f54;
	void *f55;
	struct completion test_reporting_remove_complete;

	void *rmidev;
	void *slide;

	struct synaptics_rmi4_stats stats;
	struct synaptics_rmi4_events events;
	struct delayed_work recovery_work;
	struct delayed_work lockup_work;
	struct work_struct irq_work;
	struct work_struct reset_work;
	struct work_struct fwu_done;
	bool   hw_reset;
#ifdef CONFIG_PM
	struct work_struct power_state_work;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
#endif
	struct work_struct slider_work;
	bool   lid_state;
	bool   smart_flip_state;
	uint8_t touch_obj_cnt;
	bool   ignore_touch;
	bool   resume_ignore_touch;
	unsigned long resume_notouch_jiffies;
	uint8_t init_complete;
	uint8_t slider_keys_values;
	uint8_t slider_state;
	unsigned char fb_blank;
	unsigned char fb_event;
	bool touch_ready;
	int proxi_pocket;
	struct completion proxi_completion;
	unsigned char wakeup_source;
	bool touch_edge[F12_FINGERS_TO_SUPPORT];
	unsigned char last_tap_status[SYNAPTICS_TAP_FAILURE_MAX];
	struct synaptics_enable_irq_counter en_irq_counter;

	struct synaptics_rmi4_mtouch_counter mtouch_counter;
};

struct synaptics_dsx_bus_access {
	unsigned char type;
	int (*read)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
		unsigned char *data, unsigned short length);
	int (*write)(struct synaptics_rmi4_data *rmi4_data, unsigned short addr,
		unsigned char *data, unsigned short length);
};

struct synaptics_dsx_hw_interface {
	struct synaptics_dsx_board_data *board_data;
	const struct synaptics_dsx_bus_access *bus_access;
	int (*bl_hw_init)(struct synaptics_rmi4_data *rmi4_data);
	int (*ui_hw_init)(struct synaptics_rmi4_data *rmi4_data);
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	int (*init)(struct synaptics_rmi4_data *rmi4_data);
	void (*remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*reset)(struct synaptics_rmi4_data *rmi4_data);
	void (*reinit)(struct synaptics_rmi4_data *rmi4_data);
	void (*suspend)(struct synaptics_rmi4_data *rmi4_data);
	void (*resume)(struct synaptics_rmi4_data *rmi4_data);
	void (*late_resume)(struct synaptics_rmi4_data *rmi4_data);
	int (*attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
};

int synaptics_rmi4_bus_init(void);
void synaptics_rmi4_bus_exit(void);
void synaptics_rmi4_new_function(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_exp_fn *exp_fn_module,
		bool insert);

int synaptics_rmi4_runtime_suspend(struct device *dev);
int synaptics_rmi4_runtime_resume(struct device *dev);

int synaptics_rmi4_suspend(struct device *dev);
int synaptics_rmi4_resume(struct device *dev);

void synaptics_rmi4_monitor_timer_start(
	struct synaptics_rmi4_data *rmi4_data);
void synaptics_rmi4_monitor_timer_stop(
	struct synaptics_rmi4_data *rmi4_data);

static inline int synaptics_rmi4_reg_read(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->hw_if->bus_access->read(rmi4_data, addr, data, len);
}

static inline int synaptics_rmi4_reg_write(
		struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr,
		unsigned char *data,
		unsigned short len)
{
	return rmi4_data->hw_if->bus_access->write(rmi4_data, addr, data, len);
}

static inline ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dev_warn(dev, "%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	dev_warn(dev, "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline void batohs(unsigned short *dest, unsigned char *src)
{
	*dest = src[1] * 0x100 + src[0];
}

static inline void hstoba(unsigned char *dest, unsigned short src)
{
	dest[0] = src % 0x100;
	dest[1] = src / 0x100;
}

#endif
