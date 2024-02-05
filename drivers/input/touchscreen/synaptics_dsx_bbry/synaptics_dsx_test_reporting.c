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
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#include "synaptics_dsx_test_reporting.h"

#define WATCHDOG_HRTIMER
#define WATCHDOG_TIMEOUT_S 2
#define FORCE_TIMEOUT_100MS 10
#define STATUS_WORK_INTERVAL 20 /* ms */

/*
#define RAW_HEX
#define HUMAN_READABLE
*/

#define SENSOR_RX_MAPPING_OFFSET 1
#define SENSOR_TX_MAPPING_OFFSET 2

#define CONTROL_42_SIZE 2
#define CONTROL_43_54_SIZE 13
#define CONTROL_55_56_SIZE 2
#define CONTROL_58_SIZE 1
#define CONTROL_59_SIZE 2
#define CONTROL_60_62_SIZE 3
#define CONTROL_63_SIZE 1
#define CONTROL_64_67_SIZE 4
#define CONTROL_68_73_SIZE 8
#define CONTROL_74_SIZE 2
#define CONTROL_76_SIZE 1
#define CONTROL_77_78_SIZE 2
#define CONTROL_79_83_SIZE 5
#define CONTROL_84_85_SIZE 2
#define CONTROL_86_SIZE 1
#define CONTROL_87_SIZE 1
#define CONTROL_88_SIZE 1
#define CONTROL_89_SIZE 1
#define CONTROL_90_SIZE 1
#define CONTROL_91_SIZE 1
#define CONTROL_92_SIZE 1
#define CONTROL_93_SIZE 1
#define CONTROL_94_SIZE 1
#define CONTROL_95_SIZE 1
#define CONTROL_96_SIZE 1
#define CONTROL_97_SIZE 1
#define CONTROL_98_SIZE 1
#define CONTROL_99_SIZE 1
#define CONTROL_100_SIZE 1

#define HIGH_RESISTANCE_DATA_SIZE 6
#define FULL_RAW_CAP_MIN_MAX_DATA_SIZE 4
#define TREX_DATA_SIZE 7

#define NO_AUTO_CAL_MASK 0x01

static const struct synaptics_reg_query_rule f54_data_reg_query_rules[] = {
	{ 0, NO_QUERY, 1, 1 }, /* always exists */
	{ 1, NO_QUERY, 1, 1 }, /* always exists */
	{ 2, NO_QUERY, 1, 1 }, /* always exists */
	{ 3, NO_QUERY, 1, 1 }, /* always exists */
	{ 4, SYNAPTICS_F54_ANALOG_QUERY6,   SYNAPTICS_F54_QUERY6_HASSENSEFREQCONTROL, 1 },
	{ 5, NO_QUERY, 0, 1 }, /* never exists */
	{ 6, SYNAPTICS_F54_ANALOG_QUERY6,   SYNAPTICS_F54_QUERY6_HASINTERFACEMETRIC, 2 },
	{ 7, SYNAPTICS_F54_ANALOG_QUERY6,   SYNAPTICS_F54_QUERY6_HASONEBYTEREPORTRATE, 1 },
	{ 7, SYNAPTICS_F54_ANALOG_QUERY6,   SYNAPTICS_F54_QUERY6_HASTWOBYTEREPORTRATE, 2 },
	{ 8, SYNAPTICS_F54_ANALOG_QUERY9,   SYNAPTICS_F54_QUERY9_HASVARIANCEMETRIC, 2 },
	{ 9, SYNAPTICS_F54_ANALOG_QUERY9,   SYNAPTICS_F54_QUERY9_HASMMSTATEMACHINE, 2 },
	{ 10, SYNAPTICS_F54_ANALOG_QUERY9,  SYNAPTICS_F54_QUERY9_HASMMSTATEMACHINE, 1 },
	{ 10, SYNAPTICS_F54_ANALOG_QUERY10, SYNAPTICS_F54_QUERY10_HASNOISESTATE, 1 },
	{ 11, SYNAPTICS_F54_ANALOG_QUERY9,  SYNAPTICS_F54_QUERY9_HASSTATUS, 1 },
	{ 12, SYNAPTICS_F54_ANALOG_QUERY9,  SYNAPTICS_F54_QUERY9_HASSLEWMETRIC, 2 },
	{ 13, SYNAPTICS_F54_ANALOG_QUERY9,  SYNAPTICS_F54_QUERY9_HASMMSTATEMACHINE, 2 },
	{ 14, SYNAPTICS_F54_ANALOG_QUERY13, SYNAPTICS_F54_QUERY13_HASCIDIM, 1 },
	{ 15, SYNAPTICS_F54_ANALOG_QUERY13, SYNAPTICS_F54_QUERY13_HASRAILIM, 1 },
	{ 16, SYNAPTICS_F54_ANALOG_QUERY13, SYNAPTICS_F54_QUERY13_HASNOISEMITIGENH, 1 },
	{ 17, SYNAPTICS_F54_ANALOG_QUERY16, SYNAPTICS_F54_QUERY16_HASDATA17, 1 },
	{ 18, SYNAPTICS_F54_ANALOG_QUERY21, SYNAPTICS_F54_QUERY21_HASDATA18, 1 },
	{ 19, SYNAPTICS_F54_ANALOG_QUERY21, SYNAPTICS_F54_QUERY21_HASDATA19, 1 },
	{ 20, SYNAPTICS_F54_ANALOG_QUERY25, SYNAPTICS_F54_QUERY25_HASDATA20, 1 },
	{ QUERY_RULES_END, 0, 0, 0 }
};

static inline ssize_t synaptics_rmi4_test_reporting_show_error(
		struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	printk(KERN_ERR "%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t synaptics_rmi4_test_reporting_store_error(
			struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf,
			size_t count)
{
	printk(KERN_ERR "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

#define concat(a, b) a##b

#define GROUP(_attrs) {\
	.attrs = _attrs,\
}

#define attrify(propname) (&dev_attr_##propname.attr)

#define show_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		char *buf);\
\
struct kobj_attribute dev_attr_##propname =\
		__ATTR(propname, S_IRUSR | S_IRGRP,\
		concat(synaptics_rmi4_f54, _##propname##_show),\
		synaptics_rmi4_test_reporting_store_error);

#define store_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		const char *buf, size_t count);\
\
struct kobj_attribute dev_attr_##propname =\
		__ATTR(propname, S_IWUSR | S_IWGRP,\
		synaptics_rmi4_test_reporting_show_error,\
		concat(synaptics_rmi4_f54, _##propname##_store));

#define show_store_prototype(propname)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		char *buf);\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		const char *buf, size_t count);\
\
struct kobj_attribute dev_attr_##propname =\
		__ATTR(propname, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),\
		concat(synaptics_rmi4_f54, _##propname##_show),\
		concat(synaptics_rmi4_f54, _##propname##_store));

#define simple_show_func(rtype, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		char *buf)\
{\
	struct synaptics_rmi4_data *rmi4_data;\
	struct synaptics_rmi4_f54_handle *f54;\
\
	rmi4_data = dev_get_drvdata(\
			container_of(kobj->parent, struct device, kobj));\
	f54 = rmi4_data->f54;\
\
	return snprintf(buf, PAGE_SIZE, fmt, f54->rtype.propname);\
} \

#define simple_show_func_unsigned(rtype, propname)\
simple_show_func(rtype, propname, "%u\n")

#define show_func(rtype, rgrp, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		char *buf)\
{\
	int retval;\
	struct synaptics_rmi4_data *rmi4_data;\
	struct synaptics_rmi4_f54_handle *f54;\
\
	rmi4_data = dev_get_drvdata(\
			container_of(kobj->parent, struct device, kobj));\
	f54 = rmi4_data->f54;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	retval = synaptics_rmi4_reg_read(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		dev_err(rmi4_data->pdev->dev.parent,\
				"%s: Failed to read " #rtype\
				" " #rgrp "\n",\
				__func__);\
		return retval;\
	} \
\
	return snprintf(buf, PAGE_SIZE, fmt,\
			f54->rtype.rgrp->propname);\
} \

#define show_store_func(rtype, rgrp, propname, fmt)\
show_func(rtype, rgrp, propname, fmt)\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		const char *buf, size_t count)\
{\
	int retval;\
	unsigned long setting;\
	unsigned long o_setting;\
\
	struct synaptics_rmi4_data *rmi4_data;\
	struct synaptics_rmi4_f54_handle *f54;\
\
	rmi4_data = dev_get_drvdata(\
			container_of(kobj->parent, struct device, kobj));\
	f54 = rmi4_data->f54;\
\
	retval = sstrtoul(buf, 10, &setting);\
	if (retval)\
		return retval;\
\
	mutex_lock(&f54->rtype##_mutex);\
	retval = synaptics_rmi4_reg_read(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	if (retval < 0) {\
		mutex_unlock(&f54->rtype##_mutex);\
		dev_err(rmi4_data->pdev->dev.parent,\
				"%s: Failed to read " #rtype\
				" " #rgrp "\n",\
				__func__);\
		return retval;\
	} \
\
	if (f54->rtype.rgrp->propname == setting) {\
		mutex_unlock(&f54->rtype##_mutex);\
		return count;\
	} \
\
	o_setting = f54->rtype.rgrp->propname;\
	f54->rtype.rgrp->propname = setting;\
\
	retval = synaptics_rmi4_reg_write(rmi4_data,\
			f54->rtype.rgrp->address,\
			f54->rtype.rgrp->data,\
			sizeof(f54->rtype.rgrp->data));\
	if (retval < 0) {\
		dev_err(rmi4_data->pdev->dev.parent,\
				"%s: Failed to write " #rtype\
				" " #rgrp "\n",\
				__func__);\
		f54->rtype.rgrp->propname = o_setting;\
		mutex_unlock(&f54->rtype##_mutex);\
		return retval;\
	} \
\
	mutex_unlock(&f54->rtype##_mutex);\
	return count;\
} \

#define show_store_func_unsigned(rtype, rgrp, propname)\
show_store_func(rtype, rgrp, propname, "%u\n")

#define show_replicated_func(rtype, rgrp, propname, fmt)\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_show)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		char *buf)\
{\
	int retval;\
	int size = 0;\
	unsigned char ii;\
	unsigned char length;\
	unsigned char *temp;\
\
	struct synaptics_rmi4_data *rmi4_data;\
	struct synaptics_rmi4_f54_handle *f54;\
\
	rmi4_data = dev_get_drvdata(\
		container_of(kobj->parent, struct device, kobj));\
	f54 = rmi4_data->f54;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	length = f54->rtype.rgrp->length;\
\
	retval = synaptics_rmi4_reg_read(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		dev_dbg(rmi4_data->pdev->dev.parent,\
				"%s: Failed to read " #rtype\
				" " #rgrp "\n",\
				__func__);\
		return retval;\
	} \
\
	temp = buf;\
\
	for (ii = 0; ii < length; ii++) {\
		retval = snprintf(temp, PAGE_SIZE - size, fmt " ",\
				f54->rtype.rgrp->data[ii].propname);\
		if (retval < 0) {\
			dev_err(rmi4_data->pdev->dev.parent,\
					"%s: Faild to write output\n",\
					__func__);\
			return retval;\
		} \
		size += retval;\
		temp += retval;\
	} \
\
	retval = snprintf(temp, PAGE_SIZE - size, "\n");\
	if (retval < 0) {\
		dev_err(rmi4_data->pdev->dev.parent,\
				"%s: Faild to write null terminator\n",\
				__func__);\
		return retval;\
	} \
\
	return size + retval;\
} \

#define show_replicated_func_unsigned(rtype, rgrp, propname)\
show_replicated_func(rtype, rgrp, propname, "%u")

#define show_store_replicated_func(rtype, rgrp, propname, fmt)\
show_replicated_func(rtype, rgrp, propname, fmt)\
\
static ssize_t concat(synaptics_rmi4_f54, _##propname##_store)(\
		struct kobject *kobj,\
		struct kobj_attribute *attr,\
		const char *buf, size_t count)\
{\
	int retval;\
	unsigned int setting;\
	unsigned char ii;\
	unsigned char length;\
	const unsigned char *temp;\
\
	struct synaptics_rmi4_data *rmi4_data;\
	struct synaptics_rmi4_f54_handle *f54;\
\
	rmi4_data = dev_get_drvdata(\
		container_of(kobj->parent, struct device, kobj));\
	f54 = rmi4_data->f54;\
\
	mutex_lock(&f54->rtype##_mutex);\
\
	length = f54->rtype.rgrp->length;\
\
	retval = synaptics_rmi4_reg_read(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	if (retval < 0) {\
		dev_dbg(rmi4_data->pdev->dev.parent,\
				"%s: Failed to read " #rtype\
				" " #rgrp "\n",\
				__func__);\
			mutex_unlock(&f54->rtype##_mutex);\
			return retval;\
	} \
\
	temp = buf;\
\
	for (ii = 0; ii < length; ii++) {\
		if (sscanf(temp, fmt, &setting) == 1) {\
			f54->rtype.rgrp->data[ii].propname = setting;\
		} else {\
			retval = synaptics_rmi4_reg_read(rmi4_data,\
					f54->rtype.rgrp->address,\
					(unsigned char *)f54->rtype.rgrp->data,\
					length);\
			mutex_unlock(&f54->rtype##_mutex);\
			return -EINVAL;\
		} \
\
		while (*temp != 0) {\
			temp++;\
			if (isspace(*(temp - 1)) && !isspace(*temp))\
				break;\
		} \
	} \
\
	retval = synaptics_rmi4_reg_write(rmi4_data,\
			f54->rtype.rgrp->address,\
			(unsigned char *)f54->rtype.rgrp->data,\
			length);\
	mutex_unlock(&f54->rtype##_mutex);\
	if (retval < 0) {\
		dev_err(rmi4_data->pdev->dev.parent,\
				"%s: Failed to write " #rtype\
				" " #rgrp "\n",\
				__func__);\
		return retval;\
	} \
\
	return count;\
} \

#define show_store_replicated_func_unsigned(rtype, rgrp, propname)\
show_store_replicated_func(rtype, rgrp, propname, "%u")

struct f55_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_edge_compensation:1;
			unsigned char curve_compensation_mode:2;
			unsigned char has_ctrl6:1;
			unsigned char has_alternate_transmitter_assignment:1;
			unsigned char has_single_layer_multi_touch:1;
			unsigned char has_query5:1;
		} __packed;
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f55_handle {
	unsigned char *rx_assignment;
	unsigned char *tx_assignment;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	struct f55_query query;
};

show_prototype(status)
show_prototype(report_size)
#ifdef CONFIG_BBRY_DEBUG
show_store_prototype(no_auto_cal)
show_store_prototype(report_type)
show_store_prototype(fifoindex)
store_prototype(do_preparation)
store_prototype(get_report)
store_prototype(force_cal)
store_prototype(resume_touch)
#else
show_prototype(no_auto_cal)
show_prototype(report_type)
show_prototype(fifoindex)
#endif
show_prototype(num_of_mapped_rx)
show_prototype(num_of_mapped_tx)
show_prototype(num_of_rx_electrodes)
show_prototype(num_of_tx_electrodes)
show_prototype(has_image16)
show_prototype(has_image8)
show_prototype(has_baseline)
show_prototype(clock_rate)
show_prototype(touch_controller_family)
show_prototype(has_pixel_touch_threshold_adjustment)
show_prototype(has_sensor_assignment)
show_prototype(has_interference_metric)
show_prototype(has_sense_frequency_control)
show_prototype(has_firmware_noise_mitigation)
show_prototype(has_two_byte_report_rate)
show_prototype(has_one_byte_report_rate)
show_prototype(has_relaxation_control)
show_prototype(curve_compensation_mode)
show_prototype(has_iir_filter)
show_prototype(has_cmn_removal)
show_prototype(has_cmn_maximum)
show_prototype(has_touch_hysteresis)
show_prototype(has_edge_compensation)
show_prototype(has_per_frequency_noise_control)
show_prototype(has_signal_clarity)
show_prototype(number_of_sensing_frequencies)

#ifdef CONFIG_BBRY_DEBUG
show_store_prototype(no_relax)
show_store_prototype(no_scan)
show_store_prototype(bursts_per_cluster)
show_store_prototype(saturation_cap)
show_store_prototype(pixel_touch_threshold)
show_store_prototype(rx_feedback_cap)
show_store_prototype(low_ref_cap)
show_store_prototype(low_ref_feedback_cap)
show_store_prototype(low_ref_polarity)
show_store_prototype(high_ref_cap)
show_store_prototype(high_ref_feedback_cap)
show_store_prototype(high_ref_polarity)
show_store_prototype(cbc_cap)
show_store_prototype(cbc_polarity)
show_store_prototype(cbc_tx_carrier_selection)
show_store_prototype(integration_duration)
show_store_prototype(reset_duration)
show_store_prototype(noise_sensing_bursts_per_image)
show_store_prototype(slow_relaxation_rate)
show_store_prototype(fast_relaxation_rate)
show_store_prototype(rxs_on_xaxis)
show_store_prototype(curve_comp_on_txs)
#else
show_prototype(no_relax)
show_prototype(no_scan)
show_prototype(bursts_per_cluster)
show_prototype(saturation_cap)
show_prototype(pixel_touch_threshold)
show_prototype(rx_feedback_cap)
show_prototype(low_ref_cap)
show_prototype(low_ref_feedback_cap)
show_prototype(low_ref_polarity)
show_prototype(high_ref_cap)
show_prototype(high_ref_feedback_cap)
show_prototype(high_ref_polarity)
show_prototype(cbc_cap)
show_prototype(cbc_polarity)
show_prototype(cbc_tx_carrier_selection)
show_prototype(integration_duration)
show_prototype(reset_duration)
show_prototype(noise_sensing_bursts_per_image)
show_prototype(slow_relaxation_rate)
show_prototype(fast_relaxation_rate)
show_prototype(rxs_on_xaxis)
show_prototype(curve_comp_on_txs)
#endif /* CONFIG_BBRY_DEBUG*/
show_prototype(sensor_rx_assignment)
show_prototype(sensor_tx_assignment)
show_prototype(burst_count)
show_prototype(disable)
show_prototype(filter_bandwidth)
show_prototype(stretch_duration)
#ifdef CONFIG_BBRY_DEBUG
show_store_prototype(disable_noise_mitigation)
show_store_prototype(freq_shift_noise_threshold)
show_store_prototype(medium_noise_threshold)
show_store_prototype(high_noise_threshold)
show_store_prototype(noise_density)
show_store_prototype(frame_count)
show_store_prototype(iir_filter_coef)
show_store_prototype(quiet_threshold)
show_store_prototype(cmn_filter_disable)
show_store_prototype(cmn_filter_max)
show_store_prototype(touch_hysteresis)
show_store_prototype(rx_low_edge_comp)
show_store_prototype(rx_high_edge_comp)
show_store_prototype(tx_low_edge_comp)
show_store_prototype(tx_high_edge_comp)
show_store_prototype(axis1_comp)
show_store_prototype(axis2_comp)
#else
show_prototype(disable_noise_mitigation)
show_prototype(freq_shift_noise_threshold)
show_prototype(medium_noise_threshold)
show_prototype(high_noise_threshold)
show_prototype(noise_density)
show_prototype(frame_count)
show_prototype(iir_filter_coef)
show_prototype(quiet_threshold)
show_prototype(cmn_filter_disable)
show_prototype(cmn_filter_max)
show_prototype(touch_hysteresis)
show_prototype(rx_low_edge_comp)
show_prototype(rx_high_edge_comp)
show_prototype(tx_low_edge_comp)
show_prototype(tx_high_edge_comp)
show_prototype(axis1_comp)
show_prototype(axis2_comp)
#endif /* CONFIG_BBRY_DEBUG*/
show_prototype(noise_control_1)
show_prototype(noise_control_2)
show_prototype(noise_control_3)
#ifdef CONFIG_BBRY_DEBUG
show_store_prototype(no_signal_clarity)
show_store_prototype(cbc_cap_0d)
show_store_prototype(cbc_polarity_0d)
show_store_prototype(cbc_tx_carrier_selection_0d)
#else
show_prototype(no_signal_clarity)
show_prototype(cbc_cap_0d)
show_prototype(cbc_polarity_0d)
show_prototype(cbc_tx_carrier_selection_0d)
#endif /* CONFIG_BBRY_DEBUG*/

show_store_prototype(full_raw_rt78)
show_store_prototype(keyboard_open_rt78)
static ssize_t synaptics_rmi4_f54_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct attribute *attrs[] = {
	attrify(status),
	attrify(report_size),
	attrify(no_auto_cal),
	attrify(report_type),
	attrify(fifoindex),
#ifdef CONFIG_BBRY_DEBUG
	attrify(do_preparation),
	attrify(get_report),
	attrify(force_cal),
	attrify(resume_touch),
#endif  /* CONFIG_BBRY_DEBUG*/
	attrify(num_of_mapped_rx),
	attrify(num_of_mapped_tx),
	attrify(num_of_rx_electrodes),
	attrify(num_of_tx_electrodes),
	attrify(has_image16),
	attrify(has_image8),
	attrify(has_baseline),
	attrify(clock_rate),
	attrify(touch_controller_family),
	attrify(has_pixel_touch_threshold_adjustment),
	attrify(has_sensor_assignment),
	attrify(has_interference_metric),
	attrify(has_sense_frequency_control),
	attrify(has_firmware_noise_mitigation),
	attrify(has_two_byte_report_rate),
	attrify(has_one_byte_report_rate),
	attrify(has_relaxation_control),
	attrify(curve_compensation_mode),
	attrify(has_iir_filter),
	attrify(has_cmn_removal),
	attrify(has_cmn_maximum),
	attrify(has_touch_hysteresis),
	attrify(has_edge_compensation),
	attrify(has_per_frequency_noise_control),
	attrify(has_signal_clarity),
	attrify(number_of_sensing_frequencies),
	attrify(full_raw_rt78),
	attrify(keyboard_open_rt78),
	NULL,
};

static struct attribute_group attr_group = GROUP(attrs);

static struct attribute *attrs_reg_0[] = {
	attrify(no_relax),
	attrify(no_scan),
	NULL,
};

static struct attribute *attrs_reg_1[] = {
	attrify(bursts_per_cluster),
	NULL,
};

static struct attribute *attrs_reg_2[] = {
	attrify(saturation_cap),
	NULL,
};

static struct attribute *attrs_reg_3[] = {
	attrify(pixel_touch_threshold),
	NULL,
};

static struct attribute *attrs_reg_4__6[] = {
	attrify(rx_feedback_cap),
	attrify(low_ref_cap),
	attrify(low_ref_feedback_cap),
	attrify(low_ref_polarity),
	attrify(high_ref_cap),
	attrify(high_ref_feedback_cap),
	attrify(high_ref_polarity),
	NULL,
};

static struct attribute *attrs_reg_7[] = {
	attrify(cbc_cap),
	attrify(cbc_polarity),
	attrify(cbc_tx_carrier_selection),
	NULL,
};

static struct attribute *attrs_reg_8__9[] = {
	attrify(integration_duration),
	attrify(reset_duration),
	NULL,
};

static struct attribute *attrs_reg_10[] = {
	attrify(noise_sensing_bursts_per_image),
	NULL,
};

static struct attribute *attrs_reg_11[] = {
	NULL,
};

static struct attribute *attrs_reg_12__13[] = {
	attrify(slow_relaxation_rate),
	attrify(fast_relaxation_rate),
	NULL,
};

static struct attribute *attrs_reg_14__16[] = {
	attrify(rxs_on_xaxis),
	attrify(curve_comp_on_txs),
	attrify(sensor_rx_assignment),
	attrify(sensor_tx_assignment),
	NULL,
};

static struct attribute *attrs_reg_17__19[] = {
	attrify(burst_count),
	attrify(disable),
	attrify(filter_bandwidth),
	attrify(stretch_duration),
	NULL,
};

static struct attribute *attrs_reg_20[] = {
	attrify(disable_noise_mitigation),
	NULL,
};

static struct attribute *attrs_reg_21[] = {
	attrify(freq_shift_noise_threshold),
	NULL,
};

static struct attribute *attrs_reg_22__26[] = {
	attrify(medium_noise_threshold),
	attrify(high_noise_threshold),
	attrify(noise_density),
	attrify(frame_count),
	NULL,
};

static struct attribute *attrs_reg_27[] = {
	attrify(iir_filter_coef),
	NULL,
};

static struct attribute *attrs_reg_28[] = {
	attrify(quiet_threshold),
	NULL,
};

static struct attribute *attrs_reg_29[] = {
	attrify(cmn_filter_disable),
	NULL,
};

static struct attribute *attrs_reg_30[] = {
	attrify(cmn_filter_max),
	NULL,
};

static struct attribute *attrs_reg_31[] = {
	attrify(touch_hysteresis),
	NULL,
};

static struct attribute *attrs_reg_32__35[] = {
	attrify(rx_low_edge_comp),
	attrify(rx_high_edge_comp),
	attrify(tx_low_edge_comp),
	attrify(tx_high_edge_comp),
	NULL,
};

static struct attribute *attrs_reg_36[] = {
	attrify(axis1_comp),
	NULL,
};

static struct attribute *attrs_reg_37[] = {
	attrify(axis2_comp),
	NULL,
};

static struct attribute *attrs_reg_38__40[] = {
	attrify(noise_control_1),
	attrify(noise_control_2),
	attrify(noise_control_3),
	NULL,
};

static struct attribute *attrs_reg_41[] = {
	attrify(no_signal_clarity),
	NULL,
};

static struct attribute *attrs_reg_57[] = {
	attrify(cbc_cap_0d),
	attrify(cbc_polarity_0d),
	attrify(cbc_tx_carrier_selection_0d),
	NULL,
};

static struct attribute_group attrs_ctrl_regs[] = {
	GROUP(attrs_reg_0),
	GROUP(attrs_reg_1),
	GROUP(attrs_reg_2),
	GROUP(attrs_reg_3),
	GROUP(attrs_reg_4__6),
	GROUP(attrs_reg_7),
	GROUP(attrs_reg_8__9),
	GROUP(attrs_reg_10),
	GROUP(attrs_reg_11),
	GROUP(attrs_reg_12__13),
	GROUP(attrs_reg_14__16),
	GROUP(attrs_reg_17__19),
	GROUP(attrs_reg_20),
	GROUP(attrs_reg_21),
	GROUP(attrs_reg_22__26),
	GROUP(attrs_reg_27),
	GROUP(attrs_reg_28),
	GROUP(attrs_reg_29),
	GROUP(attrs_reg_30),
	GROUP(attrs_reg_31),
	GROUP(attrs_reg_32__35),
	GROUP(attrs_reg_36),
	GROUP(attrs_reg_37),
	GROUP(attrs_reg_38__40),
	GROUP(attrs_reg_41),
	GROUP(attrs_reg_57),
};

static bool attrs_ctrl_regs_exist[ARRAY_SIZE(attrs_ctrl_regs)];

static struct bin_attribute dev_report_data = {
	.attr = {
		.name = "report_data",
		.mode = S_IRUSR | S_IRGRP,
	},
	.size = 0,
	.read = synaptics_rmi4_f54_data_read,
};

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static unsigned char *g_f54_image_output;
static bool g_flag_readrt_err; 
static bool is_report_type_valid(struct synaptics_rmi4_f54_handle *f54,
	enum f54_report_types report_type)
{
	switch (report_type) {
	case F54_8BIT_IMAGE:
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORT:
	case F54_RX_TO_RX1:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS1:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_TREX_OPENS:
	case F54_TREX_TO_GND:
	case F54_TREX_SHORTS:
	case F54_ABS_DELTA:
	case F54_ABS_RAW:
	case F54_AMP_FULL_RAW_CAP:
		return true;
		break;
	default:
		f54->report_type = INVALID_REPORT_TYPE;
		f54->report_size = 0;
		return false;
	}
}
#endif /*CONFIG_BBRY_DEBUG*/

void set_report_size(struct synaptics_rmi4_data *rmi4_data)
{

	int retval;
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;
	unsigned char rx = f54->rx_assigned;
	unsigned char tx = f54->tx_assigned;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		f54->report_size = rx * tx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
	case F54_SENSOR_SPEED:
		f54->report_size = 2 * rx * tx;
		break;
	case F54_AMP_FULL_RAW_CAP:  
		if ((rx != 0) && (tx != 0)) {
			f54->report_size = 2 * rx * tx;
		}
		else if (tx == 0) {
			f54->report_size = 2 * rx;
		}
		else if (rx == 0) {
			f54->report_size = 2 * tx;
		}
		
		pr_err("%s: report_size = %d \n", __func__, f54->report_size);
		break;
	case F54_HIGH_RESISTANCE:
		f54->report_size = HIGH_RESISTANCE_DATA_SIZE;
		break;
	case F54_TX_TO_TX_SHORT:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
		f54->report_size = (tx + 7) / 8;
		break;
	case F54_RX_TO_RX1:
	case F54_RX_OPENS1:
		if (rx < tx)
			f54->report_size = 2 * rx * rx;
		else
			f54->report_size = 2 * rx * tx;
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		f54->report_size = FULL_RAW_CAP_MIN_MAX_DATA_SIZE;
		break;
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
		if (rx <= tx)
			f54->report_size = 0;
		else
			f54->report_size = 2 * rx * (rx - tx);
		break;
	case F54_ADC_RANGE:
		if (f54->query.has_signal_clarity) {
			mutex_lock(&f54->control_mutex);
			retval = synaptics_rmi4_reg_read(rmi4_data,
					f54->control.reg_41->address,
					f54->control.reg_41->data,
					sizeof(f54->control.reg_41->data));
			mutex_unlock(&f54->control_mutex);
			if (retval < 0) {
				dev_dbg(rmi4_data->pdev->dev.parent,
						"%s: Failed to read control reg_41\n",
						__func__);
				f54->report_size = 0;
				break;
			}
			if (!f54->control.reg_41->no_signal_clarity) {
				if (tx % 4)
					tx += 4 - (tx % 4);
			}
		}
		f54->report_size = 2 * rx * tx;
		break;
	case F54_TREX_OPENS:
	case F54_TREX_TO_GND:
	case F54_TREX_SHORTS:
		f54->report_size = TREX_DATA_SIZE;
		break;
	case F54_ABS_DELTA:
		f54->report_size = (rx + tx) * 4;
		break;
	case F54_ABS_RAW:
		f54->report_size = (rx + tx) * 4;
		break;
	default:
		f54->report_size = 0;
	}

	return;
}
#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
static int set_interrupt(struct synaptics_rmi4_data *rmi4_data, bool set)
{
	int retval;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short f01_ctrl_reg;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	intr_mask = rmi4_data->intr_mask;
	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (!set) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f01_ctrl_reg,
				&zero,
				sizeof(zero));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to read control f01 control reg\n",
						__func__);
			return retval;
		}
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (set) {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						f01_ctrl_reg,
						&zero,
						sizeof(zero));
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to zero control f01 control reg\n",
						__func__);
					return retval;
				}
			} else {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						f01_ctrl_reg,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to write control f01 control reg\n",
						__func__);
					return retval;
				}
			}
		}
	}

	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (set) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f01_ctrl_reg,
				&f54->intr_mask,
				1);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to write f54 intr reg\n",
						__func__);
			return retval;
		}
	}

	return 0;
}

static int do_preparation(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char value;
	unsigned char command;
	unsigned char timeout_count;
	struct synaptics_rmi4_f01_device_status device_status;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (rmi4_data->suspend == true) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Power state must be AWAKE to run BIST test\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&f54->control_mutex);

	/*
	 * Check F01_RMI_Data00 - Bit6(FlashProg), [0-UI mode]
	 */
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		mutex_unlock(&f54->control_mutex);
		return retval;
	}

	if (device_status.flash_prog == 1) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: tp FlashProg [0x%x]: %x\n",
				__func__,
				rmi4_data->f01_data_base_addr,
				device_status.data[0]);
		mutex_unlock(&f54->control_mutex);
		return -EBUSY;
	}

	/*
	 * Disable Baseline compensation
	 */
	if (f54->query.touch_controller_family == 1) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_7->address,
				&f54->ctrl_value,
				sizeof(f54->ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to  disable CBC(read ctrl7)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}

		value = 0;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_7->address,
				&value,
				sizeof(f54->control.reg_7->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable CBC\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	} else if ((f54->query.has_ctrl88 == 1) && (bdata->disable_bc_bist == 1)) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Disable CBC\n",
			__func__);
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_88->address,
				f54->control.reg_88->data,
				sizeof(f54->control.reg_88->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable CBC (read ctrl88)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
		f54->ctrl_value = f54->control.reg_88->data[0];
		f54->control.reg_88->cbc_polarity = 0;
		f54->control.reg_88->cbc_tx_carrier_selection = 0;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_88->address,
				f54->control.reg_88->data,
				sizeof(f54->control.reg_88->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable CBC (write ctrl88)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	if (f54->query.has_0d_acquisition_control) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_57->address,
				&f54->acquisition_ctrl_value,
				sizeof(f54->acquisition_ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable 0D CBC (read ctrl57)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}

		value = 0;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_57->address,
				&value,
				sizeof(f54->control.reg_57->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable 0D CBC\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	/*
	 * Disable the SignalClarity
	 */
	if (f54->query.has_signal_clarity) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->control.reg_41->address,
				&f54->ctrl41_value,
				sizeof(f54->ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to  disable signal clarity(read ctrl41)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}

		value = 1;
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_41->address,
				&value,
				sizeof(f54->control.reg_41->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable signal clarity\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	mutex_unlock(&f54->control_mutex);

	/*
	 * Set ForceUpdate
	 */
	/* Set ForceUpdate (F54_AD_Cmd0, bit 2) to ‘1’ */
	command = (unsigned char)COMMAND_FORCE_UPDATE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);
		return -ETIMEDOUT;
	}

	/*
	 * Set ForceCal (F54_AD_Cmd0, bit 1)
	 */
	/* Write 0x02 to register F54_ANALOG_Cmd0 */
	command = (unsigned char)COMMAND_FORCE_CAL;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force cal command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force cal\n",
				__func__);
		return -ETIMEDOUT;
	}

	return 0;
}
#endif /*defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)*/

#ifdef WATCHDOG_HRTIMER
static void timeout_set_status(struct work_struct *work)
{
	int retval;
	unsigned char command;

	struct synaptics_rmi4_f54_handle *f54 =
		container_of(work,
			struct synaptics_rmi4_f54_handle,
			timeout_work);

	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->status_mutex);
	if (f54->status == STATUS_BUSY) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
		} else if (command & COMMAND_GET_REPORT) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Report type not supported by FW\n",
					__func__);
			f54->status = STATUS_IDLE;
		} else {
			queue_delayed_work(f54->status_workqueue,
					&f54->status_work,
					0);
			mutex_unlock(&f54->status_mutex);
			return;
		}
		f54->report_type = INVALID_REPORT_TYPE;
		f54->report_size = 0;
	}
	mutex_unlock(&f54->status_mutex);

	return;
}

static enum hrtimer_restart get_report_timeout(struct hrtimer *timer)
{
	struct synaptics_rmi4_f54_handle *f54 =
		container_of(timer, struct synaptics_rmi4_f54_handle, watchdog);

	schedule_work(&(f54->timeout_work));

	return HRTIMER_NORESTART;
}
#endif

#ifdef RAW_HEX
static void print_raw_hex_report(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned int ii;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	pr_info("%s: Report data (raw hex)\n", __func__);

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
		for (ii = 0; ii < f54->report_size; ii += 2) {
			pr_info("%03d: 0x%02x%02x\n",
					ii / 2,
					f54->report_data[ii + 1],
					f54->report_data[ii]);
		}
		break;
	default:
		for (ii = 0; ii < f54->report_size; ii++)
			pr_info("%03d: 0x%02x\n", ii, f54->report_data[ii]);
		break;
	}

	return;
}
#endif

#ifdef HUMAN_READABLE
static void print_image_report(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned int ii;
	unsigned int jj;
	short *report_data;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		pr_info("%s: Report data (image)\n", __func__);

		report_data = (short *)f54->report_data;

		for (ii = 0; ii < f54->tx_assigned; ii++) {
			for (jj = 0; jj < f54->rx_assigned; jj++) {
				if (*report_data < -64)
					pr_cont(".");
				else if (*report_data < 0)
					pr_cont("-");
				else if (*report_data > 64)
					pr_cont("*");
				else if (*report_data > 0)
					pr_cont("+");
				else
					pr_cont("0");

				report_data++;
			}
			pr_info("");
		}
		pr_info("%s: End of report\n", __func__);
		break;
	default:
		pr_info("%s: Image not supported for report type %d\n",
				__func__, f54->report_type);
	}

	return;
}
#endif

static void free_control_mem(struct synaptics_rmi4_f54_handle *f54)
{
	struct f54_control control = f54->control;

	kfree(control.reg_0);
	kfree(control.reg_1);
	kfree(control.reg_2);
	kfree(control.reg_3);
	kfree(control.reg_4__6);
	kfree(control.reg_7);
	kfree(control.reg_8__9);
	kfree(control.reg_10);
	kfree(control.reg_11);
	kfree(control.reg_12__13);
	kfree(control.reg_14);
	kfree(control.reg_15);
	kfree(control.reg_16);
	kfree(control.reg_17);
	kfree(control.reg_18);
	kfree(control.reg_19);
	kfree(control.reg_20);
	kfree(control.reg_21);
	kfree(control.reg_22__26);
	kfree(control.reg_27);
	kfree(control.reg_28);
	kfree(control.reg_29);
	kfree(control.reg_30);
	kfree(control.reg_31);
	kfree(control.reg_32__35);
	kfree(control.reg_36);
	kfree(control.reg_37);
	kfree(control.reg_38);
	kfree(control.reg_39);
	kfree(control.reg_40);
	kfree(control.reg_41);
	kfree(control.reg_57);
	kfree(control.reg_99);

	return;
}

static void remove_sysfs(struct synaptics_rmi4_f54_handle *f54)
{
	int reg_num;

	sysfs_remove_bin_file(f54->attr_dir, &dev_report_data);

	sysfs_remove_group(f54->attr_dir, &attr_group);

	for (reg_num = 0; reg_num < ARRAY_SIZE(attrs_ctrl_regs); reg_num++)
		sysfs_remove_group(f54->attr_dir, &attrs_ctrl_regs[reg_num]);

	kobject_put(f54->attr_dir);

	return;
}

static ssize_t synaptics_rmi4_f54_status_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->status);
}

static ssize_t synaptics_rmi4_f54_report_size_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_size);
}

static ssize_t synaptics_rmi4_f54_no_auto_cal_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->no_auto_cal);
}

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f54_no_auto_cal_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting > 1)
		return -EINVAL;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read control register\n",
				__func__);
		return retval;
	}

	if ((data & NO_AUTO_CAL_MASK) == setting)
		return count;

	data = (data & ~NO_AUTO_CAL_MASK) | (data & NO_AUTO_CAL_MASK);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write control register\n",
				__func__);
		return retval;
	}

	f54->no_auto_cal = (setting == 1);

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f54_report_type_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_type);
}

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
static int set_report_type(struct synaptics_rmi4_data *rmi4_data,
			enum f54_report_types report_type)
{
	struct synaptics_rmi4_f54_handle *f54;
	unsigned char data;
	int retval;
	f54 = rmi4_data->f54;
	f54->report_type = report_type;
	data = (unsigned char)report_type;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr,
			&data,
			sizeof(data));

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to write data register\n",
			__func__);
	}

	return retval;
}

static int check_device_busy(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_f54_handle *f54;

	f54 = rmi4_data->f54;
	if (f54->status != STATUS_IDLE) {
		if (f54->status != STATUS_BUSY) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Invalid status (%d)\n",
				__func__, f54->status);
			return -EINVAL;
		} else {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Previous get report still ongoing\n",
				__func__);
			return -EBUSY;
		}
	}
	return 0;
}

static ssize_t synaptics_rmi4_f54_report_type_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (!is_report_type_valid(f54, (enum f54_report_types)setting)) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Report type not supported by driver\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	retval = check_device_busy(rmi4_data);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}


	retval = set_report_type(rmi4_data, (enum f54_report_types)setting);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	mutex_unlock(&f54->status_mutex);
	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f54_fifoindex_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned char data[2];
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read data registers\n",
				__func__);
		return retval;
	}

	batohs(&f54->fifoindex, data);

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->fifoindex);
}
#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f54_fifoindex_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data[2];
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
		container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	f54->fifoindex = setting;

	hstoba(data, (unsigned short)setting);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write data registers\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f54_do_preparation_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;

	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	mutex_lock(&f54->status_mutex);

	retval = check_device_busy(rmi4_data);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	mutex_unlock(&f54->status_mutex);

	retval = do_preparation(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to do preparation\n",
				__func__);
		return retval;
	}

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/
#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static int get_report(struct synaptics_rmi4_data *rmi4_data)
{

	struct synaptics_rmi4_f54_handle *f54;
	unsigned char command;
	int retval;
	int i, get_report_retry = 1;

	f54 = rmi4_data->f54;

	/* Request the report and poll until ready;
	 * we may timeout if the controller happens to enter noise
	 * mitigation during the report, so retry as needed.
	 */

	do {
		command = 0;

		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to clear F54 command 0 register\n",
				__func__);
			return retval;
		}

		set_interrupt(rmi4_data, true);  // just enable the F$54 interrupt, disable it in resume touch function

		command = (unsigned char)COMMAND_GET_REPORT;

		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));

		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write get report command\n",
				__func__);
			return retval;
		}

		for (i = 0; i < SYNAPTICS_F54_GET_REPORT_MAX_RETRY; i++) {
			msleep(SYNAPTICS_SAMPLE_PERIOD_MS);
			retval = synaptics_rmi4_reg_read(rmi4_data,
					f54->command_base_addr,
					&command,
					sizeof(command));
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
				return retval;
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

		return -ETIME;
	}

	return 0;
}
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/
#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f54_get_report_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char command;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	command = (unsigned char)COMMAND_GET_REPORT;

	if (!is_report_type_valid(f54, f54->report_type)) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Invalid report type\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_IDLE) {
		if (f54->status != STATUS_BUSY) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Invalid status (%d)\n",
					__func__, f54->status);
		} else {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Previous get report still ongoing\n",
					__func__);
		}
		mutex_unlock(&f54->status_mutex);
		return -EBUSY;
	}

	set_interrupt(rmi4_data, true);

	f54->status = STATUS_BUSY;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	mutex_unlock(&f54->status_mutex);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write get report command\n",
				__func__);
		return retval;
	}

#ifdef WATCHDOG_HRTIMER
	hrtimer_start(&f54->watchdog,
			ktime_set(WATCHDOG_TIMEOUT_S, 0),
			HRTIMER_MODE_REL);
#endif

	return count;
}

static ssize_t synaptics_rmi4_f54_force_cal_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char command;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	command = (unsigned char)COMMAND_FORCE_CAL;

	if (f54->status == STATUS_BUSY) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: f54 status busy\n",
				__func__);
		return -EBUSY;
	}

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force cal command\n",
				__func__);
		return retval;
	}

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/
#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
static int resume_touch(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	struct synaptics_rmi4_f54_handle *f54;
	unsigned char value;
	unsigned char command;
	unsigned char timeout_count;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	f54 = rmi4_data->f54;

	mutex_lock(&f54->control_mutex);

	/*
	 * Restore Baseline compensation
	 */
	if (f54->query.touch_controller_family == 1) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_7->address,
				&f54->ctrl_value,
				sizeof(f54->ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to restore CBC\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	} else if ((f54->query.has_ctrl88 == 1) && (bdata->disable_bc_bist == 1)) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_88->address,
				&f54->ctrl_value,
				sizeof(f54->ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to restore CBC (write ctrl88)\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	if (f54->query.has_0d_acquisition_control) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_57->address,
				&f54->acquisition_ctrl_value,
				sizeof(f54->acquisition_ctrl_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to restore 0D CBC\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	/*
	 * Restore the SignalClarity
	 */
	if (f54->query.has_signal_clarity) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				f54->control.reg_41->address,
				&f54->ctrl41_value,
				sizeof(f54->ctrl41_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable signal clarity\n",
					__func__);
			mutex_unlock(&f54->control_mutex);
			return retval;
		}
	}

	mutex_unlock(&f54->control_mutex);

	/*
	 * Set ForceUpdate
	 */
	/* Set ForceUpdate (F54_AD_Cmd0, bit 2) to ‘1’ */
	command = (unsigned char)COMMAND_FORCE_UPDATE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);
		return -ETIMEDOUT;
	}

	/*
	 * Set ForceCal (F54_AD_Cmd0, bit 1)
	 */
	/* Write 0x02 to register F54_ANALOG_Cmd0 */
	command = (unsigned char)COMMAND_FORCE_CAL;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force cal command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force cal\n",
				__func__);
		return -ETIMEDOUT;
	}

	set_interrupt(rmi4_data, false);

	return 0;
}
#endif

#ifdef CONFIG_BBRY_DEBUG
static ssize_t synaptics_rmi4_f54_resume_touch_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;

	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	retval = resume_touch(rmi4_data);
	if (retval)
		return retval;

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static int get_report_data(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char report_index[2];
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

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
			return -ENOMEM;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

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
		return -EINVAL;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read report data\n",
				__func__);
		return -EINVAL;
	}

	return STATUS_IDLE;
}

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static int calculate_cap_min_max(struct synaptics_rmi4_data *rmi4_data,
		bool *passfail, uint16_t *bist_max_cap,
		uint16_t *bist_min_cap, uint16_t *testdata)
{
	unsigned i, cap[2];
	uint16_t capval;
	const struct synaptics_dsx_board_data *bdata;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	bdata = rmi4_data->hw_if->board_data;

	*passfail = false;
	*bist_max_cap = 0;
	*bist_min_cap = 0;

	if (f54->report_type == F54_FULL_RAW_CAP_MIN_MAX) {
		if (f54->report_size != FULL_RAW_CAP_MIN_MAX_DATA_SIZE) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Invalide report size\n",
				__func__);
			return -EINVAL;
		}
		*bist_max_cap = ((uint16_t)f54->report_data[1] << 8) + f54->report_data[0];
		*bist_min_cap = ((uint16_t)f54->report_data[3] << 8) + f54->report_data[2];
	} else if (f54->report_type == F54_FULL_RAW_CAP_RX_COUPLING_COMP
			|| f54->report_type == F54_RAW_16BIT_IMAGE
			|| f54->report_type == F54_AMP_FULL_RAW_CAP) {
		*bist_min_cap = 0xFFFF;
		for (i = 0; i < (f54->report_size / 2) ; i++) {
			cap[0] = f54->report_data[i * 2];
			cap[1] = f54->report_data[i * 2 + 1];
			capval = ((uint16_t)cap[1] << 8) + cap[0];
			testdata[i] = capval;
			if (capval > *bist_max_cap)
				*bist_max_cap = capval;
			if (capval < *bist_min_cap)
				*bist_min_cap = capval;
			dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: cap[%d] = %d\n",
				__func__, i, capval);

		}
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Invalid report type\n",
				__func__);
		return -EINVAL;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: min_cap %d, max_cap %d\n",
		__func__, *bist_min_cap, *bist_max_cap);

	if (0 != bdata->bist_min && 0 != bdata->bist_max) {
		if ((*bist_max_cap > bdata->bist_max) ||
			(*bist_min_cap < bdata->bist_min)) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: cap result (%d, %d) out of range (%d, %d)\n",
				__func__, *bist_min_cap, *bist_max_cap,
				bdata->bist_min,
				bdata->bist_max);
		} else {
			*passfail = true;
		}
	} else {
		*passfail = true;
	}

	return 0;
}

ssize_t synaptics_rmi4_f54_do_BIST(struct synaptics_rmi4_data *rmi4_data,
	bool *passfail, uint16_t *bist_max_cap, uint16_t *bist_min_cap, uint16_t *testdata)
{
	int retval;

	struct synaptics_rmi4_f54_handle *f54;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_fn *fhandler;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	f54 = rmi4_data->f54;

	mutex_lock(&f54->status_mutex);

	retval = check_device_busy(rmi4_data);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	retval = do_preparation(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to do preparation\n",
				__func__);
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	/*set BIST report type*/
	rmi = &(rmi4_data->rmi4_mod_info);

	if (bdata->report_type > 0)
		f54->report_type = bdata->report_type;
	else
		f54->report_type = F54_FULL_RAW_CAP_RX_COUPLING_COMP;

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F11) {
				f54->report_type = F54_FULL_RAW_CAP_MIN_MAX;
				break;
			}
		}
	}

	retval = set_report_type(rmi4_data,
			(enum f54_report_types)f54->report_type);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	retval = get_report(rmi4_data);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Report data size = 0\n",
				__func__);
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	retval = get_report_data(rmi4_data);
	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	retval = calculate_cap_min_max(
			rmi4_data, passfail,
			bist_max_cap, bist_min_cap, testdata);

	if (retval < 0) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	retval = resume_touch(rmi4_data);
	if (retval) {
		mutex_unlock(&f54->status_mutex);
		return retval;
	}

	mutex_unlock(&f54->status_mutex);

	return 0;
}
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f54_num_of_mapped_rx_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->rx_assigned);
}

static ssize_t synaptics_rmi4_f54_num_of_mapped_tx_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->tx_assigned);
}

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
static ssize_t synaptics_rmi4_f54_full_raw_rt78_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned long setting;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;
	struct f54_control control;
	unsigned char command;
	unsigned char timeout_count = 0;
	unsigned char temp_value;
	unsigned char report_index[2];
	const char* cmd = "78";
	
	unsigned char original_data_ctrl99[3] = {0x00, 0x00, 0x00};
	
	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;
	control = f54->control;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;
	
	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;
		
	if (setting != 1)
		return -EINVAL;
	
	if (g_f54_image_output)
		kfree(g_f54_image_output);
	
	g_f54_image_output = kzalloc(2 * rx_num, GFP_KERNEL);   // 20171016
	if (!g_f54_image_output) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to allocate mem for g_f54_image_output\n", 
			__func__);
		return -ENOMEM;
	}
		
	g_flag_readrt_err = false;
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			control.reg_99->address,
			original_data_ctrl99,
			sizeof(original_data_ctrl99));
	printk("%s %d retval=%x", __func__, __LINE__, retval);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read f54 ctrl99 \n",
			__func__);
		goto exit;
	}
	pr_err("%s: original ctrl99 data: %x %x %x \n", __func__, original_data_ctrl99[0], original_data_ctrl99[1], original_data_ctrl99[2]);
	
	control.reg_99->integration_duration_lsb = 0x32;
	control.reg_99->integration_duration_msb = 0x00;
	control.reg_99->reset_duration = 0x17;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		g_flag_readrt_err = true;
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			g_flag_readrt_err = true;
			return retval;
		}
		if (temp_value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);
		g_flag_readrt_err = true;		
		return -ETIMEDOUT;
	}

	retval = synaptics_rmi4_f54_report_type_store(kobj, attr, cmd, 2);
	if (retval < 0) {
		g_flag_readrt_err = true;
		goto exit;
	}
	
	if (!is_report_type_valid(f54, (enum f54_report_types)78)) {
		dev_err(rmi4_data->pdev->dev.parent, 
			"%s: invalid reprot type - 78\n", 
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}
	
	command = COMMAND_GET_REPORT;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write get_report command\n",
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read f54 command reg\n",
				__func__);
			g_flag_readrt_err = true;
			goto exit;
		}
		
		if (temp_value == 0x00)
			break;
		
		msleep(20);
		timeout_count++;
	} while (timeout_count < 100);
	
	if (timeout_count == 100) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: timeout waiting for command completion\n", 
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0){
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: report data size = 0\n", 
			__func__);
		g_flag_readrt_err = true;
		goto exit;
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
			g_flag_readrt_err = true;
			goto exit;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to write report data index\n",
				__func__);
		retval = -EINVAL;
		g_flag_readrt_err = true;
		goto exit;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read report data\n",
				__func__);
		retval = -EINVAL;
		g_flag_readrt_err = true;
		goto exit;
	}

	retval = STATUS_IDLE;

	memcpy(g_f54_image_output, f54->report_data, f54->report_size);

	// recover to the original data	
	control.reg_99->integration_duration_lsb = original_data_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_ctrl99[1];
	control.reg_99->reset_duration = original_data_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to recover the ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}
	
	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command in recover\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		printk("%s %d retval=%x", __func__, __LINE__, retval);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register in recover\n",
					__func__);
			return retval;
		}
		if (temp_value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update in recover\n",
				__func__);
		return -ETIMEDOUT;
	}

exit:
	return count;
}

static ssize_t synaptics_rmi4_f54_full_raw_rt78_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int i;
	int cnt;
	int count = 0;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;
	unsigned short *rt_data_16;
	
	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;
	
	if (!g_f54_image_output)
		return snprintf(buf, PAGE_SIZE, "invalid g_f54_image_output\n");
		
	if (g_flag_readrt_err) {
		kfree(g_f54_image_output);
		g_f54_image_output = NULL;
		
		return snprintf(buf, PAGE_SIZE, "fail to read report image 78\n");
	}
	
	cnt = snprintf(buf, PAGE_SIZE - count, "tx = 0 \nrx = %d \n", rx_num);
	buf += cnt;
	count += cnt;
	
	rt_data_16 = (unsigned short *)g_f54_image_output;
	for(i = 0; i < rx_num; i++) {
		cnt = snprintf(buf, PAGE_SIZE - count, "%-5d \n", *rt_data_16);
				
		rt_data_16++;
		buf += cnt;
		count += cnt;
	}
	
	cnt = snprintf(buf, PAGE_SIZE - count, "\n\n");
	buf += cnt;
	count += cnt;
	
	kfree(g_f54_image_output);
	g_f54_image_output = NULL;

	cnt = snprintf(buf, PAGE_SIZE - count, "PASS\n");
	buf += cnt;
	count += cnt;

	return count;
}

ssize_t synaptics_rmi4_f54_do_full_raw_rt78(struct synaptics_rmi4_data *rmi4_data,
	unsigned char *testdata)
{
	int retval = 0;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_f54_handle *f54;
	struct f54_control control;
	unsigned char command;
	unsigned char timeout_count = 0;
	unsigned char temp_value;
	unsigned char report_index[2];
	unsigned char original_data_ctrl99[3] = {0x00, 0x00, 0x00};

	retval = check_device_busy(rmi4_data);
	if (retval < 0) {
		//mutex_unlock(&f54->status_mutex);
		return retval;
	}

	f54 = rmi4_data->f54;
	control = f54->control;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			control.reg_99->address,
			original_data_ctrl99,
			sizeof(original_data_ctrl99));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read f54 ctrl99 \n",
			__func__);
		return retval;
	}
	pr_err("%s: original ctrl99 data: %x %x %x \n", __func__, original_data_ctrl99[0], original_data_ctrl99[1], original_data_ctrl99[2]);

	control.reg_99->integration_duration_lsb = 0x32;
	control.reg_99->integration_duration_msb = 0x00;
	control.reg_99->reset_duration = 0x17;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change ctrl_99\n",
				__func__);
		retval = -EIO;
		return retval;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}
		if((temp_value & 0x01)== 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);	
		return -ETIMEDOUT;
	}

	retval = set_report_type(rmi4_data, (enum f54_report_types)78);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to set_report_type\n",
			__func__);
		return retval;
	}

	set_interrupt(rmi4_data, true);

	command = COMMAND_GET_REPORT;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write get_report command\n",
			__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read f54 command reg\n",
				__func__);
			return retval;
		}
		
		if((temp_value & 0x01)== 0x00)
			break;
		
		msleep(20);
		timeout_count++;
	} while (timeout_count < 100);

	if (timeout_count == 100) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: timeout waiting for command completion\n", 
			__func__);
		return -ETIMEDOUT;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0){
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: report data size = 0\n", 
			__func__);
		return -ENOMEM;
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
			return -ENOMEM;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write report data index\n",
			__func__);
		return retval;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read report data\n",
			__func__);
		return retval;
	}

	retval = STATUS_IDLE;

	memcpy(testdata, f54->report_data, f54->report_size);

#if 0
	// recover to the original data 
	control.reg_99->integration_duration_lsb = original_data_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_ctrl99[1];
	control.reg_99->reset_duration = original_data_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to recover the ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command in recover\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		printk("%s %d retval=%x", __func__, __LINE__, retval);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register in recover\n",
					__func__);
			return retval;
		}
		if (temp_value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update in recover\n",
				__func__);
		return -ETIMEDOUT;
	}
#endif

	retval = resume_touch(rmi4_data);
	if (retval) {
		//mutex_unlock(&f54->status_mutex);
		return retval;
	}
	return 0;
}
static ssize_t synaptics_rmi4_f54_keyboard_open_rt78_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = 0;
	unsigned long setting;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;
	struct f54_control control;
	unsigned char command;
	unsigned char timeout_count = 0;
	unsigned char temp_value;
	unsigned char report_index[2];
	const char* cmd = "78";

	unsigned char original_data_ctrl99[3] = {0x00, 0x00, 0x00};

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;
	control = f54->control;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	if (g_f54_image_output)
		kfree(g_f54_image_output);

	g_f54_image_output = kzalloc(2 * rx_num, GFP_KERNEL);
	if (!g_f54_image_output) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to allocate mem for g_f54_image_output\n",
			__func__);
		return -ENOMEM;
	}

	g_flag_readrt_err = false;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			control.reg_99->address,
			original_data_ctrl99,
			sizeof(original_data_ctrl99));
	printk("%s %d retval=%x", __func__, __LINE__, retval);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read f54 ctrl99 \n",
			__func__);
		goto exit;
	}
	pr_err("%s: original ctrl99 data: %x %x %x \n", __func__, original_data_ctrl99[0], original_data_ctrl99[1], original_data_ctrl99[2]);

	control.reg_99->integration_duration_lsb = original_data_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_ctrl99[1];
	control.reg_99->reset_duration = 0x17;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		g_flag_readrt_err = true;
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			g_flag_readrt_err = true;
			return retval;
		}
		if (temp_value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);
		g_flag_readrt_err = true;
		return -ETIMEDOUT;
	}

	retval = synaptics_rmi4_f54_report_type_store(kobj, attr, cmd, 2);
	if (retval < 0) {
		g_flag_readrt_err = true;
		goto exit;
	}

	if (!is_report_type_valid(f54, (enum f54_report_types)78)) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: invalid reprot type - 78\n",
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}

	command = COMMAND_GET_REPORT;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write get_report command\n",
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read f54 command reg\n",
				__func__);
			g_flag_readrt_err = true;
			goto exit;
		}

		if (temp_value == 0x00)
			break;

		msleep(20);
		timeout_count++;
	} while (timeout_count < 100);

	if (timeout_count == 100) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: timeout waiting for command completion\n",
			__func__);
		g_flag_readrt_err = true;
		goto exit;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0){
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: report data size = 0\n",
			__func__);
		g_flag_readrt_err = true;
		goto exit;
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
			g_flag_readrt_err = true;
			goto exit;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to write report data index\n",
				__func__);
		retval = -EINVAL;
		g_flag_readrt_err = true;
		goto exit;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read report data\n",
				__func__);
		retval = -EINVAL;
		g_flag_readrt_err = true;
		goto exit;
	}

	retval = STATUS_IDLE;

	memcpy(g_f54_image_output, f54->report_data, f54->report_size);

	// recover to the original data
	control.reg_99->integration_duration_lsb = original_data_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_ctrl99[1];
	control.reg_99->reset_duration = original_data_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to recover the ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command in recover\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		printk("%s %d retval=%x", __func__, __LINE__, retval);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register in recover\n",
					__func__);
			return retval;
		}
		if (temp_value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update in recover\n",
				__func__);
		return -ETIMEDOUT;
	}

exit:
	return count;
}

static ssize_t synaptics_rmi4_f54_keyboard_open_rt78_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	unsigned int i;
	int cnt;
	int count = 0;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;
	unsigned short *rt_data_16;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;

	if (!g_f54_image_output)
		return snprintf(buf, PAGE_SIZE, "invalid g_f54_image_output\n");

	if (g_flag_readrt_err) {
		kfree(g_f54_image_output);
		g_f54_image_output = NULL;

		return snprintf(buf, PAGE_SIZE, "fail to read report image 78\n");
	}

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = 0 \nrx = %d \n", rx_num);
	buf += cnt;
	count += cnt;

	rt_data_16 = (unsigned short *)g_f54_image_output;
	for(i = 0; i < rx_num; i++) {
		cnt = snprintf(buf, PAGE_SIZE - count, "%-5d \n", *rt_data_16);

		rt_data_16++;
		buf += cnt;
		count += cnt;
	}

	cnt = snprintf(buf, PAGE_SIZE - count, "\n\n");
	buf += cnt;
	count += cnt;

	kfree(g_f54_image_output);
	g_f54_image_output = NULL;

	return count;
}

ssize_t synaptics_rmi4_f54_do_open_rt78(struct synaptics_rmi4_data *rmi4_data,
	unsigned char *testdata)
{
	int retval = 0;
	int tx_num;
	int rx_num;
	struct synaptics_rmi4_f54_handle *f54;
	struct f54_control control;
	unsigned char command;
	unsigned char timeout_count = 0;
	unsigned char temp_value;
	unsigned char report_index[2];
	unsigned char original_data_ctrl99[3] = {0x00, 0x00, 0x00};

	retval = check_device_busy(rmi4_data);
	if (retval < 0) {
		//mutex_unlock(&f54->status_mutex);
		return retval;
	}

	f54 = rmi4_data->f54;
	control = f54->control;
	tx_num = f54->tx_assigned;
	rx_num = f54->rx_assigned;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			control.reg_99->address,
			original_data_ctrl99,
			sizeof(original_data_ctrl99));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read f54 ctrl99 \n",
			__func__);
		return retval;
	}
	pr_err("%s: original ctrl99 data: %x %x %x \n", __func__, original_data_ctrl99[0], original_data_ctrl99[1], original_data_ctrl99[2]);

	control.reg_99->integration_duration_lsb = 0x32;
	control.reg_99->integration_duration_msb = 0x00;
	control.reg_99->reset_duration = 0x17;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change ctrl_99\n",
				__func__);
		retval = -EIO;
		return retval;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}
		if((temp_value & 0x01)== 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update\n",
				__func__);	
		return -ETIMEDOUT;
	}

	retval = set_report_type(rmi4_data, (enum f54_report_types)78);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to set_report_type\n",
			__func__);
		return retval;
	}

	set_interrupt(rmi4_data, true);

	command = COMMAND_GET_REPORT;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write get_report command\n",
			__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: fail to read f54 command reg\n",
				__func__);
			return retval;
		}
		
		if((temp_value & 0x01)== 0x00)
			break;
		
		msleep(20);
		timeout_count++;
	} while (timeout_count < 100);

	if (timeout_count == 100) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: timeout waiting for command completion\n", 
			__func__);
		return -ETIMEDOUT;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0){
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: report data size = 0\n", 
			__func__);
		return -ENOMEM;
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
			return -ENOMEM;
		}
		f54->data_buffer_size = f54->report_size;
		mutex_unlock(&f54->data_mutex);
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to write report data index\n",
			__func__);
		return retval;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: fail to read report data\n",
			__func__);
		return retval;
	}

	retval = STATUS_IDLE;

	memcpy(testdata, f54->report_data, f54->report_size);

	// recover to the original data 
	control.reg_99->integration_duration_lsb = original_data_ctrl99[0];
	control.reg_99->integration_duration_msb = original_data_ctrl99[1];
	control.reg_99->reset_duration = original_data_ctrl99[2];
	retval = synaptics_rmi4_reg_write(rmi4_data,
			control.reg_99->address,
			control.reg_99->data,
			sizeof(control.reg_99->data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to recover the ctrl_99\n",
				__func__);
		retval = -EIO;
		goto exit;
	}

	command = (unsigned char)COMMAND_FORCE_UPDATE;
	retval = synaptics_rmi4_reg_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write force update command in recover\n",
				__func__);
		return retval;
	}

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->command_base_addr,
				&temp_value,
				sizeof(temp_value));
		printk("%s %d retval=%x", __func__, __LINE__, retval);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read command register in recover\n",
					__func__);
			return retval;
		}
		if((temp_value & 0x01)== 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < FORCE_TIMEOUT_100MS);

	if (timeout_count == FORCE_TIMEOUT_100MS) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for force update in recover\n",
				__func__);
		return -ETIMEDOUT;
	}
	set_interrupt(rmi4_data, false);

exit:
	return retval;
}
#endif

simple_show_func_unsigned(query, num_of_rx_electrodes)
simple_show_func_unsigned(query, num_of_tx_electrodes)
simple_show_func_unsigned(query, has_image16)
simple_show_func_unsigned(query, has_image8)
simple_show_func_unsigned(query, has_baseline)
simple_show_func_unsigned(query, clock_rate)
simple_show_func_unsigned(query, touch_controller_family)
simple_show_func_unsigned(query, has_pixel_touch_threshold_adjustment)
simple_show_func_unsigned(query, has_sensor_assignment)
simple_show_func_unsigned(query, has_interference_metric)
simple_show_func_unsigned(query, has_sense_frequency_control)
simple_show_func_unsigned(query, has_firmware_noise_mitigation)
simple_show_func_unsigned(query, has_two_byte_report_rate)
simple_show_func_unsigned(query, has_one_byte_report_rate)
simple_show_func_unsigned(query, has_relaxation_control)
simple_show_func_unsigned(query, curve_compensation_mode)
simple_show_func_unsigned(query, has_iir_filter)
simple_show_func_unsigned(query, has_cmn_removal)
simple_show_func_unsigned(query, has_cmn_maximum)
simple_show_func_unsigned(query, has_touch_hysteresis)
simple_show_func_unsigned(query, has_edge_compensation)
simple_show_func_unsigned(query, has_per_frequency_noise_control)
simple_show_func_unsigned(query, has_signal_clarity)
simple_show_func_unsigned(query, number_of_sensing_frequencies)

#ifdef CONFIG_BBRY_DEBUG
show_store_func_unsigned(control, reg_0, no_relax)
show_store_func_unsigned(control, reg_0, no_scan)
show_store_func_unsigned(control, reg_1, bursts_per_cluster)
show_store_func_unsigned(control, reg_2, saturation_cap)
show_store_func_unsigned(control, reg_3, pixel_touch_threshold)
show_store_func_unsigned(control, reg_4__6, rx_feedback_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_feedback_cap)
show_store_func_unsigned(control, reg_4__6, low_ref_polarity)
show_store_func_unsigned(control, reg_4__6, high_ref_cap)
show_store_func_unsigned(control, reg_4__6, high_ref_feedback_cap)
show_store_func_unsigned(control, reg_4__6, high_ref_polarity)
show_store_func_unsigned(control, reg_7, cbc_cap)
show_store_func_unsigned(control, reg_7, cbc_polarity)
show_store_func_unsigned(control, reg_7, cbc_tx_carrier_selection)
show_store_func_unsigned(control, reg_8__9, integration_duration)
show_store_func_unsigned(control, reg_8__9, reset_duration)
show_store_func_unsigned(control, reg_10, noise_sensing_bursts_per_image)
show_store_func_unsigned(control, reg_12__13, slow_relaxation_rate)
show_store_func_unsigned(control, reg_12__13, fast_relaxation_rate)
show_store_func_unsigned(control, reg_14, rxs_on_xaxis)
show_store_func_unsigned(control, reg_14, curve_comp_on_txs)
show_store_func_unsigned(control, reg_20, disable_noise_mitigation)
show_store_func_unsigned(control, reg_21, freq_shift_noise_threshold)
show_store_func_unsigned(control, reg_22__26, medium_noise_threshold)
show_store_func_unsigned(control, reg_22__26, high_noise_threshold)
show_store_func_unsigned(control, reg_22__26, noise_density)
show_store_func_unsigned(control, reg_22__26, frame_count)
show_store_func_unsigned(control, reg_27, iir_filter_coef)
show_store_func_unsigned(control, reg_28, quiet_threshold)
show_store_func_unsigned(control, reg_29, cmn_filter_disable)
show_store_func_unsigned(control, reg_30, cmn_filter_max)
show_store_func_unsigned(control, reg_31, touch_hysteresis)
show_store_func_unsigned(control, reg_32__35, rx_low_edge_comp)
show_store_func_unsigned(control, reg_32__35, rx_high_edge_comp)
show_store_func_unsigned(control, reg_32__35, tx_low_edge_comp)
show_store_func_unsigned(control, reg_32__35, tx_high_edge_comp)
show_store_func_unsigned(control, reg_41, no_signal_clarity)
show_store_func_unsigned(control, reg_57, cbc_cap_0d)
show_store_func_unsigned(control, reg_57, cbc_polarity_0d)
show_store_func_unsigned(control, reg_57, cbc_tx_carrier_selection_0d)
#else
show_func(control, reg_0, no_relax, "%u\n")
show_func(control, reg_0, no_scan, "%u\n")
show_func(control, reg_1, bursts_per_cluster, "%u\n")
show_func(control, reg_2, saturation_cap, "%u\n")
show_func(control, reg_3, pixel_touch_threshold, "%u\n")
show_func(control, reg_4__6, rx_feedback_cap, "%u\n")
show_func(control, reg_4__6, low_ref_cap, "%u\n")
show_func(control, reg_4__6, low_ref_feedback_cap, "%u\n")
show_func(control, reg_4__6, low_ref_polarity, "%u\n")
show_func(control, reg_4__6, high_ref_cap, "%u\n")
show_func(control, reg_4__6, high_ref_feedback_cap, "%u\n")
show_func(control, reg_4__6, high_ref_polarity, "%u\n")
show_func(control, reg_7, cbc_cap, "%u\n")
show_func(control, reg_7, cbc_polarity, "%u\n")
show_func(control, reg_7, cbc_tx_carrier_selection, "%u\n")
show_func(control, reg_8__9, integration_duration, "%u\n")
show_func(control, reg_8__9, reset_duration, "%u\n")
show_func(control, reg_10, noise_sensing_bursts_per_image, "%u\n")
show_func(control, reg_12__13, slow_relaxation_rate, "%u\n")
show_func(control, reg_12__13, fast_relaxation_rate, "%u\n")
show_func(control, reg_14, rxs_on_xaxis, "%u\n")
show_func(control, reg_14, curve_comp_on_txs, "%u\n")
show_func(control, reg_20, disable_noise_mitigation, "%u\n")
show_func(control, reg_21, freq_shift_noise_threshold, "%u\n")
show_func(control, reg_22__26, medium_noise_threshold, "%u\n")
show_func(control, reg_22__26, high_noise_threshold, "%u\n")
show_func(control, reg_22__26, noise_density, "%u\n")
show_func(control, reg_22__26, frame_count, "%u\n")
show_func(control, reg_27, iir_filter_coef, "%u\n")
show_func(control, reg_28, quiet_threshold, "%u\n")
show_func(control, reg_29, cmn_filter_disable, "%u\n")
show_func(control, reg_30, cmn_filter_max, "%u\n")
show_func(control, reg_31, touch_hysteresis, "%u\n")
show_func(control, reg_32__35, rx_low_edge_comp, "%u\n")
show_func(control, reg_32__35, rx_high_edge_comp, "%u\n")
show_func(control, reg_32__35, tx_low_edge_comp, "%u\n")
show_func(control, reg_32__35, tx_high_edge_comp, "%u\n")
show_func(control, reg_41, no_signal_clarity, "%u\n")
show_func(control, reg_57, cbc_cap_0d, "%u\n")
show_func(control, reg_57, cbc_polarity_0d, "%u\n")
show_func(control, reg_57, cbc_tx_carrier_selection_0d, "%u\n")
#endif /*CONFIG_BBRY_DEBUG*/
show_replicated_func_unsigned(control, reg_15, sensor_rx_assignment)
show_replicated_func_unsigned(control, reg_16, sensor_tx_assignment)
show_replicated_func_unsigned(control, reg_17, disable)
show_replicated_func_unsigned(control, reg_17, filter_bandwidth)
show_replicated_func_unsigned(control, reg_19, stretch_duration)
show_replicated_func_unsigned(control, reg_38, noise_control_1)
show_replicated_func_unsigned(control, reg_39, noise_control_2)
show_replicated_func_unsigned(control, reg_40, noise_control_3)

#ifdef CONFIG_BBRY_DEBUG
show_store_replicated_func_unsigned(control, reg_36, axis1_comp)
show_store_replicated_func_unsigned(control, reg_37, axis2_comp)
#else
show_replicated_func_unsigned(control, reg_36, axis1_comp)
show_replicated_func_unsigned(control, reg_37, axis2_comp)
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t synaptics_rmi4_f54_burst_count_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	int size = 0;
	unsigned char ii;
	unsigned char *temp;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_f54_handle *f54;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));
	f54 = rmi4_data->f54;

	mutex_lock(&f54->control_mutex);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->control.reg_17->address,
			(unsigned char *)f54->control.reg_17->data,
			f54->control.reg_17->length);
	if (retval < 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Failed to read control reg_17\n",
				__func__);
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->control.reg_18->address,
			(unsigned char *)f54->control.reg_18->data,
			f54->control.reg_18->length);
	if (retval < 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Failed to read control reg_18\n",
				__func__);
	}

	mutex_unlock(&f54->control_mutex);

	temp = buf;

	for (ii = 0; ii < f54->control.reg_17->length; ii++) {
		retval = snprintf(temp, PAGE_SIZE - size, "%u ", (1 << 8) *
			f54->control.reg_17->data[ii].burst_count_b8__10 +
			f54->control.reg_18->data[ii].burst_count_b0__7);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Faild to write output\n",
					__func__);
			return retval;
		}
		size += retval;
		temp += retval;
	}

	retval = snprintf(temp, PAGE_SIZE - size, "\n");
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Faild to write null terminator\n",
				__func__);
		return retval;
	}

	return size + retval;
}

static ssize_t synaptics_rmi4_f54_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct synaptics_rmi4_f54_handle *f54;
	struct synaptics_rmi4_data *rmi4_data;

	rmi4_data = dev_get_drvdata(
			container_of(kobj->parent, struct device, kobj));

	f54 = (struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	mutex_lock(&f54->data_mutex);

	if (pos > f54->report_size) {
		mutex_unlock(&f54->data_mutex);
		return -ESPIPE;
	}
	if (pos == f54->report_size) {
		mutex_unlock(&f54->data_mutex);
		return 0; /* EOF */
	}
	if (pos + count > f54->report_size)
		count = f54->report_size - pos;
	if (f54->report_data) {
		memcpy(buf, f54->report_data, count);
		mutex_unlock(&f54->data_mutex);
		return count;
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Report type %d data not available\n",
				__func__, f54->report_type);
		mutex_unlock(&f54->data_mutex);
		return -EINVAL;
	}
}

static int synaptics_rmi4_f54_set_sysfs(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int reg_num;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	f54->attr_dir = kobject_create_and_add("f54",
			&rmi4_data->input_dev->dev.kobj);
	if (!f54->attr_dir) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs directory\n",
				__func__);
		goto exit_1;
	}

	retval = sysfs_create_bin_file(f54->attr_dir, &dev_report_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_2;
	}

	retval = sysfs_create_group(f54->attr_dir, &attr_group);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs attributes\n",
				__func__);
		goto exit_3;
	}

	for (reg_num = 0; reg_num < ARRAY_SIZE(attrs_ctrl_regs); reg_num++) {
		if (attrs_ctrl_regs_exist[reg_num]) {
			retval = sysfs_create_group(f54->attr_dir,
					&attrs_ctrl_regs[reg_num]);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to create sysfs attributes\n",
						__func__);
				goto exit_4;
			}
		}
	}
	return 0;

exit_4:
	sysfs_remove_group(f54->attr_dir, &attr_group);

	for (reg_num--; reg_num >= 0; reg_num--)
		sysfs_remove_group(f54->attr_dir, &attrs_ctrl_regs[reg_num]);

exit_3:
	sysfs_remove_bin_file(f54->attr_dir, &dev_report_data);

exit_2:
	kobject_put(f54->attr_dir);

exit_1:
	return -ENODEV;
}

static int synaptics_rmi4_f54_set_ctrl(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char length;
	unsigned char reg_num = 0;
	unsigned char num_of_sensing_freqs;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	unsigned short reg_addr = f54->control_base_addr;
	struct f54_control *control = &f54->control;

	num_of_sensing_freqs = f54->query.number_of_sensing_frequencies;

	/* control 0 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_0 = kzalloc(sizeof(*(control->reg_0)),
			GFP_KERNEL);
	if (!control->reg_0)
		goto exit_no_mem;
	control->reg_0->address = reg_addr;
	reg_addr += sizeof(control->reg_0->data);
	reg_num++;

	/* control 1 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_1 = kzalloc(sizeof(*(control->reg_1)),
				GFP_KERNEL);
		if (!control->reg_1)
			goto exit_no_mem;
		control->reg_1->address = reg_addr;
		reg_addr += sizeof(control->reg_1->data);
	}
	reg_num++;

	/* control 2 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_2 = kzalloc(sizeof(*(control->reg_2)),
			GFP_KERNEL);
	if (!control->reg_2)
		goto exit_no_mem;
	control->reg_2->address = reg_addr;
	reg_addr += sizeof(control->reg_2->data);
	reg_num++;

	/* control 3 */
	if (f54->query.has_pixel_touch_threshold_adjustment == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_3 = kzalloc(sizeof(*(control->reg_3)),
				GFP_KERNEL);
		if (!control->reg_3)
			goto exit_no_mem;
		control->reg_3->address = reg_addr;
		reg_addr += sizeof(control->reg_3->data);
	}
	reg_num++;

	/* controls 4 5 6 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_4__6 = kzalloc(sizeof(*(control->reg_4__6)),
				GFP_KERNEL);
		if (!control->reg_4__6)
			goto exit_no_mem;
		control->reg_4__6->address = reg_addr;
		reg_addr += sizeof(control->reg_4__6->data);
	}
	reg_num++;

	/* control 7 */
	if (f54->query.touch_controller_family == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_7 = kzalloc(sizeof(*(control->reg_7)),
				GFP_KERNEL);
		if (!control->reg_7)
			goto exit_no_mem;
		control->reg_7->address = reg_addr;
		reg_addr += sizeof(control->reg_7->data);
	}
	reg_num++;

	/* controls 8 9 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1)) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_8__9 = kzalloc(sizeof(*(control->reg_8__9)),
				GFP_KERNEL);
		if (!control->reg_8__9)
			goto exit_no_mem;
		control->reg_8__9->address = reg_addr;
		reg_addr += sizeof(control->reg_8__9->data);
	}
	reg_num++;

	/* control 10 */
	if (f54->query.has_interference_metric == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_10 = kzalloc(sizeof(*(control->reg_10)),
				GFP_KERNEL);
		if (!control->reg_10)
			goto exit_no_mem;
		control->reg_10->address = reg_addr;
		reg_addr += sizeof(control->reg_10->data);
	}
	reg_num++;

	/* control 11 */
	if (f54->query.has_ctrl11 == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_11 = kzalloc(sizeof(*(control->reg_11)),
				GFP_KERNEL);
		if (!control->reg_11)
			goto exit_no_mem;
		control->reg_11->address = reg_addr;
		reg_addr += sizeof(control->reg_11->data);
	}
	reg_num++;

	/* controls 12 13 */
	if (f54->query.has_relaxation_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_12__13 = kzalloc(sizeof(*(control->reg_12__13)),
				GFP_KERNEL);
		if (!control->reg_12__13)
			goto exit_no_mem;
		control->reg_12__13->address = reg_addr;
		reg_addr += sizeof(control->reg_12__13->data);
	}
	reg_num++;

	/* controls 14 15 16 */
	if (f54->query.has_sensor_assignment == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_14 = kzalloc(sizeof(*(control->reg_14)),
				GFP_KERNEL);
		if (!control->reg_14)
			goto exit_no_mem;
		control->reg_14->address = reg_addr;
		reg_addr += sizeof(control->reg_14->data);

		control->reg_15 = kzalloc(sizeof(*(control->reg_15)),
				GFP_KERNEL);
		if (!control->reg_15)
			goto exit_no_mem;
		control->reg_15->length = f54->query.num_of_rx_electrodes;
		control->reg_15->data = kzalloc(control->reg_15->length *
				sizeof(*(control->reg_15->data)), GFP_KERNEL);
		if (!control->reg_15->data)
			goto exit_no_mem;
		control->reg_15->address = reg_addr;
		reg_addr += control->reg_15->length;

		control->reg_16 = kzalloc(sizeof(*(control->reg_16)),
				GFP_KERNEL);
		if (!control->reg_16)
			goto exit_no_mem;
		control->reg_16->length = f54->query.num_of_tx_electrodes;
		control->reg_16->data = kzalloc(control->reg_16->length *
				sizeof(*(control->reg_16->data)), GFP_KERNEL);
		if (!control->reg_16->data)
			goto exit_no_mem;
		control->reg_16->address = reg_addr;
		reg_addr += control->reg_16->length;
	}
	reg_num++;

	/* controls 17 18 19 */
	if (f54->query.has_sense_frequency_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		length = num_of_sensing_freqs;

		control->reg_17 = kzalloc(sizeof(*(control->reg_17)),
				GFP_KERNEL);
		if (!control->reg_17)
			goto exit_no_mem;
		control->reg_17->length = length;
		control->reg_17->data = kzalloc(length *
				sizeof(*(control->reg_17->data)), GFP_KERNEL);
		if (!control->reg_17->data)
			goto exit_no_mem;
		control->reg_17->address = reg_addr;
		reg_addr += length;

		control->reg_18 = kzalloc(sizeof(*(control->reg_18)),
				GFP_KERNEL);
		if (!control->reg_18)
			goto exit_no_mem;
		control->reg_18->length = length;
		control->reg_18->data = kzalloc(length *
				sizeof(*(control->reg_18->data)), GFP_KERNEL);
		if (!control->reg_18->data)
			goto exit_no_mem;
		control->reg_18->address = reg_addr;
		reg_addr += length;

		control->reg_19 = kzalloc(sizeof(*(control->reg_19)),
				GFP_KERNEL);
		if (!control->reg_19)
			goto exit_no_mem;
		control->reg_19->length = length;
		control->reg_19->data = kzalloc(length *
				sizeof(*(control->reg_19->data)), GFP_KERNEL);
		if (!control->reg_19->data)
			goto exit_no_mem;
		control->reg_19->address = reg_addr;
		reg_addr += length;
	}
	reg_num++;

	/* control 20 */
	attrs_ctrl_regs_exist[reg_num] = true;
	control->reg_20 = kzalloc(sizeof(*(control->reg_20)),
			GFP_KERNEL);
	if (!control->reg_20)
		goto exit_no_mem;
	control->reg_20->address = reg_addr;
	reg_addr += sizeof(control->reg_20->data);
	reg_num++;

	/* control 21 */
	if (f54->query.has_sense_frequency_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_21 = kzalloc(sizeof(*(control->reg_21)),
				GFP_KERNEL);
		if (!control->reg_21)
			goto exit_no_mem;
		control->reg_21->address = reg_addr;
		reg_addr += sizeof(control->reg_21->data);
	}
	reg_num++;

	/* controls 22 23 24 25 26 */
	if (f54->query.has_firmware_noise_mitigation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_22__26 = kzalloc(sizeof(*(control->reg_22__26)),
				GFP_KERNEL);
		if (!control->reg_22__26)
			goto exit_no_mem;
		control->reg_22__26->address = reg_addr;
		reg_addr += sizeof(control->reg_22__26->data);
	}
	reg_num++;

	/* control 27 */
	if (f54->query.has_iir_filter == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_27 = kzalloc(sizeof(*(control->reg_27)),
				GFP_KERNEL);
		if (!control->reg_27)
			goto exit_no_mem;
		control->reg_27->address = reg_addr;
		reg_addr += sizeof(control->reg_27->data);
	}
	reg_num++;

	/* control 28 */
	if (f54->query.has_firmware_noise_mitigation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_28 = kzalloc(sizeof(*(control->reg_28)),
				GFP_KERNEL);
		if (!control->reg_28)
			goto exit_no_mem;
		control->reg_28->address = reg_addr;
		reg_addr += sizeof(control->reg_28->data);
	}
	reg_num++;

	/* control 29 */
	if (f54->query.has_cmn_removal == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_29 = kzalloc(sizeof(*(control->reg_29)),
				GFP_KERNEL);
		if (!control->reg_29)
			goto exit_no_mem;
		control->reg_29->address = reg_addr;
		reg_addr += sizeof(control->reg_29->data);
	}
	reg_num++;

	/* control 30 */
	if (f54->query.has_cmn_maximum == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_30 = kzalloc(sizeof(*(control->reg_30)),
				GFP_KERNEL);
		if (!control->reg_30)
			goto exit_no_mem;
		control->reg_30->address = reg_addr;
		reg_addr += sizeof(control->reg_30->data);
	}
	reg_num++;

	/* control 31 */
	if (f54->query.has_touch_hysteresis == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_31 = kzalloc(sizeof(*(control->reg_31)),
				GFP_KERNEL);
		if (!control->reg_31)
			goto exit_no_mem;
		control->reg_31->address = reg_addr;
		reg_addr += sizeof(control->reg_31->data);
	}
	reg_num++;

	/* controls 32 33 34 35 */
	if (f54->query.has_edge_compensation == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_32__35 = kzalloc(sizeof(*(control->reg_32__35)),
				GFP_KERNEL);
		if (!control->reg_32__35)
			goto exit_no_mem;
		control->reg_32__35->address = reg_addr;
		reg_addr += sizeof(control->reg_32__35->data);
	}
	reg_num++;

	/* control 36 */
	if ((f54->query.curve_compensation_mode == 1) ||
			(f54->query.curve_compensation_mode == 2)) {
		attrs_ctrl_regs_exist[reg_num] = true;

		if (f54->query.curve_compensation_mode == 1) {
			length = max(f54->query.num_of_rx_electrodes,
					f54->query.num_of_tx_electrodes);
		} else if (f54->query.curve_compensation_mode == 2) {
			length = f54->query.num_of_rx_electrodes;
		}

		control->reg_36 = kzalloc(sizeof(*(control->reg_36)),
				GFP_KERNEL);
		if (!control->reg_36)
			goto exit_no_mem;
		control->reg_36->length = length;
		control->reg_36->data = kzalloc(length *
				sizeof(*(control->reg_36->data)), GFP_KERNEL);
		if (!control->reg_36->data)
			goto exit_no_mem;
		control->reg_36->address = reg_addr;
		reg_addr += length;
	}
	reg_num++;

	/* control 37 */
	if (f54->query.curve_compensation_mode == 2) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_37 = kzalloc(sizeof(*(control->reg_37)),
				GFP_KERNEL);
		if (!control->reg_37)
			goto exit_no_mem;
		control->reg_37->length = f54->query.num_of_tx_electrodes;
		control->reg_37->data = kzalloc(control->reg_37->length *
				sizeof(*(control->reg_37->data)), GFP_KERNEL);
		if (!control->reg_37->data)
			goto exit_no_mem;

		control->reg_37->address = reg_addr;
		reg_addr += control->reg_37->length;
	}
	reg_num++;

	/* controls 38 39 40 */
	if (f54->query.has_per_frequency_noise_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;

		control->reg_38 = kzalloc(sizeof(*(control->reg_38)),
				GFP_KERNEL);
		if (!control->reg_38)
			goto exit_no_mem;
		control->reg_38->length = num_of_sensing_freqs;
		control->reg_38->data = kzalloc(control->reg_38->length *
				sizeof(*(control->reg_38->data)), GFP_KERNEL);
		if (!control->reg_38->data)
			goto exit_no_mem;
		control->reg_38->address = reg_addr;
		reg_addr += control->reg_38->length;

		control->reg_39 = kzalloc(sizeof(*(control->reg_39)),
				GFP_KERNEL);
		if (!control->reg_39)
			goto exit_no_mem;
		control->reg_39->length = num_of_sensing_freqs;
		control->reg_39->data = kzalloc(control->reg_39->length *
				sizeof(*(control->reg_39->data)), GFP_KERNEL);
		if (!control->reg_39->data)
			goto exit_no_mem;
		control->reg_39->address = reg_addr;
		reg_addr += control->reg_39->length;

		control->reg_40 = kzalloc(sizeof(*(control->reg_40)),
				GFP_KERNEL);
		if (!control->reg_40)
			goto exit_no_mem;
		control->reg_40->length = num_of_sensing_freqs;
		control->reg_40->data = kzalloc(control->reg_40->length *
				sizeof(*(control->reg_40->data)), GFP_KERNEL);
		if (!control->reg_40->data)
			goto exit_no_mem;
		control->reg_40->address = reg_addr;
		reg_addr += control->reg_40->length;
	}
	reg_num++;

	/* control 41 */
	if (f54->query.has_signal_clarity == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_41 = kzalloc(sizeof(*(control->reg_41)),
				GFP_KERNEL);
		if (!control->reg_41)
			goto exit_no_mem;
		control->reg_41->address = reg_addr;
		reg_addr += sizeof(control->reg_41->data);
	}
	reg_num++;

	/* control 42 */
	if (f54->query.has_variance_metric == 1)
		reg_addr += CONTROL_42_SIZE;

	/* controls 43 44 45 46 47 48 49 50 51 52 53 54 */
	if (f54->query.has_multi_metric_state_machine == 1)
		reg_addr += CONTROL_43_54_SIZE;

	/* controls 55 56 */
	if (f54->query.has_0d_relaxation_control == 1)
		reg_addr += CONTROL_55_56_SIZE;

	/* control 57 */
	if (f54->query.has_0d_acquisition_control == 1) {
		attrs_ctrl_regs_exist[reg_num] = true;
		control->reg_57 = kzalloc(sizeof(*(control->reg_57)),
				GFP_KERNEL);
		if (!control->reg_57)
			goto exit_no_mem;
		control->reg_57->address = reg_addr;
		reg_addr += sizeof(control->reg_57->data);
	}
	reg_num++;

	/* control 58 */
	if (f54->query.has_0d_acquisition_control == 1)
		reg_addr += CONTROL_58_SIZE;

	/* control 59 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_59_SIZE;

	/* controls 60 61 62 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_60_62_SIZE;

	/* control 63 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1) ||
			(f54->query.has_slew_metric == 1) ||
			(f54->query.has_slew_option == 1) ||
			(f54->query.has_noise_mitigation2 == 1))
		reg_addr += CONTROL_63_SIZE;

	/* controls 64 65 66 67 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_64_67_SIZE * 7;
	else if ((f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_64_67_SIZE;

	/* controls 68 69 70 71 72 73 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_68_73_SIZE;

	/* control 74 */
	if (f54->query.has_slew_metric == 1)
		reg_addr += CONTROL_74_SIZE;

	/* control 75 */
	if (f54->query.has_enhanced_stretch == 1)
		reg_addr += num_of_sensing_freqs;

	/* control 76 */
	if (f54->query.has_startup_fast_relaxation == 1)
		reg_addr += CONTROL_76_SIZE;

	/* controls 77 78 */
	if (f54->query.has_esd_control == 1)
		reg_addr += CONTROL_77_78_SIZE;

	/* controls 79 80 81 82 83 */
	if (f54->query.has_noise_mitigation2 == 1)
		reg_addr += CONTROL_79_83_SIZE;

	/* controls 84 85 */
	if (f54->query.has_energy_ratio_relaxation == 1)
		reg_addr += CONTROL_84_85_SIZE;

	/* control 86 */
	if ((f54->query.has_query13 == 1) && (f54->query.has_ctrl86 == 1))
		reg_addr += CONTROL_86_SIZE;

	/* control 87 */
	if ((f54->query.has_query13 == 1) && (f54->query.has_ctrl87 == 1))
		reg_addr += CONTROL_87_SIZE;

	/* control 88 */
	if (f54->query.has_ctrl88 == 1) {
		control->reg_88 = kzalloc(sizeof(*(control->reg_88)),
				GFP_KERNEL);
		if (!control->reg_88)
			goto exit_no_mem;
		control->reg_88->address = reg_addr;
		reg_addr += sizeof(control->reg_88->data);
	}

	/* control 89 */
	if (f54->query.has_cidim ||
			f54->query.has_noise_mitigation_enhancement ||
			f54->query.has_rail_im)
		reg_addr += CONTROL_89_SIZE;

	/* control 90 */
	if (f54->query_15.has_ctrl90)
		reg_addr += CONTROL_90_SIZE;

	/* control 91 */
	if (f54->query_21.has_ctrl91)
		reg_addr += CONTROL_91_SIZE;

	/* control 92 */
	if (f54->query_16.has_ctrl92)
		reg_addr += CONTROL_92_SIZE;

	/* control 93 */
	if (f54->query_16.has_ctrl93)
		reg_addr += CONTROL_93_SIZE;

	/* control 94 */
	if (f54->query_16.has_ctrl94_query18)
		reg_addr += CONTROL_94_SIZE;

	/* control 95 */
	if (f54->query_16.has_ctrl95_query19)
		reg_addr += CONTROL_95_SIZE;

	/* control 96 */
	if (f54->query_21.has_ctrl96)
		reg_addr += CONTROL_96_SIZE;

	/* control 97 */
	if (f54->query_21.has_ctrl97)
		reg_addr += CONTROL_97_SIZE;

	/* control 98 */
	if (f54->query_21.has_ctrl98)
		reg_addr += CONTROL_98_SIZE;

	/* control 99 */
	if (f54->query.touch_controller_family == 2) {
		control->reg_99 = kzalloc(sizeof(*(control->reg_99)),
				GFP_KERNEL);
		if (!control->reg_99)
			goto exit_no_mem;
		control->reg_99->address = reg_addr;
		reg_addr += CONTROL_99_SIZE;

		pr_info("%s: F54_CTRL99 addr = 0x%x\n", 
			__func__, control->reg_99->address);
	}

	return 0;

exit_no_mem:
	dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to alloc mem for control registers\n",
			__func__);
	return -ENOMEM;
}

static int synaptics_rmi4_f54_set_data_offset(
			struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;
	int num_data_regs, current_offset = 0;
	const struct synaptics_reg_query_rule *rule;

	num_data_regs = MAX_NUM_REGS(f54_data_reg_query_rules);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Discovering locations of up to %d data registers",
			__func__, num_data_regs);

	f54->data_offsets = kzalloc(num_data_regs * sizeof(unsigned char),
			GFP_KERNEL);

	if (!f54->data_offsets) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Failed to alloc mem for data offsets",
			__func__);
		return -ENOMEM;
	}

	for (rule = f54_data_reg_query_rules;
		rule->reg != QUERY_RULES_END; rule++) {
		if (NO_QUERY == rule->query) {
			if (rule->bit) {
				dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: F54_Data%d found at offset %d",
					__func__, rule->reg, current_offset);
				f54->data_offsets[rule->reg] = current_offset;
				current_offset += rule->length;
			}
		} else {
			if (f54->query.data[rule->query] & rule->bit) {
				dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: F54_Data%d found at offset %d",
					__func__, rule->reg, current_offset);
				f54->data_offsets[rule->reg] = current_offset;
				current_offset += rule->length;
			}
		}
	}
	return 0;
}

static void synaptics_rmi4_f54_status_work(struct work_struct *work)
{
	int retval;
	unsigned int patience = 250;
	unsigned char command;

	struct synaptics_rmi4_f54_handle *f54 =
		container_of((struct delayed_work *) work,
				struct synaptics_rmi4_f54_handle, status_work);

	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	if (f54->status != STATUS_BUSY)
		return;

	do {
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

		if (command & COMMAND_GET_REPORT)
			msleep(20);
		else
			break;
	} while (--patience > 0);
	if (command & COMMAND_GET_REPORT) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Timed out waiting for report ready\n",
				__func__);
		goto error_exit;
	}

	set_report_size(rmi4_data);
	if (f54->report_size == 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Report data size = 0\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}

	retval = get_report_data(rmi4_data);
	if (retval < 0)
		goto error_exit;
#ifdef RAW_HEX
	print_raw_hex_report(rmi4_data);
#endif

#ifdef HUMAN_READABLE
	print_image_report(rmi4_data);
#endif

error_exit:
	mutex_lock(&f54->status_mutex);
	f54->status = retval;
	mutex_unlock(&f54->status_mutex);

	return;
}

static void synaptics_rmi4_f54_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned char page)
{
	unsigned char ii;
	unsigned char intr_offset;

	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	f54->query_base_addr = fd->query_base_addr | (page << 8);
	f54->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f54->data_base_addr = fd->data_base_addr | (page << 8);
	f54->command_base_addr = fd->cmd_base_addr | (page << 8);

	f54->intr_reg_num = (intr_count + 7) / 8;
	if (f54->intr_reg_num != 0)
		f54->intr_reg_num -= 1;

	f54->intr_mask = 0;
	intr_offset = intr_count % 8;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++) {
		f54->intr_mask |= 1 << ii;
	}

	return;
}

static int synaptics_rmi5_f55_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	struct synaptics_rmi4_f55_handle *f55 =
		(struct synaptics_rmi4_f55_handle *) rmi4_data->f55;
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;
	// unsigned char rx_electrodes = f54->query.num_of_rx_electrodes;
	// unsigned char tx_electrodes = f54->query.num_of_tx_electrodes;
	unsigned char rx_electrodes;   
	unsigned char tx_electrodes;
	
	retval = synaptics_rmi4_reg_read(rmi4_data,
			f55->query_base_addr,
			f55->query.data,
			sizeof(f55->query.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read f55 query registers\n",
				__func__);
		return retval;
	}

	if (!f55->query.has_sensor_assignment) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: f55 has no sensor assignment\n",
				__func__);
		return -ENODATA;
	}
	
	tx_electrodes = f55->query.num_of_tx_electrodes;
	rx_electrodes = f55->query.num_of_rx_electrodes;

	f55->rx_assignment = kzalloc(rx_electrodes, GFP_KERNEL);
	if (!f55->rx_assignment) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for f54 rx_assignment\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	f55->tx_assignment = kzalloc(tx_electrodes, GFP_KERNEL);
	if (!f55->tx_assignment) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for f55 tx_assignment\n",
				__func__);
		retval = -ENOMEM;
		goto free_rx_mem;
	}

	if (rx_electrodes > 0) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->control_base_addr + SENSOR_RX_MAPPING_OFFSET,
				f55->rx_assignment,
				rx_electrodes);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read f55 rx assignment\n",
				__func__);
			return retval;
		}

		f54->rx_assigned = 0;
		for (ii = 0; ii < rx_electrodes; ii++) {
			if (f55->rx_assignment[ii] != 0xff)
				f54->rx_assigned++;
		}
	}
	else {
		f54->rx_assigned = 0;
	}

	if (tx_electrodes > 0) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f55->control_base_addr + SENSOR_TX_MAPPING_OFFSET,
				f55->tx_assignment,
				tx_electrodes);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read f55 tx assignment\n",
				__func__);
			return retval;
		}

		f54->tx_assigned = 0;
		for (ii = 0; ii < tx_electrodes; ii++) {
			if (f55->tx_assignment[ii] != 0xff)
				f54->tx_assigned++;
		}
	}
	else {
		f54->tx_assigned = 0;
	}
	pr_info("%s: rx_electrodes = %d, tx_electrodes = %d\n",
		__func__, f54->rx_assigned, f54->tx_assigned);

	return 0;

free_rx_mem:
	kfree(f55->rx_assignment);
	f55->rx_assignment = NULL;
exit:
	return retval;
}

static int synaptics_rmi4_f55_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned char page)
{
	struct synaptics_rmi4_f55_handle *f55;

	f55 = kzalloc(sizeof(*f55), GFP_KERNEL);
	if (!f55) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for f55\n",
				__func__);
		rmi4_data->f55 = NULL;
		return -ENOMEM;
	}

	f55->query_base_addr = fd->query_base_addr | (page << 8);
	f55->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f55->data_base_addr = fd->data_base_addr | (page << 8);
	f55->command_base_addr = fd->cmd_base_addr | (page << 8);

	rmi4_data->f55 = f55;

	return 0;
}

static int synaptics_rmi4_f54_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	if (!f54)
		return 0;

	if ((f54->intr_mask & intr_mask) &&
			STATUS_RAW_STREAMING != f54->status) {
		queue_delayed_work(f54->status_workqueue,
				&f54->status_work,
				msecs_to_jiffies(STATUS_WORK_INTERVAL));
	}

	return 0;
}

static int synaptics_rmi4_f54_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned short addr;
	unsigned char page;
	unsigned char intr_count = 0;
	bool f54found = false;
	bool f55found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_f54_handle *f54;
	struct synaptics_rmi4_f55_handle *f55;

	unsigned char offset;
	
	init_completion(&rmi4_data->test_reporting_remove_complete);

	if (rmi4_data->f54) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: f54 is already init\n",
			__func__);
		return 0;
	}

	f54 = kzalloc(sizeof(*f54), GFP_KERNEL);
	if (!f54) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for f54\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	f54->rmi4_data = rmi4_data;
	rmi4_data->f54 = f54;

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
			addr |= (page << 8);

			retval = synaptics_rmi4_reg_read(rmi4_data,
					addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read reg 0x%x\n",
				__func__, addr);
				goto exit_free_mem;
			}

			addr &= ~(MASK_8BIT << 8);

			if (!rmi_fd.fn_number)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F54:
				synaptics_rmi4_f54_set_regs(rmi4_data,
						&rmi_fd, intr_count, page);
				f54found = true;
				break;
			case SYNAPTICS_RMI4_F55:
				retval = synaptics_rmi4_f55_set_regs(rmi4_data,
						&rmi_fd, page);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to set f55 regs\n",
					__func__);
					goto exit_free_mem;
				}
				f55found = true;
				f55 = (struct synaptics_rmi4_f55_handle *) rmi4_data->f55;
				break;
			default:
				break;
			}

			if (f54found && f55found)
				goto pdt_done;

			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
		}
	}

	if (!f54found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to found f54\n",
				__func__);
		retval = -ENODEV;
		goto exit_free_mem;
	}

pdt_done:
	retval = synaptics_rmi4_reg_read(rmi4_data,
			f54->query_base_addr,
			f54->query.data,
			sizeof(f54->query.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read f54 query registers\n",
				__func__);
		goto exit_free_mem;
	}
	
	offset = sizeof(f54->query.data);

	if (f54->query.has_sense_frequency_control == 0)
		offset -= 1;

	/* query 14 */
	if (f54->query.has_ctrl87)
		offset += 1;

	/* query 15 */
	if (f54->query.has_query15) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_15.data,
				sizeof(f54->query_15.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 16 */
	if (f54->query_15.has_query16) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_16.data,
				sizeof(f54->query_16.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 17 */
	if (f54->query_16.has_query17)
		offset += 1;

	/* query 18 */
	if (f54->query_16.has_ctrl94_query18)
		offset += 1;

	/* query 19 */
	if (f54->query_16.has_ctrl95_query19)
		offset += 1;

	/* query 20 */
	if (f54->query_15.has_query20)
		offset += 1;

	/* query 21 */
	if (f54->query_15.has_query21) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_21.data,
				sizeof(f54->query_21.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

#if defined(CONFIG_TCT_SDM660_COMMON) || defined(BBRY_MINISW)
	if (rmi4_data->hw_if->board_data->num_of_rx_electrodes != 0)
		f54->query.num_of_rx_electrodes = rmi4_data->hw_if->board_data->num_of_rx_electrodes;
	if (rmi4_data->hw_if->board_data->num_of_tx_electrodes != 0)
		f54->query.num_of_tx_electrodes = rmi4_data->hw_if->board_data->num_of_tx_electrodes;
#endif

	f54->rx_assigned = f54->query.num_of_rx_electrodes;
	f54->tx_assigned = f54->query.num_of_tx_electrodes;

	if (f54->query.num_of_tx_electrodes == 0) {
		f54->rx_assigned = rmi4_data->num_of_rx;
		f54->tx_assigned = rmi4_data->num_of_tx;
	}

	retval = synaptics_rmi4_f54_set_ctrl(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set up f54 control registers\n",
				__func__);
		goto exit_free_control;
	}

	retval = synaptics_rmi4_f54_set_data_offset(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set up f54 data offsets\n",
				__func__);
		goto exit_free_control;
	}

	if (f55found && (f54->query.num_of_tx_electrodes > 0)) {
		retval = synaptics_rmi5_f55_init(rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to init f55\n",
				__func__);
			goto exit_free_control;
		}
	}

	mutex_init(&f54->status_mutex);
	mutex_init(&f54->data_mutex);
	mutex_init(&f54->control_mutex);

	retval = synaptics_rmi4_f54_set_sysfs(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs entries\n",
				__func__);
		goto exit_sysfs;
	}

	f54->status_workqueue =
			create_singlethread_workqueue("f54_status_workqueue");
	INIT_DELAYED_WORK(&f54->status_work,
			synaptics_rmi4_f54_status_work);

#ifdef WATCHDOG_HRTIMER
	/* Watchdog timer to catch unanswered get report commands */
	hrtimer_init(&f54->watchdog, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	f54->watchdog.function = get_report_timeout;

	/* Work function to do actual cleaning up */
	INIT_WORK(&f54->timeout_work, timeout_set_status);
#endif

	f54->status = STATUS_IDLE;
	f54->current_lockup_poll_num = 0;
	f54->prev_lockup_poll_value = 0;

	return 0;

exit_sysfs:
	if (f55found) {
		kfree(f55->rx_assignment);
		f55->rx_assignment = NULL;
		kfree(f55->tx_assignment);
		f55->tx_assignment = NULL;
	}

exit_free_control:
	free_control_mem(f54);

exit_free_mem:
	if (f55found) {
		kfree(f55);
		rmi4_data->f55 = NULL;
	}
	kfree(f54);
	rmi4_data->f54 = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_f54_remove(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_f55_handle *f55 =
		(struct synaptics_rmi4_f55_handle *) rmi4_data->f55;
	struct synaptics_rmi4_f54_handle *f54 =
		(struct synaptics_rmi4_f54_handle *) rmi4_data->f54;

	if (!f54)
		goto exit;

#ifdef WATCHDOG_HRTIMER
	hrtimer_cancel(&f54->watchdog);
#endif

	cancel_delayed_work_sync(&f54->status_work);
	flush_workqueue(f54->status_workqueue);
	destroy_workqueue(f54->status_workqueue);

	remove_sysfs(f54);

	if (f55 && f55->rx_assignment) {
		kfree(f55->rx_assignment);
		f55->rx_assignment = NULL;
	}
	if (f55 && f55->tx_assignment) {
		kfree(f55->tx_assignment);
		f55->tx_assignment = NULL;
	}

	free_control_mem(f54);
	kfree(f55);
	rmi4_data->f55 = NULL;

	kfree(f54->data_offsets);
	f54->data_offsets = NULL;

	if (f54->data_buffer_size) {
		kfree(f54->report_data);
		f54->report_data = NULL;
	}
	kfree(f54);
	rmi4_data->f54 = NULL;

exit:
	complete(&rmi4_data->test_reporting_remove_complete);

	return;
}

static void synaptics_rmi4_f54_reset(struct synaptics_rmi4_data *rmi4_data)
{
	if (!rmi4_data->f54) {
		synaptics_rmi4_f54_init(rmi4_data);
		return;
	}

	synaptics_rmi4_f54_remove(rmi4_data);
	synaptics_rmi4_f54_init(rmi4_data);

	return;
}

static struct synaptics_rmi4_exp_fn f54_module = {
	.fn_type = RMI_F54,
	.init = synaptics_rmi4_f54_init,
	.remove = NULL,
	.reset = synaptics_rmi4_f54_reset,
	.reinit = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_f54_attn,
};

int synaptics_rmi4_f54_module_init(struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_bbry_rmi4_new_function(rmi4_data, &f54_module, true);

	return 0;
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Test Reporting Module");
MODULE_LICENSE("GPL v2");
