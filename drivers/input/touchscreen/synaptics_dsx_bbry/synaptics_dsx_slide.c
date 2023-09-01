/*
 * Synaptics DSX touchscreen slide detection
 *
 * Copyright (C) 2015 BlackBerry Limited
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
#include "synaptics_dsx_slide.h"
#include <linux/input.h>
#include <linux/sort.h>
#include <linux/platform_device.h>
#include "synaptics_dsx_test_reporting.h"

#define BASELINE_DEFAULT_OPEN	1285
#define BASELINE_DEFAULT_CLOSED	1100
#define SCALE_FACTOR_DEFAULT	220
#define OUTPUT_FULL_SCALE		255

enum {
	SLIDE_UNKNOWN,
	SLIDE_OPEN,
	SLIDE_CLOSED,
	SLIDE_SLIDING
};

struct slide_position_sensor {
	struct synaptics_rmi4_data *rmi4;
	int last_state;
	int sliding_from;
	int16_t *open_baseline;
	int16_t *closed_baseline;
	int16_t baseline_extreme;
	int16_t scale_factor;
	bool update_baseline_extreme;
	bool debug;
	bool raw_stream;
	bool int_enabled;
};

#ifdef CONFIG_BBRY_DEBUG
#define slide_dbg(str, args...)	printk(KERN_INFO "[slide] " str "\n", \
									##args);

static void dump_frame(struct slide_position_sensor *slide,
		int16_t *data, const char *title)
{
	size_t r, c;

	if (slide->debug) {
		slide_dbg("%s:", title);
		for (r = 0; r < slide->rmi4->num_of_tx; r++) {
			printk(KERN_INFO "[slide] ");
			for (c = 0; c < slide->rmi4->num_of_rx; c++)
				printk("%4d ",
					data[r*slide->rmi4->num_of_rx+c]);
			printk("\n");
		}
	}
}
#else
#define dump_frame(...)
#define slide_dbg(...)
#endif

static ssize_t slide_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#ifdef CONFIG_BBRY_DEBUG
static ssize_t slide_debug_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t slide_scale_factor_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif
static int slide_request_raw_frame(struct slide_position_sensor *slide);
static int slide_set_state(struct slide_position_sensor *slide, int state);

static struct device_attribute attrs[] = {
	__ATTR(slide, (S_IWUSR | S_IWGRP),
			NULL,
			slide_attr_store),
#ifdef CONFIG_BBRY_DEBUG
	__ATTR(slide_debug, (S_IWUSR | S_IWGRP),
			NULL,
			slide_debug_attr_store),
	__ATTR(slide_scale_factor, (S_IWUSR | S_IWGRP),
			NULL,
			slide_scale_factor_attr_store),
#endif
};

static ssize_t slide_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct slide_position_sensor *slide = rmi4_data->slide;
	unsigned int state;

	if (sscanf(buf, "%u", &state) != 1)
		return -EINVAL;

	if (state == 0) {
		slide_dbg("Slide is closed");
		slide_set_state(slide, SLIDE_CLOSED);
	} else if (state == 1) {
		slide_dbg("Slide is sliding");
		slide_set_state(slide, SLIDE_SLIDING);
	} else if (state == 2) {
		slide_dbg("Slide is open");
		slide_set_state(slide, SLIDE_OPEN);
	} else {
		dev_err(slide->rmi4->pdev->dev.parent,
				"%s Slide invalid state: %d\n",
				__func__, state);
		return -EINVAL;
	}

	return count;
}

#ifdef CONFIG_BBRY_DEBUG
static ssize_t slide_debug_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct slide_position_sensor *slide = rmi4_data->slide;

	if (!slide)
		return -EINVAL;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	slide->debug = value;

	return count;
}

static ssize_t slide_scale_factor_attr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct slide_position_sensor *slide = rmi4_data->slide;

	if (!slide)
		return -EINVAL;

	if (sscanf(buf, "%i", &value) != 1)
		return -EINVAL;

	slide->scale_factor = (int16_t)value;

	return count;
}
#endif

static int slide_interrupt_enable(struct slide_position_sensor *slide,
		bool enable)
{
	int retval;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short f01_ctrl_reg;
	struct synaptics_rmi4_f54_handle *f54 = slide->rmi4->f54;

	if (enable && slide->int_enabled)
		return 0;
	if (!enable && !slide->int_enabled)
		return 0;

	intr_mask = slide->rmi4->intr_mask;
	f01_ctrl_reg = slide->rmi4->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (!enable) {
		retval = synaptics_rmi4_reg_write(slide->rmi4,
				f01_ctrl_reg,
				&zero,
				sizeof(zero));
		if (retval < 0) {
			dev_err(slide->rmi4->pdev->dev.parent,
						"%s: Failed to write control f01 control reg\n",
						__func__);
			return retval;
		}
	}

	for (ii = 0; ii < slide->rmi4->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			f01_ctrl_reg = slide->rmi4->f01_ctrl_base_addr + 1 + ii;
			if (enable) {
				retval = synaptics_rmi4_reg_write(slide->rmi4,
						f01_ctrl_reg,
						&zero,
						sizeof(zero));
				if (retval < 0) {
					dev_err(slide->rmi4->pdev->dev.parent,
						"%s: Failed to zero control f01 control reg\n",
						__func__);
					return retval;
				}
			} else {
				retval = synaptics_rmi4_reg_write(slide->rmi4,
						f01_ctrl_reg,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0) {
					dev_err(slide->rmi4->pdev->dev.parent,
						"%s: Failed to write control f01 control reg\n",
						__func__);
					return retval;
				}
			}
		}
	}

	f01_ctrl_reg = slide->rmi4->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (enable) {
		retval = synaptics_rmi4_reg_write(slide->rmi4,
				f01_ctrl_reg,
				&f54->intr_mask,
				1);
		if (retval < 0) {
			dev_err(slide->rmi4->pdev->dev.parent,
						"%s: Failed to write f54 intr reg\n",
						__func__);
			return retval;
		}
		slide->int_enabled = true;
	} else {
		slide->int_enabled = false;
		slide->raw_stream = false;
	}

	return 0;
}

static int slide_request_raw_frame(struct slide_position_sensor *slide)
{
	struct synaptics_rmi4_f54_handle *f54;
	unsigned char data;
	unsigned char command;
	int retval;

	if (slide->rmi4->suspend)
		return 0;

	f54 = slide->rmi4->f54;
	if (!f54)
		return 0;

	if (slide_interrupt_enable(slide, true)) {
		dev_warn(slide->rmi4->pdev->dev.parent,
				"%s: Failed to enable raw report interrupt\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	if (STATUS_IDLE != f54->status)
		dev_warn(slide->rmi4->pdev->dev.parent,
			"%s: Previous raw report was still ongoing\n",
			__func__);

	/* Set the report type */
	f54->report_type = F54_RAW_16BIT_IMAGE;
	data = (unsigned char)F54_RAW_16BIT_IMAGE;
	retval = synaptics_rmi4_reg_write(slide->rmi4,
			f54->data_base_addr + f54->data_offsets[0],
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
			"%s: Failed to write data register\n",
			__func__);
		goto error_exit;
	}

	command = 0;
	retval = synaptics_rmi4_reg_write(slide->rmi4,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
			"%s: Failed to clear F54 command 0 register\n",
			__func__);
		goto error_exit;
	}

	command = (unsigned char)COMMAND_GET_REPORT;
	retval = synaptics_rmi4_reg_write(slide->rmi4,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
			"%s: Failed to write get report command\n",
			__func__);
		goto error_exit;
	}

	f54->status = STATUS_RAW_STREAMING;

error_exit:
	mutex_unlock(&f54->status_mutex);
	return retval;
}

static int slide_read_raw(struct slide_position_sensor *slide,
		int16_t *buf, size_t buf_len)
{
	int retval = 0;
	unsigned char report_index[2] = {0};
	unsigned int report_size = 2 * buf_len;
	struct synaptics_rmi4_f54_handle *f54 = slide->rmi4->f54;

	if (report_size == 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
				"%s: buf size = 0\n",
				__func__);
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_RAW_STREAMING)
		goto error_exit;

	retval = synaptics_rmi4_reg_write(slide->rmi4,
			f54->data_base_addr + DATA_REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
				"%s: Failed to write report data index\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}

	retval = synaptics_rmi4_reg_read(slide->rmi4,
			f54->data_base_addr + DATA_REPORT_DATA_OFFSET,
			(unsigned char *)buf,
			report_size);
	if (retval < 0) {
		dev_err(slide->rmi4->pdev->dev.parent,
				"%s: Failed to read report data\n",
				__func__);
		retval = -EINVAL;
		goto error_exit;
	}
	f54->status = STATUS_IDLE;

error_exit:
	mutex_unlock(&f54->status_mutex);
	return retval;
}

static int compare_int32(const void *a, const void *b)
{
	int32_t x = *(int32_t *)a, y = *(int32_t *)b;
	return x < y ? -1 : x == y ? 0 : 1;
}

static int slide_raw_frame_isr(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	struct slide_position_sensor *slide = rmi4_data->slide;
	struct synaptics_rmi4_f54_handle *f54;

	if (!slide)
		return 0;

	f54 = rmi4_data->f54;
	if (!f54)
		return 0;

	if (slide->rmi4->suspend)
		return 0;

	if ((intr_mask & f54->intr_mask)) {
		const size_t num_tx = slide->rmi4->num_of_tx;
		const size_t num_rx = slide->rmi4->num_of_rx;
		int16_t frame[num_tx * num_rx];

		if (slide_read_raw(slide, frame, ARRAY_SIZE(frame)) < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s Failed to read raw frame data\n",
					__func__);
			return -EAGAIN;
		}

		if (!slide->raw_stream) {
			if (slide_interrupt_enable(slide, false) < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s Failed to read raw frame data\n",
						__func__);
				return -EAGAIN;
			}

			if (SLIDE_OPEN == slide->sliding_from) {
				if (f54) {
					memcpy(slide->open_baseline,
							frame, sizeof(frame));
					dump_frame(slide,
							slide->open_baseline,
							"Updated open baseline");
				}
			} else if (SLIDE_CLOSED == slide->sliding_from) {
				if (f54) {
					memcpy(slide->closed_baseline,
							frame, sizeof(frame));
					dump_frame(slide,
							slide->closed_baseline,
							"Updated closed baseline");
				}
			}
		} else {
			size_t i, r, c;
			int32_t col_sums[num_rx];
			int32_t sum = 0;
			int32_t num_cols_to_avg = num_rx - 2*num_rx/3;
#ifdef CONFIG_BBRY_DEBUG
			bool normalized_data_changed = false;
#endif

			/* request the next frame right away */
			if (slide_request_raw_frame(slide) < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s Failed to request next raw frame\n",
						__func__);
				return -EAGAIN;
			}

			dump_frame(slide, frame, "raw frame");

			if (SLIDE_OPEN == slide->sliding_from) {
				int16_t min = frame[0] -
						slide->open_baseline[0];

				for (i = 0; i < ARRAY_SIZE(frame); i++) {
					frame[i] = frame[i] -
							slide->open_baseline[i];
					if (frame[i] < min)
						min = frame[i];
				}

				if (slide->update_baseline_extreme) {
					slide->baseline_extreme = min;
					slide->update_baseline_extreme = false;
#ifdef CONFIG_BBRY_DEBUG
					if (slide->debug)
						slide_dbg("updated min to %d",
								min);
					normalized_data_changed = true;
#endif
					dump_frame(slide, frame, "delta frame");
				}
			} else if (SLIDE_CLOSED == slide->sliding_from) {
				int16_t max = frame[0] -
						slide->closed_baseline[0];

				for (i = 0; i < ARRAY_SIZE(frame); i++) {
					frame[i] = frame[i] -
							slide->closed_baseline[i];
					if (frame[i] > max)
						max = frame[i];
				}

				/* update max_min if necessary */
				if (slide->update_baseline_extreme) {
					slide->baseline_extreme = max;
					slide->update_baseline_extreme = false;
#ifdef CONFIG_BBRY_DEBUG
					if (slide->debug)
						slide_dbg("updated max to %d",
								max);
					normalized_data_changed = true;
#endif
					dump_frame(slide, frame, "delta frame");
				}
			}

			/* "normalize" and add columns */
			memset(col_sums, 0, sizeof(col_sums));
			for (i = 0, r = 0; r < num_tx; r++) {
				for (c = 0; c < num_rx; c++) {
					frame[i] -= slide->baseline_extreme;
					col_sums[c] += frame[i];
					i++;
				}
			}

#ifdef CONFIG_BBRY_DEBUG
			if (normalized_data_changed) {
				dump_frame(slide, frame, "normalized frame");
				if (slide->debug)
					for (c = 0; c < num_rx; c++)
						slide_dbg("col_sum[%d] = %d",
								(int)c, col_sums[c]);
			}
#endif

			sort(col_sums, sizeof(col_sums)/sizeof(col_sums[0]),
					sizeof(col_sums[0]),
					compare_int32, NULL);

			/* avg the middle colums and scale to output */
			for (i = num_rx/3; i <  num_rx/3 + num_cols_to_avg; i++)
				sum += col_sums[i];
			if (SLIDE_OPEN == slide->sliding_from) {
				slide_dbg("position = %d (col_avg = %d)",
						OUTPUT_FULL_SCALE*
						sum / num_cols_to_avg /
						slide->scale_factor,
						sum / num_cols_to_avg);
			} else if (SLIDE_CLOSED == slide->sliding_from) {
				slide_dbg("position = %d, (col_avg = %d)",
						OUTPUT_FULL_SCALE +
						OUTPUT_FULL_SCALE *
						sum / num_cols_to_avg/
						slide->scale_factor,
						sum / num_cols_to_avg);
			}
		}
	}

	return 0;
}

static int slide_set_state(struct slide_position_sensor *slide, int state)
{
	int ret = 0;

	if (state <= SLIDE_UNKNOWN || state > SLIDE_SLIDING)
		return -EINVAL;

	if (SLIDE_SLIDING == state) {
		if (!slide->raw_stream) {
			slide->raw_stream = true;
			if (slide_request_raw_frame(slide) < 0) {
				dev_err(slide->rmi4->pdev->dev.parent,
						"%s Failed to request next raw frame\n",
						__func__);
				ret = -EAGAIN;
			}
		}
	} else {
		/* The frame currently pending will be used to
		 * update the open or closed baseline when it
		 * becomes ready.  If there is no frame pending,
		 * request one.
		 */
		if (!slide->raw_stream)
			if (slide_request_raw_frame(slide) < 0) {
				dev_err(slide->rmi4->pdev->dev.parent,
						"%s Failed to request next raw frame\n",
						__func__);
				ret = -EAGAIN;
			}
		slide->raw_stream = false; /* TODO fix race w/ISR */
		slide->update_baseline_extreme = true;
		slide->sliding_from = state;
	}
	slide->last_state = state;

	return ret;
}

static void slide_suspend(struct synaptics_rmi4_data *rmi4_data)
{
	struct slide_position_sensor *slide = rmi4_data->slide;
	struct synaptics_rmi4_f54_handle *f54;

	if (slide) {
		slide->raw_stream = false;
		f54 = slide->rmi4->f54;

		if (f54) {
			mutex_lock(&f54->status_mutex);
			if (STATUS_RAW_STREAMING == f54->status)
				f54->status = STATUS_IDLE;
			mutex_unlock(&f54->status_mutex);
		}
	}
}

static void slide_resume(struct synaptics_rmi4_data *rmi4_data)
{
	struct slide_position_sensor *slide = rmi4_data->slide;

	if (slide && SLIDE_UNKNOWN != slide->last_state)
		if (slide_set_state(slide, slide->last_state)) {
			dev_warn(rmi4_data->pdev->dev.parent,
					"%s Failed to set slide state\n",
					__func__);
		}
}

static int slide_init(struct synaptics_rmi4_data *rmi4_data)
{
	uint16_t frame_size = rmi4_data->num_of_rx * rmi4_data->num_of_tx;
	struct slide_position_sensor *slide;
	size_t i;
	int ret;

	if (0 == frame_size) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s Sensor size unknown\n", __func__);
		return -EINVAL;
	}

	if (rmi4_data->num_of_rx < 3) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s Slide position requires at least 3 rx lines\n",
				__func__);
		return -EINVAL;
	}

	slide = kzalloc(sizeof(*slide), GFP_KERNEL);
	if (!slide) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s Failed to alloc mem for slide sensor\n",
				__func__);
		ret = -ENOMEM;
		goto fail;
	}
	slide->scale_factor = SCALE_FACTOR_DEFAULT;

	slide->open_baseline = kzalloc(frame_size*sizeof(uint16_t),
			GFP_KERNEL);
	if (!slide->open_baseline) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s Failed to alloc mem for open baseline\n",
				__func__);
		ret = -ENOMEM;
		goto fail;
	}

	slide->closed_baseline = kzalloc(frame_size*sizeof(uint16_t),
			GFP_KERNEL);
	if (!slide->closed_baseline) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s Failed to alloc mem for closed baseline\n",
				__func__);
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < frame_size; i++) {
		slide->open_baseline[i] = BASELINE_DEFAULT_OPEN;
		slide->closed_baseline[i] = BASELINE_DEFAULT_CLOSED;
	}

	for (i = 0; i < ARRAY_SIZE(attrs); i++) {
		ret = device_create_file(&rmi4_data->input_dev->dev,
				&attrs[i]);
		if (ret < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs attributes\n",
				__func__);
			for (i--; i >= 0; i--) {
				device_remove_file(&rmi4_data->input_dev->dev,
						&attrs[i]);
			}
			goto fail;
		}
	}

	rmi4_data->slide = slide;
	slide->rmi4 = rmi4_data;

	return 0;

fail:
	if (slide) {
		kfree(slide->open_baseline);
		kfree(slide->closed_baseline);
	}
	kfree(slide);
	return ret;
}

static struct synaptics_rmi4_exp_fn slide_module = {
	.fn_type = RMI_SLIDE_POSITION,
	.init = slide_init,
	.remove = NULL,
	.reset = NULL,
	.reinit = NULL,
	.suspend = slide_suspend,
	.resume = slide_resume,
	.late_resume = NULL,
	.attn = slide_raw_frame_isr,
};

int synaptics_rmi4_slide_module_init(struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_bbry_rmi4_new_function(rmi4_data, &slide_module, true);

	return 0;
}
