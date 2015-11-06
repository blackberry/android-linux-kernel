/*
 * Copyright (C) 2015 BlackBerry Limited
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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/input/matrix_keypad.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/version.h>
#ifdef CONFIG_STMPE_KEYPAD_DDT
#include <misc/ddt.h>
#include <misc/lw_event_types.h>

#define LW_EVENT_KEYPAD_UNEXPECTED_RESET	LW_EVENT_KEYPAD_00002525
#endif
#define REG_CHIP_ID				0x00
#define REG_INT_CTRL				0x04
#define REG_INT_EN_MASK			0x06
#define REG_INT_STA				0x08
#define REG_GPIO_SET_LOW			0x10
#define REG_GPIO_SET_DIR_LOW			0x19
#define REG_KPC_ROW				0x30
#define REG_KPC_COL				0x31
#define REG_KPC_CTRL_LOW			0x33
#define REG_KPC_CTRL_MID			0x34
#define REG_KPC_CTRL_HIGH			0x35
#define REG_KPC_CMD				0x36
#define REG_KPC_DATA				0x3a

#define CHIP_ID				0xc1
#define INT_I0_WAKEUP				0x01
#define INT_I1_KEYPAD				0x02
#define INT_I2_KEYPAD_OVERFLOW			0x04
#define INT_I3_GPIO				0x08
#define INT_I4_COMBO_KEY			0x10
#define INT_CTRL_MASK				0x01
#define KPC_SCAN_FREQ_60			0x00
#define KPC_SCAN_FREQ_30			0x01
#define KPC_SCAN_FREQ_15			0x02
#define KPC_SCAN_FREQ_275			0x03
#define KPC_CMD_SCAN_ENABLE			0x01
#define KPC_DATA_LENGTH			5
#define KPC_DATA_NO_KEY_MASK			0x78
#define KEYPAD_ROW_SHIFT			3
#define KEYPAD_MAX_ROWS			8
#define KEYPAD_MAX_COLS			8
#define KEYPAD_KEYMAP_SIZE	\
	(KEYPAD_MAX_ROWS * KEYPAD_MAX_COLS)
#define KEYPAD_MAX_KEY_BUFFER		3
#define KEYPAD_STATS_TIMER		5000
#define KEYPAD_STUCK_KEY_TIME		10
#define KEYPAD_SPACE_TIMER		200
#define KEYPAD_SPACE_KEY		0x39
#ifdef CONFIG_STMPE_KEYPAD_DDT
#define MAX_RESET_DDT_SEND		50
#endif
/* TEMP */
#define debug(str, args...) /* dev_err(&keypad->i2c_client->dev, "%s: " str "\n", __func__, ##args)*/
#define info(str, args...) dev_err(&keypad->i2c_client->dev, "%s: " str "\n", __func__, ##args)
#define warn(str, args...) dev_err(&keypad->i2c_client->dev, "%s: " str "\n", __func__, ##args)
#define error(str, args...) dev_err(&keypad->i2c_client->dev, "%s: " str "\n", __func__, ##args)
typedef enum {
	STMPE_RESET_DETECT_GPIO,
	STMPE_RESET_DETECT_WATCHDOG
} reset_detect_type_t;

struct stmpe_keypad {
	struct i2c_client *i2c_client;
	struct input_dev *input_dev;
	int ctrl_irq;
	int reset_irq;
	int reset_count;
	int interrupts;
	int keys_pressed;
	int stuck_keys;
	int multi_key;
	int extra_key;
	reset_detect_type_t reset_detect;
	struct timeval last_keypress;
	struct regulator *regulator_vdd;
	unsigned short keymap[KEYPAD_KEYMAP_SIZE];
	struct task_struct *wd_task;
	struct mutex keypad_mutex;
	struct workqueue_struct *workqueue;
	struct work_struct input_work;
	struct timer_list timer;
	struct timer_list space_timer;
	int down_keys[KEYPAD_MAX_KEY_BUFFER];
	bool slide_open;
	bool slide_transitioning;
	bool irq_enabled;
	bool keypad_enabled;
	struct {
		u8 row_mask;
		u16 column_mask;
		int reset_det_gpio;
		int scan_count;
		int debounce;
		int scan_frequency;
		int rst_int_gpio;
		int kp_int_gpio;
		int reset_gpio;
		int interrupt_polarity;
		bool edge_interrupt;
	} config;
};

typedef enum {
	KEY_EVENT_UP,
	KEY_EVENT_DOWN
} key_event_type_t;
#define KEY_EVENT_FIFO_SIZE	10

struct key_event_data {
	key_event_type_t type;
	uint8_t row;
	uint8_t col;
	uint8_t code;
	uint8_t key;
};

struct space_adjacent {
	uint8_t key;
	int count;
	int blocked;
};
static struct space_adjacent sp_adj_keys[] = {
	{0x2E, 0, 0},
	{0x2F, 0, 0},
	{0x30, 0, 0},
	{0x31, 0, 0},
	{0x0B, 0, 0},
	{0x64, 0, 0}
};
const static char *keypad_name[] = {
	"stmpe_keypad",
	"stmpe_azerty_keypad",
	"stmpe_qwertz_keypad",
};


static const struct input_device_id input_event_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_KEYPAD_SLIDE)] =
					BIT_MASK(SW_KEYPAD_SLIDE) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_KEYPAD_TRANSITION)] =
					BIT_MASK(SW_KEYPAD_TRANSITION) },
	},
	{ },			/* Terminating zero entry */
};

MODULE_DEVICE_TABLE(input, evdev_ids);

static ssize_t stmpe_keypad_store_kl(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t stmpe_keypad_show_counters(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t stmpe_keypad_store_enabled(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t stmpe_keypad_show_enabled(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t stmpe_keypad_show_keys(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t stmpe_keypad_show_reset_count(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t stmpe_keypad_show_status(struct device *dev,
				struct device_attribute *attr, char *buf);
static void stmpe_keypad_reset_counters(struct stmpe_keypad *keypad);
static void stmpe_input_event(struct input_handle *handle,
				unsigned int type,
				unsigned int code, int value);
static int stmpe_input_event_connect(struct input_handler *handler,
					struct input_dev *dev,
					const struct input_device_id *id);
static void stmpe_input_event_disconnect(struct input_handle *handle);
int stmpe_keypad_watchdog(void *data);
int stmpe_keypad_check_status(struct stmpe_keypad *keypad);
static irqreturn_t stmpe_keypad_irq_handler(int irq, void *dev);
static irqreturn_t stmpe_reset_irq_handler(int irq, void *dev);
static int stmpe_enable_keypad(struct stmpe_keypad *keypad, bool enable);
static int stmpe_keypad_status(struct stmpe_keypad *keypad);

static struct input_handler input_event_handler = {
	.event		= stmpe_input_event,
	.connect	= stmpe_input_event_connect,
	.disconnect	= stmpe_input_event_disconnect,
	.minor		= 0,
	.name		= "stmpe_keypad",
	.id_table	= input_event_ids,
};

static DEVICE_ATTR(kl, S_IWUSR | S_IWGRP, NULL, stmpe_keypad_store_kl);
static DEVICE_ATTR(keys, S_IRUSR | S_IRGRP, stmpe_keypad_show_keys, NULL);
static DEVICE_ATTR(status, S_IRUSR | S_IRGRP, stmpe_keypad_show_status, NULL);
static DEVICE_ATTR(reset_count, S_IRUSR | S_IRGRP,
					stmpe_keypad_show_reset_count, NULL);
static DEVICE_ATTR(enabled, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
					stmpe_keypad_show_enabled,
					stmpe_keypad_store_enabled);
static DEVICE_ATTR(keypad_counters, S_IRUSR | S_IRGRP,
					stmpe_keypad_show_counters, NULL);

static struct attribute *stmpe_keypad_attrs[] = {
	&dev_attr_kl.attr,
	&dev_attr_enabled.attr,
	&dev_attr_keys.attr,
	&dev_attr_reset_count.attr,
	&dev_attr_status.attr,
	&dev_attr_keypad_counters.attr,
	NULL,
};

static struct attribute_group stmpe_keypad_attr_group = {
	.attrs = stmpe_keypad_attrs,
};

static ssize_t stmpe_keypad_store_kl(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);
	uint32_t kl = 0;
	uint8_t max_num = (sizeof(keypad_name)/sizeof(char *));
	int ret;

	ret = sscanf(buf, "%u *", &kl);
	if (ret >= 1)
		keypad->input_dev->name = ((kl > 0) && (kl <= max_num)) ?
					keypad_name[kl-1] : keypad_name[0];
	else
		dev_err(&keypad->input_dev->dev, "sscanf returned %d\n", ret);

	return count;
}

static ssize_t stmpe_keypad_store_enabled(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;

	if (stmpe_enable_keypad(keypad, enable) == -1)
		return -EINVAL;

	return count;
}

static ssize_t stmpe_keypad_show_enabled(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", keypad->keypad_enabled);
}

static ssize_t stmpe_keypad_show_keys(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X\n", keypad->down_keys[0],
						keypad->down_keys[1],
						keypad->down_keys[2]);
}

static ssize_t stmpe_keypad_show_reset_count(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", keypad->reset_count);
}

static ssize_t stmpe_keypad_show_status(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);

	if (keypad->keypad_enabled)
		return snprintf(buf, PAGE_SIZE, "%d\n", stmpe_keypad_status(keypad));
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t stmpe_keypad_show_counters(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmpe_keypad *keypad = dev_get_drvdata(dev);
	ssize_t  size = 0;
	int down_count = 0;
	int i;

	for (i = 0; i < KEYPAD_MAX_KEY_BUFFER; i++)
		if (keypad->down_keys[i] != 0)
			down_count++;

	buf[0] = 0;
	size += snprintf(buf, PAGE_SIZE, "reset count = %d\n",
						keypad->reset_count);
	size += snprintf(buf + size, PAGE_SIZE - size,
			"down keys = %d\n", down_count);
	size += snprintf(buf + size, PAGE_SIZE - size,
			"stuck keys = %d\n", keypad->stuck_keys);
	size += snprintf(buf + size, PAGE_SIZE - size,
			"total keys pressed = %d\n", keypad->keys_pressed);
	size += snprintf(buf + size, PAGE_SIZE - size,
			"total multi-key = %d\n", keypad->multi_key);
	size += snprintf(buf + size, PAGE_SIZE - size,
				"total extra_keys = %d\n", keypad->extra_key);
	size += snprintf(buf + size, PAGE_SIZE - size,
			"total interrupts = %d\n", keypad->interrupts);
	for (i = 0; i < ARRAY_SIZE(sp_adj_keys); i++) {
		size += snprintf(buf + size, PAGE_SIZE - size,
					"total inadv space 0x%X detected = %d\n",
						sp_adj_keys[i].key,
						sp_adj_keys[i].count);
		size += snprintf(buf + size, PAGE_SIZE - size,
					"total inadv space 0x%X blocked = %d\n",
						sp_adj_keys[i].key,
						sp_adj_keys[i].blocked);
	}

	stmpe_keypad_reset_counters(keypad);

	return size;
}

static void stmpe_keypad_reset_counters(struct stmpe_keypad *keypad)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sp_adj_keys); i++) {
		sp_adj_keys[i].count = 0;
		sp_adj_keys[i].blocked = 0;
	}
	keypad->interrupts = 0;
	keypad->keys_pressed = 0;
	keypad->multi_key = 0;
	keypad->reset_count = 0;
	keypad->stuck_keys = 0;
	keypad->extra_key = 0;

}
static int read_block(struct stmpe_keypad *keypad,
			uint8_t reg, uint8_t length, uint8_t *out)
{
	struct i2c_msg msg[] = {
		{
			.addr = keypad->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = keypad->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = out,
		},
	};
	int i, ret;

	ret = i2c_transfer(keypad->i2c_client->adapter, msg, 2);
	if (ret < 0 || ret != 2) {
		error("i2c error while reading register 0x%x %d", reg, ret);
		return -EINVAL;
	}
	debug("read block %d bytes at 0x%x", length, reg);
	for (i = 0; i < length; i++)
		debug("  [0x%x] = 0x%x", reg + i, out[i]);

	return 0;
}

static int write_block(struct stmpe_keypad *keypad,
			uint8_t addr, uint8_t length, uint8_t *data)
{
	int ret;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = keypad->i2c_client->addr,
			.flags = 0,
			.len = sizeof(data),
			.buf = buf,
		}
	};
	buf[0] = addr;
	memcpy(&buf[1], &data[0], length);

	ret = i2c_transfer(keypad->i2c_client->adapter, msg, 1);
	if (ret < 0 || ret != 1)
		error("i2c error while writing block to adress 0x%x %d",
								addr, ret);

	return 0;
}

static int read_reg(struct stmpe_keypad *keypad, uint8_t reg)
{
	uint8_t data[1] = {0}; /* read buf */
	struct i2c_msg msg[] = {
		{
			.addr = keypad->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = keypad->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};
	int ret;


	ret = i2c_transfer(keypad->i2c_client->adapter, msg, 2);
	if (ret < 0 || ret != 2) {
		error("i2c error while reading register 0x%x %d", reg, ret);
		return -EINVAL;
	}

	return data[0];
}

static int write_reg(struct stmpe_keypad *keypad, uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };
	struct i2c_msg msg[] = {
		{
			.addr = keypad->i2c_client->addr,
			.flags = 0,
			.len = sizeof(data),
			.buf = data,
		}
	};
	int ret;

	ret = i2c_transfer(keypad->i2c_client->adapter, msg, 1);
	if (ret < 0 || ret != 1)
		error("i2c error while writing register 0x%x %d", reg, ret);

	return 0;
}

static int stmpe_keypad_status(struct stmpe_keypad *keypad)
{
	int reg = 0;

	mutex_lock(&keypad->keypad_mutex);
	reg = read_reg(keypad, REG_INT_EN_MASK);
	mutex_unlock(&keypad->keypad_mutex);

	return (reg == (INT_I1_KEYPAD|INT_I2_KEYPAD_OVERFLOW)) ? 1 : 0;
}

static int stmpe_power_enable(struct stmpe_keypad *keypad, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = regulator_enable(keypad->regulator_vdd);
		if (ret) {
			error("Failed to enable vcc regulator, %d", ret);
			return ret;
		}

		ret = gpio_direction_output(keypad->config.reset_gpio, 1);
		if (ret < 0) {
			error("Failed to configure reset GPIO %d", ret);
			return ret;
		}

		msleep(20);

	} else {
		ret = regulator_disable(keypad->regulator_vdd);
		if (ret) {
			error("Failed to disable vcc regulator, %d", ret);
			return ret;
		}

		ret = gpio_direction_output(keypad->config.reset_gpio, 0);
		if (ret < 0) {
			error("Failed to configure reset GPIO %d", ret);
			return ret;
		}
	}

	return ret;
}

static int stmpe_irq_enable(struct stmpe_keypad *keypad, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = request_threaded_irq(keypad->ctrl_irq, NULL,
						stmpe_keypad_irq_handler,
						IRQF_TRIGGER_LOW|IRQF_ONESHOT,
						"stmpe-keypad", keypad);
		if (ret < 0) {
			error("Failed to create irq thread, %d", ret);
			return ret;
		}

		if (keypad->reset_detect == STMPE_RESET_DETECT_GPIO) {
			ret = request_threaded_irq(keypad->reset_irq, NULL,
					stmpe_reset_irq_handler,
					IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
					"stmpe-keypad-reset", keypad);
			if (ret < 0)
				error("Failed to attach reset int handler");
		}

		keypad->irq_enabled = true;
	} else {
		if (keypad->irq_enabled) {
			disable_irq(keypad->ctrl_irq);
			free_irq(keypad->ctrl_irq, keypad);
			if (keypad->reset_detect == STMPE_RESET_DETECT_GPIO) {
				disable_irq(keypad->reset_irq);
				free_irq(keypad->reset_irq, keypad);
			}
			keypad->irq_enabled = false;
		}
	}

	return 0;
}

static void stmpe_input_event(struct input_handle *handle, unsigned int type,
						unsigned int code, int value)
{
	struct stmpe_keypad *keypad = handle->private;

	if (type == EV_SW) {
		switch (code) {
		case SW_KEYPAD_SLIDE:
			keypad->slide_open = value;
			break;
		case SW_KEYPAD_TRANSITION:
			keypad->slide_transitioning = value;
			break;
		default:
			return;
			break;
		}

		queue_work(keypad->workqueue,
				&keypad->input_work);
	}
}

static int stmpe_input_event_connect(struct input_handler *handler,
					struct input_dev *dev,
					const struct input_device_id *id)
{
	int ret;
	struct input_handle *handle;
	struct stmpe_keypad *keypad = handler->private;

	if (test_bit(SW_KEYPAD_SLIDE, dev->swbit)) {
		if (test_bit(SW_KEYPAD_SLIDE, dev->sw)) {
			keypad->slide_open = 1;
			queue_work(keypad->workqueue, &keypad->input_work);
		}
	}

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->private = handler->private;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "stmpe_keypad";

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

static void stmpe_input_event_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static int stmpe_keypad_configure(struct stmpe_keypad *keypad)
{
	int ret = 0;
	u8 scan_frequency;

	mutex_lock(&keypad->keypad_mutex);

	/* Configure keypad controller */
	if (keypad->reset_detect == STMPE_RESET_DETECT_GPIO) {
		ret = write_reg(keypad, REG_GPIO_SET_DIR_LOW,
					(1 << keypad->config.reset_det_gpio));
		if (ret < 0)
			goto unlock_ret;

		ret = write_reg(keypad, REG_GPIO_SET_LOW,
					(1 << keypad->config.reset_det_gpio));
		if (ret < 0)
			goto unlock_ret;
	}

	ret = write_reg(keypad, REG_KPC_ROW, keypad->config.row_mask);
	if (ret < 0)
		goto unlock_ret;

	ret = write_block(keypad, REG_KPC_COL, 2,
				(uint8_t *) &keypad->config.column_mask);
	if (ret < 0)
		goto unlock_ret;

	ret = write_reg(keypad, REG_KPC_CTRL_LOW,
				(u8) (keypad->config.scan_count << 4));
	if (ret < 0)
		goto unlock_ret;

	ret = write_reg(keypad, REG_KPC_CTRL_MID,
				(u8) (keypad->config.debounce << 1));
	if (ret < 0)
		goto unlock_ret;

	switch(keypad->config.scan_frequency) {
	case 60:
		scan_frequency = KPC_SCAN_FREQ_60;
		break;
	case 30:
		scan_frequency = KPC_SCAN_FREQ_30;
		break;
	case 15:
		scan_frequency = KPC_SCAN_FREQ_15;
		break;
	case 275:
		scan_frequency = KPC_SCAN_FREQ_275;
		break;
	default:
		scan_frequency = KPC_SCAN_FREQ_60;
		error("Unsupported keypad scan frequency %d",
					keypad->config.scan_frequency);
		break;
	}
	ret = write_reg(keypad, REG_KPC_CTRL_HIGH, scan_frequency);
	if (ret < 0)
		goto unlock_ret;

	/* Set INT_EN_MASK */
	ret = write_reg(keypad, REG_INT_EN_MASK,
				INT_I1_KEYPAD|INT_I2_KEYPAD_OVERFLOW);
	if (ret < 0)
		goto unlock_ret;

	/* Set polarity/type and enable global ints (INT_CTRL) */
	ret = write_reg(keypad, REG_INT_CTRL, INT_CTRL_MASK);
	if (ret < 0)
		goto unlock_ret;

	ret = write_reg(keypad, REG_KPC_CMD, KPC_CMD_SCAN_ENABLE);
	if (ret < 0)
		goto unlock_ret;

unlock_ret:
	mutex_unlock(&keypad->keypad_mutex);

	return ret;
}

int stmpe_handle_keypress(struct stmpe_keypad *keypad)
{
	int ret = 0;
	uint8_t data[KPC_DATA_LENGTH];
	int i, n, num_events;
	bool space_key_down;
	static int adj_key_index;
	static bool indv_space;
	bool adj_key_down = false;
	bool inject_key;
	int pressed = 0;
	int released = 0;
	int down_keys[KEYPAD_MAX_KEY_BUFFER];
	struct key_event_data key_data[KEYPAD_MAX_KEY_BUFFER];

	do {
		num_events = 0;
		space_key_down = false;
		adj_key_down = false;

		for (n = 0; n < KEYPAD_MAX_KEY_BUFFER; n++)
			down_keys[n] = 0;

		ret = read_block(keypad, REG_KPC_DATA, KPC_DATA_LENGTH, data);
		if (ret < 0) {
			error("failed to read keypad data");
			return IRQ_NONE;
		}

		for (i = 0; i < KEYPAD_MAX_KEY_BUFFER; i++) {
			if (KPC_DATA_NO_KEY_MASK !=
				(data[i] & KPC_DATA_NO_KEY_MASK)) {
				key_data[num_events].type =
						data[i] & 0x80 ? KEY_EVENT_UP :
							KEY_EVENT_DOWN;
				key_data[num_events].row = data[i] & 0x07;
				key_data[num_events].col =
						(data[i] >> 3) & 0x0f;
				key_data[num_events].code =
					MATRIX_SCAN_CODE(
						key_data[num_events].row,
						key_data[num_events].col,
							KEYPAD_ROW_SHIFT);
				key_data[num_events].key =
					keypad->keymap[key_data[num_events].code];

				if (key_data[num_events].type ==
							KEY_EVENT_DOWN) {
					if (key_data[num_events].key ==
							KEYPAD_SPACE_KEY)
						space_key_down = true;

				for (n = 0; n < ARRAY_SIZE(sp_adj_keys); n++)
					if (sp_adj_keys[n].key ==
						key_data[num_events].key) {
						adj_key_down = true;
						adj_key_index = n;
					}
				}
				num_events++;
			}
		}

		for (i = 0; i < num_events; i++) {
			inject_key = true;

			/* Check if key already reported */
			if (key_data[i].type == KEY_EVENT_DOWN) {
				for (n = 0; n < KEYPAD_MAX_KEY_BUFFER; n++) {
					if (keypad->down_keys[n] ==
							key_data[i].code)
						inject_key = false;
				}
			}

			/* Check for inadvertent space key */
			if ((key_data[i].type == KEY_EVENT_DOWN) &&
				(key_data[i].key == KEYPAD_SPACE_KEY) &&
				(inject_key != false) && /* already reported */
				adj_key_down) {
				pressed++;
				indv_space = true;
				sp_adj_keys[adj_key_index].count++;
				mod_timer(&keypad->space_timer,
				(jiffies +
					msecs_to_jiffies(KEYPAD_SPACE_TIMER)));
				inject_key = false;
				info("Possible inadvertent space down: 0x%X",
					sp_adj_keys[adj_key_index].key);
			}

			/* Cancel space key timer on release */
			if ((key_data[i].type == KEY_EVENT_UP) &&
					(key_data[i].key == KEYPAD_SPACE_KEY)) {
				if (indv_space) {
					indv_space = false;
					info("Space key up");
				}
				if (timer_pending(&keypad->space_timer)) {
					del_timer(&keypad->space_timer);
					sp_adj_keys[adj_key_index].blocked++;
					info("Space timer canceled. Key ignored");
				}
			}

			if (inject_key) {
				if (key_data[i].type == KEY_EVENT_DOWN)
					pressed++;
				else
					released++;

				input_event(keypad->input_dev,
						EV_MSC,
						MSC_SCAN, key_data[i].code);

				input_report_key(keypad->input_dev,
					key_data[i].key,
					key_data[i].type);

				input_sync(keypad->input_dev);
			}

			if (key_data[i].type == KEY_EVENT_DOWN)
				down_keys[i] = key_data[i].code;

			do_gettimeofday(&keypad->last_keypress);

			/* Start the stats timer */
			if (!timer_pending(&keypad->timer))
				mod_timer(&keypad->timer,
					(jiffies +
					msecs_to_jiffies(
					KEYPAD_STATS_TIMER)));
		}

		if (num_events > 0) {
			for (n = 0; n < KEYPAD_MAX_KEY_BUFFER; n++)
				keypad->down_keys[n] = down_keys[n];

			if ((keypad->down_keys[0] != 0) &&
					(keypad->down_keys[1] != 0) &&
						(keypad->down_keys[2] != 0)) {
				error("Possible unintentional adjacent key.");
				error("0x%X 0x%X 0x%X ", keypad->down_keys[0],
							 keypad->down_keys[1],
							 keypad->down_keys[2]);
			}
		}

	} while (num_events > 0);

	/* Update counters */
	keypad->keys_pressed += pressed;

	if ((pressed + released) > 1)
		keypad->extra_key += ((pressed + released) - 1);

	if ((pressed > 0)  && ((keypad->down_keys[1] != 0)
				|| (keypad->down_keys[2] != 0)))
		keypad->multi_key++;

	return ret;
}

static int stmpe_keypad_process_events(struct stmpe_keypad *keypad)
{
	int status, ret = 0;

	mutex_lock(&keypad->keypad_mutex);
	/* Read the interrupt status */
	status = read_reg(keypad, REG_INT_STA);

	if (status < 0) {
		error("Failed reading interrupt status %d", status);
		goto unlock_ret;
	}
	if (0 == status) {
		warn("No interrupt source");
		goto unlock_ret;
	}

	/* Handle overflow interrupt */
	if (status & INT_I2_KEYPAD_OVERFLOW)
		error("Keypad overflow interrupt");

	/* Handle keypad interrupt */
	if (status & INT_I1_KEYPAD)
		ret = stmpe_handle_keypress(keypad);

unlock_ret:
	mutex_unlock(&keypad->keypad_mutex);

	return ret;
}

static irqreturn_t stmpe_keypad_irq_handler(int irq, void *dev)
{
	struct stmpe_keypad *keypad = dev;
	int ret;

	keypad->interrupts++;

	ret = stmpe_keypad_process_events(keypad);
	if (ret < 0) {
		warn("Error handling keypad interrupt");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static irqreturn_t stmpe_reset_irq_handler(int irq, void *dev)
{
	struct stmpe_keypad *keypad = dev;

	stmpe_keypad_check_status(keypad);

	return IRQ_HANDLED;
}

int stmpe_keypad_watchdog(void *data)
{
	struct stmpe_keypad *keypad = data;

	while (!kthread_should_stop()) {
		stmpe_keypad_check_status(keypad);
		msleep_interruptible(1000);
	}

	return 0;
}


int stmpe_keypad_check_status(struct stmpe_keypad *keypad)
{
#ifdef CONFIG_STMPE_KEYPAD_DDT
	struct logworthy_event_details_t details;
	int ret = 0;
#endif

	if (!stmpe_keypad_status(keypad)) {
		keypad->reset_count++;
		error("Keypad reset!! Count: %d", keypad->reset_count);

#ifdef CONFIG_STMPE_KEYPAD_DDT
		if (keypad->reset_count < MAX_RESET_DDT_SEND) {
			memset(&details, 0,
				sizeof(struct logworthy_event_details_t));
			details.d1 = keypad->reset_count;
			details.creator_id = (char *)keypad->input_dev->name;
			ret = ddt_send(LW_EVENT_KEYPAD_UNEXPECTED_RESET, &details,
					"['logcat://main',"
					"'exec:///system/bin/dmesg']");
			error("ddt send event");
			if (ret < 0)
				error("error sending: %d", ret);
		}
#endif

		/* toggle power */
		stmpe_power_enable(keypad, false);
		msleep_interruptible(20);
		stmpe_power_enable(keypad, true);

		while (read_reg(keypad, REG_CHIP_ID) != CHIP_ID) {
			error("Unable to communicate with Keypad IC");
			msleep_interruptible(1000);
		}
		stmpe_keypad_configure(keypad);
		error("Keypad Controller - Reconfigured");
	}

	return 0;
}

static void stmpe_keypad_space_handler(unsigned long data)
{
	struct stmpe_keypad *keypad = (struct stmpe_keypad *) data;
	int i;

	for (i = 0; i < KEYPAD_MAX_KEY_BUFFER; i++) {
		if (keypad->keymap[keypad->down_keys[i]] == KEYPAD_SPACE_KEY) {
			info("inject space key");
			input_event(keypad->input_dev, EV_MSC,
					MSC_SCAN, keypad->down_keys[i]);
			input_report_key(keypad->input_dev,
					keypad->keymap[keypad->down_keys[i]],
					KEY_EVENT_DOWN);
			input_sync(keypad->input_dev);
		}
	}
}

static void stmpe_keypad_timer_handler(unsigned long data)
{
	struct stmpe_keypad *keypad = (struct stmpe_keypad *) data;
	struct timeval current_time;
	int last_kp = 0;
	int down_count = 0;
	int inv_space_count = 0;
	int i;
	static bool stuck_key;

	for (i = 0; i < KEYPAD_MAX_KEY_BUFFER; i++)
		if (keypad->down_keys[i] != 0)
			down_count++;

	for (i = 0; i < ARRAY_SIZE(sp_adj_keys); i++)
		inv_space_count += sp_adj_keys[i].count;

	info("STATS: ints: %d ex:%d kp: %d mk:%d sk:%d rc: %d dc: %d isp: %d",
						keypad->interrupts,
						keypad->extra_key,
						keypad->keys_pressed,
						keypad->multi_key,
						keypad->stuck_keys,
						keypad->reset_count,
						down_count,
						inv_space_count);
	do_gettimeofday(&current_time);
	last_kp = current_time.tv_sec - keypad->last_keypress.tv_sec;

	if ((last_kp < 5) || down_count > 0)
		mod_timer(&keypad->timer, (jiffies + msecs_to_jiffies(
							KEYPAD_STATS_TIMER)));

	if (down_count > 0) {
		if (last_kp >= KEYPAD_STUCK_KEY_TIME) {
			if (!stuck_key) {
				stuck_key = true;
				keypad->stuck_keys++;
			}
			error("Key(s) down for > 10 sec.");
			error("Stuck key? 0x%X 0x%X 0x%X ",
					keypad->down_keys[0],
					keypad->down_keys[1],
					keypad->down_keys[2]);
		}
	} else
		stuck_key = false;
}

static void stmp_keypad_input_wq(struct work_struct *work)
{
	struct stmpe_keypad *keypad =
			container_of(work, struct stmpe_keypad,
							input_work);
	stmpe_enable_keypad(keypad,
			(keypad->slide_open && !keypad->slide_transitioning));
}

static int stmpe_enable_keypad(struct stmpe_keypad *keypad, bool enable)
{
	int ret = 0;

	if (enable) {
		if (!keypad->keypad_enabled) {
			ret = stmpe_power_enable(keypad, true);
			if (ret < 0)
				error("Failed to enable keypad power: %d\n",
									ret);

			ret = stmpe_keypad_configure(keypad);
			if (ret < 0)
				error("Failed to configure the keypad: %d\n",
									ret);

			ret = stmpe_irq_enable(keypad, true);
			if (ret < 0)
				error("Failed to enable the keypad IRQ: %d\n",
									ret);

			if ((keypad->reset_detect ==
					STMPE_RESET_DETECT_WATCHDOG) &&
							(!keypad->wd_task)) {
				keypad->wd_task =
					kthread_run(&stmpe_keypad_watchdog,
					(void *)keypad, "keypad_watchdog");
				if (keypad->wd_task == ERR_PTR(-ENOMEM))
					error("Failed to create wd thread");
			}
			keypad->keypad_enabled = true;
		}
	} else {
		if (keypad->keypad_enabled) {
			if (keypad->wd_task)
				kthread_stop(keypad->wd_task);
			keypad->wd_task = NULL;

			if (timer_pending(&keypad->timer))
				del_timer(&keypad->timer);

			ret = stmpe_irq_enable(keypad, false);
			if (ret < 0)
				error("Failed to disable the keypad IRQ: %d\n",
									ret);

			ret = stmpe_power_enable(keypad, false);
			if (ret < 0) {
				error("Failed to disable keypad power: %d\n",
									ret);
			}
			keypad->keypad_enabled = false;
		}
	}
	return ret;
}
static int stmpe_keypad_parse_dt(struct device *dev,
					struct stmpe_keypad *keypad)
{
	struct device_node *np = dev->of_node;
	int ret;

	/* Retrieve device tree data */
	ret = of_property_read_u8_array(np, "st,row-mask",
					&keypad->config.row_mask, 1);
	if (ret) {
		error("Failed to retrieve row-mask, %d", ret);
		return -EINVAL;
	}

	ret = of_property_read_u16(np, "st,column-mask",
					&keypad->config.column_mask);
	if (ret) {
		error("Failed to retrieve column-mask, %d", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "st,scan-count",
					&keypad->config.scan_count);
	if (ret) {
		error("Failed to retrieve scan-count, %d", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "st,debounce",
					&keypad->config.debounce);
	if (ret) {
		error("Failed to retrieve debounce, %d", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "st,scan-frequency",
					&keypad->config.scan_frequency);
	if (ret) {
		error("Failed to retrieve scan-frequency, %d", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "st,reset_det_gpio",
					&keypad->config.reset_det_gpio);
	if (!ret) {
		keypad->reset_detect = STMPE_RESET_DETECT_GPIO;

		keypad->config.rst_int_gpio =
				of_get_named_gpio(np, "st,keypad-rst-int", 0);
		if (!gpio_is_valid(keypad->config.rst_int_gpio)) {
			error("Reset GPIO is invalid");
			keypad->reset_detect = STMPE_RESET_DETECT_WATCHDOG;
		}
	} else
		keypad->reset_detect = STMPE_RESET_DETECT_WATCHDOG;


	keypad->config.kp_int_gpio =
			of_get_named_gpio(np, "st,keypad-kp-int", 0);
	if (!gpio_is_valid(keypad->config.kp_int_gpio)) {
		error("Interrupt GPIO is invalid");
		return -EINVAL;
	}

	keypad->config.reset_gpio = of_get_named_gpio(np, "st,keypad-reset", 0);
	if (!gpio_is_valid(keypad->config.reset_gpio)) {
		error("Reset GPIO is invalid");
		return -EINVAL;
	}

	keypad->regulator_vdd =
			regulator_get(&keypad->i2c_client->dev, "st,vdd");
	if (IS_ERR(keypad->regulator_vdd)) {
		error("Failed to get regulator %d",
					(int)PTR_ERR(keypad->regulator_vdd));
		return -EINVAL;
	}

	return 0;
}

static int stmpe_keypad_probe(struct i2c_client *i2c,
		const struct i2c_device_id *dev_id)
{
	struct stmpe_keypad *keypad;
	int ret;
	int i;

	/* Initialize the keypad structure */
	keypad = kzalloc(sizeof(*keypad), GFP_KERNEL);
	if (NULL == keypad) {
		error("Failed to allocate memory for keypad device");
		ret = -ENOMEM;
		goto fail;
	}

	keypad->slide_transitioning = 0;
	keypad->i2c_client = i2c;
	keypad->wd_task = NULL;
	keypad->keypad_enabled = false;

	stmpe_keypad_reset_counters(keypad);

	mutex_init(&keypad->keypad_mutex);

	dev_set_drvdata(&i2c->dev, keypad);

	ret = stmpe_keypad_parse_dt(&i2c->dev, keypad);
	if (ret < 0) {
		error("Failed to parse device tree parameters %d\n", ret);
		goto fail_free;
	}

	ret = gpio_request(keypad->config.reset_gpio, "keypad-rst");
	if (ret < 0) {
		error("Failed to request reset GPIO %d, error %d\n",
				keypad->config.reset_gpio, ret);
		goto fail_free;
	}

	/* Set up irq gpio */
	ret = gpio_request(keypad->config.kp_int_gpio, "keypad-int");
	if (ret < 0) {
		error("Failed to request interrupt GPIO %d, error %d\n",
					keypad->config.kp_int_gpio, ret);
		goto fail_free;
	}

	keypad->ctrl_irq = gpio_to_irq(keypad->config.kp_int_gpio);

	/* Set up the reset detect interrupt */
	if (keypad->reset_detect == STMPE_RESET_DETECT_GPIO) {

		if (!gpio_is_valid(keypad->config.rst_int_gpio)) {
			error("Reset GPIO is invalid");
			ret = -EINVAL;
			goto fail_free;
		}

		ret = gpio_request(keypad->config.rst_int_gpio, "reset-int");
		if (ret < 0) {
			error("Failed to request reset int GPIO %d, err %d\n",
					keypad->config.rst_int_gpio, ret);
		}

		keypad->reset_irq = gpio_to_irq(keypad->config.rst_int_gpio);
	}

	/* Enable keypad power */
	ret = stmpe_power_enable(keypad, true);
	if (ret < 0) {
		error("Failed to initialize the keypad controller: %d\n", ret);
		goto fail_free;
	}
	/* Read out the chip ID. Verify version. */
	ret = read_reg(keypad, REG_CHIP_ID);
	if (ret < 0) {
		error("Unable to communicate with IC %d", ret);
		ret = -EAGAIN;
		goto fail_free;
	}
	if (CHIP_ID != ret) {
		error("Found unknown IC (chip ID = 0x%x)", ret);
		ret = -ENODEV;
		goto fail_free;
	}

	/* Disable power */
	ret = stmpe_power_enable(keypad, false);
	if (ret < 0) {
		error("Failed to disable power: %d\n", ret);
		goto fail_free;
	}

	/* Create sysfs entry for keypad language */
	ret = sysfs_create_group(&i2c->dev.kobj, &stmpe_keypad_attr_group);
	if (ret)
		dev_err(&i2c->dev, "Unable to create group. error: %d\n", ret);

	ret = sysfs_create_link(i2c->dev.kobj.parent->parent->parent->parent,
						&i2c->dev.kobj, "keypad");
	if (ret)
		dev_err(&i2c->dev, "Unable to create link, rc=%d\n", ret);

	/* Register as an input device */
	keypad->input_dev = input_allocate_device();
	if (NULL == keypad->input_dev) {
		error("Unable to register keypad as input device");
		ret = -ENOMEM;
		goto fail_free;;
	}

	keypad->input_dev->name = keypad_name[0];
	keypad->input_dev->id.bustype = BUS_I2C;
	keypad->input_dev->dev.parent = &i2c->dev;

	input_set_capability(keypad->input_dev, EV_MSC, MSC_SCAN);

	ret = input_register_device(keypad->input_dev);
	if (ret) {
		error("Unable to register input device : %d", ret);
		goto fail_input_free;
	}

	/* Build keypad matrix using device tree data */
	ret = matrix_keypad_build_keymap(NULL, "linux,keymap",
					KEYPAD_MAX_ROWS,
					KEYPAD_MAX_COLS,
					keypad->keymap, keypad->input_dev);
	if (ret)
		error("Unable build keymap : %d", ret);

	/* Create input handler for slide events */
	input_event_handler.private = keypad;
	ret = input_register_handler(&input_event_handler);
	if (ret)
		dev_err(&i2c->dev,
			"Unable to register with input, rc=%d\n", ret);

	/* Set up work queue for input events */
	keypad->workqueue =
			create_singlethread_workqueue("background_workqueue");
	INIT_WORK(&keypad->input_work, stmp_keypad_input_wq);

	for (i = 0; i < KEYPAD_MAX_KEY_BUFFER; i++)
		keypad->down_keys[i] = 0;
	init_timer(&keypad->timer);
	keypad->timer.function = stmpe_keypad_timer_handler;
	keypad->timer.data = (unsigned long) keypad;

	init_timer(&keypad->space_timer);
	keypad->space_timer.function = stmpe_keypad_space_handler;
	keypad->space_timer.data = (unsigned long) keypad;

	return 0;

fail_input_free:
	input_free_device(keypad->input_dev);
fail_free:
	kfree(keypad);
fail:
	return ret;
}

static int stmpe_keypad_remove(struct i2c_client *client)
{
	dev_err(&client->dev, "%s: exiting stmpe-keypad!\n", __func__);
	return 0;
}

static const struct i2c_device_id stmpe_id_table[] = {
	{"stmpe-keypad", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, stmpe_id_table);

#ifdef CONFIG_OF
static struct of_device_id stmpe_match_table[] = {
	{ .compatible = "st,stmpe-keypad",},
	{ },
};
#else
#define stmpe_match_table NULL
#endif

static struct i2c_driver stmpe_keypad_driver = {
	.driver.name    = "stmpe-keypad",
	.driver.owner   = THIS_MODULE,
	.driver.of_match_table = stmpe_match_table,
	.probe      = stmpe_keypad_probe,
	.remove     = stmpe_keypad_remove,
	.id_table   = stmpe_id_table,
};

static int __init stmpe_kp_init(void)
{
	return i2c_add_driver(&stmpe_keypad_driver);
}

static void __exit stmpe_kp_exit(void)
{
	i2c_del_driver(&stmpe_keypad_driver);
}

module_init(stmpe_kp_init);
module_exit(stmpe_kp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STMPE keypad driver");
MODULE_AUTHOR("BlackBerry Limited");
