/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-aw2013.h>

#if defined(CONFIG_KRYPTON)
#define AW2013_MAX_RETRY 3
static u8 probe_retries=0;
extern int i2c_check_status_create(const char *name,int value);
#endif


/* register address */
#define AW_REG_RESET			0x00
#define AW_REG_GLOBAL_CONTROL		0x01
#define AW_REG_LED_STATUS		0x02
#define AW_REG_LED_ENABLE		0x30
#define AW_REG_LED_CONFIG_BASE		0x31
#define AW_REG_LED_BRIGHTNESS_BASE	0x34
#define AW_REG_TIMESET0_BASE		0x37
#define AW_REG_TIMESET1_BASE		0x38

/* register bits */
#define AW2013_CHIPID			0x33
#define AW_LED_MOUDLE_ENABLE_MASK	0x01
#define AW_LED_FADE_OFF_MASK		0x40
#define AW_LED_FADE_ON_MASK		0x20
#define AW_LED_BREATHE_MODE_MASK	0x10
#define AW_LED_RESET_MASK		0x55

#define AW_LED_RESET_DELAY		8
#define AW2013_VDD_MIN_UV		2600000
#define AW2013_VDD_MAX_UV		3300000
#define AW2013_VI2C_MIN_UV		1800000
#define AW2013_VI2C_MAX_UV		1800000

#define MAX_RISE_TIME_MS		7
#define MAX_HOLD_TIME_MS		5
#define MAX_FALL_TIME_MS		7
#define MAX_OFF_TIME_MS			5

#ifdef CONFIG_KRYPTON
#define LED_AW2013_OEM_PATCH
#endif

struct aw2013_led {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct aw2013_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex lock;
	struct regulator *vdd;
	struct regulator *vcc;
	int num_leds;

#if defined(CONFIG_KRYPTON)
	struct workqueue_struct *wq;
	unsigned long id;
#else
	int id;
#endif
	bool poweron;
};

#if defined(CONFIG_KRYPTON)
static int aw2013_write(struct aw2013_led *led, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(led->client, reg, val);
	if(ret < 0) {
		dev_err(&led->client->dev,
				"aw2013 write reg %x with %x failed, ret:%d \n", 
				reg, val, ret);
	}

	return ret;
}

static int aw2013_read(struct aw2013_led *led, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(led->client, reg);
	if (ret < 0) {
		dev_err(&led->client->dev,
				"aw2013 read reg %x failed, ret:%d \n", reg, ret);
		return ret;
	}

	*val = ret;
	return 0;
}

#else
static int aw2013_write(struct aw2013_led *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}

static int aw2013_read(struct aw2013_led *led, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(led->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}
#endif


static int aw2013_power_on(struct aw2013_led *led, bool on)
{
	int rc;

	if (on) {
		rc = regulator_enable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(led->vcc);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vcc enable failed rc=%d\n", rc);
			goto fail_enable_reg;
		}
		led->poweron = true;

#if defined(CONFIG_KRYPTON)
		mdelay(10);
#endif
	} else {
		rc = regulator_disable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(led->vcc);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vcc disable failed rc=%d\n", rc);
			goto fail_disable_reg;
		}
		led->poweron = false;

#if defined(CONFIG_KRYPTON)
		mdelay(10);
#endif
	}
	return rc;

fail_enable_reg:
	rc = regulator_disable(led->vdd);
	if (rc)
		dev_err(&led->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);

	return rc;

fail_disable_reg:
	rc = regulator_enable(led->vdd);
	if (rc)
		dev_err(&led->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);

	return rc;
}

static int aw2013_power_init(struct aw2013_led *led, bool on)
{
	int rc;

	if (on) {
		led->vdd = regulator_get(&led->client->dev, "vdd");
		if (IS_ERR(led->vdd)) {
			rc = PTR_ERR(led->vdd);
			dev_err(&led->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(led->vdd) > 0) {
			rc = regulator_set_voltage(led->vdd, AW2013_VDD_MIN_UV,
						   AW2013_VDD_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		led->vcc = regulator_get(&led->client->dev, "vcc");
		if (IS_ERR(led->vcc)) {
			rc = PTR_ERR(led->vcc);
			dev_err(&led->client->dev,
				"Regulator get failed vcc rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(led->vcc) > 0) {
			rc = regulator_set_voltage(led->vcc, AW2013_VI2C_MIN_UV,
						   AW2013_VI2C_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
				"Regulator set_vtg failed vcc rc=%d\n", rc);
				goto reg_vcc_put;
			}
		}
	} else {
		if (regulator_count_voltages(led->vdd) > 0)
			regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);

		regulator_put(led->vdd);

		if (regulator_count_voltages(led->vcc) > 0)
			regulator_set_voltage(led->vcc, 0, AW2013_VI2C_MAX_UV);

		regulator_put(led->vcc);
	}
	return 0;

reg_vcc_put:
	regulator_put(led->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(led->vdd) > 0)
		regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(led->vdd);
	return rc;
}


#if defined(CONFIG_KRYPTON)
#define NUM_LEDS_PATH (3)
#define LEDS_MASK_BITS (0x07)
static void aw2013_brightness_work(struct work_struct *work)
{
	struct aw2013_led *led = container_of(work, struct aw2013_led,
					brightness_work);
	u8 val;
	u8 idx=0, led_on_mask=0;
	u8 temp_rgb_max_current[NUM_LEDS_PATH] = {2,1,1};

	mutex_lock(&led->pdata->led->lock);

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	led_on_mask = (u8)led->id;
	pr_debug_ratelimited("%s('%s'): id:0x%x, brightness:%d\n", 
			__func__, led->cdev.name,
			led_on_mask, led->cdev.brightness);

	if (led->cdev.brightness > 0) {
		if (led->cdev.brightness > led->cdev.max_brightness)
			led->cdev.brightness = led->cdev.max_brightness;

		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);

		for_each_set_bit(idx, &led->id, NUM_LEDS_PATH) {
			if(!strcmp("yellow", led->cdev.name)
				|| !strcmp("white", led->cdev.name))
				aw2013_write(led, AW_REG_LED_CONFIG_BASE + idx,
					temp_rgb_max_current[idx]);
			else
				aw2013_write(led, AW_REG_LED_CONFIG_BASE + idx,
					led->pdata->max_current);

			aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + idx,
				led->cdev.brightness);
		}
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, (val | led_on_mask));
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, (val & (~led_on_mask)));
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if ((val&LEDS_MASK_BITS) == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	mutex_unlock(&led->pdata->led->lock);
}

static void aw2013_led_blink_set(struct aw2013_led *led, unsigned long blinking)
{
	u8 val;
	u8 idx=0, led_on_mask=0;
	u8 temp_rgb_max_current[NUM_LEDS_PATH] = {2,1,1};

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			return;
		}
	}

	led->cdev.brightness = blinking ? 
				(clamp(blinking, 100UL, (unsigned long)led->cdev.max_brightness)) : 0;

	led_on_mask = (u8)led->id;

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	aw2013_write(led, AW_REG_LED_ENABLE, val & (~LEDS_MASK_BITS));
	val &= (~LEDS_MASK_BITS);

	for(idx=0; idx<NUM_LEDS_PATH; idx++)
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + idx, 0);

	pr_debug_ratelimited("%s('%s'): id:0x%x, val:0x%x, brightness:%d\n", 
			__func__, led->cdev.name,
			led_on_mask, val, led->cdev.brightness);

	if (blinking > 0) {
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);

		for_each_set_bit(idx, &led->id, NUM_LEDS_PATH) {
			if(!strcmp("yellow", led->cdev.name)
				|| !strcmp("white", led->cdev.name))
				aw2013_write(led, AW_REG_LED_CONFIG_BASE + idx,
					AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
					AW_LED_BREATHE_MODE_MASK | temp_rgb_max_current[idx]);
			else
				aw2013_write(led, AW_REG_LED_CONFIG_BASE + idx,
					AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
					AW_LED_BREATHE_MODE_MASK | led->pdata->max_current);

			aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + idx,
				led->cdev.brightness);
			aw2013_write(led, AW_REG_TIMESET0_BASE + idx * 3,
				led->pdata->rise_time_ms << 4 |
				led->pdata->hold_time_ms);
			aw2013_write(led, AW_REG_TIMESET1_BASE + idx * 3,
				led->pdata->fall_time_ms << 4 |
				led->pdata->off_time_ms);
		}
		aw2013_write(led, AW_REG_LED_ENABLE, val | led_on_mask);
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if ((val&LEDS_MASK_BITS) == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			return;
		}
	}
}

static void aw2013_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);

	if(!led || !led->pdata || !led->pdata->led 
		|| !led->pdata->led->wq) {
		pr_err("%s(): pointer invalid\n", __func__);
		return;
	}

	led->cdev.brightness = brightness;
	queue_work(led->pdata->led->wq, &led->brightness_work);
}

static ssize_t aw2013_store_blink(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;

	if(!led || !led->pdata || !led->pdata->led 
		|| !led->pdata->led->wq) {
		pr_err("%s(): pointer invalid\n", __func__);
		return len;
	}

	/* before blink, make sure all before brightness done. */
	flush_workqueue(led->pdata->led->wq);

	mutex_lock(&led->pdata->led->lock);
	aw2013_led_blink_set(led, blinking);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}


#else
static void aw2013_brightness_work(struct work_struct *work)
{
	struct aw2013_led *led = container_of(work, struct aw2013_led,
					brightness_work);
	u8 val;

	mutex_lock(&led->pdata->led->lock);

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	if (led->cdev.brightness > 0) {
		if (led->cdev.brightness > led->cdev.max_brightness)
			led->cdev.brightness = led->cdev.max_brightness;
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
			led->pdata->max_current);
		aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,
			led->cdev.brightness);
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if (val == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	mutex_unlock(&led->pdata->led->lock);
}

static void aw2013_led_blink_set(struct aw2013_led *led, unsigned long blinking)
{
	u8 val;

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			return;
		}
	}

	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;

	if (blinking > 0) {
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
			AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
			AW_LED_BREATHE_MODE_MASK | led->pdata->max_current);
		aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,
			led->cdev.brightness);
		aw2013_write(led, AW_REG_TIMESET0_BASE + led->id * 3,
			led->pdata->rise_time_ms << 4 |
			led->pdata->hold_time_ms);
		aw2013_write(led, AW_REG_TIMESET1_BASE + led->id * 3,
			led->pdata->fall_time_ms << 4 |
			led->pdata->off_time_ms);
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if (val == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			return;
		}
	}
}

static void aw2013_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);

	led->cdev.brightness = brightness;

	schedule_work(&led->brightness_work);
}

static ssize_t aw2013_store_blink(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	mutex_lock(&led->pdata->led->lock);
	aw2013_led_blink_set(led, blinking);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}
#endif


static ssize_t aw2013_led_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			led->pdata->rise_time_ms, led->pdata->hold_time_ms,
			led->pdata->fall_time_ms, led->pdata->off_time_ms);
}

static ssize_t aw2013_led_time_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);
	int rc, rise_time_ms, hold_time_ms, fall_time_ms, off_time_ms;

	rc = sscanf(buf, "%d %d %d %d",
			&rise_time_ms, &hold_time_ms,
			&fall_time_ms, &off_time_ms);

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = (rise_time_ms > MAX_RISE_TIME_MS) ?
				MAX_RISE_TIME_MS : rise_time_ms;
	led->pdata->hold_time_ms = (hold_time_ms > MAX_HOLD_TIME_MS) ?
				MAX_HOLD_TIME_MS : hold_time_ms;
	led->pdata->fall_time_ms = (fall_time_ms > MAX_FALL_TIME_MS) ?
				MAX_FALL_TIME_MS : fall_time_ms;
	led->pdata->off_time_ms = (off_time_ms > MAX_OFF_TIME_MS) ?
				MAX_OFF_TIME_MS : off_time_ms;

#if !defined(CONFIG_KRYPTON)
	aw2013_led_blink_set(led, 1);
#endif

	mutex_unlock(&led->pdata->led->lock);
	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, aw2013_store_blink);
static DEVICE_ATTR(led_time, 0664, aw2013_led_time_show, aw2013_led_time_store);

static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_blink.attr,
	&dev_attr_led_time.attr,
	NULL,
};

static struct attribute_group aw2013_led_attr_group = {
	.attrs = aw2013_led_attributes
};

static int aw_2013_check_chipid(struct aw2013_led *led)
{
	u8 val;

	aw2013_write(led, AW_REG_RESET, AW_LED_RESET_MASK);
#if defined(CONFIG_KRYPTON)
	mdelay(10);
#else
	usleep(AW_LED_RESET_DELAY);
#endif
	aw2013_read(led, AW_REG_RESET, &val);

#if defined(LED_AW2013_OEM_PATCH)
	pr_err("%s(): aw2013 chipid: 0x%02x \n", __func__, val);
#endif

	if (val == AW2013_CHIPID)
		return 0;
	else
		return -EINVAL;
}

static int aw2013_led_err_handle(struct aw2013_led *led_array,
				int parsed_leds)
{
	int i;

#if defined(CONFIG_KRYPTON)
	if(led_array->wq) {
		destroy_workqueue(led_array->wq);
		led_array->wq = NULL;
	}
#endif

	/*
	 * If probe fails, cannot free resource of all LEDs, only free
	 * resources of LEDs which have allocated these resource really.
	 */
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);

#if !defined(CONFIG_KRYPTON)
		cancel_work_sync(&led_array[i].brightness_work);
#endif

		devm_kfree(&led_array->client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}

static int aw2013_led_parse_child_node(struct aw2013_led *led_array,
				struct device_node *node)
{
	struct aw2013_led *led;
	struct device_node *temp;
	struct aw2013_platform_data *pdata;
	int rc = 0, parsed_leds = 0;

#if defined(CONFIG_KRYPTON)
	u32 temp_id=0;
#endif

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->client = led_array->client;

		pdata = devm_kzalloc(&led->client->dev,
				sizeof(struct aw2013_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&led->client->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = led_array;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "aw2013,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

#if defined(CONFIG_KRYPTON)
		rc = of_property_read_u32(temp, "aw2013,id",
			&temp_id);
		if (rc < 0 || temp_id > LEDS_MASK_BITS) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d, id=%d\n", 
				rc, temp_id);
			goto free_pdata;
		} else
			led->id = temp_id;
#else
		rc = of_property_read_u32(temp, "aw2013,id",
			&led->id);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}
#endif

		rc = of_property_read_u32(temp, "aw2013,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,max-current",
			&led->pdata->max_current);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-current, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		INIT_WORK(&led->brightness_work, aw2013_brightness_work);

		led->cdev.brightness_set = aw2013_set_brightness;

		rc = led_classdev_register(&led->client->dev, &led->cdev);
		if (rc) {
			dev_err(&led->client->dev,
#if defined(CONFIG_KRYPTON)
				"unable to register led %ld,rc=%d\n",
#else
				"unable to register led %d,rc=%d\n",
#endif
				led->id, rc);
			goto free_pdata;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw2013_led_attr_group);
		if (rc) {
			dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		parsed_leds++;
	}

	return 0;

free_class:
	aw2013_led_err_handle(led_array, parsed_leds);
	led_classdev_unregister(&led_array[parsed_leds].cdev);

#if !defined(CONFIG_KRYPTON)
	cancel_work_sync(&led_array[parsed_leds].brightness_work);
#endif

	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	led_array[parsed_leds].pdata = NULL;
	return rc;

free_pdata:
	aw2013_led_err_handle(led_array, parsed_leds);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	return rc;

free_err:
	aw2013_led_err_handle(led_array, parsed_leds);
	return rc;
}

static int aw2013_led_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct aw2013_led *led_array;
	struct device_node *node;
	int ret, num_leds = 0;

#if defined(CONFIG_KRYPTON)
	const char *temp_string = NULL;
#endif

	node = client->dev.of_node;
	if (node == NULL)
		return -EINVAL;

	num_leds = of_get_child_count(node);

	if (!num_leds)
		return -EINVAL;

	led_array = devm_kzalloc(&client->dev,
			(sizeof(struct aw2013_led) * num_leds), GFP_KERNEL);
	if (!led_array)
		return -ENOMEM;

	led_array->client = client;
	led_array->num_leds = num_leds;

	mutex_init(&led_array->lock);

#if defined(CONFIG_KRYPTON)
	ret = of_property_read_string(node, "func-name",
			&temp_string);
	if (ret) {
		dev_err(&client->dev,
			"Failure reading func-name, ret %d\n", ret);
		goto notry;
	}

	ret = aw2013_power_init(led_array, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto free_led_arry;
	}

	if (aw2013_power_on(led_array, true)) {
		dev_err(&client->dev, "power on failed");
		goto free_led_arry;
	}

	led_array->wq = create_singlethread_workqueue(temp_string);
	if (!led_array->wq) {
		dev_err(&client->dev, "create wq failed\n");
		ret = -ENOMEM;
		goto free_led_arry;
	}
#endif

	ret = aw_2013_check_chipid(led_array);
	if (ret) {
		dev_err(&client->dev, "Check chip id error\n");
#if defined(LED_AW2013_OEM_PATCH)
		aw2013_power_on(led_array, false);
#endif
		goto free_led_arry;
	}

#if defined(LED_AW2013_OEM_PATCH)
	if (aw2013_power_on(led_array, false)) {
		dev_err(&client->dev, "power off failed");
		goto free_led_arry;
	}
#endif

	ret = aw2013_led_parse_child_node(led_array, node);
	if (ret) {
		dev_err(&client->dev, "parsed node error\n");
		goto free_led_arry;
	}

	i2c_set_clientdata(client, led_array);

#if !defined(LED_AW2013_OEM_PATCH)
	ret = aw2013_power_init(led_array, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto fail_parsed_node;
	}
#endif

#if defined(CONFIG_KRYPTON)
	i2c_check_status_create(temp_string, 1);
#endif

	return 0;

#if !defined(LED_AW2013_OEM_PATCH)
fail_parsed_node:
	aw2013_led_err_handle(led_array, num_leds);
#endif

free_led_arry:

#if defined(CONFIG_KRYPTON)
	if (led_array->wq) {
		destroy_workqueue(led_array->wq);
		led_array->wq = NULL;
	}

	if(++probe_retries < AW2013_MAX_RETRY) {
		pr_err("aw2013 probe failed with ret:%d at times:%d  \n", 
				ret, probe_retries);
		ret = -EPROBE_DEFER;
	} else {
		i2c_check_status_create(temp_string, 0);
	}
#endif

#if defined(CONFIG_KRYPTON)
notry:
#endif
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return ret;
}


#if defined(CONFIG_KRYPTON)
static int aw2013_led_remove(struct i2c_client *client)
{
	struct aw2013_led *led_array = i2c_get_clientdata(client);
	int i, parsed_leds = led_array->num_leds;

	if(led_array->wq) {
		destroy_workqueue(led_array->wq);
		led_array->wq = NULL;
	}

	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);

		devm_kfree(&client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return 0;
}
#else
static int aw2013_led_remove(struct i2c_client *client)
{
	struct aw2013_led *led_array = i2c_get_clientdata(client);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return 0;
}
#endif


static const struct i2c_device_id aw2013_led_id[] = {
	{"aw2013_led", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, aw2013_led_id);

static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "awinic,aw2013",},
	{ },
};

static struct i2c_driver aw2013_led_driver = {
	.probe = aw2013_led_probe,
	.remove = aw2013_led_remove,
	.driver = {
		.name = "aw2013_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw2013_match_table),
	},
	.id_table = aw2013_led_id,
};

static int __init aw2013_led_init(void)
{
	return i2c_add_driver(&aw2013_led_driver);
}
module_init(aw2013_led_init);

static void __exit aw2013_led_exit(void)
{
	i2c_del_driver(&aw2013_led_driver);
}
module_exit(aw2013_led_exit);

MODULE_DESCRIPTION("AWINIC aw2013 LED driver");
MODULE_LICENSE("GPL v2");
