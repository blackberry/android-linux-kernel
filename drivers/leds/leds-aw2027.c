/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#include <linux/leds-aw2027.h>

extern int i2c_check_status_create(char *name,int value);

/* register address */
#define AW2027_REG_RESET			0x00
#define AW2027_REG_GCR1				0x01
#define AW2027_REG_STATUS			0x02
#define AW2027_REG_PATST			0x03
#define AW2027_REG_GCR2				0x04
#define AW2027_REG_LEDEN			0x30
#define AW2027_REG_LCFG1			0x31
#define AW2027_REG_LCFG2			0x32
#define AW2027_REG_LCFG3			0x33
#define AW2027_REG_PWM1				0x34
#define AW2027_REG_PWM2				0x35
#define AW2027_REG_PWM3				0x36
#define AW2027_REG_LED1T0			0x37
#define AW2027_REG_LED1T1			0x38
#define AW2027_REG_LED1T2			0x39
#define AW2027_REG_LED2T0			0x3A
#define AW2027_REG_LED2T1			0x3B
#define AW2027_REG_LED2T2			0x3C
#define AW2027_REG_LED3T0			0x3D
#define AW2027_REG_LED3T1			0x3E
#define AW2027_REG_LED3T2			0x3F

/* register bits */
#define AW2027_CHIPID					0x09
#define AW2027_RESET_MASK				0x55
#define AW2027_CHIP_DISABLE_MASK		0x00
#define AW2027_CHIP_ENABLE_MASK			0x01
#define AW2027_CHARGE_DISABLE_MASK		0x02
#define AW2027_LED_BREATH_MODE_MASK 	0x10
#define AW2027_LED_MANUAL_MODE_MASK		0x00
#define AW2027_LED_BREATHE_PWM_MASK		0xFF
#define AW2027_LED_MANUAL_PWM_MASK		0xFF
#define AW2027_LED_FADEIN_MODE_MASK		0x20
#define AW2027_LED_FADEOUT_MODE_MASK	0x40

#define MAX_RISE_TIME_MS				15
#define MAX_HOLD_TIME_MS				15
#define MAX_FALL_TIME_MS				15
#define MAX_OFF_TIME_MS					15

/* aw2027 register read/write access*/
#define REG_NONE_ACCESS					0
#define REG_RD_ACCESS					1 << 0
#define REG_WR_ACCESS					1 << 1
#define AW2027_REG_MAX					0x7F

#define AW2027_VDD_MIN_UV		2600000
#define AW2027_VDD_MAX_UV		3300000
#define AW2027_VI2C_MIN_UV		1800000
#define AW2027_VI2C_MAX_UV		1800000


const unsigned char aw2027_reg_access[AW2027_REG_MAX] = {
	[AW2027_REG_RESET]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_GCR1]   = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_STATUS] = REG_RD_ACCESS,
	[AW2027_REG_PATST]  = REG_RD_ACCESS,
	[AW2027_REG_GCR2]   = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LEDEN]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LCFG1]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LCFG2]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LCFG3]  = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_PWM1]   = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_PWM2]   = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_PWM3]   = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED1T0] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED1T1] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED1T2] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED2T0] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED2T1] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED2T2] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED3T0] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED3T1] = REG_RD_ACCESS|REG_WR_ACCESS,
	[AW2027_REG_LED3T2] = REG_RD_ACCESS|REG_WR_ACCESS,
};
struct aw2027_led {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct aw2027_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex lock;
	int num_leds;
	int id;
	struct regulator *vdd;
	bool poweron;
};

static int aw2027_write(struct aw2027_led *led, u8 reg, u8 val)
{
	int ret = -EINVAL, retry_times = 0;

	do {
		ret = i2c_smbus_write_byte_data(led->client, reg, val);
		retry_times ++;
		if(retry_times == 5)
			break;
	}while (ret < 0);
	
	return ret;	
}

static int aw2027_read(struct aw2027_led *led, u8 reg, u8 *val)
{
	int ret = -EINVAL, retry_times = 0;

	do{
		ret = i2c_smbus_read_byte_data(led->client, reg);
		retry_times ++;
		if(retry_times == 5)
			break;
	}while (ret < 0);

	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int aw2027_power_on(struct aw2027_led *led, bool on)
{
	int rc;

	if (on) {
		rc = regulator_enable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		led->poweron = true;

		mdelay(10);
	} else {
		rc = regulator_disable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		led->poweron = false;

		mdelay(10);
	}

	return rc;
}

static int aw2027_power_init(struct aw2027_led *led, bool on)
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
			rc = regulator_set_voltage(led->vdd, AW2027_VDD_MIN_UV,
						   AW2027_VDD_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}
	} else {
		if (regulator_count_voltages(led->vdd) > 0)
			regulator_set_voltage(led->vdd, 0, AW2027_VDD_MAX_UV);

		regulator_put(led->vdd);
	}
	return 0;

reg_vdd_put:
	regulator_put(led->vdd);
	return rc;
}

static void aw2027_brightness_work(struct work_struct *work)
{
	struct aw2027_led *led = container_of(work, struct aw2027_led,
					brightness_work);
	u8 val, i, j;

	mutex_lock(&led->pdata->led->lock);

	/* enable aw2027 if disabled */
	aw2027_read(led, AW2027_REG_GCR1, &val);
	if (!(val&AW2027_CHIP_ENABLE_MASK)) {
		aw2027_write(led, AW2027_REG_GCR1, AW2027_CHARGE_DISABLE_MASK | AW2027_CHIP_ENABLE_MASK);
		msleep(2);
	}

	if (led->cdev.brightness > 0) {
		if (led->cdev.brightness > led->cdev.max_brightness)
			led->cdev.brightness = led->cdev.max_brightness;
		aw2027_write(led, AW2027_REG_GCR2, led->pdata->imax);
		for (j=0,i=led->id; i; i >>= 1, j++) {
			if (i & 0x01) {
				aw2027_write(led, AW2027_REG_LCFG1 + j, 
							(AW2027_LED_MANUAL_MODE_MASK | led->pdata->led_current));
				aw2027_write(led, AW2027_REG_PWM1 + j, led->cdev.brightness);
				aw2027_read(led, AW2027_REG_LEDEN, &val);
				//printk("%s111 i=%d j=%d val=%x \n", __func__, i, j, val | (1 << j));
				aw2027_write(led, AW2027_REG_LEDEN, val | (1 << j));
			}
		}
	} else {
		for (j=0,i=led->id; i; i >>= 1, j++) {
			if (i & 0x01) {
				aw2027_read(led, AW2027_REG_LEDEN, &val);
				//printk("%s222 i=%d j=%d val=%x \n", __func__, i, j, val & (~(1 << j)));
				aw2027_write(led, AW2027_REG_LEDEN, val & (~(1 << j)));
			}
		}
	}

	/*
	 * If value in AW2027_REG_LEDEN is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	aw2027_read(led, AW2027_REG_LEDEN, &val);
	if (val == 0) {
		aw2027_write(led, AW2027_REG_GCR1, AW2027_CHARGE_DISABLE_MASK | AW2027_CHIP_DISABLE_MASK);
		mutex_unlock(&led->pdata->led->lock);
		return;
	}

	mutex_unlock(&led->pdata->led->lock);
}

#if 0
static void aw2027_led_blink_set(struct aw2027_led *led, unsigned long blinking)
{
	u8 val;

	/* enable aw2027 if disabled */
	aw2027_read(led, AW2027_REG_GCR1, &val);
	if (!(val&AW2027_CHIP_ENABLE_MASK)) {
		aw2027_write(led, AW2027_REG_GCR1, AW2027_CHARGE_DISABLE_MASK | AW2027_CHIP_ENABLE_MASK);
		msleep(2);
	}

	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
	
	if (blinking > 0) {
		aw2027_write(led, AW2027_REG_GCR2, led->pdata->imax);
		aw2027_write(led, AW2027_REG_LCFG1 + led->id, 
					(AW2027_LED_BREATH_MODE_MASK | led->pdata->led_current));
		aw2027_write(led, AW2027_REG_PWM1 + led->id, led->cdev.brightness);
		aw2027_write(led, AW2027_REG_LED1T0 + led->id*3, 
					(led->pdata->rise_time_ms << 4 | led->pdata->hold_time_ms));
		aw2027_write(led, AW2027_REG_LED1T1 + led->id*3, 
					(led->pdata->fall_time_ms << 4 | led->pdata->off_time_ms));
		aw2027_read(led, AW2027_REG_LEDEN, &val);
		aw2027_write(led, AW2027_REG_LEDEN, val | (1 << led->id));
	} else {
		aw2027_read(led, AW2027_REG_LEDEN, &val);
		aw2027_write(led, AW2027_REG_LEDEN, val & (~(1 << led->id)));
	}

	/*
	 * If value in AW2027_REG_LEDEN is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	aw2027_read(led, AW2027_REG_LEDEN, &val);
	if (val == 0) {
		aw2027_write(led, AW2027_REG_GCR1, AW2027_CHARGE_DISABLE_MASK | AW2027_CHIP_DISABLE_MASK);
	}
}
#endif

static void aw2027_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct aw2027_led *led = container_of(cdev, struct aw2027_led, cdev);

	led->cdev.brightness = brightness;

	schedule_work(&led->brightness_work);
}

#if 0
static ssize_t aw2027_store_blink(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2027_led *led =
			container_of(led_cdev, struct aw2027_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	mutex_lock(&led->pdata->led->lock);
	aw2027_led_blink_set(led, blinking);
	mutex_unlock(&led->pdata->led->lock);

	return len;
}

static ssize_t aw2027_led_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2027_led *led =
			container_of(led_cdev, struct aw2027_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			led->pdata->rise_time_ms, led->pdata->hold_time_ms,
			led->pdata->fall_time_ms, led->pdata->off_time_ms);
}

static ssize_t aw2027_led_time_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2027_led *led =
			container_of(led_cdev, struct aw2027_led, cdev);
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
	aw2027_led_blink_set(led, 1);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}
#endif

static ssize_t aw2027_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2027_led *led =
			container_of(led_cdev, struct aw2027_led, cdev);

	unsigned char i, reg_val;
	ssize_t len = 0;

	for(i=0; i<AW2027_REG_MAX; i++) {
		if(!(aw2027_reg_access[i]&REG_RD_ACCESS))
		continue;
		aw2027_read(led, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t aw2027_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2027_led *led =
			container_of(led_cdev, struct aw2027_led, cdev);

	unsigned int databuf[2];

	if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
	{
		aw2027_write(led, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return len;
}
#if 0
static DEVICE_ATTR(blink, 0664, NULL, aw2027_store_blink);
static DEVICE_ATTR(led_time, 0664, aw2027_led_time_show, aw2027_led_time_store);
#endif
static DEVICE_ATTR(reg, 0664, aw2027_reg_show, aw2027_reg_store);

static struct attribute *aw2027_led_attributes[] = {
	//&dev_attr_blink.attr,
	//&dev_attr_led_time.attr,
	&dev_attr_reg.attr,
	NULL,
};

static struct attribute_group aw2027_led_attr_group = {
	.attrs = aw2027_led_attributes
};
static int aw2027_check_chipid(struct aw2027_led *led)
{
	u8 val;
	u8 cnt;

	for(cnt = 5; cnt > 0; cnt --)
	{
		aw2027_read(led, AW2027_REG_RESET, &val);
		dev_notice(&led->client->dev,"aw2027 chip id %0x",val);
		if (val == AW2027_CHIPID)
			return 0;
	}
	
	return -EINVAL;
}

static int aw2027_led_err_handle(struct aw2027_led *led_array,
				int parsed_leds)
{
	int i;
	/*
	 * If probe fails, cannot free resource of all LEDs, only free
	 * resources of LEDs which have allocated these resource really.
	 */
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2027_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}

static int aw2027_led_parse_child_node(struct aw2027_led *led_array,
				struct device_node *node)
{
	struct aw2027_led *led;
	struct device_node *temp;
	struct aw2027_platform_data *pdata;
	int rc = 0, parsed_leds = 0;

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->client = led_array->client;

		pdata = devm_kzalloc(&led->client->dev,
				sizeof(struct aw2027_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&led->client->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = led_array;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "aw2027,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,id",
			&led->id);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,imax",
			&led->pdata->imax);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,led-current",
			&led->pdata->led_current);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading led-current, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2027,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		INIT_WORK(&led->brightness_work, aw2027_brightness_work);

		led->cdev.brightness_set = aw2027_set_brightness;

		rc = led_classdev_register(&led->client->dev, &led->cdev);
		if (rc) {
			dev_err(&led->client->dev,
				"unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_pdata;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw2027_led_attr_group);
		if (rc) {
			dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		parsed_leds++;
	}

	return 0;

free_class:
	aw2027_led_err_handle(led_array, parsed_leds);
	led_classdev_unregister(&led_array[parsed_leds].cdev);
	cancel_work_sync(&led_array[parsed_leds].brightness_work);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	led_array[parsed_leds].pdata = NULL;
	return rc;

free_pdata:
	aw2027_led_err_handle(led_array, parsed_leds);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	return rc;

free_err:
	aw2027_led_err_handle(led_array, parsed_leds);
	return rc;
}

static int aw2027_led_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct aw2027_led *led_array;
	struct device_node *node;
	int ret = -EINVAL, num_leds = 0;
	node = client->dev.of_node;
	if (node == NULL)
		return -EINVAL;

	num_leds = of_get_child_count(node);

	if (!num_leds)
		return -EINVAL;

	led_array = devm_kzalloc(&client->dev,
			(sizeof(struct aw2027_led) * num_leds), GFP_KERNEL);
	if (!led_array)
		return -ENOMEM;

	led_array->client = client;
	led_array->num_leds = num_leds;

	mutex_init(&led_array->lock);

	ret = aw2027_power_init(led_array, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto free_led_arry;
	}

	if (aw2027_power_on(led_array, true)) {
		dev_err(&client->dev, "power on failed");
		goto free_led_arry;
	}

	ret = aw2027_led_parse_child_node(led_array, node);
	if (ret) {
		dev_err(&client->dev, "parsed node error\n");
		goto free_led_arry;
	}

	i2c_set_clientdata(client, led_array);

	ret = aw2027_check_chipid(led_array);
	if (ret) {
		i2c_check_status_create("keypad_led", 0);
		dev_err(&client->dev, "Check chip id error\n");
		goto fail_parsed_node;
	}
	i2c_check_status_create("keypad_led", 1);

	return 0;

fail_parsed_node:
	aw2027_led_err_handle(led_array, num_leds);
free_led_arry:
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return ret;
}

static int aw2027_led_remove(struct i2c_client *client)
{
	struct aw2027_led *led_array = i2c_get_clientdata(client);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2027_led_attr_group);
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

static const struct i2c_device_id aw2027_led_id[] = {
	{"aw2027_led", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, aw2027_led_id);

static struct of_device_id aw2027_match_table[] = {
	{ .compatible = "awinic,aw2027_led",},
	{ },
};


static struct i2c_driver aw2027_led_driver = {
	.probe = aw2027_led_probe,
	.remove = aw2027_led_remove,
	.driver = {
		.name = "aw2027_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw2027_match_table),
	},
	.id_table = aw2027_led_id,
};

static int __init aw2027_led_init(void)
{
	return i2c_add_driver(&aw2027_led_driver);
}
module_init(aw2027_led_init);

static void __exit aw2027_led_exit(void)
{
	i2c_del_driver(&aw2027_led_driver);
}
module_exit(aw2027_led_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW2027 LED driver");
MODULE_LICENSE("GPL v2");
