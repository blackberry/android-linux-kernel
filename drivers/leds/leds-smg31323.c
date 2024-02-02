/*
 * Copyright (C) 2016 BlackBerry Limited
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

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include "leds.h"

//#define FAKE_I2C
#define DEBUG(name, format, ...) pr_err("smg31323 %s:" format "\n", (name), ## __VA_ARGS__);
#define INFO(name, format, ...) pr_info("smg31323 %s:" format "\n", (name), ## __VA_ARGS__);

#define SGM31323_CH1      0x06
#define SGM31323_CH2      0x07
#define SGM31323_CH3      0x08

#define SGM31323_CONTROL  0x04

static const u8 SGM31323_CH[] = { SGM31323_CH1, SGM31323_CH2, SGM31323_CH3 };

extern int i2c_check_status_create(char *name,int value);

/* [FETURE]-Add- by TCTNB.XQJ, RR-3902379, 2017/1/11, add led ic ktd2026 for mercury*/
//extern int get_board_version(void);
#define BOARD_PIO04 4
#define BOARD_PIO05 5
#define BAORD_OTHERS 0Xff
/* [BUGFIX]-End- by TCLNB.XQJ*/

#define SMG31323_NAME "leds-smg31323"

struct smg31323_platform_data {
	struct i2c_client       *client;
	const char              *name;
	u8                      control_val;
	struct workqueue_struct *workqueue;   // One work queue for all the leds under the driver
	int                     num_leds;
	bool                    is_blinking;
	bool                    blink_on;
	unsigned long           delay_on;
	unsigned long           delay_off;
	int                     en_gpio;
	struct regulator        *vdd_vreg;
	bool                    is_powered;
	struct delayed_work     blink_work;
};

struct smg31323_led_data {
	struct led_classdev     led_dev;
	struct i2c_client       *client;
	struct smg31323_platform_data *pdata;
	u8                      led_mask;
	bool                    use_blink;
	bool                    is_blinking;
	int                     current_brightness;
	struct work_struct      brightness_set_work;
};

#if 0
static ssize_t blink_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count);


static DEVICE_ATTR(blink, 0664, NULL, blink_store);

static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};


static void update_power_state(struct smg31323_led_data *led)
{
	struct smg31323_platform_data *pdata;
	struct smg31323_led_data *led_array = i2c_get_clientdata(led->client);
	int rc, i, num_leds = led->pdata->num_leds;
	bool power_required = false;
	pdata = led->pdata;

	for (i=0; i<num_leds; i++){
		if (led_array[i].led_dev.brightness != LED_OFF) {
			power_required = true;
			break;
		}
	}
	if (power_required && !pdata->is_powered) {
		INFO(pdata->name, "Power on");
		if (pdata->vdd_vreg) {
			rc = regulator_enable(pdata->vdd_vreg);
			if (rc) {
				INFO(pdata->name, "vdd_vreg enable failed rc=%d", rc);
			}
		}

		rc = gpio_direction_output(led->pdata->en_gpio, 1);
		if (rc < 0) {
			INFO(pdata->name, "Failed to gpio_direction_output GPIO %d", rc);
		}
		pdata->control_val = SMG31323_CONTROL_DEFAULT;
	} else if (!power_required && led->pdata->is_powered) {
		INFO(pdata->name, "Power off");
		if (pdata->vdd_vreg) {
			rc = regulator_disable(pdata->vdd_vreg);
			if (rc) {
				INFO(pdata->name, "vdd_vreg enable failed rc=%d", rc);
			}
		}

		rc = gpio_direction_output(led->pdata->en_gpio, 0);
		if (rc < 0) {
			INFO(pdata->name, "Failed to gpio_direction_output GPIO %d", rc);
		}

		for (i=0; i<num_leds; i++) {
			led_array[i].current_brightness = -1; // force brightness register update when we turn back on
		}
	}
	pdata->is_powered = power_required;
}
#endif
// Actual work to set the brightness
static void brightness_set_work(struct work_struct *work)
{
	struct smg31323_platform_data *pdata;
	struct smg31323_led_data *led;
	int i, brt_val;
	u8 enable, val;

	led = container_of(work, struct smg31323_led_data, brightness_set_work);
	pdata = led->pdata;
	brt_val = led->led_dev.brightness;

#if 0
	update_power_state(led);
	if(!pdata->is_powered){
		//don't bother with i2c writes after we've powered off the chip.
		return;
	}
#endif
	if(brt_val == 0){
		enable = 0;
	}else{
		enable = led->led_mask;
	}
	// Update the brightness first
	if(enable && led->current_brightness != brt_val){
		// Only update brightness if non zero
		led->current_brightness = brt_val;
		for(val=0,i=led->led_mask; i; i >>= 2, val++){
			if(i & 0x01){
				#ifndef FAKE_I2C
				if (i2c_smbus_write_byte_data(led->client, SGM31323_CH[val], brt_val) < 0)
						INFO(pdata->name, "Failed to write %02X to %02X",SGM31323_CH[val], brt_val);
				#else
				INFO(pdata->name, "Setting %02X to %02X",SGM31323_CH[val], brt_val);
				#endif
			}
		}
	}
	// Then change the enable bit (if needed)
	val = pdata->control_val & ~led->led_mask;
	if(enable)
		val |= led->led_mask;
	if (pdata->control_val != val && led->is_blinking == false) {
		// No need to update this if its blinking. blink_work will take care of it
		#ifndef FAKE_I2C
		if (i2c_smbus_write_byte_data(led->client, SGM31323_CONTROL, val) < 0)
				INFO(pdata->name, "Failed to write %02X to %02X",SGM31323_CONTROL, val);
		#else
		INFO(pdata->name, "Setting %02X to %02X",SGM31323_CONTROL, val);
		#endif
		pdata->control_val = val;
	}
}

#if 0
// Periodic blink work to keep all bliking leds in sync
static void blink_work(struct work_struct *awork)
{
	struct delayed_work *work = to_delayed_work(awork);
	struct smg31323_platform_data *pdata = container_of(work, struct smg31323_platform_data, blink_work);
	struct smg31323_led_data *led_array;
	int i, num_leds;
	u8 control;

	if(pdata->is_blinking == false)
		return;   // We aren't blinking anymore

	// Enable/Disable all blinking ones at once to keep them in sync
	led_array = i2c_get_clientdata(pdata->client);
	num_leds = pdata->num_leds;
	control = pdata->control_val;
	for(i=0; i<num_leds; i++){
		struct smg31323_led_data *led=&led_array[i];

		if(led->is_blinking){
			control &= ~led->led_mask;
			if(pdata->blink_on)
				control |= led->led_mask;
		}
	}

	if(pdata->control_val != control){
		#ifndef FAKE_I2C
		if (i2c_smbus_write_byte_data(pdata->client, SMG31323_CONTROL, control) < 0) {
				INFO(pdata->name, "Failed to write %02X to %02X",SMG31323_CONTROL, control);
		} else {
				DEBUG(pdata->name, "Setting %02X to %02X",SMG31323_CONTROL, control);
		}
		#else
		INFO(pdata->name, "Setting %02X to %02X",SMG31323_CONTROL, control);
		#endif
		pdata->control_val = control;
	}
	queue_delayed_work(pdata->workqueue, &pdata->blink_work, msecs_to_jiffies( (pdata->blink_on) ? pdata->delay_on : pdata->delay_off ));
	pdata->blink_on = !pdata->blink_on;
}

// If none of the led is blinking, stop the blink work as well
static void stop_blinking(struct smg31323_led_data *led)
{
	struct smg31323_led_data *led_array;
	int i, num_leds;

	led_array=i2c_get_clientdata(led->client);
	num_leds = led->pdata->num_leds;
	// Stop blinking
	led->is_blinking = false;
	// If this is the last one blinking, disable blink delayed work as well
	for(i=0; i<num_leds; i++){
		if(led_array[i].is_blinking) break;
	}
	if(i>=num_leds){
		// Loop exchausted, no led is blinking
		led->pdata->is_blinking = false;
		cancel_delayed_work_sync(&led_array->pdata->blink_work);
	}
}
#endif

// Called by led class driver in response of brightess API or write to brightness sysfs entry
static void smg31323_brightness_set(struct led_classdev *led_cdev,
						 enum led_brightness brt_val)
{
	struct smg31323_led_data *led;

	led = container_of(led_cdev, struct smg31323_led_data, led_dev);
	//INFO(led->pdata->name, "Set %s to %d", led_cdev->name, led_cdev->brightness);
#if 0
	if(led->is_blinking && brt_val == LED_OFF){
		stop_blinking(led);
	}
#endif
	queue_work(led->pdata->workqueue, &led->brightness_set_work);
}

#if 0
// Called by sysfs for blink entry
static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct smg31323_led_data *led;
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct smg31323_platform_data *pdata;
	ssize_t ret = -EINVAL;
	led = container_of(led_cdev, struct smg31323_led_data, led_dev);
	pdata = led->pdata;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	// Stock liblight writes 0, 1 or 2 to the blink. Currently we only support 0 or non-zero
	if(led->is_blinking && blinking == 0){
		INFO(pdata->name, "Stopping blink for %s", led->led_dev.name);
		stop_blinking(led);
		// Back to what it was based on current brightness
		smg31323_brightness_set(&led->led_dev, led->led_dev.brightness);
	}else if(led->led_dev.brightness != LED_OFF){
		led->is_blinking = true;;
		if(pdata->is_blinking){
			// Already blinking, nothing to do
			DEBUG(pdata->name, "Already blinking, nothing to do for %s", led->led_dev.name);
		}else{
			pdata->is_blinking = true;
			pdata->blink_on = true;   // First tick is to turn on
			INFO(pdata->name, "Starting blink for %s", led->led_dev.name);
			queue_delayed_work(pdata->workqueue, &pdata->blink_work, msecs_to_jiffies(1));  // Start blinking in ~ a msec
		}
	}
	return count;
}
#endif
static int smg31323_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc = 0, num_leds = 0, parsed_leds = 0, i;
	u32 temp_u32;
	struct smg31323_led_data *led, *led_array;
	struct device_node *node, *temp;
	struct smg31323_platform_data  *pdata;
	bool use_blink=false;

	node = client->dev.of_node;
	if (node == NULL) {
		INFO("","Probe error: null of node!");
		return -ENODEV;
	}

	temp = NULL;
	// Figure out the number of led class we support, so we can allocate buffers accordingly
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds) {
		INFO("", "Probe error: no child leds!");
		return -ECHILD;
	}

	led_array = devm_kzalloc(&client->dev,
		(sizeof(struct smg31323_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		INFO("", "Probe error: Unable to allocate memory for led_array");
		return -ENOMEM;
	}
	pdata = devm_kzalloc(&client->dev,
		sizeof(struct smg31323_platform_data), GFP_KERNEL);
	if (!pdata) {
		INFO("", "Probe error: Unable to allocate memory for pdata");
		return -ENOMEM;
	}

	rc = of_property_read_string(node, "linux,name", &pdata->name);
	if (rc < 0) {
		INFO("", "Probe error:Failure reading led name, rc = %d", rc);
		goto fail_node;
	}
	INFO(pdata->name, "Probing");

	// Allocate ordered workqueue, to keep code simpler
	pdata->workqueue = alloc_ordered_workqueue
					("smg31323_workqueue", 0);
	if (!pdata->workqueue) {
		INFO(pdata->name, "Probe error: alloc_ordered_workqueue failed");
		return -ENOMEM;
	}

	//INIT_DELAYED_WORK(&pdata->blink_work, blink_work);
	pdata->client = client;
	pdata->num_leds = num_leds;

#if 0
	pdata->en_gpio = of_get_named_gpio(node, "en-gpio", 0);
	if (!gpio_is_valid(pdata->en_gpio)) {
		INFO(pdata->name, "Probe error: en-gpio GPIO is invalid");
		goto fail_node;
	}

	rc = gpio_request(pdata->en_gpio, "smg31323-en-gpio");
	if (rc < 0) {
			INFO(pdata->name, "Probe error: Failed to request en-gpio %d, error %d", pdata->en_gpio, rc);
			goto fail_node;
	}

	i2c_en_gpio = of_get_named_gpio(node, "i2c-en-gpio", 0);
	if (!gpio_is_valid(i2c_en_gpio)) {
		DEBUG(pdata->name, "en-gpio2 GPIO not used");
	} else {
		/* don't call gpio_request, because we are not the sole owners of this GPIO. Just set it high and assume nobody will turn it off. */
		rc = gpio_direction_output(i2c_en_gpio, 1);
		if (rc < 0) {
			INFO(pdata->name, "Failed to gpio_direction_output %d", rc);
		}
	}
#endif
	pdata->vdd_vreg = regulator_get(&client->dev, "vdd");
	if (IS_ERR(pdata->vdd_vreg)) {
		rc = PTR_ERR(pdata->vdd_vreg);
		DEBUG(pdata->name, "Regulator not used. rc=%d", rc);
		pdata->vdd_vreg = NULL;
	} else {
		rc = regulator_set_voltage(pdata->vdd_vreg, 2800000, 2800000);
		if (rc) {
			INFO(pdata->name, "Regulator set_vtg failed vdd rc=%d", rc);
			pdata->vdd_vreg = NULL;
		}
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->client = client;
		led->pdata = pdata;
		led->led_dev.brightness_set = smg31323_brightness_set;

		rc = of_property_read_string(temp, "linux,name",
			&led->led_dev.name);
		if (rc < 0) {
			INFO(pdata->name, "Failure reading led name, rc = %d", rc);
			goto fail_node;
		}
#if 0
/* [FETURE]-Add- by TCTNB.XQJ, RR-3902379, 2017/1/11, add led ic ktd2026 for mercury*/
		if(get_board_version()==BOARD_PIO05) {//if  use new hardware,RGB will not use smg31323,so let skip to crate RGB node  avoiding conlict with new led ic
           if(strcmp(pdata->name,"nav+rgb")==0)
           {
               if(strcmp(led->led_dev.name,"red")==0 || strcmp(led->led_dev.name,"blue")==0 || strcmp(led->led_dev.name,"green")==0)
               {
                 if(parsed_leds>0)
	                parsed_leds--;
                 continue;
               }
           }
        }
/* [BUGFIX]-End- by TCLNB.XQJ*/
#endif
		rc = of_property_read_u8(temp, "led,mask",
			&led->led_mask);
		if (rc < 0) {
			INFO(pdata->name, "Failure reading led mask, rc = %d", rc);
			goto fail_node;
		}

		rc = of_property_read_u32(temp, "led,max_brightness",
			&temp_u32);
		if (rc < 0) {
			INFO(pdata->name, "Failure reading led max_brightness, rc = %d", rc);
			goto fail_node;
		}
		led->led_dev.max_brightness = temp_u32;

		led->use_blink = of_property_read_bool(temp, "led,use-blink");
		if(led->use_blink){
			use_blink = true;
		}

		DEBUG(pdata->name, "Registering %s, brightness=%d, mask=%d", led->led_dev.name, led->led_dev.max_brightness, led->led_mask);

		INIT_WORK(&led->brightness_set_work, brightness_set_work);

		rc = led_classdev_register(&led->client->dev, &led->led_dev);
		if (rc) {
			INFO(pdata->name, "unable to register led %s,rc=%d", led->led_dev.name, rc);
			goto fail_node;
		}
	#if 0
		if (led->use_blink) {
			rc = sysfs_create_group(&led->led_dev.dev->kobj,
				&blink_attr_group);
			if (rc)
				goto fail_node;
			}
	#endif
		parsed_leds++;
	}
	// Read duty cycle paramanets if at least one led supports blinking
	if(use_blink){
		rc = of_property_read_u32(node, "led,delay_on",
			&temp_u32);
		if (rc < 0) {
			INFO(pdata->name, "Failure reading led delay_on, rc = %d", rc);
			goto fail_node;
		}
		pdata->delay_on = temp_u32;
		rc = of_property_read_u32(node, "led,delay_off",
			&temp_u32);
		if (rc < 0) {
			INFO(pdata->name, "Failure reading led delay_off, rc = %d", rc);
			goto fail_node;
		}
		pdata->delay_off = temp_u32;
	}

	i2c_set_clientdata(client, led_array);
/* [BUGFIX]-Add- by TCLNB.XQJ,bug 3379224,11/07,add checking i2c status*/
#if 1//def CONFIG_BBRY
#ifndef FAKE_I2C
    if (pdata->vdd_vreg) {
        rc = regulator_enable(pdata->vdd_vreg);
        if (rc) {
                INFO(pdata->name, "vdd_vreg enable failed rc=%d", rc);
		}
    }
#if 0
        else
        {
            gpio_direction_output(pdata->en_gpio, 1);
            rc=i2c_smbus_read_byte_data(pdata->client, SMG31323_CONTROL);
            if (rc < 0) {
                /* MODIFIED-BEGIN by hongwei.tian, 2017-01-14,BUG-3989478*/
                i2c_check_status_create("keypad_led",0);
		  /* MODIFIED-BEGIN by hongwei.tian, 2017-01-17,BUG-4013973*/
		  if(get_board_version()!=BOARD_PIO05)
			i2c_check_status_create("indicator_led",0);
                 INFO(pdata->name, "Failed to read %02X ",SMG31323_CONTROL);
            } else {
                i2c_check_status_create("keypad_led",1);
		  if(get_board_version()!=BOARD_PIO05)
		  /* MODIFIED-END by hongwei.tian,BUG-4013973*/
			i2c_check_status_create("indicator_led",1);
            }
             gpio_direction_output(pdata->en_gpio, 0);
         }
         rc = regulator_disable(pdata->vdd_vreg);
     }
#endif
	rc = i2c_smbus_read_byte_data(pdata->client, SGM31323_CONTROL);
	if (rc < 0) {
		DEBUG(pdata->name, "Failure reading SGM31323_CONTROL, rc = %d", rc);
		if (!strcmp("nav-led", pdata->name))
			i2c_check_status_create("navkey_led", 0);
		goto fail_node;
	}
	if (!strcmp("nav-led", pdata->name))
		i2c_check_status_create("navkey_led", 1);
#else
     i2c_check_status_create("keypad_led",1)
     if(get_board_version()!=BOARD_PIO05 ) // MODIFIED by hongwei.tian, 2017-01-17,BUG-4013973
	i2c_check_status_create("indicator_led",1);
	/* MODIFIED-END by hongwei.tian,BUG-3989478*/

     INFO(pdata->name, "fake read i2c %02X",SMG31323_CONTROL);
#endif
#endif
/* [BUGFIX]-End- by TCLNB.XQJ*/
	return 0;

fail_node:
	for (i = 0; i < parsed_leds; i++) {
		led_classdev_unregister(&led_array[i].led_dev);
	}
	destroy_workqueue(pdata->workqueue);

	return rc;
}

static int smg31323_remove(struct i2c_client *client)
{
	struct smg31323_led_data *led_array = i2c_get_clientdata(client);
	struct smg31323_platform_data  *pdata=led_array->pdata;
	int i, num_leds = pdata->num_leds;

	for (i = 0; i < num_leds; i++) {
		led_classdev_unregister(&led_array[i].led_dev);
	}
	destroy_workqueue(pdata->workqueue);

	return 0;
}


static const struct i2c_device_id smg31323_id[] = {
	{SMG31323_NAME, 0},    // This dummy id. Class driver will update it from dts
	{}
};


static struct of_device_id smg31323_match_table[] = {
	{.compatible = "qcom,leds-smg31323",},
	{ },
};

static struct i2c_driver smg31323_i2c_driver = {
	.probe = smg31323_probe,
	.remove = smg31323_remove,
	.id_table = smg31323_id,
	.driver = {
		.name = SMG31323_NAME,
		.owner = THIS_MODULE,
		.of_match_table = smg31323_match_table,
	},
};

module_i2c_driver(smg31323_i2c_driver);

MODULE_DESCRIPTION("SMG31323 LED driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BlackBerry Limited");
