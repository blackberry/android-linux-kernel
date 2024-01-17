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

 * [FETURE]-Add- by TCTNB.XQJ, RR-3902379, 2017/1/5, add led ic ktd2026 for mercury

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
#define DEBUG(name, format, ...) pr_debug("ktd2026 %s:" format "\n", (name), ## __VA_ARGS__);
#define INFO(name, format, ...) pr_info("ktd2026 %s:" format "\n", (name), ## __VA_ARGS__);

#define ktd2026_SLOT   0x00
#define ktd2026_CONTROL   0x04
//#define ktd2026_CONFIG    0x20
#define ktd2026_RED       0x06
/* MODIFIED-BEGIN by hongwei.tian, 2017-02-22,BUG-4255998*/
#define ktd2026_BLUE  0x08
#define ktd2026_GREEN  0x07
#define ktd2026_PERIOD      0x01
#define ktd2026_FLASHON     0x02
#define ktd2026_FLASHON2     0x03

#define ktd2026_CONTROL_DEFAULT     0x00

static const u8 ktd2026_CH[] = { ktd2026_RED,ktd2026_GREEN,ktd2026_BLUE};
/* MODIFIED-END by hongwei.tian,BUG-4255998*/
#ifdef CONFIG_BBRY  /* [BUGFIX]-Add- by TCLNB.XQJ,bug 3379224,11/07,add i2c status  */
extern int i2c_check_status_create(char *name,int value);
#endif


#define ktd2026_NAME "leds-ktd2026"

struct ktd2026_platform_data {
    struct i2c_client       *client;
    const char              *name;
    u8                      control_val;
    struct workqueue_struct *workqueue;   // One work queue for all the leds under the driver
    int                     num_leds;
    bool                    is_blinking;
    bool                    blink_on;

    int                     en_gpio;
    struct regulator        *i2c_vreg;
    bool                    is_powered;
};

struct ktd2026_led_data {
    struct led_classdev     led_dev;
    struct i2c_client       *client;
    struct ktd2026_platform_data *pdata;
    u8                      led_mask;
    bool                    use_blink;
    u8           flash_ontime;
    u8           flash_period;
    bool                    is_blinking;
    int                     current_brightness;
    struct work_struct      brightness_set_work;
};

static ssize_t blink_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t flashperiod_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t flashon_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count);
/* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
static ssize_t blink_show(struct device *dev, struct device_attribute *attr, char *buf);


static DEVICE_ATTR(blink, 0664, blink_show, blink_store);
/* MODIFIED-END by hongwei.tian,BUG-4255998*/
static DEVICE_ATTR(flashperiod, 0664, NULL, flashperiod_store);
static DEVICE_ATTR(flashon, 0664, NULL, flashon_store);

extern int get_board_version(void);
#define BOARD_PIO04 4
#define BOARD_PIO05 5
#define BAORD_OTHERS 0Xff

static struct attribute *blink_attrs[] = {
    &dev_attr_blink.attr,
    &dev_attr_flashperiod.attr,
    &dev_attr_flashon.attr,
    NULL
};

static const struct attribute_group blink_attr_group = {
    .attrs = blink_attrs,
};

static void update_power_state(struct ktd2026_led_data *led)
{
    return ;//when use ldo control led,supply
}

/* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
static s32 ktd2026_i2c_write(const struct i2c_client *client, u8 command,
			      u8 value)
{
//	printk(KERN_ERR"ktd2026 write 0x%02X to 0x%02X \n",value,command);
	return i2c_smbus_write_byte_data(client,command,value);

}
// Actual work to set the brightness
static void brightness_set_work(struct ktd2026_led_data *led) // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998
{
    struct ktd2026_platform_data *pdata;
    int  brt_val;
    u8 enable, val=0,blink = 0;
    /* MODIFIED-END by hongwei.tian,BUG-4255998*/

    pdata = led->pdata;
    brt_val = led->led_dev.brightness;
    INFO(pdata->name, "brightness =%02X",led->led_dev.brightness);

    update_power_state(led);

    if(brt_val == 0){
        enable = 0;
    }else{
        enable = 1;
    }
    // Update the brightness first
    // Only update brightness if non zero
    /* MODIFIED-BEGIN by guangchao.su, 2017-03-01,BUG-4275264*/
    led->current_brightness = brt_val;

    //brt_val=brt_val-1;

 //   brt_val=brt_val*191/255;
    if(brt_val>255)
      brt_val=255;
    if(brt_val<0)
      brt_val=0;
    //led->current_brightness = brt_val;
    /* MODIFIED-END by guangchao.su,BUG-4275264*/

    if(led->led_mask==1)
    {
       /* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
       if(ktd2026_i2c_write(led->client, ktd2026_CH[0], brt_val) < 0) {
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[0], brt_val);
        }
       else {
            INFO(pdata->name, "Setting red 0x%02X to 0x%02X",ktd2026_CH[0], brt_val);
        }
	blink = pdata->control_val & 0x03;
	if(blink == 0x00 || blink == 0x011)
	{
	       val = pdata->control_val & 0xfc;//1111 1100
	       if(enable)
	         val |= 0x1;
	}
	else
	{
		val = pdata->control_val ;
		if(!enable)
			val = pdata->control_val & 0xfc;//1111 1100
	}
    }
    else if(led->led_mask==2)
    {
       if(ktd2026_i2c_write(led->client, ktd2026_CH[1], brt_val) < 0) {
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[1], brt_val);
        }
       else {
            INFO(pdata->name, "Setting green 0x%02X to 0x%02X",ktd2026_CH[1], brt_val); // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998
        }
	blink = pdata->control_val & 0x0C;
	if(blink == 0x00 || blink == 0x04)
	{
	       val = pdata->control_val & 0xf3;//1111 0011
	       if(enable)
	         val |= 0x4;
	}
	else
	{
		val = pdata->control_val ;
		if(!enable)
			val = pdata->control_val & 0xf3;//1111 0011
	}
    }
    else if(led->led_mask==4)
    {
       if(ktd2026_i2c_write(led->client, ktd2026_CH[2], brt_val) < 0) {
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[2], brt_val);
        }
       else {
            INFO(pdata->name, "Setting blue 0x%02X to 0x%02X",ktd2026_CH[2], brt_val); // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998
        }
	blink = pdata->control_val & 0x30;
	if(blink == 0x00 || blink == 0x10)
	{
	       val = pdata->control_val & 0x0f;//0000 1111
	       if(enable)
	         val |= 0x10;
	}
	else
	{
		val = pdata->control_val ;
		if(!enable)
			val = pdata->control_val & 0x0f;//0000 1111
	}
	/* MODIFIED-END by hongwei.tian,BUG-4255998*/
    }
    else //appear abnormal
    {
       INFO(pdata->name, "set led failue NOK,exit mask=%d\n",led->led_mask);
        return;
    }

    pdata->control_val = val;

        // No need to update this if its blinking. blink_work will take care of it
    ktd2026_i2c_write(led->client, 0x00, 0x00); // MODIFIED by guangchao.su, 2017-03-15,BUG-4354642
    if (ktd2026_i2c_write(led->client, ktd2026_CONTROL, val) < 0) { // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
        INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CONTROL, val);
    } else {
        INFO(pdata->name, "Setting 0x%02X to 0x%02X",ktd2026_CONTROL, val);


    }
}

// Periodic blink work to keep all bliking leds in sync
static void blink_work(struct ktd2026_led_data *led,bool enable)
{

    u8 reg_period;
    u8 reg_flashon;
    u8 val;
//    int  brt_val; // MODIFIED by guangchao.su, 2017-03-01,BUG-4275264
    struct ktd2026_platform_data *pdata  = led->pdata;
    if(enable==true)
    {
      if(led->flash_period>16 || led->flash_period<1 )
      {
        INFO(led->pdata->name, "blink period time error, %02d", led->flash_period);
        return;
      }
       reg_period=6+((led->flash_period-1)*78+2)/10;//from 1sstart
      if(led->flash_ontime>100 || led->flash_ontime<1)
      {
        INFO(led->pdata->name,"blink flash  time error ,%d,",led->flash_ontime);
         return;
       }
        reg_flashon=led->flash_ontime*255/100;
       /* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
       ktd2026_i2c_write(led->client, 0x00, 0x00);// DEFAULT slot1
       ktd2026_i2c_write(led->client, 0x02, 0x00);//
       /* MODIFIED-BEGIN by guangchao.su, 2017-03-01,BUG-4275264*/
       ktd2026_i2c_write(led->client, 0x05, 0x00);//rase time 480ms
       /* MODIFIED-BEGIN by guangchao.su, 2017-03-13,BUG-4354642*/
       //ktd2026_i2c_write(led->client, 0x01, reg_period);//
       //ktd2026_i2c_write(led->client, 0x02, reg_flashon);//
       ktd2026_i2c_write(led->client, 0x01, 0x12);//
       ktd2026_i2c_write(led->client, 0x02, 0x32);//
       /* MODIFIED-END by guangchao.su,BUG-4354642*/
       /* MODIFIED-END by hongwei.tian,BUG-4255998*/

/*        brt_val=led->current_brightness ;
        if(brt_val==0)
            brt_val=255;*/
    }
/*    else
    {
      brt_val=0;
    }*/
    if(led->led_mask==1)
    {
/*       if(ktd2026_i2c_write(led->client, ktd2026_CH[0], brt_val) < 0) { // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[0], brt_val);
        }
       else {
            INFO(pdata->name, "Setting red 0x%02X to 0x%02X",ktd2026_CH[0], brt_val);
        }*/
       val = pdata->control_val & 0xfc;//1111 1100
       if(enable)
         val |= 0x2;
    }
    else if(led->led_mask==2)
    {
/*       if(ktd2026_i2c_write(led->client, ktd2026_CH[1], brt_val) < 0) { // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[1], brt_val);
        }
       else {
            INFO(pdata->name, "Setting green 0x%02X to 0x%02X",ktd2026_CH[1], brt_val); // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998
        }*/
        /* MODIFIED-END by guangchao.su,BUG-4275264*/
       val = pdata->control_val & 0xf3;//1111 0011
       if(enable)
         val |= 0x8;

    }
    else if(led->led_mask==4)
    {
/* MODIFIED-BEGIN by guangchao.su, 2017-03-01,BUG-4275264*/
/*       if(ktd2026_i2c_write(led->client, ktd2026_CH[2], brt_val) < 0) { // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
            INFO(pdata->name, "Failed to write 0x%02X to 0x%02X",ktd2026_CH[2], brt_val);
        }
       else {
            INFO(pdata->name, "Setting blue 0x%02X to 0x%02X",ktd2026_CH[2], brt_val); // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998
        }*/
        /* MODIFIED-END by guangchao.su,BUG-4275264*/
       val = pdata->control_val & 0x0f;//0000 1111
       if(enable)
         val |= 0x20;

    }
    else //appear abnormal
    {
       INFO(pdata->name, "set led failue NOK,exit mask=%d\n",led->led_mask);
        return;
    }

    pdata->control_val = val;


/* MODIFIED-BEGIN by guangchao.su, 2017-03-01,BUG-4275264*/
/*     if (ktd2026_i2c_write(led->client, ktd2026_CONTROL, val) < 0) { // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
           INFO(led->pdata->name, "Failed to write %02X to %02X",ktd2026_CONTROL, val);
     } else {
           INFO(led->pdata->name, "Setting %02X to %02X",ktd2026_CONTROL, val);
    }
    led->pdata->control_val = val;*/
    /* MODIFIED-END by guangchao.su,BUG-4275264*/
}



// Called by led class driver in response of brightess API or write to brightness sysfs entry
static void ktd2026_brightness_set(struct led_classdev *led_cdev,
                         enum led_brightness brt_val)
{
    struct ktd2026_led_data *led;

    led = container_of(led_cdev, struct ktd2026_led_data, led_dev);
    led->led_dev.brightness=brt_val;
    INFO(led->pdata->name, "Set %s to %d,led_mask=%d", led_cdev->name, led_cdev->brightness,led->led_mask);
/* MODIFIED-BEGIN by hongwei.tian, 2017-02-22,BUG-4255998*/
//    queue_work(led->pdata->workqueue, &led->brightness_set_work);
    brightness_set_work(led);
    /* MODIFIED-END by hongwei.tian,BUG-4255998*/
}

/* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
static ssize_t
blink_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ktd2026_led_data *led;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    led = container_of(led_cdev, struct ktd2026_led_data, led_dev);

	return sprintf(buf, "%d \n", led->is_blinking);
}
/* MODIFIED-END by hongwei.tian,BUG-4255998*/

// Called by sysfs for blink entry
static ssize_t blink_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
    struct ktd2026_led_data *led;
    unsigned long blinking;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct ktd2026_platform_data *pdata;
    ssize_t ret = -EINVAL;
    led = container_of(led_cdev, struct ktd2026_led_data, led_dev);
    pdata = led->pdata;

    ret = kstrtoul(buf, 10, &blinking);
    if (ret)
        return ret;
    // it need set flash_period,flash_on parameter,or else use default
  /* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
  if(blinking == 0){
        INFO(pdata->name, "Stopping blink for %s", led->led_dev.name);
	 led->is_blinking = false;
        blink_work(led,false);
        // Back to what it was based on current brightness
        led->led_dev.brightness=0;
        ktd2026_brightness_set(&led->led_dev, led->led_dev.brightness);
    }else {
        pdata->blink_on = true;   // First tick is to turn on
	 led->is_blinking = true;
	 /* MODIFIED-END by hongwei.tian,BUG-4255998*/
        INFO(pdata->name, "Starting blink for %s", led->led_dev.name);
        blink_work(led,true);
    }


    return count;
}
// Called by sysfs for blink entry
#if 1
static ssize_t flashperiod_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{
    struct ktd2026_led_data *led;
    ssize_t ret = -EINVAL;
    unsigned long flash_cycle;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    led = container_of(led_cdev, struct ktd2026_led_data, led_dev);
    ret = kstrtoul(buf, 10, &flash_cycle);
    if (ret)
        return ret;
    if(flash_cycle>16 || flash_cycle <1)
        return ret;
    led->flash_period=flash_cycle;
    return count;

}
static ssize_t flashon_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count)
{

  struct ktd2026_led_data *led;
  ssize_t ret = -EINVAL;
  unsigned long flash_ontime;
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  led = container_of(led_cdev, struct ktd2026_led_data, led_dev);
  ret = kstrtoul(buf, 10, &flash_ontime);
  if (ret)
      return ret;
  if(flash_ontime>100 || flash_ontime <1)
      return ret;
  led->flash_ontime=flash_ontime;
  return count;
}
#endif
static int ktd2026_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int rc = 0, num_leds = 0, parsed_leds = 0, i2c_en_gpio, i;
    u32 temp_u32;
    struct ktd2026_led_data *led, *led_array;
    struct device_node *node, *temp;
    struct ktd2026_platform_data  *pdata;
    bool use_blink=false;
	if(get_board_version()==BOARD_PIO05) {
      INFO("", "new hardware,Uing ktd2026 ic Probe start!");
	}
    else
   	{
      INFO("", "Old hardware,directly return");
	  return 0;
   	}
    node = client->dev.of_node;
    if (node == NULL) {
        INFO("","Probe error: null of node!");
        return -ENODEV;
    }
    temp = NULL;
    // Figure out the number of led class we support, so we can allocate buffers accordingly
    while ((temp = of_get_next_child(node, temp)))
        num_leds++;
    pr_err("num_leds=%d\n",num_leds);
    if (!num_leds) {
        INFO("", "Probe error: no child leds!");
        return -ECHILD;
    }
    led_array = devm_kzalloc(&client->dev,
        (sizeof(struct ktd2026_led_data) * num_leds), GFP_KERNEL);
    if (!led_array) {
        INFO("", "Probe error: Unable to allocate memory for led_array");
        return -ENOMEM;
    }
    pdata = devm_kzalloc(&client->dev,
        sizeof(struct ktd2026_platform_data), GFP_KERNEL);
    if (!pdata) {
        INFO("", "Probe error: Unable to allocate memory for pdata");
        return -ENOMEM;
    }


    rc = of_property_read_string(node, "linux,name", &pdata->name);
    if (rc < 0) {
        INFO("", "Probe error:Failure reading led name, rc = %d", rc);
        goto fail_node;
    }

    // Allocate ordered workqueue, to keep code simpler
    pdata->workqueue = alloc_ordered_workqueue
                    ("ktd2026_workqueue", 0);
    if (!pdata->workqueue) {
        INFO(pdata->name, "Probe error: alloc_ordered_workqueue failed");
        return -ENOMEM;
    }

    pdata->client = client;
    pdata->num_leds = num_leds;
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

    pdata->i2c_vreg = regulator_get(&client->dev, "i2c");
    if (IS_ERR(pdata->i2c_vreg)) {
        rc = PTR_ERR(pdata->i2c_vreg);
        DEBUG(pdata->name, "Regulator not used. rc=%d", rc);
        pdata->i2c_vreg = NULL;
    } else {
        rc = regulator_set_voltage(pdata->i2c_vreg, 1800000, 1800000);
        if (rc) {
            INFO(pdata->name, "Regulator set_vtg failed vdd rc=%d", rc);
            pdata->i2c_vreg = NULL;
        }
    }



    for_each_child_of_node(node, temp) {
        led = &led_array[parsed_leds];
        led->client = client;
        led->pdata = pdata;
        led->led_dev.brightness_set = ktd2026_brightness_set;

        rc = of_property_read_string(temp, "linux,name",
            &led->led_dev.name);
        if (rc < 0) {
            INFO(pdata->name, "Failure reading led name, rc = %d", rc);
            goto fail_node;
        }

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
        led->flash_ontime=45;//default // MODIFIED by guangchao.su, 2017-03-01,BUG-4275264
        led->flash_period=5;//default
        led->use_blink = of_property_read_bool(temp, "led,use-blink");
        if(led->use_blink){
            use_blink = true;
        }

        DEBUG(pdata->name, "Registering %s, brightness=%d, mask=%d", led->led_dev.name, led->led_dev.max_brightness, led->led_mask);

//        INIT_WORK(&led->brightness_set_work, brightness_set_work); // MODIFIED by hongwei.tian, 2017-02-22,BUG-4255998

        rc = led_classdev_register(&led->client->dev, &led->led_dev);
        if (rc) {
            INFO(pdata->name, "unable to register led %s,rc=%d", led->led_dev.name, rc);
            goto fail_node;
        }

        if (led->use_blink) {
            rc = sysfs_create_group(&led->led_dev.dev->kobj,
                &blink_attr_group);
            if (rc)
                goto fail_node;
            }
        parsed_leds++;
    }
    // Read duty cycle paramanets if at least one led supports blinking
#if 0
    if(use_blink){
        rc = of_property_read_u32(node, "led,delay_on",
            &temp_u32);
        if (rc < 0) {
            INFO(pdata->name, "Failure reading led delay_on, rc = %d", rc);
            goto fail_node;
        }

        rc = of_property_read_u32(node, "led,delay_off",
            &temp_u32);
        if (rc < 0) {
            INFO(pdata->name, "Failure reading led delay_off, rc = %d", rc);
            goto fail_node;
        }

    }
#endif
    i2c_set_clientdata(client, led_array);
/* [BUGFIX]-Add- by TCLNB.XQJ,bug 3379224,11/07,add checking i2c status*/
  pdata->control_val=0;
  if (pdata->i2c_vreg)
     rc = regulator_enable(pdata->i2c_vreg);

   /* MODIFIED-BEGIN by hongwei.tian, 2017-02-21,BUG-4255998*/
   ktd2026_i2c_write(led->client, 0x02, 0x00);//
   ktd2026_i2c_write(led->client, 0x05, 0x00);//
   ktd2026_i2c_write(led->client, 0x01, 0x00);//
   ktd2026_i2c_write(led->client, 0x06, 0x00);//
   ktd2026_i2c_write(led->client, ktd2026_CONTROL, 0);

#ifdef CONFIG_BBRY
    rc=ktd2026_i2c_write(led->client, ktd2026_CONTROL, 0);
    /* MODIFIED-END by hongwei.tian,BUG-4255998*/
    if (rc < 0) {
         i2c_check_status_create("indicator_led",0);
         INFO(pdata->name, "Failed to read %02X ",ktd2026_CONTROL);
    } else {
         INFO(pdata->name, "check ic  setted 1 ");
         i2c_check_status_create("indicator_led",1);
    }
#endif
/* [BUGFIX]-End- by TCLNB.XQJ*/
    INFO("", "Probe scuessflly !");

    return 0;

fail_node:
    for (i = 0; i < parsed_leds; i++) {
        led_classdev_unregister(&led_array[i].led_dev);
    }
    destroy_workqueue(pdata->workqueue);

    return rc;
}

static int ktd2026_remove(struct i2c_client *client)
{
    struct ktd2026_led_data *led_array = i2c_get_clientdata(client);
    struct ktd2026_platform_data  *pdata=led_array->pdata;
    int i, num_leds = pdata->num_leds;

    for (i = 0; i < num_leds; i++) {
        led_classdev_unregister(&led_array[i].led_dev);
    }
    destroy_workqueue(pdata->workqueue);

    return 0;
}
static void ktd2026_shutdown(struct i2c_client *client)
{
       if(client !=NULL)
        ktd2026_i2c_write(client, 0x04, 0);//SET reg4  oo off // MODIFIED by hongwei.tian, 2017-02-21,BUG-4255998
}


static const struct i2c_device_id ktd2026_id[] = {
    {ktd2026_NAME, 0},    // This dummy id. Class driver will update it from dts
    {}
};


static struct of_device_id ktd2026_match_table[] = {
    {.compatible = "qcom,leds-ktd2026",},
    { },
};

static struct i2c_driver ktd2026_i2c_driver = {
    .probe = ktd2026_probe,
    .remove = ktd2026_remove,
    .id_table = ktd2026_id,
    .driver = {
        .name = ktd2026_NAME,
        .owner = THIS_MODULE,
        .of_match_table = ktd2026_match_table,
    },
   .shutdown   = ktd2026_shutdown,
};
static int __init ktd2026_i2c_init(void)
{
    return i2c_add_driver(&ktd2026_i2c_driver);
}
late_initcall(ktd2026_i2c_init);

//module_i2c_driver(ktd2026_i2c_driver);

MODULE_DESCRIPTION("ktd2026 LED driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TCL NINGBO Limited");
