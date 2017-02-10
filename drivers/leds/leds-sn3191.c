/*
 * leds-sn3191.c - RGB LED Driver
 *
 * Copyright (C) 2014 TCL Communication Technology (shanghai) Co., LTD.
 * Sun zhangyang <zhangyang.sun@tcl.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * now is only green led is used,so disalbe other led,but ic is support RGB
 * Datasheet:
 * ===================================================================================
 *                             EDIT HISTORY FOR MODULE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *	when       who        what, where, why
 * ------------------------------------------------------------------------------------
* [PLATFORM]-Add- by TCTSH.SZY, FR-, 2015/10/30, add for idol4 led 
* 11/04/2015  XQJ       FR-526565, sn31xx extern led ic chip
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define sn31xx_VDD_MIN_UV  2000000
#define sn31xx_VDD_MAX_UV  3300000
#define sn31xx_VIO_MIN_UV        1750000
#define sn31xx_VIO_MAX_UV       1950000

enum led_bits {
    sn3191_OFF = 0,
    sn3191_ON,
    sn3191_BLINK1,
    sn3191_BLINK2,
    sn3191_UNKNOWN,
};

struct sn3191_led {
    struct i2c_client       *client;
    struct rw_semaphore     rwsem;
    struct regulator        *vio;
    struct regulator        *vdd;
    const char *vio_name;    
    int                     power_enabled;
    unsigned int	    en_gpio;
    /*
     * Making led_classdev as array is not recommended, because array
     * members prevent using 'container_of' macro. So repetitive works
     * are needed.
     */
    struct led_classdev     cdev_led1g;
    /*
     * Advanced Configuration Function(ADF) mode:
     * In ADF mode, user can set registers of sn3191GU directly,
     * therefore sn3191GU doesn't enter reset state.
     */
    int			    adf_on;	
    enum led_bits           state;
};
static int sn3191_power_init(struct sn3191_led *data, bool on)
{  
    int rc;

    if (!on) {
        if (regulator_count_voltages(data->vio) > 0)
            regulator_set_voltage(data->vio, 0,sn31xx_VIO_MAX_UV);
            regulator_put(data->vio);
    } else {
        data->vio = regulator_get(&data->client->dev, data->vio_name);
        if (IS_ERR(data->vio)) {
            rc = PTR_ERR(data->vio);
            dev_err(&data->client->dev,
                        "Regulator get failed vio rc=%d\n", rc);
            return rc;
        }
        if (regulator_count_voltages(data->vio) > 0) {
            rc = regulator_set_voltage(data->vio,
                                    sn31xx_VIO_MIN_UV, sn31xx_VIO_MAX_UV);
            if (rc) {
                dev_err(&data->client->dev,
                            "Regulator set failed vio rc=%d\n", rc);
                return rc;
            }
        }
    }

    return 0;

    
}
static int sn3191_power_set(struct sn3191_led *data, bool on)
{
    int rc = 0;

    if (!on && data->power_enabled) {
        rc = regulator_disable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                          "Regulator vio disable failed rc=%d\n", rc);
            return rc;
        }
        data->power_enabled = false;
        return rc;
    } else if (on && !data->power_enabled) {
        rc = regulator_enable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                          "Regulator vio enable failed rc=%d\n", rc);
            return rc;
        }
        data->power_enabled = true;

        /*
         * The max time for the power supply rise time is 50ms.
         * Use 80ms to make sure it meets the requirements.
         */
        msleep(80);
        return rc;
    } else {
        dev_warn(&data->client->dev,
                         "Power on=%d. enabled=%d\n", on, data->power_enabled);
        return rc;
    }
}
/*--------------------------------------------------------------*/
/*  sn3191GU core functions                    */
/*--------------------------------------------------------------*/

static int sn3191_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret >= 0)
        return 0;

    dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
                        __func__, reg, val, ret);

    return ret;
}

#define sn3191_SET_REGISTER(reg_addr, reg_name)                \
static ssize_t sn3191_store_reg##reg_addr(struct device *dev,      \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn3191_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    msleep(500);                                                \
    sn3191_write_byte(led->client, reg_addr, (u8) val);        \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute sn3191_reg##reg_addr##_attr = {     \
    .attr = {.name = reg_name, .mode = 0644},           \
    .store = sn3191_store_reg##reg_addr,               \
};

sn3191_SET_REGISTER(0x00, "0x00");
sn3191_SET_REGISTER(0x01, "0x01");
sn3191_SET_REGISTER(0x02, "0x02");
sn3191_SET_REGISTER(0x03, "0x03");
sn3191_SET_REGISTER(0x04, "0x04");
sn3191_SET_REGISTER(0x07, "0x07");
sn3191_SET_REGISTER(0x0a, "0x0a");
sn3191_SET_REGISTER(0x10, "0x10");
sn3191_SET_REGISTER(0x16, "0x16");
sn3191_SET_REGISTER(0x1c, "0x1c");
sn3191_SET_REGISTER(0x1d, "0x1d");
sn3191_SET_REGISTER(0x2f, "0x2f");


static struct device_attribute *sn3191_addr_attributes[] = {
    &sn3191_reg0x00_attr,
    &sn3191_reg0x01_attr,
    &sn3191_reg0x02_attr,
    &sn3191_reg0x03_attr,
    &sn3191_reg0x04_attr,
    &sn3191_reg0x07_attr,
    &sn3191_reg0x0a_attr,
    &sn3191_reg0x10_attr,
    &sn3191_reg0x16_attr,
    &sn3191_reg0x1c_attr,
    &sn3191_reg0x1d_attr,
    &sn3191_reg0x2f_attr,
};

static void sn3191_enable_adv_conf(struct sn3191_led *led)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(sn3191_addr_attributes); i++) {
        ret = device_create_file(&led->client->dev,
                        sn3191_addr_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn3191_addr_attributes[i]->attr.name);
            goto failed_remove_files;
        }
    }
    led->adf_on = 1;

    return;

failed_remove_files:
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev,
                        sn3191_addr_attributes[i]);
}

static void sn3191_disable_adv_conf(struct sn3191_led *led)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(sn3191_addr_attributes); i++)
        device_remove_file(&led->client->dev,
                        sn3191_addr_attributes[i]);
    led->adf_on = 0;
}

static ssize_t sn3191_show_adv_conf(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct sn3191_led *led = i2c_get_clientdata(to_i2c_client(dev));
    ssize_t ret;

    down_read(&led->rwsem);
    if (led->adf_on)
        ret = sprintf(buf, "on\n");
    else
        ret = sprintf(buf, "off\n");
    up_read(&led->rwsem);

    return ret;
}

static ssize_t sn3191_store_adv_conf(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct sn3191_led *led = i2c_get_clientdata(to_i2c_client(dev));
    if (!count)
        return -EINVAL;

    down_write(&led->rwsem);
    if (!led->adf_on && !strncmp(buf, "on", 2))
        sn3191_enable_adv_conf(led);
    else if (led->adf_on && !strncmp(buf, "off", 3))
        sn3191_disable_adv_conf(led);
    up_write(&led->rwsem);

    return count;
}

static struct device_attribute sn3191_adv_conf_attr = {
    .attr = {
        .name = "advanced_configuration",
        .mode = 0644,
    },
    .show = sn3191_show_adv_conf,
    .store = sn3191_store_adv_conf,
};

#define sn3191_CONTROL_ATTR(attr_name, name_str)           \
static ssize_t sn3191_show_##attr_name(struct device *dev,     \
    struct device_attribute *attr, char *buf)           \
{                                   \
    struct sn3191_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    ssize_t ret;                            \
    down_read(&led->rwsem);                     \
    ret = sprintf(buf, "0x%02x\n", led->attr_name);         \
    up_read(&led->rwsem);                       \
    return ret;                         \
}                                   \
static ssize_t sn3191_store_##attr_name(struct device *dev,        \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn3191_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    led->attr_name = val;                       \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute sn3191_##attr_name##_attr = {       \
    .attr = {                           \
        .name = name_str,                   \
        .mode = 0644,                       \
    },                              \
    .show = sn3191_show_##attr_name,               \
    .store = sn3191_store_##attr_name,             \
};

static struct device_attribute *sn3191_attributes[] = {
    &sn3191_adv_conf_attr,
};
static void sn3191_set_led1g_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value)  
{               
    struct sn3191_led *led =       
        container_of(led_cdev, struct sn3191_led, cdev_led1g);

  
    //pr_err("  gpio=%d brightness:%d\n",led->en_gpio, value);
   
    sn3191_power_set(led,1);
    if (value == LED_OFF)
    {   
        led->state = sn3191_OFF;
        sn3191_write_byte(led->client,0x2f, 0x00); //reset
        sn3191_write_byte(led->client,0x00, 0x01); //led2 constant on  ,light on
        gpio_direction_output(led->en_gpio, 0);
		sn3191_power_set(led,0);
    }
    else
    {
        gpio_direction_output(led->en_gpio, 1);        
        msleep(100);
        sn3191_write_byte(led->client,0x2f, 0x00); //reset
        sn3191_write_byte(led->client,0x00, 0x20); //
        sn3191_write_byte(led->client,0x02, 0x00); // 
        sn3191_write_byte(led->client,0x03,0x08); //i max 5ma 
        if(value>255)
             value=255;
        else if(value<1)
             value=1;
        sn3191_write_byte(led->client,0x04, value); 
        sn3191_write_byte(led->client,0x07, 0xff); //update
        led->state = sn3191_ON;      
    }
}                                   
static void sn3191_set_led1g_blink(struct led_classdev *led_cdev,u8 bblink)
{                               
	struct sn3191_led *led = container_of(led_cdev,
										struct sn3191_led, cdev_led1g);

   
    gpio_direction_output(led->en_gpio, 1);
    msleep(100);
    if(bblink==1) {
        //pr_err("blink on mode 1 \n");
        sn3191_write_byte(led->client,0x2f, 0x00); //reset
        sn3191_write_byte(led->client,0x02, 0x20); //reset
        sn3191_write_byte(led->client,0x00, 0x20); //reset         
        sn3191_write_byte(led->client,0x03,0x08); //i max 5ma
#if defined(CONFIG_TCT_8X76_IDOL4)
        sn3191_write_byte(led->client,0x04, 0x14); //sun zhangyang modify for defect1505463,2016.1.25
#else
        sn3191_write_byte(led->client,0x04, 0x0f);
#endif
        sn3191_write_byte(led->client,0x07, 0xff); 

        sn3191_write_byte(led->client,0x0a, 0x00); //T0 0S
/* [FEATURE]-ADD-BEGIN by TCTCD.liyue, 2015/11/10, Modify the indicator led rules follow Vodafone's request */
#if defined(CONFIG_TCT_8X76_IDOL4S_VDF)
        sn3191_write_byte(led->client,0x10, 0x40);//T1,Tt2
        sn3191_write_byte(led->client,0x16, 0x48);//T3,T4
/* [FEATURE]-ADD-END by TCTCD.liyue */ 
#elif defined(CONFIG_TCT_8X76_IDOL4S)
        sn3191_write_byte(led->client,0x10, 0x80);//T1,Tt2
        sn3191_write_byte(led->client,0x16, 0x88);//T3,T4
#elif defined(CONFIG_TCT_8X76_IDOL4)

#ifdef CONFIG_BBRY
	    sn3191_write_byte(led->client,0x10, 0x00);//T1 0.13s, T2 0.00s
	    sn3191_write_byte(led->client,0x16, 0x0A);//T3 0.13s, T4 2.08s
#else
        sn3191_write_byte(led->client,0x10, 0x80);//T1 2.08s,T2 0s
        sn3191_write_byte(led->client,0x16, 0x86);//T3 2.08s,T4 0.52s
#endif /* CONFIG_BBRY */

#endif
        sn3191_write_byte(led->client,0x1C, 0x00);//UPDATE
        led->state = sn3191_BLINK1;       
	} else if(bblink==0) {			//constant 
           //pr_err("blink off\n");
           sn3191_write_byte(led->client,0x2f, 0x00); //reset
           sn3191_write_byte(led->client,0x00, 0x01); //
           sn3191_write_byte(led->client,0x02, 0x00); // 
	   sn3191_write_byte(led->client,0x03, 0x08); //i max 5ma
	   sn3191_write_byte(led->client,0x04, 0x0f);
           sn3191_write_byte(led->client,0x07, 0xff); //update
       
           led->state = sn3191_OFF;
           gpio_direction_output(led->en_gpio, 0);          
	} else if (bblink==2) {
	    //pr_err("blink on mode 2 \n");
	    sn3191_write_byte(led->client,0x2f, 0x00); //reset
	    sn3191_write_byte(led->client,0x02, 0x20); //led mode
	    sn3191_write_byte(led->client,0x00, 0x20);

	    sn3191_write_byte(led->client,0x03,0x08); //i max 5ma
#if defined(CONFIG_TCT_8X76_IDOL4)
          sn3191_write_byte(led->client,0x04, 0x14); //sun zhangyang modify for defect1505463,2016.1.25
#else
          sn3191_write_byte(led->client,0x04, 0x0f);
#endif
	    sn3191_write_byte(led->client,0x07, 0xff); 

	    sn3191_write_byte(led->client,0x0a, 0x00); //T0 0S
/* [FEATURE]-ADD-BEGIN by TCTCD.liyue, 2015/11/10, Modify the indicator led rules follow Vodafone's request */
#if defined(CONFIG_TCT_8X76_IDOL4S_VDF)
	sn3191_write_byte(led->client,0x10, 0x46);//T1,Tt2
        sn3191_write_byte(led->client,0x16, 0x48);//T3,T4
/* [FEATURE]-ADD-END by TCTCD.liyue */ 
#elif defined(CONFIG_TCT_8X76_IDOL4S)
        sn3191_write_byte(led->client,0x10, 0x60);//T1,Tt2
        sn3191_write_byte(led->client,0x16, 0x66);//T3,T4
#elif defined(CONFIG_TCT_8X76_IDOL4)

#ifdef CONFIG_BBRY
	    sn3191_write_byte(led->client,0x10, 0x00);//T1 0.13s, T2 0.00s
	    sn3191_write_byte(led->client,0x16, 0x0C);//T3 0.13s, T4 4.16s
#else
	    sn3191_write_byte(led->client,0x10, 0x60);//T1 1s,Tt2 0s
	    sn3191_write_byte(led->client,0x16, 0x66);//T3 1s,T4 0.5s
#endif /* CONFIG_BBRY */

#endif
	    sn3191_write_byte(led->client,0x1C, 0x00);//UPDATE
            led->state = sn3191_BLINK2;
	}
}


static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
{

      struct led_classdev *led_cdev = dev_get_drvdata(dev);
      u8 bblink;
      //pr_err("in %s,name=%s\n",__func__,led_cdev->name);
#ifdef CONFIG_BBRY
      pr_err("%s(), blink=%c\n", __func__, *buf);
#endif /* CONFIG_BBRY */
      if(*buf=='0')
        bblink=0;
      else if(*buf=='2')
        bblink=2;
     else
	bblink=1;

      sn3191_set_led1g_blink(led_cdev,bblink);
      return count;
}


static DEVICE_ATTR(blink, S_IWUSR, NULL, store_blink);


/* TODO: HSB, fade, timeadj, script ... */

static int sn3191_register_led_classdev(struct sn3191_led *led)
{
    int ret;
    led->cdev_led1g.name = "red";
    led->cdev_led1g.brightness = LED_OFF;
    led->cdev_led1g.brightness_set = sn3191_set_led1g_brightness;

    ret = led_classdev_register(&led->client->dev, &led->cdev_led1g);
    if (ret < 0) {
        dev_err(&led->client->dev, "couldn't register LED %s\n",
                            led->cdev_led1g.name);
        goto failed_unregister_led1_G;
    }
    return 0;

failed_unregister_led1_G:
    led_classdev_unregister(&led->cdev_led1g);

    return ret;
}

static int sn3191_init(struct i2c_client *client)
{
     int ret=0;
     ret= sn3191_write_byte(client,0x2f, 0x00); //reset
     return ret;
}

static int sn3191_parse_dt(struct device *dev, struct sn3191_led *led)
{
   int r = 0;
   const char *name;
   led->en_gpio = of_get_named_gpio_flags(dev->of_node,
			"sn31,en-gpio", 0, NULL);
   if ((!gpio_is_valid(led->en_gpio)))
	return -EINVAL;
    r = of_property_read_string(dev->of_node, "sn31,vi2c", &name);
    led->vio_name=name;
    return r;
}

/* [PLATFORM]-ADD-BEGIN by TCTNB.XQJ,898336 , 2015/11/14, add i2c scan function */
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4)//sun zhangyang modify for task937744
static int i2c_ok = 0;
module_param_named(i2c_ok, i2c_ok, int, 0644);
#endif
/* [PLATFORM]-ADD-END by TCTNB.XQJ */

static int sn3191_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int ret;
    int i =0;
    struct sn3191_led *led = devm_kzalloc(&client->dev,sizeof(struct sn3191_led), GFP_KERNEL);
   
    if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
        return -ENOMEM;
    }

    led->client = client;
    i2c_set_clientdata(client, led);
  
    ret = sn3191_parse_dt(&client->dev, led);
    if(ret<0)
    {
        dev_err(&client->dev, "failed to parse dts\n");
        goto exit0;
    }
     ret = sn3191_power_init(led, 1);
    if (ret <0)
        goto exit1;
    ret = sn3191_power_set(led, 1);//let i2c init test can pass
    if (ret <0)
        goto free_led;
    ret = gpio_request(led->en_gpio, "sn31_en");
    if (ret)
    {
        dev_err(&client->dev, "failed to request led gpio.\n");
        goto free_led;
    }
    gpio_direction_output(led->en_gpio, 0);
    led->state = sn3191_UNKNOWN;
    sn3191_power_set(led, 0);
    udelay(100);
    do{
        i++;
        ret = sn3191_init(client);
    }while(ret < 0 && i <=3);    
    if (ret < 0)
       {
         dev_err(&client->dev, "failed to init sn3191.\n");
         goto free_led;
       }
/* [PLATFORM]-ADD-BEGIN by TCTNB.XQJ, 898336, 2015/04/04, add i2c scan function */
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4) //sun zhangyang modify for task937744
       else
       {
          // printk("i2c test ok\n");
           i2c_ok = 1;
       }
#endif
/* [PLATFORM]-ADD-END by TCTNB.XQJ */
 	    
    init_rwsem(&led->rwsem);

    ret = sn3191_register_led_classdev(led);	
    if (ret < 0)
       {
          dev_err(&client->dev, "failed to register led\n");
          goto free_led;
       } 
	
    for (i = 0; i < ARRAY_SIZE(sn3191_attributes); i++) {   
        ret = device_create_file(&led->client->dev,
                        sn3191_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn3191_attributes[i]->attr.name);
            goto remove_files;
        }
    }

    ret= sysfs_create_file(&led->cdev_led1g.dev->kobj, &dev_attr_blink.attr);
    if(ret)
        {
           dev_err(&led->client->dev,"fail to sys create.\n");
           goto free_led;
        }

    return 0;

remove_files:
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev,sn3191_addr_attributes[i]);
free_led:
    sn3191_power_set(led, 0);
exit1:
    sn3191_power_init(led, 0);
exit0:
    devm_kfree(&client->dev, led);
    led = NULL;

  return ret;
}

static int sn3191_remove(struct i2c_client *client)
{
    struct sn3191_led *led = i2c_get_clientdata(client);
    int i;
    led_classdev_unregister(&led->cdev_led1g);
    if (led->adf_on)
        sn3191_disable_adv_conf(led);
    for (i = 0; i < ARRAY_SIZE(sn3191_attributes); i++)
        device_remove_file(&led->client->dev, sn3191_attributes[i]);
    return 0;
}

static void sn3191_shutdown(struct i2c_client *client)
{
    struct sn3191_led *led = i2c_get_clientdata(client);

    gpio_direction_output(led->en_gpio, 0);

}

static const struct i2c_device_id sn3191_id[] = {
    { "sn3191", 0 },
    { }
};

static struct of_device_id sn3191_match_table[] = {
    { .compatible = "sn31,sn3191", },
    { },
};

static struct i2c_driver sn3191_i2c_driver = {
    .probe		= sn3191_probe,
    .remove		= sn3191_remove,
    .id_table	= sn3191_id,
    .driver = {
				.name    = "sn3191",
				.owner  = THIS_MODULE,
				.of_match_table = sn3191_match_table,
	},
    .shutdown   = sn3191_shutdown,
};
static int __init sn3191_i2c_init(void)
{
	return i2c_add_driver(&sn3191_i2c_driver);
}
late_initcall(sn3191_i2c_init);
MODULE_AUTHOR("zhangyang.sun@tcl.com>");
MODULE_DESCRIPTION("sn3191 LED driver");
MODULE_LICENSE("GPL v2");
