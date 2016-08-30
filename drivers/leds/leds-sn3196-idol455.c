/*
 * leds_sn3196.c - RGB LED Driver
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
* [PLATFORM]-Add- by TCTSH.SZY, add for IDOL4 led 
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/qpnp/pin.h>
//#include <linux/regulator/consumer.h>

#define	SN3196_REG_CHIP_ONOFF  	0x00
#define	SN3196_REG_LED_CTRL  	0x01 //control out1-out6 on or off                 
#define	SN3196_REG_CONF1  	    0x03 // set ic work mode
#define	SN3196_REG_CONF2		0x04 //set imax and audio gain
#define	SN3196_REG_BT_HT	  	0x05
#define	SN3196_REG_BT_FLAG	  	0x06

#define	SN3196_REG_PWM_OUT	 	0x07//07-0c：out1-out6(T0)

#define SN3196_REG_T0			0x11//11-16：out1-out6(T1-T2-T3)
#define SN3196_REG_T1_T2_T3_RGB1     0x1a // rgb1
#define SN3196_REG_T1_T2_T3_RGB2     0x1b // rgb2
#define SN3196_REG_T4			0x1d//1d-22 out1-out6(T4)
 
#define	SN3196_REG_DATA_UPDATE  0x10
#define	SN3196_REG_TIME_UPDATE	0x26
#define	SN3196_REG_RST 			0xff

// CHANELE FUNCTION
#define AUDIO_LED1 	 1 
#define AUDIO_LED2   2
#define AUDIO_LED3   4
#define AUDIO_LED4   5

static unsigned char g_Ledctrl1=0,g_LedCurrentSet=1, g_LedAudioGainSet =4; 


enum led_bits {
    SN3196_UNKNOW,
    SN3196_OFF,
    SN3196_ON,
    SN3196_BLINK1,
    SN3196_BLINK2,
    SN3196_BLINK3,
    SN3196_BLINK4,
};


enum IMAX
{
	IMAX_20_MA,
	IMAX_15_MA,
	IMAX_10_MA,
	IMAX_5_MA,
	IMAX_40_MA,
	IMAX_35_MA,
	IMAX_30_MA,
	IMAX_25_MA
};
enum AGS
{
	AGS_0_dB,
	AGS_3_dB,
	AGS_6_dB,
	AGS_9_dB,
	AGS_12_dB,
	AGS_15_dB,
	AGS_18_dB,
	AGS_21_dB
};

struct sn3196_led {
    struct i2c_client       *client;
    struct device			*dev;
    struct rw_semaphore     rwsem;
    unsigned int	en_gpio;/*add  led ic sn3196,gpio is used for enable led ic*/
    #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
    unsigned int	en_gpio2;/*led ic sn3196,gpio2 is used for contorl led*/
    #endif
    /*
     * Making led_classdev as array is not recommended, because array
     * members prevent using 'container_of' macro. So repetitive works
     * are needed.
     */
    struct led_classdev     cdev_led1g;
    /*
     * Advanced Configuration Function(ADF) mode:
     * In ADF mode, user can set registers of sn3196 directly,
     * therefore sn3196 doesn't enter reset state.
     */
    int						adf_on;//control debug
    struct pinctrl			*sn3196_pinctrl;
    struct pinctrl_state	*gpio_state;
 
    enum led_bits           state;
};

/*--------------------------------------------------------------*/
/*  sn3196 core functions                    */
/*--------------------------------------------------------------*/

static int sn3196_write_byte(struct sn3196_led *led, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(led->client, reg, val);
    if (ret < 0)
	{
		dev_err(led->dev, "fail to write reg 0x%x, val 0x%x, err %d\n",reg, val, ret);
		return ret;
	}
    return 0;
}


//sn3196 on
static void sn3196_power_on(struct sn3196_led *led)
{
	gpio_direction_output(led->en_gpio, 1);
	udelay(5000);
	sn3196_write_byte(led,SN3196_REG_CHIP_ONOFF,1);
}

//sn3196 off
static void sn3196_power_off(struct sn3196_led *led)
{
    sn3196_write_byte(led,SN3196_REG_RST,0); //reset chip
	sn3196_write_byte(led,SN3196_REG_CHIP_ONOFF,0);
	gpio_direction_output(led->en_gpio, 0);
}

/***************************************************************************************************/
//Work mode 
//0:PWM MODE 1:one progame mode 
//RGBx: x=1,2  D5D4 取值范围为(0-3) 
//D5:D4--RGB2:RGB1
//Rgbx= 00代表Rgb1,2均为PWM模式,01代表Rgb2，3为一次编程呼吸模式Rgb1为PWM模式 
//11代表Rgb1,2均为一次编程呼吸模式

//音频同步呼吸模式  AudioSync = D2D1D0 取值范围为(0-7) 
//（D2=1表示打开，D2=0表示关闭 音频同步呼吸模式）
//（D1=1表示关闭，D1=0表示启动 AGC自动增益控制功能）
//（D0=1表示AGC快速调制模式，D0=0表示慢速调制模式）
/***************************************************************************************************/
static void sn3196_set_workmode(struct sn3196_led *led,u8 RGBx, u8 AudioSync )
{
	 if ( (RGBx >3) || (AudioSync > 7)) 
	{	        
           printk("please set right RGBx(0-3) and AudioSync(0-7).\n");
           return;
        }

	 sn3196_write_byte(led,SN3196_REG_CONF1,(RGBx<<4)|(AudioSync));
}
/*********************************************************/
//设置Mono色灯的亮度
//Ch = 1,2,3,4,5,6,brightness = 0-255
/********************************************************/
static void sn3196_SetBrightness(struct sn3196_led *led,u8 Ch,u8 brightness)
{

	if  ( (Ch < 1) || (Ch > 6) ) 
        {
           printk("please set right ch(1-6)\n");
           return;
        }

	sn3196_write_byte(led,SN3196_REG_PWM_OUT+Ch-1,brightness);
	sn3196_write_byte(led,SN3196_REG_DATA_UPDATE,0x00);
}
/******************************************************************/
//Ch = 123, RGB1;    Ch =456 ,RGB2, 
//Ch =1,2,3,4,5,6   mono led1,2,3,4,5,6
/******************************************************************/
static void sn3196_TurnOn(struct sn3196_led *led,unsigned int Ch)
{
	switch (Ch){
		case 1:	
			g_Ledctrl1|=0x01;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 2:	//mono Led 2	
			g_Ledctrl1|=0x02;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 3:	//mono Led 3
			g_Ledctrl1|=0x04;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 4:	//mono Led 4
			g_Ledctrl1|=0x10;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;break;
		case 5://mono Led 5
			g_Ledctrl1|=0x20;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 6:	 //mono Led 6
			g_Ledctrl1|=0x40;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 123: //rgb Led 1
			g_Ledctrl1|=0x07;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 456: //rgb Led 2
			g_Ledctrl1|=0x70;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		}
}
//
static void sn3196_TurnOff(struct sn3196_led *led,unsigned int Ch)
{	
	switch (Ch){
		case 1://mono Led 1
			g_Ledctrl1&=(~0x01);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 2://mono Led 2
			g_Ledctrl1&=(~0x02);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 3://mono Led 3
			g_Ledctrl1&=(~0x04);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 4://mono Led 4	
			g_Ledctrl1&=(~0x10);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 5://mono Led 5			
			g_Ledctrl1&=(~0x20);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 6://mono Led 6		
			g_Ledctrl1&=(~0x40);
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 123: //rgb Led 1
			g_Ledctrl1&=0x70;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		case 456: //rgb Led 2
			g_Ledctrl1&=0x07;
			sn3196_write_byte(led,SN3196_REG_LED_CTRL,g_Ledctrl1);
			break;
		}
}

/************************************************/
//设置LED 电流大小
//0= 20mA , 1 = 15mA, 2= 10mA, 3=5mA, 4= 40mA, 5= 35mA, 6=30mA,7=25mA
/*************************************************/
static void sn3196_set_imax(struct sn3196_led *led,u8 level)
{
	if ( level > 7 ) 
        {
           printk("please set right level(0-7)\n");
           return;
        }

	g_LedCurrentSet = level;
	sn3196_write_byte(led,SN3196_REG_CONF2,(g_LedAudioGainSet + (level<<4))); 
}

/************************************************/
//0= 0dB , 1 = +3dB, 2= +6dB, 3=+9dB, 4= +12dB, 5= +15dB, 6=+18dB,7=+21dB
/*************************************************/
static void sn3196_SetAudioGain(struct sn3196_led *led,u8 level)
{

	 if ( level > 7 ) 
        {
           printk("please set right level(0-7)\n");
           return;
        }
	g_LedAudioGainSet = level;
	sn3196_write_byte(led,SN3196_REG_CONF2,((g_LedCurrentSet<<4) + g_LedAudioGainSet)); 
 
}

/************************************************/
//data update
/*************************************************/
static void sn3196_DataUpdate(struct sn3196_led *led)
{															
 	sn3196_write_byte(led,SN3196_REG_DATA_UPDATE,0x00);
}

/************************************************/
//time update
/*************************************************/
static void sn3196_TimeUpdate(struct sn3196_led *led)
{
   	sn3196_write_byte(led,SN3196_REG_TIME_UPDATE,0x00);  
}
/*******************************************************************************/
//T0启动时间   			公式= 0.26s*T0A*2^T0B  范围 T0A（0-15) T0B(0-3)
//T4为保持灭掉的时间		公式= 0.26s*T0A*2^T0B  范围 T4A(0-15),T4B(0-3)
/********************************************************************************/

static void sn3196_SetBreathTime(struct sn3196_led *led,u8 Ch, u8 T0A,u8 T0B, u8 T4A,u8 T4B)
{														 

	  if ( (Ch >6) || (T0A >15) || (T0B>3) || (T4A> 15) || (T4B >3)) 
          {
             printk("please set right ch(1-6),T0A（0-15),T0B(0-3),T4A(0-15) and T4B(0-3).\n");
             return;
          }	   
	  if (Ch)
	  {
			sn3196_write_byte(led,SN3196_REG_T0+Ch-1,T0A|(T0B<<4));
			sn3196_write_byte(led,SN3196_REG_T4+Ch-1,T4A|(T4B<<4));
	  }
	  else
	  {
			sn3196_write_byte(led,0x11,0x10);
			sn3196_write_byte(led,0x12,0x11);	 //T0
			sn3196_write_byte(led,0x13,0x12);

			sn3196_write_byte(led,0x1a,0x01); //T1,T2,T3
	
		   	sn3196_write_byte(led,0x1d,0x10);	  
			sn3196_write_byte(led,0x1e,0x11);	 
			sn3196_write_byte(led,0x1f,0x12);	//T4
	
			sn3196_write_byte(led,0x14,0x13);
			sn3196_write_byte(led,0x15,0x14);	//T0
			sn3196_write_byte(led,0x16,0x15);
	
		   	sn3196_write_byte(led,0x1b,0x01); //T1,T2,T3
	
			sn3196_write_byte(led,0x20,0x13);	
			sn3196_write_byte(led,0x21,0x14);	//T4
			sn3196_write_byte(led,0x22,0x15);			
	}	
}


/*******************************************************************************/
//RGB Time , 只对RGB的灯有效
//T1 Ramp up ,T2 hold  T3 Ramp down   T1= 0-7, T2 = 0-7,     ( if DT = 0 then T3= T1 else if DT= 1,T3 =2*T1)
/********************************************************************************/

static void sn3196_SetRgbBreathTime(struct sn3196_led *led,unsigned int Ch, u8 T1,u8 T2,u8 DT)
{
	u8 reg = 0;															 

	  if (( T1> 7)||(T2>7)||(DT >1) ) 
          {
             printk("please set right T1(0-7),T2(0-7),DT(0,1).\n");
             return;
          }	   										  
	  switch (Ch)
	  {
		case 123:
			if ( DT )
			{
				reg = 0x80 |T1|(T2 << 4) ; 
				sn3196_write_byte(led,SN3196_REG_T1_T2_T3_RGB1,reg); //T1,T2,T3
			}
			else
			{
				reg = T1|(T2 << 4) ; 
				sn3196_write_byte(led,SN3196_REG_T1_T2_T3_RGB1,reg); //T1,T2,T3
			}
			break;
			case 456:
			if ( DT )
			{
				reg = 0x80 |T1|(T2 << 4) ; 
				sn3196_write_byte(led,SN3196_REG_T1_T2_T3_RGB2,reg); //T1,T2,T3
			}
			else
			{
				reg = T1|(T2 << 4) ; 
				sn3196_write_byte(led,SN3196_REG_T1_T2_T3_RGB2,reg); //T1,T2,T3
			}
			break;
		}	
}


#define sn3196_CONTROL_REGISTER(reg_addr, reg_name)                \
static ssize_t sn3196_store_reg##reg_addr(struct device *dev,      \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn3196_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    msleep(500);                                                \
    sn3196_write_byte(led, reg_addr, (u8) val);        \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute sn3196_reg##reg_addr##_attr = {     \
    .attr = {.name = reg_name, .mode = 0644},           \
    .store = sn3196_store_reg##reg_addr,               \
};

sn3196_CONTROL_REGISTER(0x00, "0x00");
sn3196_CONTROL_REGISTER(0x01, "0x01");
sn3196_CONTROL_REGISTER(0x03, "0x03");
sn3196_CONTROL_REGISTER(0x04, "0x04");
sn3196_CONTROL_REGISTER(0x05, "0x05");
sn3196_CONTROL_REGISTER(0x06, "0x06");
sn3196_CONTROL_REGISTER(0x07, "0x07");
sn3196_CONTROL_REGISTER(0x08, "0x08");
sn3196_CONTROL_REGISTER(0x09, "0x09");
sn3196_CONTROL_REGISTER(0x0a, "0x0a");
sn3196_CONTROL_REGISTER(0x0b, "0x0b");
sn3196_CONTROL_REGISTER(0x0c, "0x0c");
sn3196_CONTROL_REGISTER(0x10, "0x10");
sn3196_CONTROL_REGISTER(0x11, "0x11");
sn3196_CONTROL_REGISTER(0x12, "0x12");
sn3196_CONTROL_REGISTER(0x13, "0x13");
sn3196_CONTROL_REGISTER(0x14, "0x14");
sn3196_CONTROL_REGISTER(0x15, "0x15");
sn3196_CONTROL_REGISTER(0x16, "0x16");
sn3196_CONTROL_REGISTER(0x1a, "0x1a");
sn3196_CONTROL_REGISTER(0x1b, "0x1b");

sn3196_CONTROL_REGISTER(0x1d, "0x1d");
sn3196_CONTROL_REGISTER(0x1e, "0x1e");
sn3196_CONTROL_REGISTER(0x1f, "0x1f");
sn3196_CONTROL_REGISTER(0x20, "0x20");
sn3196_CONTROL_REGISTER(0x21, "0x21");
sn3196_CONTROL_REGISTER(0x22, "0x22");

sn3196_CONTROL_REGISTER(0x26, "0x26");
sn3196_CONTROL_REGISTER(0xff, "0xff");


static struct device_attribute *sn3196_addr_attributes[] = {
    &sn3196_reg0x00_attr,
    &sn3196_reg0x01_attr,
    &sn3196_reg0x03_attr,
    &sn3196_reg0x04_attr,
    &sn3196_reg0x05_attr,
    &sn3196_reg0x06_attr,
    &sn3196_reg0x07_attr,
    &sn3196_reg0x08_attr,
    &sn3196_reg0x09_attr,
    &sn3196_reg0x0a_attr,
    &sn3196_reg0x0b_attr,
    &sn3196_reg0x0c_attr,
    &sn3196_reg0x10_attr,
    &sn3196_reg0x11_attr,
    &sn3196_reg0x12_attr,
    &sn3196_reg0x13_attr,
    &sn3196_reg0x14_attr,
    &sn3196_reg0x15_attr,
    &sn3196_reg0x16_attr,
    &sn3196_reg0x1a_attr,
    &sn3196_reg0x1b_attr,
    &sn3196_reg0x1d_attr,
    &sn3196_reg0x1e_attr,
    &sn3196_reg0x1f_attr,
    &sn3196_reg0x20_attr,
    &sn3196_reg0x21_attr,
    &sn3196_reg0x22_attr,
    &sn3196_reg0x26_attr,
    &sn3196_reg0xff_attr,
};

static void sn3196_enable_adv_conf(struct sn3196_led *led)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(sn3196_addr_attributes); i++) {
        ret = device_create_file(&led->client->dev,
                        sn3196_addr_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn3196_addr_attributes[i]->attr.name);
            goto failed_remove_files;
        }
    }
    led->adf_on = 1;

    return;

failed_remove_files:
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev,
                        sn3196_addr_attributes[i]);
}

static void sn3196_disable_adv_conf(struct sn3196_led *led)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(sn3196_addr_attributes); i++)
        device_remove_file(&led->client->dev,
                        sn3196_addr_attributes[i]);
    led->adf_on = 0;
}

static ssize_t sn3196_show_adv_conf(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct sn3196_led *led = i2c_get_clientdata(to_i2c_client(dev));
    ssize_t ret;

    down_read(&led->rwsem);
    if (led->adf_on)
        ret = sprintf(buf, "on\n");
    else
        ret = sprintf(buf, "off\n");
    up_read(&led->rwsem);

    return ret;
}

static ssize_t sn3196_store_adv_conf(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct sn3196_led *led = i2c_get_clientdata(to_i2c_client(dev));
    if (!count)
        return -EINVAL;

    down_write(&led->rwsem);
    if (!led->adf_on && !strncmp(buf, "on", 2))
        sn3196_enable_adv_conf(led);
    else if (led->adf_on && !strncmp(buf, "off", 3))
        sn3196_disable_adv_conf(led);
    up_write(&led->rwsem);

    return count;
}

static struct device_attribute sn3196_adv_conf_attr = {
    .attr = {
        .name = "advanced_configuration",
        .mode = 0644,
    },
    .show = sn3196_show_adv_conf,
    .store = sn3196_store_adv_conf,
};

/*#define sn3196_CONTROL_ATTR(attr_name, name_str)           \
static ssize_t sn3196_show_##attr_name(struct device *dev,     \
    struct device_attribute *attr, char *buf)           \
{                                   \
    struct sn3196_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    ssize_t ret;                            \
    down_read(&led->rwsem);                     \
    ret = sprintf(buf, "0x%02x\n", led->attr_name);         \
    up_read(&led->rwsem);                       \
    return ret;                         \
}                                   \
static ssize_t sn3196_store_##attr_name(struct device *dev,        \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn3196_led *led = i2c_get_clientdata(to_i2c_client(dev));\
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
static struct device_attribute sn3196_##attr_name##_attr = {       \
    .attr = {                           \
        .name = name_str,                   \
        .mode = 0644,                       \
    },                              \
    .show = sn3196_show_##attr_name,               \
    .store = sn3196_store_##attr_name,             \
};
*/
static struct device_attribute *sn3196_attributes[] = {
    &sn3196_adv_conf_attr,
};
static void sn3196_set_led1g_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value)  
{               
    struct sn3196_led *led =       
        container_of(led_cdev, struct sn3196_led, cdev_led1g);
    if(((value == LED_OFF) && (led->state == SN3196_OFF))
        || ((value != LED_OFF) && (led->state == SN3196_ON)))
    {
        pr_err(" %s same value, don't change.", __func__);
        return ;
    }
	pr_err("  gpio=%d brightness:%d\n",led->en_gpio, value);

    if (value == LED_OFF)
       {   
		  sn3196_TurnOff(led,AUDIO_LED1);
		  sn3196_TurnOff(led,AUDIO_LED2);
                  sn3196_TurnOff(led,AUDIO_LED3);
                  sn3196_TurnOff(led,AUDIO_LED4);	  
		  sn3196_power_off(led);
		  led->state = SN3196_OFF;
		  #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                  gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                  #endif
    }else
      {
		  sn3196_power_on(led);
	          msleep(100);			
                  sn3196_set_workmode(led,0,0);
	          sn3196_set_imax(led,IMAX_10_MA);
		  sn3196_TurnOn(led,AUDIO_LED1);
	          sn3196_TurnOn(led,AUDIO_LED2);
                  sn3196_TurnOn(led,AUDIO_LED3);
                  sn3196_TurnOn(led,AUDIO_LED4);
		  sn3196_SetBrightness(led,AUDIO_LED1,value);
		  sn3196_SetBrightness(led,AUDIO_LED2,value);
                  sn3196_SetBrightness(led,AUDIO_LED3,value);
                  sn3196_SetBrightness(led,AUDIO_LED4,value);
		  sn3196_DataUpdate(led);
		  led->state = SN3196_ON;
		  #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                  gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                  #endif
      }
}                                   
static void sn3196_set_led1g_blink(struct led_classdev *led_cdev,u8 bblink)
{                               
	struct sn3196_led *led = container_of(led_cdev,struct sn3196_led, cdev_led1g);
    if(((bblink == 1) && (led->state == SN3196_BLINK1))
        || ((bblink == 2) && (led->state == SN3196_BLINK2)) || ((bblink == 3) && (led->state == SN3196_BLINK3))
        || ((bblink == 4) && (led->state == SN3196_BLINK4)) || ((bblink == 0) && (led->state == SN3196_OFF)))
    {
        pr_err(" %s same value, don't change.", __func__);
        return ;
    }
	pr_err("  gpio=%d\n",led->en_gpio);
	sn3196_power_on(led);
	msleep(100);
	if(bblink==1) {// lower battery
		pr_err("blink on mode 1 \n");
                sn3196_set_workmode(led,3,0);
		sn3196_set_imax(led,IMAX_10_MA);
		sn3196_SetBreathTime(led,AUDIO_LED1,0,0,5,1);//t4 = 2.5s
		sn3196_SetBreathTime(led,AUDIO_LED2,0,0,5,1);//t4 = 2.5s
		sn3196_SetRgbBreathTime(led,123,7,2,0);//t1=t3=0.1ms, t2 = 0.5s
		sn3196_TimeUpdate(led);
		sn3196_TurnOn(led,AUDIO_LED1);
		sn3196_TurnOn(led,AUDIO_LED2);
                sn3196_TurnOff(led,AUDIO_LED3);
		sn3196_TurnOff(led,AUDIO_LED4);
		sn3196_SetBrightness(led,AUDIO_LED1,0xff);
		sn3196_SetBrightness(led,AUDIO_LED2,0xff);
		sn3196_DataUpdate(led);
		led->state = SN3196_BLINK1;
		#if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                #endif		
	} else if (bblink==2) {//events
		pr_err("blink on mode 2 \n");
		sn3196_set_workmode(led,3,0);
		sn3196_set_imax(led,IMAX_10_MA);
		sn3196_SetBreathTime(led,AUDIO_LED1,0,0,0,0);//t0=t4=0s
		sn3196_SetBreathTime(led,AUDIO_LED2,0,0,0,0);//t0=t4=0s
		sn3196_SetRgbBreathTime(led,123,2,1,0);//t1=t3=1.04s, t2 = 0.26s
		sn3196_TimeUpdate(led);
		sn3196_TurnOn(led,AUDIO_LED1);
		sn3196_TurnOn(led,AUDIO_LED2);
                sn3196_TurnOff(led,AUDIO_LED3);
		sn3196_TurnOff(led,AUDIO_LED4);
		sn3196_SetBrightness(led,AUDIO_LED1,0xff);
		sn3196_SetBrightness(led,AUDIO_LED2,0xff);
		sn3196_DataUpdate(led);
                led->state = SN3196_BLINK2;
		#if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                #endif   		
	}else if(bblink==3){//The first 5 mins of Charging
		pr_err("blink on mode 3 \n");
		sn3196_set_workmode(led,3,0);
		sn3196_set_imax(led,IMAX_10_MA);
		sn3196_SetBreathTime(led,AUDIO_LED1,0,0,0,0);//t0=t4=0s
		sn3196_SetBreathTime(led,AUDIO_LED2,0,0,0,0);//t0=t4=0s
		sn3196_SetRgbBreathTime(led,123,3,0,0);//t1=t3=2.08s, t2 = 0.1s
		sn3196_TimeUpdate(led);
		sn3196_TurnOn(led,AUDIO_LED1);
		sn3196_TurnOn(led,AUDIO_LED2);
                sn3196_TurnOff(led,AUDIO_LED3);
		sn3196_TurnOff(led,AUDIO_LED4);
		sn3196_SetBrightness(led,AUDIO_LED1,0xff);
		sn3196_SetBrightness(led,AUDIO_LED2,0xff);
		sn3196_DataUpdate(led);
                led->state = SN3196_BLINK3;
		#if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                #endif  
	}else if(bblink==4)//audio mode
	{
		pr_err("blink on mode 4 \n");
		sn3196_set_workmode(led,0,5);
		sn3196_SetAudioGain(led,AGS_21_dB);
		sn3196_set_imax(led,IMAX_10_MA);
		sn3196_TurnOn(led,AUDIO_LED1);
		sn3196_TurnOn(led,AUDIO_LED2);
		sn3196_TurnOn(led,AUDIO_LED3);
		sn3196_TurnOn(led,AUDIO_LED4);
		sn3196_SetBrightness(led,AUDIO_LED1,0xff);
		sn3196_SetBrightness(led,AUDIO_LED2,0xff);
		sn3196_SetBrightness(led,AUDIO_LED3,0xff);
		sn3196_SetBrightness(led,AUDIO_LED4,0xff);
		sn3196_DataUpdate(led);
		led->state = SN3196_BLINK4;
		#if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                #endif
	}else if(bblink==0)
        {
                sn3196_TurnOff(led,AUDIO_LED1);
		sn3196_TurnOff(led,AUDIO_LED2);
                sn3196_TurnOff(led,AUDIO_LED3);
                sn3196_TurnOff(led,AUDIO_LED4);	  
		sn3196_power_off(led);
		led->state = SN3196_OFF;
		#if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
                gpio_direction_output(led->en_gpio2, 1);/*led ic sn3196,gpio2 is used for ind_led*/
                #endif 
        }
}

static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
{

      struct led_classdev *led_cdev = dev_get_drvdata(dev);
      u8 bblink;
      pr_err("in %s,name=%s\n",__func__,led_cdev->name);
      if(*buf=='0')
        bblink=0;
      else if(*buf=='1')
        bblink=1;
      else if(*buf=='2')
	    bblink=2;
	  else if(*buf=='3')
		bblink=3;
	  else 
		bblink=4;

      sn3196_set_led1g_blink(led_cdev,bblink);
      return count;
}


static DEVICE_ATTR(blink, S_IWUSR, NULL, store_blink);


static int sn3196_register_led_classdev(struct sn3196_led *led)
{
    int ret;
    led->cdev_led1g.name = "led_G_3196";
    led->cdev_led1g.brightness = LED_OFF;
    led->cdev_led1g.brightness_set = sn3196_set_led1g_brightness;

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

static int sn3196_init(struct sn3196_led *led)
{
    int ret=0;
    ret= sn3196_write_byte(led,SN3196_REG_RST, 0x00); //reset
    return ret;
}

static int sn3196_parse_dt(struct device *dev, struct sn3196_led *led)
{
	int r = 0;

   led->en_gpio = of_get_named_gpio_flags(dev->of_node,
			"sn31,en-gpio", 0, NULL);

/*led ic sn3196,gpio2 is used for ind_led*/
    #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
    led->en_gpio2 = of_get_named_gpio_flags(dev->of_node,
			"sn31,en-gpio2", 0, NULL);
    #endif
    if ((!gpio_is_valid(led->en_gpio)))
	return -EINVAL;

  return r;
}

static int sn3196_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int ret, i;
    struct sn3196_led *led = devm_kzalloc(&client->dev,
										sizeof(struct sn3196_led), GFP_KERNEL);
    if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
        return -ENOMEM;
    }
    sn3196_parse_dt(&client->dev, led);
	led->client = client;
	led->dev = &client->dev;
 
	i2c_set_clientdata(client, led);
		
    pr_err("sn31 gpio=%d\n",led->en_gpio);
	ret = gpio_request(led->en_gpio, "sn31_en");
	if (ret)
		return  -ENODEV;

    #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
	ret = gpio_request(led->en_gpio2, "sn31_en2");
	if (ret)
		return  -ENODEV;
   #endif
    gpio_direction_output(led->en_gpio, 0);/* led ic sn3196,gpio is used for led ic*/
    
    #if defined(CONFIG_TCT_8X76_IDOL455) || defined(CONFIG_TCT_8X76_POP457)
    gpio_direction_output(led->en_gpio2, 0);/*led ic sn3196,gpio2 is used for ind_led*/
    #endif
	led->state = SN3196_UNKNOW;

    udelay(100);
    i=0;
    do{
        i++;
        ret = sn3196_init(led);
    }while(ret < 0 && i <=3);    
  	    
    init_rwsem(&led->rwsem);

    ret = sn3196_register_led_classdev(led);
	
    for (i = 0; i < ARRAY_SIZE(sn3196_attributes); i++) {   
        ret = device_create_file(&led->client->dev,
                        sn3196_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn3196_attributes[i]->attr.name);
            goto exit;
        }
    }
	ret= sysfs_create_file (&led->cdev_led1g.dev->kobj, &dev_attr_blink.attr);
    return 0;

exit:
	i=0;
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev, sn3196_attributes[i]);
    return ret;
}
static int sn3196_remove(struct i2c_client *client)
{
    struct sn3196_led *led = i2c_get_clientdata(client);
    int i;

	led_classdev_unregister(&led->cdev_led1g);
    if (led->adf_on)
        sn3196_disable_adv_conf(led);
    for (i = 0; i < ARRAY_SIZE(sn3196_attributes); i++)
        device_remove_file(&led->client->dev, sn3196_attributes[i]);
    return 0;
}

static const struct i2c_device_id sn3196_id[] = {
    { "sn3196", 0 },
    { }
};

static struct of_device_id sn3196_match_table[] = {
    { .compatible = "sn31,sn3196", },
    { },
};

static struct i2c_driver sn3196_i2c_driver = {
    .probe		= sn3196_probe,
    .remove		= sn3196_remove,
    .id_table	= sn3196_id,
    .driver = {
				.name    = "sn3196",
				.owner  = THIS_MODULE,
				.of_match_table = sn3196_match_table,
	},
};
static int __init sn3196_i2c_init(void)
{
	return i2c_add_driver(&sn3196_i2c_driver);
}
late_initcall(sn3196_i2c_init);
MODULE_AUTHOR("<zhangyang.sun@tcl.com>");
MODULE_DESCRIPTION("sn3196 LED driver");
MODULE_LICENSE("GPL v2");
