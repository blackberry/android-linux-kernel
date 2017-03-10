/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2012, 2013 STMicroelectronics Limited.
 * Authors: AMS(Analog Mems Sensor)
 *        : Victor Phay <victor.phay@st.com>
 *        : Li Wu <li.wu@st.com>
 *        : Giuseppe Di Giore <giuseppe.di-giore@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define DEBUG
#include <linux/device.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/completion.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#ifdef CONFIG_HAS_EARLYSUSPEND //hank modified
#include <linux/earlysuspend.h>
#endif

#include "fts.h"

#include <linux/notifier.h>
#include <linux/fb.h>

#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#ifdef CONFIG_JANUARY_BOOSTER
#include <linux/input/janeps_booster.h>
#endif

//#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
//#endif

#ifdef CONFIG_EXYNOS_TOUCH_DAEMON
#include <linux/exynos_touch_daemon.h>
extern struct exynos_touch_daemon_data exynos_touch_daemon_data;
#endif
#define VPS_NAME "virtual-proximity"
#define	LINK_KOBJ_NAME	"tp"
static struct class * tp_class;
static struct device * tp_gesture_dev;
static struct device * tp_sensitivity_dev;
static struct device * tp_glove_dev;
static struct device * tp_hover_dev;
static struct device * tp_fw_dev;
static struct device * tp_debug_dev;
static struct device * tp_vr_dev;
struct fts_ts_info *info_globle;
static int wake_up_enable_counter = 0;
static int backlight_level = 127;
/*
 * Uncomment to use polling mode insead of interrupt mode.
 *
 */
 // #define FTS_USE_POLLING_MODE
#if defined(CONFIG_TCT_8X76_IDOL4S)
static char *fts_fhd_fw_filename[] = {
	"st_fts.bin",
	"fts_fw.bin",
	"fts_config_fw.bin",
	"fts64_fw.bin"
};
static char *fts_wqhd_fw_filename[] = {
	"st_fts_wqhd.bin",
	"fts_fw_wqhd.bin",
	"fts_config_fw_wqhd.bin",
	"fts64_fw_wqhd.bin"
};
#else
static char *fts_fw_filename[] = {
	"st_fts.bin",
	"fts_fw.bin",
	"fts_config_fw.bin",
	"fts64_fw.bin"
};
#endif

struct virtualpsensor2 {
	char const *name;
//	struct i2c_client *client;
	struct input_dev *virtualdevice;
	bool vps_enabled;
	struct sensors_classdev vps_cdev;
	bool virtual_proximity_data;
};

struct virtualpsensor2 *vps2;

 struct sensors_classdev virtual_sensors_proximity_cdev2 = {
	.name = VPS_NAME,
	.vendor = "NULL",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


/*
 * Event installer helpers
 */
#define event_id(_e)     EVENTID_##_e
#define handler_name(_h) fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd) \
do { \
	_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd); \
} while (0)


/*
 * Asyncronouns command helper
 */
#define WAIT_WITH_TIMEOUT(_info, _timeout, _command) \
do { \
	if (wait_for_completion_timeout(&_info->cmd_done, _timeout) == 0) { \
		dev_warn(_info->dev, "Waiting for %s command: timeout\n", \
		#_command); \
	} \
} while (0)

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

unsigned char afe_version_same = 0x0;
unsigned char reg_backup[4];
/* forward declarations */
static int fts_suspend(struct i2c_client *client, pm_message_t mesg);
static int fts_resume(struct i2c_client *client);
static void fts_interrupt_enable(struct fts_ts_info *info);
static int fts_fw_upgrade(struct fts_ts_info *info, int mode,int fw_forceupdate,int crc_err);
static int fts_init_hw(struct fts_ts_info *info);
static int fts_init_flash_reload(struct fts_ts_info *info);
static int fts_command(struct fts_ts_info *info, unsigned char cmd);
static void fts_interrupt_set(struct fts_ts_info *info, int enable);
static int fts_systemreset(struct fts_ts_info *info);
extern int input_register_notifier_client(struct notifier_block * nb);
extern int input_unregister_notifier_client(struct notifier_block * nb);
static int fts_get_init_status(struct fts_ts_info *info);
static int fts_get_fw_version(struct fts_ts_info *info);
static int fts_chip_powercycle(struct fts_ts_info *info);
static int fts_chip_initialization(struct fts_ts_info *info);
static void fts_glove_on(void);
static void fts_glove_off(void);
extern u32 panel_res_type_select(void);
static int fts_enable_reg(struct fts_ts_info *rmi4_data,	bool enable);
static int fts_write_reg(struct fts_ts_info *info, unsigned char *reg,
						 unsigned short len)
{
	struct i2c_msg xfer_msg[1];

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = len;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	return (i2c_transfer(info->client->adapter, xfer_msg, 1) != 1);
}


static int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum,
						unsigned char *buf, int num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = info->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return (i2c_transfer(info->client->adapter, xfer_msg, 2) != 2);
}

static inline void fts_set_sensor_mode(struct fts_ts_info *info,int mode)
{
	if(!info)
		return ;
	mutex_lock(&info->fts_mode_mutex);
	info->mode = mode ;
	mutex_unlock(&info->fts_mode_mutex);
	return ;
}

#if 0
static void swipe_gesture_control(char *data, char *reg)
{
	if (!reg)
		return;

	if(data[0]&0x01)
		reg[0] |= (1<<7);
	else
		reg[0] &= ~(1<<7);
	
	if(data[0]&0x02)
		reg[1] |= (1);
	else
		reg[1] &= ~(1);
	
	if(data[0]&0x04)
		reg[1] |= (1<<1);
	else
		reg[1] &= ~(1<<1);
	
	if(data[0]&0x08)
		reg[1] |= (1<<2);
	else
		reg[1] &= ~(1<<2);
}


static void unicode_gesture_control(char *data, char *reg)
{
	/*handler V*/
	if(data[0]&0x01)
		reg[1] |= (1<<5);
	else
		reg[1] &= ~(1<<5);
	/*handler C*/
	if(data[0]&0x02)
		reg[0] |= (1<<3);
	else
		reg[0] &= ~(1<<3);
	/*handler E*/
	if(data[0]&0x04)
		reg[0] |= (1<<6);
	else
		reg[0] &= ~(1<<6);
	/*handler W*/
	if(data[0]&0x08)
		reg[0] |= (1<<5);
	else
		reg[0] &= ~(1<<5);
	/*handler M*/
	if(data[0]&0x10)
		reg[0] |= (1<<4);
	else
		reg[0] &= ~(1<<4);
	/*handler S*/
	if(data[0]&0x20)
		reg[1] |= (1<<7);
	else
		reg[1] &= ~(1<<7);
	/*handler O*/
	if(data[0]&0x80)
		reg[0] |= (1<<2);
	else
		reg[0] &= ~(1<<2);
	/*handler Z*/
	if(data[0]&0x40)
		reg[2] |= (1);
	else
		reg[2] &= ~(1);
}

static void tap_gesture_control(char *data, char *reg)
{
	if(data[0])
		reg[0] |= (1<<1);
	else
		reg[0] &= ~(1<<1);
}

static int fts_set_gesture_reg(struct fts_ts_info *info, char *mode)
{
	int i;
 	unsigned char reg[6] = {0xC1, 0x06};
	unsigned char regcmd[6] = {0xC2, 0x06, 0xFF, 0xFF, 0xFF, 0xFF};

	for(i = 0; i < 4; i++){
		reg[i+2] = *(mode + i);
	}

	fts_write_reg(info, regcmd, sizeof(regcmd));
	usleep_range(5000,6000);
	fts_write_reg(info, reg, sizeof(reg));
	usleep_range(5000,6000);
	#ifdef DEBUG
 	tp_log(" set gesture mode %d %d %d\n", *mode, *(mode+1), *(mode+2));
	#endif
	return 0;
}
#endif
static void fts_set_gesture(struct fts_ts_info *info)
{
	//unsigned long val = 0;
	int error;
	//ssize_t ret = -EINVAL;
	u8 reg1[10] = {0xc3,0x01,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0x00};//enable
	u8 reg2[10] = {0xc3,0x02,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0x00}; //disable
	//struct i2c_client *client = to_i2c_client(dev);
	//struct fts_ts_info *info = i2c_get_clientdata(client);

	//info_globle->gesture_disall;
	pr_err("%s:  %d \n",__func__,info_globle->gesture_disall);
		
	if (( 1 == info_globle->gesture_disall )&&(!(info_globle->doubleclick_enable)))
	{
  		error = fts_write_reg(info_globle,reg2,sizeof(reg2)); //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			pr_err("FTS :%s Cannot write reg2 ,disable gesture.\n",__func__);
		}
	} else	if (( 0 == info_globle->gesture_disall )||(info_globle->doubleclick_enable))
	{
  		error = fts_write_reg(info_globle,reg1,sizeof(reg1)); //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			pr_err("FTS :%s Cannot write reg1 ,disable gesture.\n",__func__);
		}
	fts_command(info_globle, ENTER_GESTURE_MODE);
	} else {
		pr_err("invalid  command! \n");
		return;
	}

}


void fts_cover_on(void)
{
	int ret =0;
	unsigned char regAdd[2] = {0xC1, 0x04};//cover on
	tp_log("fts cover on set begin.\n");
	//close glove and hover before enable cover
	if(info_globle->glove_bit)
	fts_glove_off();
	if(info_globle->hover_bit)
	fts_command(info_globle, HOVER_OFF);
	ret = fts_write_reg(info_globle, regAdd, sizeof(regAdd)); 
	usleep_range(5000,6000);
	fts_command(info_globle, FORCECALIBRATION);
	if(ret){
		tp_log("fts cover on set failed.\n");
	}
}
EXPORT_SYMBOL_GPL(fts_cover_on);

void fts_cover_off(void)
{
	int ret =0;
	unsigned char regAdd[2] = {0xC2, 0x04};//cover off
	tp_log("fts cover off set begin.\n");
	ret = fts_write_reg(info_globle, regAdd, sizeof(regAdd)); 
	usleep_range(5000,6000);
	if(ret){
		tp_log("fts cover off set failed.\n");
	}
	//recovery glove and hover after disable cover
	if(info_globle->glove_bit){
		fts_glove_on();
		fts_set_sensor_mode(info_globle, MODE_GLOVE);
	}
	if(info_globle->hover_bit){
		fts_command(info_globle, HOVER_ON);
		fts_set_sensor_mode(info_globle, MODE_HOVER);

	}
}
EXPORT_SYMBOL_GPL(fts_cover_off);

void fts_vr_on(void)
{
	int ret =0;
	unsigned char regAdd[2] = {0xC1, 0x10};//vr on
	if(!info_globle->vr_bit)
	{
		tp_log("vr_bit is 0, so don't enable vr function \n");
		return;
	}
	tp_log("fts vr on set begin.\n");
	//close glove and hover before enable vr
	if(info_globle->glove_bit)
	fts_glove_off();
	if(info_globle->hover_bit)
	fts_command(info_globle, HOVER_OFF);
	ret = fts_write_reg(info_globle, regAdd, sizeof(regAdd));
	usleep_range(5000,6000);
	fts_command(info_globle, FORCECALIBRATION);
	if(ret){
		tp_log("fts vr on set failed.\n");
	}
}

void fts_vr_off(void)
{
	int ret =0;
	unsigned char regAdd[2] = {0xC2, 0x10};//vr off
	tp_log("fts vr off set begin.\n");
	ret = fts_write_reg(info_globle, regAdd, sizeof(regAdd));
	usleep_range(5000,6000);
	if(ret){
		tp_log("fts vr off set failed.\n");
	}
	//recovery glove and hover after disable vr
	if(info_globle->glove_bit){
		fts_glove_on();
		fts_set_sensor_mode(info_globle, MODE_GLOVE);
	}
	if(info_globle->hover_bit){
		fts_command(info_globle, HOVER_ON);
		fts_set_sensor_mode(info_globle, MODE_HOVER);

	}
}

void fts_vr_on_work(struct work_struct *work)
{
	fts_vr_on();
}

void fts_glove_on(void)
{
	int ret =0;
	u8 reg[3] = {0xc1,0x01,0x01};//enable
	tp_log("fts glove on set begin.\n");
	ret = fts_write_reg(info_globle,reg,sizeof(reg)); //error = fts_write_reg(info_gl,reg,sizeof(reg));
	if(ret){
		tp_log("fts cover on set failed.\n");
	}
}

void fts_glove_off(void)
{
	int ret =0;
	u8 reg[3] = {0xc2,0x01,0x01}; //disable
	tp_log("fts glove off set begin.\n");
	ret = fts_write_reg(info_globle, reg, sizeof(reg));
	if(ret){
		tp_log("fts cover off set failed.\n");
	}
}


static ssize_t fts_gesture_data_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	
	int count = snprintf(buf, PAGE_SIZE, "%u\n",info->gesture_value);
	#ifdef DEBUG
	tp_log("gesture %x detect \n",info->gesture_value);
	#endif
	info->gesture_value = GESTURE_ERROR ;
	return count ;
}

/* force update firmware*/
/*
static ssize_t fts_fw_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", info->fw_version);
}
*/
static ssize_t fts_fw_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* reading out firmware upgrade mode */
	sscanf(buf, "%d", &mode);

	info->fw_force = 1;
	ret = fts_fw_upgrade(info, mode,0,0);
	info->fw_force = 0;
	if (ret)
		dev_err(dev, "Unable to upgrade firmware\n");
	return count;
}


static ssize_t fts_gesture_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	//return scnprintf(buf, PAGE_SIZE, "%04x\n", info->gesture_bit);
	int *p = (int *)info->gesture_mask;
	memcpy(buf, p, 4);
	#ifdef DEBUG
	tp_log("gesture mask %x %p\n", *buf, buf);
	#endif
	return 4;

}

static ssize_t fts_gesture_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int temp;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	
#if 1
	const char * data = buf ;
	tp_log("turn on/off [%x][%x][%x][%x] gesture control\n",
			  *(data+3),*(data+2),*(data+1),*(data));

#else
	int data[4];
	sscanf(buf, "%d %d %d %d", data+3, data+2, data+1, data);
		 tp_log("%s: turn on/off [%x][%x][%x][%x] gesture control\n",
		 __func__, *(data+3),*(data+2),*(data+1),*(data));

#endif

	if(data[2] == ALL_CTR) {
		info->gesture_disall = !data[0];
	}else if(data[2]==SWIPE_CTR){
		info->gesture_mask[SWIPE_INDEX] = 0x0F&data[0] ;
		info->gesture_mask[ALL_INDEX]   = 2<<6 ;
	}else if(data[2]==UNICODE_CTR){
		info->gesture_mask[UNICODE_INDEX] = 0xFF&data[0] ;
		info->gesture_mask[ALL_INDEX]     = 2<<6 ;
	}else if(data[2]==TAP_CTR){
		info->gesture_mask[TAP_INDEX] = 0x01&data[0] ;
		info->gesture_mask[ALL_INDEX] = 2<<6 ;
	}else {
		tp_log("parse gesture type error\n");
		return -EIO ;
	}

	temp= ((info->gesture_mask[SWIPE_INDEX]==0x0F)&&
			   (info->gesture_mask[UNICODE_INDEX]==0xFF)&&
			   (info->gesture_mask[TAP_INDEX]==0x01));
	info->gesture_mask[ALL_INDEX] = (temp?1:2)<<6 ;

	return count;
}

static ssize_t fts_glove_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", info->glove_bit);
}

static ssize_t fts_glove_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* reading out firmware upgrade mode */
	sscanf(buf, "%d", &info->glove_bit);
	tp_log("%s: glove mode: %d\n",__func__, info->glove_bit);
	if(info->glove_bit) {
		if(info->hover_bit){
			info->hover_bit = 0;
			fts_command(info, HOVER_OFF);
		}
		ret = fts_command(info, GLOVE_ON);
		fts_set_sensor_mode(info, MODE_GLOVE);
	}else {
		ret = fts_command(info, GLOVE_OFF);
		fts_set_sensor_mode(info, MODE_NORMAL);
	}
	if (ret)
		tp_log("%s: glove mode: %d : failed !\n",__func__, info->glove_bit);
	return count;
}

static ssize_t fts_hover_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", info->hover_bit);
}

static ssize_t fts_hover_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* reading out firmware upgrade mode */
	sscanf(buf, "%d", &info->hover_bit);
	tp_log("%s: hover mode: %d\n",__func__, info->hover_bit);
	if(info->hover_bit) {
		if(info->glove_bit){
			info->glove_bit = 0;
			fts_command(info, GLOVE_OFF);
		}
		ret = fts_command(info, HOVER_ON);
		fts_set_sensor_mode(info, MODE_HOVER);
	}else {
		ret = fts_command(info, HOVER_OFF);
		fts_set_sensor_mode(info, MODE_NORMAL);
	}
	if (ret)
		tp_log("%s: hover mode: %d : failed !\n",__func__, info->hover_bit);
	return count;
}

static ssize_t fts_cover_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", info->cover_bit);
}

static ssize_t fts_cover_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* reading out firmware upgrade mode */
	sscanf(buf, "%d", &info->cover_bit);
	tp_log("%s: cover mode: %d",__func__, info->cover_bit);
	
	if(info->cover_bit) {
		if(info->resume_bit){
			fts_command(info, GLOVE_OFF);
			fts_command(info, HOVER_OFF);
			fts_cover_on();
			usleep_range(5000,6000);
			fts_command(info, FORCECALIBRATION);
		}
		fts_set_sensor_mode(info, MODE_COVER);
 	}else {
 		    if(info->resume_bit){
 			   fts_cover_off();
 			   usleep_range(5000,6000);
 			   fts_command(info, FORCECALIBRATION);
 			}
 			fts_set_sensor_mode(info, MODE_NORMAL);
 
 			if(info->glove_bit){
 				if(info->resume_bit){
 					fts_command(info, GLOVE_ON);
 				}
 				fts_set_sensor_mode(info, MODE_GLOVE);
 		    }
 
 			if(info->hover_bit){
 				if(info->resume_bit){
 					fts_command(info, HOVER_ON);
 				}
 				fts_set_sensor_mode(info, MODE_HOVER);
 			}
 			
 	}
 	return count;
}

#if 0
static int fts_cover_notity_func(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct fts_ts_info * info =
				container_of(nb, struct fts_ts_info,cover_notifier);

	tp_log("hall_notify action[%ld]mode[%d]\n",action,info->mode);

    if(HALL_COVER==action){

		if(info->resume_bit){
			fts_command(info, GLOVE_OFF);
			fts_command(info, HOVER_OFF);
			fts_cover_on(info);
			mdelay(5);
			fts_command(info, FORCECALIBRATION);
		}
		
		fts_set_sensor_mode(info, MODE_COVER);
	}else {	
		 if(info->resume_bit){
 			   fts_cover_off(info);
 			   mdelay(5);
 			   fts_command(info, FORCECALIBRATION);
 			}
 			fts_set_sensor_mode(info, MODE_NORMAL);
 
 			if(info->glove_bit){
 				if(info->resume_bit)
 				fts_command(info, GLOVE_ON);
 				fts_set_sensor_mode(info, MODE_GLOVE);
 		    }
 
 			if(info->hover_bit){
 				if(info->resume_bit)
 				fts_command(info, HOVER_ON);
 				fts_set_sensor_mode(info, MODE_HOVER);
 			}
	}
	
   return 0 ;
}


static void fts_register_cover_notify(struct fts_ts_info *info)
{
	info->cover_notifier.notifier_call = fts_cover_notity_func ;
	input_register_notifier_client(&(info->cover_notifier));
}
#endif

static ssize_t fts_sysfs_config_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int error;		

    error = fts_get_fw_version(info);
	if(error) {
		tp_log("%s: can not get fw version!\n", __func__);
	}

	error = snprintf(buf, PAGE_SIZE, "%s:%x:%s:%x,,\n","86FTS",info->config_id,"FTM3BD54",info->fw_version);
	return error;
}
static ssize_t fts_sysfs_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

    //fwupdate_stat: 2:  sucess,;  1: failed; 0: No firmware update.
	return sprintf(buf, "%d\n", info->fwupdate_stat);
}

static unsigned int le_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] + (unsigned int)ptr[1] * 0x100;
}
static unsigned int be_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[1] + (unsigned int)ptr[0] * 0x100;
}
static ssize_t fts_fw_test_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	const struct firmware *fw = NULL;
	unsigned char *data;
	unsigned int size;
	char *firmware_name = "st_fts.bin";
	int fw_version;
	int config_version;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int retval;
	
	retval = request_firmware(&fw,firmware_name,info->dev);
	if(retval){
		tp_log("%s: request_firmware failed!\n",__func__);
	}
	
	data =(unsigned char *)fw->data;
	size = fw->size;

	fw_version = le_to_uint(&data[FILE_FW_VER_OFFSET]);	
	config_version = be_to_uint(&data[FILE_CONFIG_VER_OFFSET]);  // read correct address to get config ID in file

	tp_log("%s: fw_version = %x, config_version = %x, size = %d\n", __func__, fw_version, config_version, size);

	return 0;
}


static ssize_t fts_touch_debug_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	sscanf(buf,"%d", &info->touch_debug);
	return count;
}

/* *******************************************Production test****************************** */
/*INITIALIZATION Test*/
static ssize_t fts_initialization_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char result=0,ret  = 0 ; 
	char buff[CMD_STR_LEN] = { 0 };
	char all_strbuff[CMD_STR_LEN]={0};
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int size = 0;
	int error;
	size = sizeof(all_strbuff);
	
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	ret =  fts_chip_initialization(info);
	if (!ret)
		result = 0x00;
		else
		result = 0x01;
	error = fts_systemreset(info);
	msleep(150);
	error += fts_command(info, FORCECALIBRATION);
	error += fts_command(info, SENSEON);
#ifdef PHONE_KEY
	fts_command(info, KEYON);
#endif					
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);
	error += fts_command(info, FLUSHBUFFER);
	if (error) 
	{
		tp_log("%s: Cannot reset the device----------\n", __func__);
	}	
	snprintf(buff, sizeof(buff), "%02X",result);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);

}
/*ITO TEST*/
static ssize_t fts_ito_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	unsigned char retry = 0;
	unsigned char error ;
	unsigned char regAdd = 0;
	unsigned int ito_check_status = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	char buff[CMD_STR_LEN] = { 0 };
	char all_strbuff[CMD_STR_LEN]={0};
	int size = 0;
	size = sizeof(all_strbuff);	
	
	fts_systemreset(info);
	msleep(200);
	fts_command(info, SENSEOFF);
	msleep(10);
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);
	fts_command(info, ITO_CHECK);
	msleep(200);
	tp_log("fts ITO Check Start \n");

	for (retry = 0; retry < READ_CNT_ITO; retry++) 
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			tp_log("fts_ito_test_show : i2C READ ERR , Cannot read device info\n");
			return -ENODEV;
		}
		tp_log("FTS ITO event : %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x0F) && (tune_flag == 0x05)) 
		{
			if((data[2] == 0x00) && (data[3] == 0x00)){
				ito_check_status = 0;
				tp_log("fts ITO check ok \n");
				break;
			}
			else
			{
				ito_check_status = 1;
				tp_log("fts ITO check fail \n");

				switch (data[2]) 
				{
					case ERR_ITO_PANEL_OPEN_FORCE :
						tp_log("ITO Test result : Force channel [%d] open.\n",
							data[3]);
						break;
					case ERR_ITO_PANEL_OPEN_SENSE :
						tp_log("ITO Test result : Sense channel [%d] open.\n",
							data[3]);
						break;
					case ERR_ITO_F2G :
						tp_log("ITO Test result : Force channel [%d] short to GND.\n",
							data[3]);
						break;
					case ERR_ITO_S2G :
						tp_log("ITO Test result : Sense channel [%d] short to GND.\n",
							data[3]);
						break;
					case ERR_ITO_F2VDD :
						tp_log("ITO Test result : Force channel [%d] short to VDD.\n",
							data[3]);
						break;
					case ERR_ITO_S2VDD :
						tp_log("ITO Test result : Sense channel [%d] short to VDD.\n",
							data[3]);
						break;
					case ERR_ITO_P2P_FORCE :
						tp_log("ITO Test result : Force channel [%d] ,Pin to Pin short.\n",
							data[3]);
						break;
					case ERR_ITO_P2P_SENSE :
						tp_log("ITO Test result : Sense channel [%d] Pin to Pin short.\n",
							data[3]);
						break;					
			
					default:
						break;
				}//switch case
				break;//for loop
			}
		}
		else
		{
			msleep(5);
			if (retry == READ_CNT_ITO) 
			{
				tp_log("Time over - wait for result of ITO test\n");
			}
		}
	}
	
	fts_systemreset(info);
	msleep(200);
//	fts_command(info, SENSEON);
//	msleep(10);
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);
	fts_command(info, SENSEON);
	fts_command(info, FORCECALIBRATION);
		
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",ito_check_status);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);	
}//ITO Test

/*  2.2	Read Mutual Tune Data  */
static ssize_t fts_read_cx_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *mutual_cx_data =NULL;
	unsigned char cx1_num = 0;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int 	j = 0, 
		error =0 , 
		address_offset = 0,
		start_tx_offset = 0, 
		retry = 0,
		size = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEOFF);	
	fts_command(info, FLUSHBUFFER);
	msleep(50);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x02;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x02)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
	
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	cx1_num = buff_read[10];
	
	all_strbuff = (unsigned char *)kmalloc(((tx_num * rx_num)+5)*2, GFP_KERNEL);
	mutual_cx_data = (unsigned char *)kmalloc(((tx_num * rx_num)+1), GFP_KERNEL);// added one to accommodate  dummy byte

	memset(all_strbuff,0,sizeof(char)*((tx_num*rx_num+5)*2));	//size 3  ex(45,)
	size = ((tx_num*rx_num+5)*2);//size of message byte * 3()
	/* <0xAA> <f_cnt>*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	snprintf(buff, sizeof(buff), "%02X",cx1_num);
	strncat(all_strbuff, buff, size);	
	// ============================================ Cx data Reading ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_cx_data, ((tx_num * rx_num)+1));

	
	/*Copying CX1 and CX2 data  */
	for (j = 1; j < ((tx_num * rx_num)+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",mutual_cx_data[j]);
			strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	kfree(all_strbuff);
	kfree(mutual_cx_data);
	return count;
}//Read Mutual Tune Data

/*  2.3	Read Self Tune Data  */
static ssize_t fts_read_self_tune_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *self_tune_data =NULL;
	unsigned char f_ix1_num = 0;
	unsigned char s_ix1_num = 0;
	unsigned char f_cx1_num = 0;
	unsigned char s_cx1_num = 0;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int 	j = 0, 
		error =0 , 
		address_offset = 0,
		start_tx_offset = 0, 
		retry = 0,
		size = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEOFF);	
	fts_command(info, FLUSHBUFFER);
	msleep(50);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
	
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	
	f_ix1_num = buff_read[10];
	s_ix1_num = buff_read[11];
	f_cx1_num = buff_read[12];
	s_cx1_num = buff_read[13];
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/text */
	all_strbuff = (unsigned char *)kmalloc(((tx_num + rx_num)* 2 + 8) *2, GFP_KERNEL);
	self_tune_data = (unsigned char *)kmalloc(((tx_num + rx_num)* 2 + 1), GFP_KERNEL);// added one to accommodate  dummy byte ((M+N) + (M +N))

	memset(all_strbuff,0,sizeof(char)*(((tx_num + rx_num)* 2 + 8) *2));	
	size = (((tx_num + rx_num)* 2 + 8) *2);//size of message byte * 2
	
	/* <0xAA> <f_cnt>*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	snprintf(buff, sizeof(buff), "%02X",f_ix1_num);
	strncat(all_strbuff, buff, size);	
	
	snprintf(buff, sizeof(buff), "%02X",s_ix1_num);
	strncat(all_strbuff, buff, size);

	snprintf(buff, sizeof(buff), "%02X",f_cx1_num);
	strncat(all_strbuff, buff, size);

	snprintf(buff, sizeof(buff), "%02X",s_cx1_num);
	strncat(all_strbuff, buff, size);	
	// ============================================ Self tune data Reading ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, self_tune_data, ((tx_num + rx_num)*2+1));
	
	/*Copying Self tune data data  */
	for (j = 1; j < ((tx_num + rx_num)*2+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",self_tune_data[j]);
			strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	
	kfree(all_strbuff);
	kfree(self_tune_data);
	return count;
}//Read Self Tune Data 

/*  2.4	Read Mutual Raw Data  */ 
static ssize_t fts_read_mutual_raw_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *mutual_raw_data =NULL;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		error =0 , 
		address_offset = 0,
	//	start_tx_offset = 0, 
		retry = 0,
		size = 0,
		size_hex =0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
	fts_command(info, SENSEON);
	msleep(100);
	fts_command(info, SENSEOFF);
	msleep(50);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	size = (((tx_num * rx_num)* 2 + 4) *2);//size of message byte * 2
	size_hex = (tx_num * rx_num)* 2 ;
	
	/* Read Offset Address for Mutual Raw data*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x00;//Offset address for MS screen raw frame, Please verify this address from supporting fw engineer
	fts_read_reg(info,regAdd,3, &buff_read[0], 4);
	
	address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
	//address_offset  = start_tx_offset + 0x10;
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
	all_strbuff = (unsigned char *)kmalloc(size+1, GFP_KERNEL);
	mutual_raw_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)

	memset(all_strbuff,0,sizeof(char)*size);	
		
	/* <0xAA> <f_cnt>...*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	// ============================================ Mutual Raw data Reading ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_raw_data, (size_hex+1));
	
	/*Copying Mutual Raw data  */
	for (j = 1; j < (size_hex+1); j++) 
	{
		snprintf(buff, sizeof(buff), "%02X",mutual_raw_data[j]);
		strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	
	kfree(all_strbuff);
	kfree(mutual_raw_data);
	return count;
}//2.4	Read Mutual Raw Data 

/*  2.5	Read Self Raw Data  */
static ssize_t fts_read_self_raw_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *self_raw_data_force =NULL;
	unsigned char *self_raw_data_sense =NULL;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		error =0 , 
		address_offset_force = 0,
		address_offset_sense = 0,
		retry = 0,
		size = 0 ,
		size_hex_force= 0,
		size_hex_sense = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
	fts_command(info, SENSEON);
	msleep(100);
	fts_command(info, SENSEOFF);
	msleep(50);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	
	size = (((tx_num * rx_num)* 2 + 4) *2);//size of message byte * 2
	size_hex_force = tx_num * 2 ;
	size_hex_sense = rx_num * 2 ;
	
	/* Read Offset Address for Self Raw data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x1A;//Offset address for SS touch raw force line, Please verify this address from supporting fw engineer 
	fts_read_reg(info,regAdd, 3, &buff_read[0], 5);
	
	address_offset_force = ((buff_read[2]<<8) |buff_read[1]);
	address_offset_sense = ((buff_read[4]<<8) |buff_read[3]);
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ASCII/text */
	all_strbuff = (unsigned char *)kmalloc(size, GFP_KERNEL);
	self_raw_data_force = (unsigned char *)kmalloc((size_hex_force + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	self_raw_data_sense = (unsigned char *)kmalloc((size_hex_sense + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	
	memset(all_strbuff,0,sizeof(char)*(((tx_num * rx_num)* 2 + 4) *2));	
		
	/* <0xAA> <f_cnt>...*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	// ============================================ Self Raw data Reading(force) ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_force & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_force & 0xFF);
	fts_read_reg(info,regAdd, 3, self_raw_data_force, (size_hex_force+1));
	
	/*Copying self raw data Force  */
	for (j = 1; j < (size_hex_force+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",self_raw_data_force[j]);
			strncat(all_strbuff, buff, size);
	}
	
	// ============================================ Self Raw data Reading(sense) ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_sense & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_sense & 0xFF);
	fts_read_reg(info,regAdd, 3, self_raw_data_sense, (size_hex_sense+1));
	
	/*Copying self raw data Sense  */
	for (j = 1; j < (size_hex_sense+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",self_raw_data_sense[j]);
			strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	
	kfree(all_strbuff);
	kfree(self_raw_data_force);
	kfree(self_raw_data_sense);
	return count;
	
}//2.5	Read Self Raw Data 

/*  2.6	Read Mutual Strength Data  */
static ssize_t fts_read_mutual_strength_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *mutual_strength_data =NULL;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		error =0 , 
		address_offset = 0,
	//	start_tx_offset = 0, 
		retry = 0,
		size = 0,
		size_hex =0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEON);	
	fts_command(info, FLUSHBUFFER);
	msleep(50); // TODO: add longer delay time after sense on ?
	
	/* Request Compensation Data,just get sense and force lines*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_mutual_Strength_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_mutual_raw_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	size = (((tx_num * rx_num)* 2 + 4) *2);//size of message byte * 2
	size_hex = (tx_num * rx_num)* 2 ;
	
	/* Read Offset Address for Mutual Strength data*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x04;//Offset address for MS screen Strength frame, Please verify this address from supporting fw engineer
	fts_read_reg(info,regAdd,3, &buff_read[0], 4);
	
	address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
	//address_offset  = start_tx_offset + 0x10;
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
	all_strbuff = (unsigned char *)kmalloc(size+1, GFP_KERNEL);
	mutual_strength_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)

	memset(all_strbuff,0,sizeof(char)*size);	
		
	/* <0xAA> <f_cnt>...*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	// ============================================ Mutual strength data Reading ======================================//	
	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_strength_data, (size_hex+1));
	
	/*Copying Mutual strength data  */
	for (j = 1; j < (size_hex+1); j++) 
	{
		snprintf(buff, sizeof(buff), "%02X",mutual_strength_data[j]);
		strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	//fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	
	kfree(all_strbuff);
	kfree(mutual_strength_data);
	return count;
}//2.6	Read Mutual Strength Data 

/*  2.7	Read Self Strength Data  */
static ssize_t fts_read_self_strength_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char *all_strbuff =NULL;
	unsigned char *self_strength_data_force =NULL;
	unsigned char *self_strength_data_sense =NULL;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		error =0 , 
		address_offset_force = 0,
		address_offset_sense = 0,
		retry = 0,
		size = 0 ,
		size_hex_force= 0,
		size_hex_sense = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEON);	
	fts_command(info, FLUSHBUFFER);
	msleep(50);  //TODO: add longer delay time after sense on ?
	
	/* Request Compensation Data,just get sense and force lines*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_self_strength_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_self_strength_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	
	/* f_cnt and s_cnt*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	
	size = (((tx_num * rx_num)* 2 + 4) *2);//size of message byte * 2
	size_hex_force = tx_num * 2 ;
	size_hex_sense = rx_num * 2 ;
	
	/* Read Offset Address for Self strength data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x22;//Offset address for SS touch strength force line, Please verify this address from supporting fw engineer 
	fts_read_reg(info,regAdd, 3, &buff_read[0], 5);
	
	address_offset_force = ((buff_read[2]<<8) |buff_read[1]);
	address_offset_sense = ((buff_read[4]<<8) |buff_read[3]);
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ASCII/text */
	all_strbuff = (unsigned char *)kmalloc(size, GFP_KERNEL);
	self_strength_data_force = (unsigned char *)kmalloc((size_hex_force + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	self_strength_data_sense = (unsigned char *)kmalloc((size_hex_sense + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	
	memset(all_strbuff,0,sizeof(char)*(((tx_num * rx_num)* 2 + 4) *2));	
		
	/* <0xAA> <f_cnt>...*/
	snprintf(buff, sizeof(buff), "%02X",0xAA);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",tx_num);
	strncat(all_strbuff, buff, size);
	
	snprintf(buff, sizeof(buff), "%02X",rx_num);
	strncat(all_strbuff, buff, size);

	// ============================================ Self strength data Reading(force) ======================================//	
	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_force & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_force & 0xFF);
	fts_read_reg(info,regAdd, 3, self_strength_data_force, (size_hex_force+1));
	
	/*Copying self strength data Force  */
	for (j = 1; j < (size_hex_force+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",self_strength_data_force[j]);
			strncat(all_strbuff, buff, size);
	}
	
	// ============================================ Self strength data Reading(sense) ======================================//	
	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_sense & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_sense & 0xFF);
	fts_read_reg(info,regAdd, 3, self_strength_data_sense, (size_hex_sense+1));
	
	/*Copying self strength data Sense  */
	for (j = 1; j < (size_hex_sense+1); j++) {
			snprintf(buff, sizeof(buff), "%02X",self_strength_data_sense[j]);
			strncat(all_strbuff, buff, size);
	}
	
	snprintf(buff, sizeof(buff), "%02X",0xBB);
	strncat(all_strbuff, buff, size);
	
	fts_interrupt_set(info, INT_ENABLE);
	//fts_command(info, SENSEON);
	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	
	kfree(all_strbuff);
	kfree(self_strength_data_force);
	kfree(self_strength_data_sense);
	return count;
	
}//2.7	Read Self strength Data 
static DEVICE_ATTR(fwupdate, (S_IRUGO|S_IWUSR|S_IWGRP), fts_sysfs_fwupdate_show, fts_fw_control_store);
static DEVICE_ATTR(gesture_control, (S_IRUGO|S_IWUSR|S_IWGRP), fts_gesture_control_show, fts_gesture_control_store);
static DEVICE_ATTR(gesture_data1, S_IRUGO, fts_gesture_data_read, NULL);
static DEVICE_ATTR(glove_control, (S_IRUGO|S_IWUSR|S_IWGRP), fts_glove_control_show, fts_glove_control_store);
static DEVICE_ATTR(appid, (S_IRUGO), fts_sysfs_config_id_show, NULL);
static DEVICE_ATTR(update_test, (S_IRUGO),fts_fw_test_show, NULL);
static DEVICE_ATTR(cover_control, (S_IRUGO|S_IWUSR|S_IWGRP),fts_cover_control_show, fts_cover_control_store);
static DEVICE_ATTR(hover_control, (S_IRUGO|S_IWUSR|S_IWGRP), fts_hover_control_show, fts_hover_control_store);
/** factory test */
static DEVICE_ATTR(init_test,(S_IRUGO), fts_initialization_test_show, NULL);
static DEVICE_ATTR(ito_test,(S_IRUGO), fts_ito_test_show, NULL);
static DEVICE_ATTR(touch_debug,(S_IWUSR|S_IWGRP), NULL, fts_touch_debug_store);
static DEVICE_ATTR(read_mutual_cx,(S_IRUGO), fts_read_cx_show, NULL);
static DEVICE_ATTR(read_self_cx,(S_IRUGO), fts_read_self_tune_show, NULL);
static DEVICE_ATTR(read_mutual_raw,(S_IRUGO), fts_read_mutual_raw_show, NULL);
static DEVICE_ATTR(read_self_raw,(S_IRUGO), fts_read_self_raw_show, NULL);
static DEVICE_ATTR(read_mutual_strength,(S_IRUGO), fts_read_mutual_strength_show, NULL);
static DEVICE_ATTR(read_self_strength,(S_IRUGO), fts_read_self_strength_show, NULL);

static struct attribute *fts_attr_group[] = {
	&dev_attr_fwupdate.attr,
	&dev_attr_gesture_control.attr,
	&dev_attr_gesture_data1.attr,
	&dev_attr_glove_control.attr,
	&dev_attr_appid.attr,
	&dev_attr_update_test.attr,
	&dev_attr_cover_control.attr,
	&dev_attr_hover_control.attr,
	&dev_attr_init_test.attr,
	&dev_attr_ito_test.attr,
	&dev_attr_touch_debug.attr,
	&dev_attr_read_mutual_cx.attr,
	&dev_attr_read_self_cx.attr,
	&dev_attr_read_mutual_raw.attr,
	&dev_attr_read_self_raw.attr,
	&dev_attr_read_mutual_strength.attr,
	&dev_attr_read_self_strength.attr,
	NULL,
};
static ssize_t fts_gesture_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", !(info_globle->gesture_disall));
	return ret;
}

static ssize_t  fts_gesture_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val = 0;
	//int error;
	ssize_t ret = -EINVAL;
	//u8 reg1[6] = {0xc3,0x01,0xff,0xff,0xff,0xff};//enable
	//u8 reg2[6] = {0xc3,0x02,0x00,0x00,0x00,0x00}; //disable
	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	printk("gesture enable = %ld \n", val);
	info_globle->gesture_filter = val;
	info_globle->gesture_disall = !val;
	return size;

		return 0;
}

static ssize_t fts_gesture_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	//char *temp_buff=NULL;
	unsigned char regAdd[3] = {0xd0,0x00,0x00};
	u16 count,i;
	char data[73];
	char *temp_buff=NULL;
	//u16 count,i;
	char * pbuff;

	temp_buff = devm_kzalloc(dev,(50)*10, GFP_KERNEL);
	if(temp_buff == NULL)
		printk("temp_buff");

	printk("info_globle->gesture_reg[0]:%x,gesture_reg[1]:%x \n",info_globle->gesture_reg[0],info_globle->gesture_reg[1]);
	regAdd[1] = info_globle->gesture_reg[1];    // Ofset address
	regAdd[2] = info_globle->gesture_reg[0];    // Ofset address
	fts_read_reg(info_globle, regAdd, 3, data, 73);

	pbuff = temp_buff;
	count = 0;
	for(i=0;i<36;i++)
	{
		count = sprintf(pbuff,"%02x%02x,",data[2*i+2],data[2*i+1]);
		pbuff += count;
	}

	count = sprintf(buf, "0x%02x,0x%02x:%s\n", info_globle->gesture_value,info_globle->gesture_type,(char *)temp_buff);

	//for(i=0;i<36;i++)
	//printk("data[%d] = %02x%02x \n",i,data[2*i+2],data[2*i+1]);

	devm_kfree(dev,temp_buff);
	return count;


		return 0;
}

static ssize_t  fts_gesture_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{

		return 0;
}

static ssize_t fts_db_enalbe_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->doubleclick_enable);
	return ret;
}

static ssize_t  fts_db_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val = 0;
	int error;
	ssize_t ret = -EINVAL;
	u8 reg1[6] = {0xc3,0x01,0xff,0xff,0xff,0xff};//enable
	u8 reg2[6] = {0xc3,0x02,0x00,0x00,0x00,0x00}; //disable
	//struct i2c_client *client = to_i2c_client(dev);
	//struct fts_ts_info *info = i2c_get_clientdata(client);

	
	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;
	printk("db enable = %ld \n", val);
	//info_globle->gesture_disall = !val;
	info_globle->doubleclick_enable = val;
	return size;
	
	if ( 0 == val )
	{
  		error = fts_write_reg(info_globle,reg2,sizeof(reg2)); //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			pr_err("FTS :%s Cannot write reg2 ,disable gesture.\n",__func__);
		}
	} else	if ( 1 == val )
	{
  		error = fts_write_reg(info_globle,reg1,sizeof(reg1)); //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			pr_err("FTS :%s Cannot write reg2 ,disable gesture.\n",__func__);
		}
	fts_command(info_globle, ENTER_GESTURE_MODE);
	} else {
		pr_err("invalid  command! \n");
	//	return size;
	}
	printk("db enable = %ld \n", val);

	return size;
	
	return 0;
}

static ssize_t fts_cover_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->cover_bit);
	return ret;
}

static ssize_t  fts_cover_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return size;
	info_globle->cover_bit = val;
	if(info_globle->sensor_sleep)
	{
		tp_log("tp is sleep mode ,can't enter/exit cover mode\n");
		return size;
	}
	if(val)
		fts_cover_on();
	else
		fts_cover_off();

	return size;
}

static ssize_t fts_glove_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->glove_bit);
	return ret;
}

static ssize_t  fts_glove_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	//int error;
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	//u8 reg1[3] = {0xc1,0x01,0x01};//enable
	//u8 reg2[3] = {0xc2,0x01,0x01}; //disable

	ret = kstrtoul(buf, 10, &val);
	if(ret)
		return size;
	info_globle->glove_bit = val;
	if((info_globle->cover_bit)||(info_globle->cover_bit)||(info_globle->proximity_bit))
	{
		tp_log("cover:%d, proximity:%d, vr:%d return \n",info_globle->cover_bit,info_globle->proximity_bit,info_globle->vr_bit);
		return size;
	}
	if( 1 == val )
	{
		fts_glove_on();
		fts_set_sensor_mode(info_globle, MODE_GLOVE);
	} else if( 0 == val )
	{
		fts_glove_off();
		fts_set_sensor_mode(info_globle, MODE_NORMAL);
	}
		return size;
}

static ssize_t fts_hover_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->hover_bit);
	return ret;
}

static ssize_t  fts_hover_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{

	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	//struct i2c_client *client = to_i2c_client(dev);
	//struct fts_ts_info *info = i2c_get_clientdata(client);


	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return size;
	printk("hover enable = %ld \n", val);
	info_globle->hover_bit = val;
	if((info_globle->vr_bit)||(info_globle->cover_bit)||(info_globle->proximity_bit))
	{
		tp_log("cover:%d, proximity:%d, vr:%d return \n",info_globle->cover_bit,info_globle->proximity_bit,info_globle->vr_bit);
		return size;
	}
	if(val == 1) {
		if(info_globle->sensor_sleep)
			printk("tp is sleep mode, can't set hover mode \n");
		else
			ret = fts_command(info_globle, HOVER_ON);
			fts_set_sensor_mode(info_globle, MODE_HOVER);
	}else if(val == 0) {
		if(info_globle->sensor_sleep)
			printk("tp is sleep mode, can't disable hover mode \n");
		else
			ret = fts_command(info_globle, HOVER_OFF);
			fts_set_sensor_mode(info_globle, MODE_NORMAL);
	}
	if (ret)
		tp_log("%s: hover mode: %ld : failed !\n",__func__, val);
	return size;

}

static ssize_t fts_fw_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "ST-tp fw-version:0x%x, config-version:0x%x\n", info_globle->fw_version,info_globle->config_id);
	return ret;
}

static ssize_t  fts_fw_version_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return size;
}

static ssize_t fts_tp_log_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->touch_log_switch);
	return ret;
}

static ssize_t  fts_tp_log_switch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;

	unsigned char *mutual_cx_data =NULL;
	unsigned char *self_tune_data =NULL;
	unsigned char *mutual_raw_data =NULL;
	unsigned char *self_raw_data_force =NULL;
	unsigned char *self_raw_data_sense =NULL;
	unsigned char cx1_num = 0;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char f_ix1_num = 0;
	unsigned char s_ix1_num = 0;
	unsigned char f_cx1_num = 0;
	unsigned char s_cx1_num = 0;
	unsigned int rx_num, tx_num;
//	int count = 0;
	int 	j = 0,
			error =0 ,
			address_offset = 0,
			start_tx_offset = 0,
			retry = 0,
			size_hex =0,
			address_offset_force = 0,
			address_offset_sense = 0,
			size_hex_force= 0,
			size_hex_sense = 0;


	ret = kstrtoul(buf, 10, &val);
	if(ret)
		return size;
	info_globle->touch_log_switch = val;
	tp_log("cover:%d, proximity:%d, vr:%d \n",info_globle->cover_bit,info_globle->proximity_bit,info_globle->vr_bit);
	tp_log("glove:%d, hover:%d \n",info_globle->glove_bit,info_globle->hover_bit);
	if(val)
	{
		fts_interrupt_set(info_globle, INT_DISABLE);
		fts_command(info_globle, SENSEOFF);
		fts_command(info_globle, FLUSHBUFFER);
		msleep(50);

		/* Request Compensation Data*/
		regAdd[0] = 0xB8;
		regAdd[1] = 0x02;
		regAdd[2] = 0x00;
		fts_write_reg(info_globle,regAdd, 3);

		/* Read completion event*/
		for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
		{
			regAdd[0] = READ_ONE_EVENT;
			error = fts_read_reg(info_globle, regAdd,1, data, FTS_EVENT_SIZE);
			if (error)
			{
				tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
				return size;
			}
			printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					data[0], data[1], data[2], data[3],
					data[4], data[5], data[6], data[7]);

			if ((data[0] == 0x13) && (data[1] == 0x02))
			{
				break;
			}
			else
			{
				msleep(10);
				if(retry == READ_CNT)
				{
					tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
				}
			}
		}

		/* Read Offset Address for Compensation*/
		regAdd[0] = 0xD0;
		regAdd[1] = 0x00;
		regAdd[2] = 0x50;
		fts_read_reg(info_globle,regAdd, 3, &buff_read[0], 4);

		start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
		address_offset  = start_tx_offset + 0x10;

		/* Read Offset Address for f_cnt and s_cnt*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);
		regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, &buff_read[0], 17);

		/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
		tx_num = data[4];//M
		rx_num = data[5];//N
		cx1_num = buff_read[10];

		mutual_cx_data = (unsigned char *)kmalloc(((tx_num * rx_num)+1), GFP_KERNEL);// added one to accommodate  dummy byte

		//size = ((tx_num*rx_num+5)*2);//size of message byte * 3()
		/* <0xAA> <f_cnt>*/
		printk("reading Cx data \n");
		printk("%02X",0xAA);

		printk("%02X",tx_num);

		printk("%02X",rx_num);

		printk("%02X",cx1_num);

		// ============================================ Cx data Reading ======================================//	

		/* Read compensation data*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);	
		regAdd[2] = (unsigned char)(address_offset & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, mutual_cx_data, ((tx_num * rx_num)+1));


		/*Copying CX1 and CX2 data  */
		for (j = 1; j < ((tx_num * rx_num)+1); j++) {
				printk("%02X",mutual_cx_data[j]);
		}

		printk("%02X",0xBB);

		fts_interrupt_set(info_globle, INT_ENABLE);
		fts_command(info_globle, SENSEON);





		fts_interrupt_set(info_globle, INT_DISABLE);
		fts_command(info_globle, SENSEOFF);	
		fts_command(info_globle, FLUSHBUFFER);
		msleep(50);

		/* Request Compensation Data*/
		regAdd[0] = 0xB8;
		regAdd[1] = 0x20;
		regAdd[2] = 0x00;
		fts_write_reg(info_globle,regAdd, 3);

		/* Read completion event*/
		for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
		{
			regAdd[0] = READ_ONE_EVENT;
			error = fts_read_reg(info_globle, regAdd,1, data, FTS_EVENT_SIZE);
			if (error) 
			{
				tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
				return size;
			}
			printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					data[0], data[1], data[2], data[3],
					data[4], data[5], data[6], data[7]);

			if ((data[0] == 0x13) && (data[1] == 0x20)) 
			{
				break;
			}
			else
			{
				msleep(10);
				if(retry == READ_CNT)
				{
					tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
				}
			}
		}	

		/* Read Offset Address for Compensation*/
		regAdd[0] = 0xD0;
		regAdd[1] = 0x00;
		regAdd[2] = 0x50;
		fts_read_reg(info_globle,regAdd, 3, &buff_read[0], 4);

		start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
		address_offset  = start_tx_offset + 0x10;

		/* Read Offset Address for f_cnt and s_cnt*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
		regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, &buff_read[0], 17);

		/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
		tx_num = data[4];//M
		rx_num = data[5];//N

		f_ix1_num = buff_read[10];
		s_ix1_num = buff_read[11];
		f_cx1_num = buff_read[12];
		s_cx1_num = buff_read[13];

		/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/text */
		self_tune_data = (unsigned char *)kmalloc(((tx_num + rx_num)* 2 + 1), GFP_KERNEL);// added one to accommodate  dummy byte ((M+N) + (M +N))
		printk("reading Self tune data \n");
		/* <0xAA> <f_cnt>*/
		printk("%02X",0xAA);

		printk("%02X",tx_num);

		printk("%02X",rx_num);

		printk("%02X",f_ix1_num);

		printk("%02X",s_ix1_num);

		printk("%02X",f_cx1_num);

		printk("%02X",s_cx1_num);

		// ============================================ Self tune data Reading ======================================//	

		/* Read compensation data*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
		regAdd[2] = (unsigned char)(address_offset & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, self_tune_data, ((tx_num + rx_num)*2+1));

		/*Copying Self tune data data  */
		for (j = 1; j < ((tx_num + rx_num)*2+1); j++) {
				printk("%02X",self_tune_data[j]);
		}

		printk("%02X",0xBB);
		fts_interrupt_set(info_globle, INT_ENABLE);
		fts_command(info_globle, SENSEON);
		kfree(self_tune_data);


		fts_interrupt_set(info_globle, INT_DISABLE);
		fts_command(info_globle, SENSEON);	
		fts_command(info_globle, FLUSHBUFFER);
		msleep(50);

		/* Request Compensation Data*/
		regAdd[0] = 0xB8;
		regAdd[1] = 0x20;
		regAdd[2] = 0x00;
		fts_write_reg(info_globle,regAdd, 3);

		/* Read completion event*/
		for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
		{
			regAdd[0] = READ_ONE_EVENT;
			error = fts_read_reg(info_globle, regAdd,1, data, FTS_EVENT_SIZE);
			if (error) 
			{
				tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
				return -ENODEV;
			}
			printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					data[0], data[1], data[2], data[3],
					data[4], data[5], data[6], data[7]);

			if ((data[0] == 0x13) && (data[1] == 0x20)) 
			{
				break;
			}
			else
			{
				msleep(10);
				if(retry == READ_CNT)
				{
					tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
				}
			}
		}
		/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
		tx_num = data[4];//M
		rx_num = data[5];//N
		size_hex = (tx_num * rx_num)* 2 ;

		/* Read Offset Address for Mutual Raw data*/
		regAdd[0] = 0xD0;
		regAdd[1] = 0x00;
		regAdd[2] = 0x00;//Offset address for MS screen raw frame, Please verify this address from supporting fw engineer
		fts_read_reg(info_globle,regAdd,3, &buff_read[0], 4);

		address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
		//address_offset  = start_tx_offset + 0x10;

		/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
		mutual_raw_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
		printk("reading Mutual Raw data \n");
		/* <0xAA> <f_cnt>...*/
		printk("%02X",0xAA);

		printk("%02X",tx_num);

		printk("%02X",rx_num);

		// ============================================ Mutual Raw data Reading ======================================//	
		
		/* Read compensation data*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);
		regAdd[2] = (unsigned char)(address_offset & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, mutual_raw_data, (size_hex+1));

		/*Copying Mutual Raw data  */
		for (j = 1; j < (size_hex+1); j++)
		{
			printk("%02X",mutual_raw_data[j]);
		}

		printk("%02X",0xBB);

		fts_interrupt_set(info_globle, INT_ENABLE);
		fts_command(info_globle, SENSEON);

		kfree(mutual_raw_data);

		fts_interrupt_set(info_globle, INT_DISABLE);
		fts_command(info_globle, SENSEON);
		fts_command(info_globle, FLUSHBUFFER);
		msleep(50);

		/* Request Compensation Data*/
		regAdd[0] = 0xB8;
		regAdd[1] = 0x20;
		regAdd[2] = 0x00;
		fts_write_reg(info_globle,regAdd, 3);

		/* Read completion event*/
		for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
		{
			regAdd[0] = READ_ONE_EVENT;
			error = fts_read_reg(info_globle, regAdd,1, data, FTS_EVENT_SIZE);
			if (error)
			{
				tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
				return -ENODEV;
			}
			printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					data[0], data[1], data[2], data[3],
					data[4], data[5], data[6], data[7]);

			if ((data[0] == 0x13) && (data[1] == 0x20))
			{
				break;
			}
			else
			{
				msleep(10);
				if(retry == READ_CNT)
				{
					tp_log("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
				}
			}
		}

		/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
		tx_num = data[4];//M
		rx_num = data[5];//N

		//size = (((tx_num * rx_num)* 2 + 4) *2);//size of message byte * 2
		size_hex_force = tx_num * 2 ;
		size_hex_sense = rx_num * 2 ;

		/* Read Offset Address for Self Raw data */
		regAdd[0] = 0xD0;
		regAdd[1] = 0x00;
		regAdd[2] = 0x1A;//Offset address for SS touch raw force line, Please verify this address from supporting fw engineer 
		fts_read_reg(info_globle,regAdd, 3, &buff_read[0], 5);

		address_offset_force = ((buff_read[2]<<8) |buff_read[1]);
		address_offset_sense = ((buff_read[4]<<8) |buff_read[3]);

		/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ASCII/text */
		self_raw_data_force = (unsigned char *)kmalloc((size_hex_force + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
		self_raw_data_sense = (unsigned char *)kmalloc((size_hex_sense + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
		printk("reading  Self Raw data \n");
		/* <0xAA> <f_cnt>...*/
		printk("%02X",0xAA);

		printk("%02X",tx_num);

		printk("%02X",rx_num);

		// ============================================ Self Raw data Reading(force) ======================================//	

		/* Read compensation data*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((address_offset_force & 0xFF00) >> 8);
		regAdd[2] = (unsigned char)(address_offset_force & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, self_raw_data_force, (size_hex_force+1));

		/*Copying self raw data Force  */
		for (j = 1; j < (size_hex_force+1); j++) {
				printk("%02X",self_raw_data_force[j]);
		}

		// ============================================ Self Raw data Reading(sense) ======================================//	

		/* Read compensation data*/
		regAdd[0] = 0xD0;
		regAdd[1] = (unsigned char)((address_offset_sense & 0xFF00) >> 8);
		regAdd[2] = (unsigned char)(address_offset_sense & 0xFF);
		fts_read_reg(info_globle,regAdd, 3, self_raw_data_sense, (size_hex_sense+1));

		/*Copying self raw data Sense  */
		for (j = 1; j < (size_hex_sense+1); j++) {
				printk("%02X",self_raw_data_sense[j]);
		}

		printk("%02X",0xBB);

		fts_interrupt_set(info_globle, INT_ENABLE);
		fts_command(info_globle, SENSEON);

		kfree(self_raw_data_force);
		kfree(self_raw_data_sense);
	}
	return size;
}

static ssize_t fts_vr_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", info_globle->vr_bit);
	return ret;
}

static ssize_t  fts_vr_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return size;
	info_globle->vr_bit = val;
	if(info_globle->sensor_sleep)
	{
		tp_log("tp is sleep mode ,can't enter/exit vr mode\n");
		return size;
	}
	if(val)
		queue_delayed_work(info_globle->vr_workqueue, &info_globle->vr_work, msecs_to_jiffies(3000));
	else
		fts_vr_off();

	return size;
}


static DEVICE_ATTR(gesture_enable, 0664, fts_gesture_enable_show, fts_gesture_enable_store);

static DEVICE_ATTR(gesture_data, 0444, fts_gesture_data_show, fts_gesture_data_store);

static DEVICE_ATTR(doubleclick_enable, 0664, fts_db_enalbe_show, fts_db_enable_store);

static DEVICE_ATTR(cover_enable, 0664, fts_cover_enable_show, fts_cover_enable_store);

static DEVICE_ATTR(glove_enable, 0664, fts_glove_enable_show, fts_glove_enable_store);

static DEVICE_ATTR(hover_enable, 0664, fts_hover_enable_show, fts_hover_enable_store);

static DEVICE_ATTR(fw_version, 0444, fts_fw_version_show, fts_fw_version_store);
static DEVICE_ATTR(tp_log_switch, 0664, fts_tp_log_switch_show, fts_tp_log_switch_store);
static DEVICE_ATTR(vr_enable, 0664, fts_vr_enable_show, fts_vr_enable_store);

void tp_device_register ( void )
{
	int rc = 0;
	tp_class = class_create(THIS_MODULE, "tp_device");
	if (IS_ERR(tp_class))
		pr_err("Failed to create class(tp_device_class)!\n");

	tp_gesture_dev = device_create(tp_class, NULL, 0, NULL, "tp_gesture");
	if (IS_ERR(tp_gesture_dev))
		pr_err("Failed to create device(tp_gesture_dev)!\n");

	// tp_gesture
	rc = device_create_file(tp_gesture_dev, &dev_attr_gesture_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gesture_enable.attr.name);

	rc = device_create_file(tp_gesture_dev, &dev_attr_gesture_data);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gesture_data.attr.name);

	rc = device_create_file(tp_gesture_dev, &dev_attr_doubleclick_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_doubleclick_enable.attr.name);

	tp_sensitivity_dev = device_create(tp_class, NULL, 0, NULL, "tp_sensitivity");
	if (IS_ERR(tp_sensitivity_dev))
		pr_err("Failed to create device(tp_sensitivity)!\n");

	rc = device_create_file(tp_sensitivity_dev, &dev_attr_cover_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_cover_enable.attr.name);
	
	tp_glove_dev = device_create(tp_class, NULL, 0, NULL, "tp_glove");
	if (IS_ERR(tp_glove_dev))
		pr_err("Failed to create device(tp_glove)!\n");

	rc = device_create_file(tp_glove_dev, &dev_attr_glove_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_glove_enable.attr.name);
	
	tp_hover_dev = device_create(tp_class, NULL, 0, NULL, "tp_hover");
	if (IS_ERR(tp_hover_dev))
		pr_err("Failed to create device(tp_hover)!\n");

	rc = device_create_file(tp_hover_dev, &dev_attr_hover_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_hover_enable.attr.name);

	tp_fw_dev = device_create(tp_class, NULL, 0, NULL, "tp_fw");
	if (IS_ERR(tp_fw_dev))
		pr_err("Failed to create device(tp_hover)!\n");

	rc = device_create_file(tp_fw_dev, &dev_attr_fw_version);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_fw_version.attr.name);

	tp_debug_dev = device_create(tp_class, NULL, 0, NULL, "tp_debug");
	if (IS_ERR(tp_debug_dev))
		pr_err("Failed to create device(tp_hover)!\n");

	rc = device_create_file(tp_debug_dev, &dev_attr_tp_log_switch);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tp_log_switch.attr.name);

	tp_vr_dev = device_create(tp_class, NULL, 0, NULL, "tp_vr");
	if (IS_ERR(tp_vr_dev))
		pr_err("Failed to create device(tp_hover)!\n");

	rc = device_create_file(tp_vr_dev, &dev_attr_vr_enable);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_vr_enable.attr.name);

//	dev_set_drvdata(tp_dev, data);
//	dev_set_drvdata(lcd_ce_dev, NULL);
}

/*I2C CMd functions: functions to interface with GUI without script */

static int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum,
						unsigned char *buf, int num);

static int fts_write_reg(struct fts_ts_info *info, unsigned char *reg,
						 unsigned short len);
unsigned int data[512] = {0};
unsigned char pAddress_i2c[512] = {0};
int byte_count_read = 0 ;
char Out_buff[512];

static ssize_t fts_i2c_wr_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i ;
	char buff[16];
	memset(Out_buff, 0x00, ARRAY_SIZE(Out_buff));
	if(byte_count_read == 0)
	{
		snprintf(Out_buff, sizeof(Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "{%s}\n", Out_buff);
	}
#ifdef DEBUG
	 printk("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		printk(" %02X",(unsigned int )info->cmd_wr_result[i]);
		if(i < (byte_count_read-1))
		{
			printk(" ");
		}
	}
	printk("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strncat(Out_buff, buff, 512);
	for (i = 0; i < (byte_count_read+2); i++)
	{
		if((i == 0))
		{
			char temp_byte_count_read = (byte_count_read >> 8) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X",temp_byte_count_read);
		}
		else if(i == 1)
		{
			char temp_byte_count_read = (byte_count_read) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);

		}
		else
		{
			snprintf(buff, sizeof(buff), "%02X", info->cmd_wr_result[i-2]);
		}
		//snprintf(buff, sizeof(buff), "%02X", info->cmd_wr_result[i]);
		strncat(Out_buff, buff, 512);
		if(i < (byte_count_read+1))
		{
			snprintf(buff, sizeof(buff), " ");
			strncat(Out_buff, buff, 512);
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strncat(Out_buff, buff, 512);
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", Out_buff);
}

static ssize_t fts_i2c_wr_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[8] = {0};
	unsigned int byte_count =0 ;
	int i ;

	unsigned int data[8] = {0};

	memset(info->cmd_wr_result, 0x00, ARRAY_SIZE(info->cmd_wr_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x ", (data+7), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6));

	byte_count = data[7];

	/*if(sizeof(buf) != byte_count )
	{
		printk("%s : Byte count is wrong\n",__func__);
		return count;
	}*/
#ifdef DEBUG
	printk(" \n");
	printk("%s: Input Data 1:",__func__);

	for(i =0 ; i <7; i++)
	{
		 printk(" %02X",data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	printk("\n");
#else
	for(i =0 ; i <7; i++)
	{
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	byte_count_read = data[byte_count-1];

	ret = fts_write_reg(info,pAddress,3);
	msleep(20);
	ret = fts_read_reg(info,&pAddress[3], (byte_count-4), info->cmd_wr_result ,byte_count_read );

#ifdef DEBUG
	 printk("%s:DATA READ \n{", __func__);
	for(i=0;i<(2+byte_count_read);i++)
	{
		if((i == 0))
		{
			char temp_byte_count_read = (byte_count_read >> 8) & 0xFF;
			printk("%02X",(unsigned int )temp_byte_count_read);
		}
		else if(i == 1)
		{
			char temp_byte_count_read = (byte_count_read) & 0xFF;
			printk("%02X",(unsigned int )temp_byte_count_read);

		}
		else
		{
			printk("%02X",(unsigned int )info->cmd_read_result[i-2]);
		}
		if(i < (byte_count_read+1))
		{
			printk( " ");
		}

	}
	printk("}\n");
#endif
	if (ret)
		dev_err(dev, "Unable to read register \n");
	return count;
}

static ssize_t fts_i2c_read_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i ;
	char buff[16];

	memset(Out_buff, 0x00, ARRAY_SIZE(Out_buff));
	if(byte_count_read == 0)
	{
		snprintf(Out_buff, sizeof(Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "{%s}\n", Out_buff);
	}
#ifdef DEBUG
	printk("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		printk("%02X",(unsigned int )info->cmd_read_result[i]);
		if(i < (byte_count_read-1))
		{
			printk(" ");
		}
	}
	printk("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strncat(Out_buff, buff, 512);
	for (i = 0; i < (byte_count_read+2); i++)
	{
		if((i == 0))
		{
			char temp_byte_count_read = (byte_count_read >> 8) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X",temp_byte_count_read);
		}
		else if(i == 1)
		{
			char temp_byte_count_read = (byte_count_read) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);

		}
		else
		{
			snprintf(buff, sizeof(buff), "%02X", info->cmd_read_result[i-2]);
		}
		strncat(Out_buff, buff, 512);
		if(i < (byte_count_read+1))
		{
			snprintf(buff, sizeof(buff), " ");
			strncat(Out_buff, buff, 512);
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strncat(Out_buff, buff, 512);

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", Out_buff);
}

static ssize_t fts_i2c_read_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[8] = {0};
	unsigned int byte_count =0 ;
	int i ;
	unsigned int data[8] = {0};

	byte_count_read = 0;
	memset(info->cmd_read_result, 0x00, ARRAY_SIZE(info->cmd_read_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x ", (data+7), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6));
	byte_count = data[7];


	if(byte_count >7 )
	{
#ifdef DEBUG
		printk("%s : Byte count is more than 7\n",__func__);
#endif
		return count;
	}
	/*if(sizeof(buf) != byte_count )
	{
		printk("%s : Byte count is wrong\n",__func__);
		return count;
	}*/
#ifdef DEBUG
	printk(" \n");
	printk("%s: Input Data 1:",__func__);
	for(i =0 ; i < byte_count; i++)
	{
		 printk(" %02X",data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	printk(" \n");
#else
	for(i =0 ; i < byte_count; i++)
	{
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	byte_count_read = data[byte_count-1];

	ret = fts_read_reg(info,pAddress, (byte_count-1), info->cmd_read_result ,byte_count_read );
#ifdef DEBUG
	printk("%s:DATA READ \n{", __func__);
	for(i=0;i<(byte_count_read+2);i++)
	{
		if((i == 0))
		{
			char temp_byte_count_read = (byte_count_read >> 8) & 0xFF;
			printk("%02X",(unsigned int )temp_byte_count_read);
		}
		else if(i == 1)
		{
			char temp_byte_count_read = (byte_count_read) & 0xFF;
			printk("%02X",(unsigned int )temp_byte_count_read);

		}
		else
		{
			printk("%02X",(unsigned int )info->cmd_read_result[i-2]);
		}
		if(i < (byte_count_read+1))
		{
			printk(" ");
		}
	}
	printk("}\n");
#endif
	if (ret)
		dev_err(dev, "Unable to read register \n");
	return count;
}


static ssize_t fts_i2c_write_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	return snprintf(buf, TSP_BUF_SIZE, "%s", info->cmd_write_result);

}

static ssize_t fts_i2c_write_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned short byte_count =0 ;
	int i ;

	memset(pAddress_i2c, 0x00, ARRAY_SIZE(pAddress_i2c));
	memset(info->cmd_write_result, 0x00, ARRAY_SIZE(info->cmd_write_result));
	sscanf(buf, "%x ", data);
	byte_count = data[0];

	if(byte_count <= 512)
	{
		/*if(sizeof(buf) != byte_count )
		{
			printk("%s : Byte count is wrong\n",__func__);
			snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
		}*/
		for(i=1; i <= byte_count ;i++)
		{
			sscanf(&buf[3*i] , "%x ", (data+(i-1)));
		}
	}else
	{
#ifdef DEBUG
		printk("%s : message size is more than allowed limit of 512 bytes\n",__func__);
#endif
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	}
#ifdef DEBUG
	printk(" \n");
	//printk("%s: Byte_count=  %02d| Count = %02d | size of buf:%02d\n",__func__,byte_count,count,sizeof(buf));
	//printk("%s: Byte_count=  %02d | size of buf:%02d\n",__func__,byte_count,sizeof(buf));
	printk("%s: Input Data 1:",__func__);
	for(i =0 ; i < byte_count; i++)
	{
		printk(" %02X",data[i]);
		pAddress_i2c[i] = (unsigned char)data[i];
	}
	printk(" \n");
#else
	for(i =0 ; i < byte_count; i++)
	{
		pAddress_i2c[i] = (unsigned char)data[i];
	}
#endif
	if((pAddress_i2c[0] == 0xb3)&&(pAddress_i2c[3] == 0xb1))
	{
		ret = fts_write_reg(info, pAddress_i2c, 3);
		msleep(20);
		ret = fts_write_reg(info,&pAddress_i2c[3], byte_count-3);
	}else
	{
		ret = fts_write_reg(info,pAddress_i2c, byte_count);
	}

#ifdef DEBUG
	printk("%s:DATA :", __func__);
	for(i=0;i<byte_count;i++)
	{
		printk(" %02X",(unsigned int )pAddress_i2c[i]);
	}
	printk(" byte_count: %02X\n",byte_count);
#endif
	if (ret)
	{
		dev_err(dev, "{Write NOT OK}\n");
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	}else
	{
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write OK}\n");
#ifdef DEBUG
		printk("%s : {Write OK}\n",__func__);
#endif
	}
	return count;
}

static DEVICE_ATTR(iread,(S_IWUSR|S_IWGRP), NULL, fts_i2c_read_store);
static DEVICE_ATTR(iread_result,(S_IRUGO), fts_i2c_read_show, NULL);
static DEVICE_ATTR(iwr,(S_IWUSR|S_IWGRP), NULL, fts_i2c_wr_store);
static DEVICE_ATTR(iwr_result,(S_IRUGO), fts_i2c_wr_show, NULL);
static DEVICE_ATTR(iwrite,(S_IWUSR|S_IWGRP), NULL, fts_i2c_write_store);
static DEVICE_ATTR(iwrite_result,(S_IRUGO), fts_i2c_write_show, NULL);


static struct attribute *i2c_cmd_attributes[] = {
	&dev_attr_iread.attr,
	&dev_attr_iread_result.attr,
	&dev_attr_iwr.attr,
	&dev_attr_iwr_result.attr,
	&dev_attr_iwrite.attr,
	&dev_attr_iwrite_result.attr,
	NULL,
};

static struct attribute_group i2c_cmd_attr_group = {
	.attrs = i2c_cmd_attributes,
};
 /*I2C cmd funtions :END*/



#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_early_suspend(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Suspend entered\n");
	if (fts_suspend(info->client, PMSG_SUSPEND))
		dev_err(&info->client->dev, "Early suspend failed\n");
	dev_info(dev, "FTS Early Suspended\n");
}

static void fts_late_resume(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Resume entered\n");
	if (fts_resume(info->client))
		dev_err(&info->client->dev, "Late resume failed\n");
	dev_info(dev, "FTS Early Resumed\n");
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int fts_command(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd;
	int ret;

	regAdd = cmd;
	ret = fts_write_reg(info, &regAdd, sizeof(regAdd)); // 0 = ok
	#ifdef DEBUG
	tp_log( " fts:Issued command 0x%02x, return value %d\n", cmd, ret);
	#endif
	return ret;
}


static int fts_systemreset(struct fts_ts_info *info)
{
	int ret;
	unsigned char regAdd[4] = { 0xB6, 0x00, 0x23, 0x01 };

	dev_dbg(info->dev, "Doing a system reset\n");

	ret = fts_write_reg(info, regAdd, sizeof(regAdd));

	usleep_range(5000, 6000);

	return ret;
}

#if 0
static int fts_get_mode(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char regAdd[4] = { 0xB2, 0x00, 0x02, 0x01 };

	fts_write_reg(info, regAdd, sizeof(regAdd));

	msleep(20);

	regAdd[0] = READ_ONE_EVENT;
	info->mode = fts_read_reg(info, regAdd, 1, data, FTS_EVENT_SIZE) ?
			MODE_NORMAL : data[3];

	return 0;
}
#endif

static int fts_get_fw_version(struct fts_ts_info *info)
{
	unsigned char val[8];
	unsigned char regAdd[3] = {0xB6, 0x00, 0x07};
	//unsigned char regAdd1[1];
	unsigned char regAdd2[4]={0xB2, 0x00, 0x01,0x08};
	unsigned char regAdd3 = READ_ONE_EVENT;
	int error;
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	error = fts_read_reg(info, regAdd, sizeof(regAdd), val, sizeof(val));
	/*check for chip id*/
	if ((val[1] != FTS_ID0) || (val[2] != FTS_ID1)) {
		pr_err("!!!read 0xAA result: val[0]:%x,val[1]:%x,val[2]:%x,val[3]:%x,val[4]:%x,val[5]:%x,val[6]:%x,val[7]:%x \n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
		dev_err(info->dev,
			"Wrong version id (read 0x%02x%02x, expected 0x%02x%02x)\n",
				val[1], val[2], FTS_ID0, FTS_ID1);
		return -ENODEV;
	}else {
	//pr_err("!!!read 0xAA result: val[0]:%x,val[1]:%x,val[2]:%x,val[3]:%x,val[4]:%x,val[5]:%x,val[6]:%x,val[7]:%x \n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
		info->fw_version = (val[5] << 8) | val[4];
	}


	fts_write_reg(info, regAdd2, sizeof(regAdd2));
	msleep(30);
	error = fts_read_reg(info, &regAdd3,sizeof(regAdd3), val, FTS_EVENT_SIZE);
	if (error) {
		info->config_id = 0;
		tp_log("Cannot read device info\n");
		return -ENODEV;
		}else {
		info->config_id = (val[3] << 8) | val[4];
		//pr_err("!!!read 0xAA result: val[0]:%x,val[1]:%x,val[2]:%x,val[3]:%x,val[4]:%x,val[5]:%x,val[6]:%x,val[7]:%x \n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
	}
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);

#if 0

    if(info->fw_version > 0)
    {
    	regAdd1[0] = 0xAA;
		error = fts_read_reg(info, regAdd1, sizeof(regAdd1), val, sizeof(val));
		pr_err("read 0xAA result: val[0]:%x,val[1]:%x,val[2]:%x,val[3]:%x,val[4]:%x,val[5]:%x,val[6]:%x,val[7]:%x \n",val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7]);
		if (error) {
			dev_err(info->dev, "Cannot read config id\n");
			return -ENODEV;
		}else{
			info->config_id = (val[4] << 8) | val[5];
		}

    }else{
		info->config_id = 0;
    }
#endif
	return 0;
}

static int fts_flash_status(struct fts_ts_info *info,
				unsigned int timeout, unsigned int steps)
{
	int ret, status;
	unsigned char data;
	unsigned char regAdd[2];

	do {
		regAdd[0] = FLASH_READ_STATUS;
		regAdd[1] = 0;
		
		msleep(20);

		ret = fts_read_reg(info, regAdd, sizeof(regAdd), &data, sizeof(data));
		if (ret)
			status = FLASH_STATUS_UNKNOWN;
		else
			status = (data & 0x01) ? FLASH_STATUS_BUSY : FLASH_STATUS_READY;

		if (status == FLASH_STATUS_BUSY) {
			timeout -= steps;
			msleep(steps);
		}

	} while ((status == FLASH_STATUS_BUSY) && (timeout));

	return status;
}


static int fts_flash_unlock(struct fts_ts_info *info)
{
	int ret;
	unsigned char regAdd[4] = { FLASH_UNLOCK,
				FLASH_UNLOCK_CODE_0,
				FLASH_UNLOCK_CODE_1,
				0x00 };

	ret = fts_write_reg(info, regAdd, sizeof(regAdd));

	if (ret)
		dev_err(info->dev, "Cannot unlock flash\n");
	else
		dev_dbg(info->dev, "Flash unlocked\n");

	return ret;
}

static int fts_flash_load(struct fts_ts_info *info,
			int cmd, int address, const char *data, int size)
{
	int ret;
	unsigned char *cmd_buf;
	unsigned int loaded;

	cmd_buf = kmalloc(FLASH_LOAD_COMMAND_SIZE, GFP_KERNEL);
	if (cmd_buf == NULL) {
		dev_err(info->dev, "Out of memory when programming flash\n");
		return -ENOMEM;
	}
	tp_log("%s: siz = %d  \n",__func__,size);
	loaded = 0;
	while (loaded < size) {
		cmd_buf[0] = cmd;
		cmd_buf[1] = (address >> 8) & 0xFF;
		cmd_buf[2] = (address) & 0xFF;

		memcpy(&cmd_buf[3], data, FLASH_LOAD_CHUNK_SIZE);
		ret = fts_write_reg(info, cmd_buf, FLASH_LOAD_COMMAND_SIZE);
		if (ret) {
			dev_err(info->dev, "Cannot load firmware in RAM\n");
			break;
		}

		data += FLASH_LOAD_CHUNK_SIZE;
		loaded += FLASH_LOAD_CHUNK_SIZE;
		address += FLASH_LOAD_CHUNK_SIZE;
		tp_log("%s: loaded = %d  \n",__func__,loaded);
	}

	kfree(cmd_buf);
	tp_log("!!!  %s: loaded = %d size=%d  \n",__func__,loaded,size);
	return (loaded == size) ? 0 : -1;
}


static int fts_flash_erase(struct fts_ts_info *info, int cmd)
{
	int ret;
	unsigned char regAdd = cmd;

	ret = fts_write_reg(info, &regAdd, sizeof(regAdd));

	if (ret)
		dev_err(info->dev, "Cannot erase flash\n");
	else
		dev_dbg(info->dev, "Flash erased\n");

	return ret;
}


static int fts_flash_program(struct fts_ts_info *info, int cmd)
{
	int ret;
	unsigned char regAdd = cmd;

	ret = fts_write_reg(info, &regAdd, sizeof(regAdd));

	if (ret)
		dev_err(info->dev, "Cannot program flash\n");
	else
		dev_dbg(info->dev, "Flash programmed\n");

	return ret;
}


static int fts_fw_upgrade(struct fts_ts_info *info, int mode,int fw_forceupdate,int crc_err)
{
	int ret;
	const struct firmware *fw = NULL;
	unsigned char *data = NULL;
	unsigned int size;
	int updata_loop = 0;
	int status, fw_ver = 0, config_ver = 0;
	int program_command, erase_command, load_command, load_address = 0;

	tp_log("Firmware upgrade...\n");
	info->fwupdate_stat = 1;
#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 1){
		if(!info->fw_force){
			ret = request_firmware(&fw, fts_wqhd_fw_filename[0], info->dev);
		}else{
			ret = request_firmware(&fw, fts_wqhd_fw_filename[1], info->dev);
		}
		if (ret) {
			tp_log("Unable to open firmware file '%s'\n",fts_wqhd_fw_filename[0]);
			return ret;
		}

		if ((fw->size == 0) /*|| (fw->size != fts_fw_size[mode])*/) {
			dev_err(info->dev, "Wrong firmware file '%s'\n", fts_wqhd_fw_filename[mode]);
			goto fw_done;
		}
	}
	else{
		if(!info->fw_force){
			ret = request_firmware(&fw, fts_fhd_fw_filename[0], info->dev);
		}else{
			ret = request_firmware(&fw, fts_fhd_fw_filename[1], info->dev);
		}
		if (ret) {
			tp_log("Unable to open firmware file '%s'\n",fts_fhd_fw_filename[0]);
			return ret;
		}

		if ((fw->size == 0) /*|| (fw->size != fts_fw_size[mode])*/) {
			dev_err(info->dev, "Wrong firmware file '%s'\n", fts_fhd_fw_filename[mode]);
			goto fw_done;
		}
	}
#else
	if(!info->fw_force){
	    ret = request_firmware(&fw, fts_fw_filename[0], info->dev);
	}else{
        ret = request_firmware(&fw, fts_fw_filename[1], info->dev);
	}
	if (ret) {
		tp_log("Unable to open firmware file '%s'\n",fts_fw_filename[0]);
		return ret;
	}

	if ((fw->size == 0) /*|| (fw->size != fts_fw_size[mode])*/) {
		dev_err(info->dev, "Wrong firmware file '%s'\n", fts_fw_filename[mode]);
		goto fw_done;
	}
#endif
	data = (unsigned char *)fw->data;
	size = fw->size;
	fw_ver = le_to_uint(&data[FILE_FW_VER_OFFSET]);	
	config_ver = be_to_uint(&data[FILE_CONFIG_VER_OFFSET]);

	if(!info->fw_force){

		//tp_log("%s: fw update probe begin!\n", __func__);
		ret = fts_get_fw_version(info);
		if(ret) {
			tp_log("%s: can not get fw version!\n", __func__);
		}
		tp_log("%s: tp:fw_version = %x, config_id = %x. bin: fw_ver = %x, config_ver = %x\n", __func__, 
			info->fw_version, info->config_id, fw_ver, config_ver);

		if(fw_ver != info->fw_version || config_ver != info->config_id || fw_forceupdate == 1)	
		{
			mode = MODE_RELEASE_AND_CONFIG_128;
			tp_log("%s: mode = %d",__func__, mode);
		}else{
			info->fwupdate_stat = 0;
			tp_log("%s: no need to update \n",__func__);
			return 0;
		}
	}

fts_updat:
	data = (unsigned char *)fw->data;
	size = fw->size;
	dev_dbg(info->dev, "Flash programming...\n");
	ret = fts_systemreset(info);
	if (ret) {
		dev_warn(info->dev, "Cannot reset the device 00\n");
		goto fw_done;
	}
	msleep(150);

	switch (mode) {
	case MODE_CONFIG_ONLY:
		program_command = FLASH_PROGRAM;
		erase_command = FLASH_ERASE;
		load_command = FLASH_LOAD_FIRMWARE_UPPER_64K;
		load_address = FLASH_LOAD_INFO_BLOCK_OFFSET;
		break;
	case MODE_RELEASE_AND_CONFIG_128:
		/* skip 32 bytes header */
		data += 32;
		size = size - 32;
		/* fall throug */
	case MODE_RELEASE_ONLY:
		program_command = FLASH_PROGRAM;
		erase_command = FLASH_ERASE;
		load_command = FLASH_LOAD_FIRMWARE_LOWER_64K;
		load_address = FLASH_LOAD_FIRMWARE_OFFSET;
		break;
	default:
		/* should never be here, already checked mode value before */
		break;
	}

	dev_info(info->dev, "1) checking for status.\n");
	status = fts_flash_status(info, 1000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		dev_err(info->dev, "Wrong flash status\n");
		goto fw_done;
	}

	dev_info(info->dev, "2) unlock the flash.\n");
	ret = fts_flash_unlock(info);
	if (ret) {
		dev_err(info->dev, "Cannot unlock the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		dev_err(info->dev, "Wrong flash status 2\n");
		goto fw_done;
	}

	dev_info(info->dev, "3) load the program.\n");
	if(load_command == FLASH_LOAD_FIRMWARE_LOWER_64K)
	{
		ret = fts_flash_load(info, load_command, load_address, data, FLASH_SIZE_F0_CMD);
		load_command = FLASH_LOAD_FIRMWARE_UPPER_64K;
		if((crc_err == 0)&&(size == (FLASH_SIZE_FW_CONFIG + FLASH_SIZE_CXMEM))) //only for D2 chip 
		{
			//if size is 128 K, then adjust the size to include only fw and config(124 K)
			size = size - FLASH_SIZE_CXMEM;
		}
		
		ret = fts_flash_load(info, load_command, load_address, (data+FLASH_SIZE_F0_CMD), (size-FLASH_SIZE_F0_CMD));
	}else
	{
		ret = fts_flash_load(info, load_command, load_address, data, size);
	}
	if (ret) {
		dev_err(info->dev,
			"Cannot load program to for the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		dev_err(info->dev, "Wrong flash status 3\n");
		goto fw_done;
	}

	dev_info(info->dev, "4) erase the flash.\n");
	ret = fts_flash_erase(info, erase_command);
	if (ret) {
		dev_err(info->dev, "Cannot erase the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	dev_info(info->dev, "5) checking for status.\n");
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		dev_err(info->dev, "Wrong flash status 4\n");
		goto fw_done;
	}

	dev_info(info->dev, "6) program the flash.\n");
	ret = fts_flash_program(info, program_command);
	if (ret) {
		dev_err(info->dev, "Cannot program the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		dev_err(info->dev, "Wrong flash status 5\n");
		goto fw_done;
	}

	dev_info(info->dev, "Flash programming: done.\n");


	dev_info(info->dev, "Perform a system reset\n");
	ret = fts_systemreset(info);
	if (ret) {
		dev_warn(info->dev, "Cannot reset the device\n");
		goto fw_done;
	}

	ret = fts_get_fw_version(info);
	if (ret) {
		dev_warn(info->dev, "Cannot retrieve firmware version\n");
		goto fw_done;
	}

	tp_log("%s: tp:fw_version = %x, config_id = %x. bin: fw_ver = %x, config_ver = %x\n", __func__, 
			info->fw_version, info->config_id, fw_ver, config_ver);

	if(fw_ver == info->fw_version && config_ver == info->config_id)	
	{
		info->fwupdate_stat = 2;
		tp_log("%s: firmware update OK!", __func__);
	}else{
		if (updata_loop < 3){
			updata_loop++;
			tp_log("%s: firmware updata failed, update again %d******************\n", __func__, updata_loop);
			goto fts_updat;
		}
		tp_log("%s: firmware update failed!", __func__);
	}

	dev_info(info->dev,
		"New firmware version 0x%04x installed\n",
		info->fw_version);

fw_done:
	release_firmware(fw);

	return ret;
}

static void fts_interrupt_set(struct fts_ts_info *info, int enable)
{
	unsigned char regAdd[4] = { 0xB6, 0x00, 0x1C, enable };

	fts_write_reg(info, &regAdd[0], 4);
}


/*
 * New Interrupt handle implementation
 */



static inline unsigned char *fts_next_event(unsigned char *evt)
{
	/* Nothing to do with this event, moving to the next one */
	evt += FTS_EVENT_SIZE;

	/* the previous one was the last event ?  */
	return (evt[-1] & 0x1F) ? evt : NULL;
}


/* EventId : 0x00 */
static unsigned char *fts_nop_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	// GIUSEPPE dev_dbg(info->dev, "Doing nothing for event 0x%02x\n", *event);
	return fts_next_event(event);
}


/* EventId : 0x03 */
static unsigned char *fts_enter_pointer_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	unsigned char touchId,touchcount;
	int x, y, z;

	if((!info->resume_bit)||(backlight_level == 0))
		goto no_report;

	touchId = event[1] & 0x0F;
	touchcount = (event[1] & 0xF0) >> 4;

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0x3F);
	if(info->touch_log_switch)
		tp_log("touchcount:%d, x: %d, y: %d \n",touchcount,x,y);
#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 0){
	if (x == X_AXIS_MAX_FHD)
		x--;

	if (y == Y_AXIS_MAX_FHD)
		y--;
	}
	else if(panel_res_type_select() == 1){
	if (x == X_AXIS_MAX_WQHD)
		x--;

	if (y == Y_AXIS_MAX_WQHD)
		y--;
	}
#else
	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;
#endif

#if 0
	/* for LCD */
	x = 1080 - x;
	y = 1920 - y;
	////////////
#endif
	//input_mt_slot(info->input_dev, finger);

#ifdef CONFIG_EXYNOS_TOUCH_DAEMON
	// Record Touch Point Coordinates
	if(exynos_touch_daemon_data.record == 1) {
		if(exynos_touch_daemon_data.tp.count < TOUCHPOINT) {
			exynos_touch_daemon_data.tp.x[exynos_touch_daemon_data.tp.count] = x;
			exynos_touch_daemon_data.tp.y[exynos_touch_daemon_data.tp.count] = y;
			exynos_touch_daemon_data.tp.wx[exynos_touch_daemon_data.tp.count] = z;
			exynos_touch_daemon_data.tp.wy[exynos_touch_daemon_data.tp.count] = z;
			exynos_touch_daemon_data.tp.count++;
		} else {
			printk("%s: Recordable touch point exceeds %d\n", __func__, TOUCHPOINT);
			exynos_touch_daemon_data.record = 0;
		}
	}
#endif
	
	input_mt_slot(info->input_dev, touchId);
	input_mt_report_slot_state(info->input_dev,MT_TOOL_FINGER, 1);

#ifdef CONFIG_JANUARY_BOOSTER
	if(touchcount >= 1) {
		janeps_input_report(PRESS, x, y);
	}
#endif

	if(touchcount == 1){
		input_report_key(info->input_dev, BTN_TOUCH, 1);
		input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);
	}
	//input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	if(info->touch_log_switch)
		tp_log("event is report:  x: %d, y: %d \n",x,y);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, z);
	//input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);

no_report:
	return fts_next_event(event);
}


/* EventId : 0x04 */
static unsigned char *fts_leave_pointer_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	unsigned char touchId, touchcount;
#ifdef CONFIG_JANUARY_BOOSTER
	int x, y;

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);

#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 0){
	if (x == X_AXIS_MAX_FHD)
		x--;

	if (y == Y_AXIS_MAX_FHD)
		y--;
	}
	else if(panel_res_type_select() == 1){
	if (x == X_AXIS_MAX_WQHD)
		x--;

	if (y == Y_AXIS_MAX_WQHD)
		y--;
	}
#else
	if (x == X_AXIS_MAX)
		x--;
	if (y == Y_AXIS_MAX)
		y--;
#endif

#if 0
	/* for LCD */
	x = 1080 - x;
	y = 1920 - y;
#endif
#endif

	touchId = event[1] & 0x0F;
	touchcount = (event[1] & 0xF0) >> 4;

	__clear_bit(touchId, &info->touch_id);

	input_mt_slot(info->input_dev, touchId);
	input_mt_report_slot_state(info->input_dev,MT_TOOL_FINGER, 0);
	if (touchcount == 0){
		input_report_key(info->input_dev, BTN_TOUCH, 0);
		input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);
#ifdef CONFIG_EXYNOS_TOUCH_DAEMON
		if (exynos_touch_daemon_data.record == 1) {
			printk("%s Touch point recording is completed ad %d points\n", __func__, exynos_touch_daemon_data.tp.count);
			exynos_touch_daemon_data.record = 0;
		}
#endif
#ifdef CONFIG_JANUARY_BOOSTER
		janeps_input_report(RELEASE, x, y);
#endif
	}
	//input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);

	return fts_next_event(event);
}

/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler


/* EventId : 0x07 */
static unsigned char *fts_hover_enter_pointer_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;


	touchId = event[1] & 0x0F;

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
#define HOVER_ENTER_Z_VALUE 0
	z = HOVER_ENTER_Z_VALUE;

#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 0){
	if (x == X_AXIS_MAX_FHD)
		x--;

	if (y == Y_AXIS_MAX_FHD)
		y--;
	}
	else if(panel_res_type_select() == 1){
	if (x == X_AXIS_MAX_WQHD)
		x--;

	if (y == Y_AXIS_MAX_WQHD)
		y--;
	}
#else
	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;
#endif

	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, z);
	//input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);


	return fts_next_event(event);
}


/* EventId : 0x08 */
#define fts_hover_leave_pointer_event_handler fts_leave_pointer_event_handler


/* EventId : 0x09 */
#define fts_hover_motion_pointer_event_handler fts_enter_pointer_event_handler


/* EventId : 0x0B */
static unsigned char *fts_proximity_enter_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
//	unsigned char touchId;
//	int x, y, z;

	dev_err(info->dev, "Received event 0x%02x\n", event[0]);
	tp_log("virtual-proximity: NEAR \n");
//	touchId = event[1] & 0x0F;
	input_report_abs(vps2->virtualdevice, ABS_DISTANCE, 0);
	input_sync(vps2->virtualdevice);

	return fts_next_event(event);
}


/* EventId : 0x0C */
static unsigned char *fts_proximity_leave_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
//	unsigned char touchId;
//	int x, y, z;

	dev_err(info->dev, "Received event 0x%02x\n", event[0]);
	tp_log("virtual-proximity: FAR \n");
//	touchId = event[1] & 0x0F;
	input_report_abs(vps2->virtualdevice, ABS_DISTANCE, 1);
	input_sync(vps2->virtualdevice);

	return fts_next_event(event);
}

/* EventId : 0x0E */
static unsigned char *fts_button_status_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	int i;
	unsigned int buttons, changed;

	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);

	/* get current buttons status */
	buttons = event[1] | (event[2] << 8);

	/* check what is changed */
	changed = buttons ^ info->buttons;

	for (i = 0; i < 16; i++)
		if (changed & (1 << i))
			input_report_key(info->input_dev,
				BTN_0 + i,
				(!(info->buttons & (1 << i))));

	/* save current button status */
	info->buttons = buttons;

	dev_dbg(info->dev, "Event 0x%02x -  SS = 0x%02x, MS = 0x%02x\n",
				event[0], event[1], event[2]);

	return fts_next_event(event);
}


/* EventId : 0x0F */
static unsigned char *fts_error_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	int error,i;
	dev_err(info->dev, "ESD check:Received event event[0]:0x%02x,event[1]:0x%02x\n", event[0],event[1]);
	return fts_next_event(event);
	if(event[1] == 0x0a){
        if (info->bus_reg) {
		    error = regulator_disable(info->bus_reg);
		    if (error < 0) {
			    dev_err(info->dev,
					"%s: Failed to enable bus pullup regulator\n",__func__);
		    }
	    }
	    if (info->pwr_reg) {
		    error = regulator_disable(info->pwr_reg);
		    if (error < 0) {
			    dev_err(info->dev,
					"%s: Failed to enable power regulator\n",__func__);
		    }
	    }
	    msleep(300);
	    if (info->pwr_reg)
		    error = regulator_enable(info->pwr_reg);

	    if (info->bus_reg)
		    error = regulator_enable(info->bus_reg);
        msleep(300);


		for (i = 0; i < TOUCH_ID_MAX; i++){//before reset clear all slot
				input_mt_slot(info->input_dev, i);
				input_mt_report_slot_state(info->input_dev,
				(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
		}
		input_sync(info->input_dev);

		error = fts_systemreset(info);
		msleep(150);
		error += fts_command(info, SENSEON);
		usleep_range(10000,11000);
		error += fts_command(info, FORCECALIBRATION);
		error += fts_command(info, SENSEON);
		error += fts_command(info, FLUSHBUFFER);
		if (error) {
			tp_log("%s: Cannot reset the device----------\n", __func__);
		}
		if(event[2] >= 0x80)
			{ printk("ESD or Low battery at gesture mode recovery \n");
				fts_set_gesture(info);
				fts_command(info, ENTER_GESTURE_MODE);
				//enable_irq_wake(info->client->irq);
				fts_set_sensor_mode(info, MODE_GESTURE);
				info->gesture_enable = 1;				
			}			
	}

	return fts_next_event(event);
}


/* EventId : 0x10 */
static unsigned char *fts_controller_ready_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);
	info->touch_id = 0;
	info->buttons = 0;
	input_sync(info->input_dev);
	return fts_next_event(event);
}


/* EventId : 0x11 */
static unsigned char *fts_sleepout_controller_ready_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);
	complete(&info->cmd_done);
	return fts_next_event(event);
}


/* EventId : 0x16 */
static unsigned char *fts_status_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);

	switch (event[1]) {
	case FTS_STATUS_MUTUAL_TUNE:
	case FTS_STATUS_SELF_TUNE:
	case FTS_FORCE_CAL_SELF_MUTUAL:
		complete(&info->cmd_done);
		break;

	case FTS_FLASH_WRITE_CONFIG:
	case FTS_FLASH_WRITE_COMP_MEMORY:
	case FTS_FORCE_CAL_SELF:
	case FTS_WATER_MODE_ON:
	case FTS_WATER_MODE_OFF:
	default:
		dev_dbg(info->dev,
			"Received unhandled status event = 0x%02x\n", event[1]);
		break;
	}

	return fts_next_event(event);
}


/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler


/* EventId : 0x20 */
static unsigned char *fts_gesture_event_handler(
                     struct fts_ts_info *info, unsigned char *event)

{

	unsigned char touchId;
	//static int x_off, y_off;

	//unsigned char gesture_direction;
	printk( "%s Received event 0x%02x  event[2]  = %d\n", __func__,event[1], event[2] );
 
	/* always use touchId zero */
	touchId = 0;
	__set_bit(touchId, &info->touch_id);
	info->gesture_reg[0] = event[3];
	info->gesture_reg[1] = event[4];

	switch(event[2]){
	case 0x03:/*add 03-->C*/
		info->gesture_value = UNICODE_C;
		break;
	case 0x05:/*add 05-->W*/
		info->gesture_value = UNICODE_W;
		break;
	case 0x04:/*add 04-->M*/
		info->gesture_value = UNICODE_M;
		break;
	case 0x06:/*add 06-->E*/
		info->gesture_value = UNICODE_E;
		break;
	case 0x0a:/*add 0a-->UP*/
		info->gesture_value = SWIPE_Y_UP;
		break;
	case 0x09:/*add 09-->down*/
		info->gesture_value = SWIPE_Y_DOWN;
		break;
	case 0x07:/*add 07-->left*/
		info->gesture_value = SWIPE_X_RIGHT;
			break;
	case 0x08:/*add 08-->right*/
		info->gesture_value = SWIPE_X_LEFT;
			break;
	case 0x01:/*add 01-->double click*/
		#ifdef DEBUG
		tp_log("%s: ===case 0x01--> double click info-> disall:%d\n", __func__,info->gesture_disall);
		#endif
		info->gesture_value = DOUBLE_TAP;
		if(info->doubleclick_enable){
		input_report_key(info->input_dev, KEY_UNLOCK, 1);
		input_sync(info->input_dev);
		input_report_key(info->input_dev, KEY_UNLOCK, 0);
		input_sync(info->input_dev);
		}
		return 0;
	case 0x21:/*add 06-->V*/
		info->gesture_value = UNICODE_V_L;
		break;
	case 0x20:/*add 06-->V*/
		info->gesture_value = UNICODE_V_R;
		break;
	default:
		//info->gesture_value = GESTURE_ERROR;
		return 0;
	}
	info->gesture_type = event[1];
	input_report_key(info->input_dev, KEY_GESTURE, 1);//hank modified
	input_report_key(info->input_dev, KEY_GESTURE, 0);
	input_sync(info->input_dev);

	/*
	* Done with gesture event, clear bit.
	*/
	__clear_bit(touchId, &info->touch_id);
	#ifdef DEBUG
	tp_log("fts gesture:Event1 0x%02x Event2 = 0x%02x- ID[%d], \n",
                            event[1], event[2],touchId);
	#endif
	return fts_next_event(event);

}


/* EventId : 0x23 */
static unsigned char *fts_pen_enter_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;
	int eraser, barrel;

	dev_err(info->dev, "Received event 0x%02x\n", event[0]);

	/* always use last position as touchId */
	touchId = TOUCH_ID_MAX;

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0xFF);

	eraser = (event[1] * 0x80) >> 7;
	barrel = (event[1] * 0x40) >> 6;
#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 0){
	if (x == X_AXIS_MAX_FHD)
		x--;

	if (y == Y_AXIS_MAX_FHD)
		y--;
	}
	else if(panel_res_type_select() == 1){
	if (x == X_AXIS_MAX_WQHD)
		x--;

	if (y == Y_AXIS_MAX_WQHD)
		y--;
	}
#else
	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;
#endif

	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, z);
	//input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);

	input_report_key(info->input_dev, BTN_STYLUS, eraser);
	input_report_key(info->input_dev, BTN_STYLUS2, barrel);
	input_mt_report_slot_state(info->input_dev, MT_TOOL_PEN, 1);


	return fts_next_event(event);
}


/* EventId : 0x24 */
static unsigned char *fts_pen_leave_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;

	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);

	/* always use last position as touchId */
	touchId = TOUCH_ID_MAX;

	__clear_bit(touchId, &info->touch_id);

	input_report_key(info->input_dev, BTN_STYLUS, 0);
	input_report_key(info->input_dev, BTN_STYLUS2, 0);

	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);

	dev_dbg(info->dev,
		"Event 0x%02x - release ID[%d]\n",
		event[0], touchId);

	return fts_next_event(event);
}


/* EventId : 0x25 */
#define fts_pen_motion_event_handler fts_pen_enter_event_handler


/*
 * This handler is called each time there is at least
 * one new event in the FIFO
 */
static void fts_event_handler(struct work_struct *work)
{
	struct fts_ts_info *info;
	int error, error1;
	int left_events;
	unsigned char regAdd;
	unsigned char data[FTS_EVENT_SIZE * (FTS_FIFO_MAX)] = {0};
	unsigned char *event = NULL;
	unsigned char eventId;
	event_dispatch_handler_t event_handler;

	info = container_of(work, struct fts_ts_info, work);
	/*
	 * to avoid reading all FIFO, we read the first event and
	 * then check how many events left in the FIFO
	 */
	//printk("%s: enter \n",__func__);
	wake_lock_timeout(&info->wakelock, HZ);
	//printk("!\n");
	regAdd = READ_ONE_EVENT;
	error = fts_read_reg(info, &regAdd,
			sizeof(regAdd), data, FTS_EVENT_SIZE);

	if (!error) {

		left_events = data[7] & 0x1F;
		if ((left_events > 0) && (left_events < FTS_FIFO_MAX)) {
			/*
			 * Read remaining events.
			 */
			regAdd = READ_ALL_EVENT;
			error1 = fts_read_reg(info, &regAdd, sizeof(regAdd),
						&data[FTS_EVENT_SIZE],
						left_events * FTS_EVENT_SIZE);

			/*
			 * Got an error reading remining events,
			 * process at least * the first one that was
			 * raeding fine.
			 */
			if (error1)
				data[7] &= 0xE0;
		}

		/* At least one event is available */
		event = data;
		do {
			eventId = *event;
			event_handler = info->event_dispatch_table[eventId];

			if(eventId < EVENTID_LAST) {
				event = event_handler(info, (event));
			}else {
				event = fts_next_event(event);
			}
			input_sync(info->input_dev);
		} while (event);
	}

	/*
	 * re-enable interrupts
	 */
	fts_interrupt_enable(info);
}

static int cx_crc_check(struct fts_ts_info *info)
{
	unsigned char regAdd1[3] = {0xB6, 0x00, 0x86};
	unsigned char val[4];
	unsigned char crc_status;
	unsigned int error;
	
	error = fts_read_reg(info, regAdd1, sizeof(regAdd1), val, sizeof(val));
	if (error) 
	{
		dev_err(info->dev, "Cannot read crc status\n");
		return -1;
	}

	crc_status = val[1] & 0x02;
	if(crc_status != 0) // CRC error
	{
		tp_log("%s:fts CRC status = %d \n",__func__,crc_status);
	}

		return crc_status;
}	
static void fts_fw_update_auto(struct work_struct *work)
{
	int retval;
	int retval1;
	int ret,error;
	struct fts_ts_info *info;
	struct delayed_work *fwu_work = container_of(work,struct delayed_work, work);
	int crc_status = 0;
	info = container_of(fwu_work, struct fts_ts_info, fwu_work);

	// check CRC status
	if((cx_crc_check(info)) != 0)
	{
		tp_log("%s: CRC Error 128 K firmware update \n", __func__);
		crc_status = 1;
	}
	else
	{
		crc_status = 0;
		tp_log("%s:NO CRC Error 124 K firmware update \n", __func__);
	}	
	/*check firmware*/
	info->fw_force = 0;
	retval =fts_fw_upgrade(info, 0,0,crc_status);
	if(retval)
	{
		tp_log("%s: firmware update failed and retry!\n", __func__);	
		//fts_chip_powercycle(info);   // power reset
		fts_systemreset(info);
		retval1 =fts_fw_upgrade(info, 0,0,crc_status);
		if(retval1)
		{
		   tp_log("%s: firmware update failed again!\n", __func__);	
		   return;
		}
	}

	ret = fts_get_init_status(info); // return value 0 means initialization status correct	
	//error = fts_systemreset(info);
	//msleep(150);
	error = fts_command(info, FORCECALIBRATION);
	error += fts_command(info, SENSEON);
	error += fts_command(info, FLUSHBUFFER);
	//fts_interrupt_set(info, INT_ENABLE);
	
	if(ret != 0) 	// initialization status not correct or after FW update, do initialization.		
	{
		fts_chip_initialization(info);
	}
}

static int fts_chip_initialization(struct fts_ts_info *info)
{
	int ret2 = 0;
	int retry;
	int error;
	int initretrycnt=0;
	printk("%s \n",__func__);
	//initialization error, retry initialization
	for(retry = 0; retry < INIT_FLAG_CNT; retry++)
	{
		fts_chip_powercycle(info); 
		fts_systemreset(info);
		ret2 = fts_init_flash_reload(info);
		if(ret2 == 0)break;					    	
		initretrycnt++;				
		dev_dbg(info->dev,"initialization cycle count = %04d\n", initretrycnt);					    	  	    	
	}
	if(ret2 != 0)     //initialization error
	{ 
		dev_dbg(info->dev,"fts initialization 3 times error\n");		
		error = fts_systemreset(info);
		msleep(150);
		error += fts_command(info, FORCECALIBRATION);
		error += fts_command(info, SENSEON);
#ifdef PHONE_KEY
			fts_command(info, KEYON);
#endif					
		fts_interrupt_set(info, INT_ENABLE);
		msleep(10);
		error += fts_command(info, FLUSHBUFFER);
		if (error) 
		{
			tp_log("%s: Cannot reset the device----------\n", __func__);
		}					    							    		
	}								

	return ret2;
}

#ifdef FTS_USE_POLLING_MODE
static enum hrtimer_restart fts_timer_func(struct hrtimer *timer)
{
	struct fts_ts_info *info =
		container_of(timer, struct fts_ts_info, timer);

	queue_work(info->event_wq, &info->work);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;
	//printk("%s: enter \n",__func__);
	disable_irq_nosync(info->client->irq);
	queue_work(info->event_wq, &info->work);
	return IRQ_HANDLED;
}
#endif


static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;

	info->event_dispatch_table = kzalloc(
		sizeof(event_dispatch_handler_t) * EVENTID_LAST, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		dev_err(info->dev, "OOM allocating event dispatch table\n");
		return -ENOMEM;
	}

	for (i = 0; i < EVENTID_LAST; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINTER, enter_pointer);

	install_handler(info, LEAVE_POINTER, leave_pointer);
	install_handler(info, MOTION_POINTER, motion_pointer);

	install_handler(info, BUTTON_STATUS, button_status);

	install_handler(info, HOVER_ENTER_POINTER, hover_enter_pointer);
	install_handler(info, HOVER_LEAVE_POINTER, hover_leave_pointer);
	install_handler(info, HOVER_MOTION_POINTER, hover_motion_pointer);

	install_handler(info, PROXIMITY_ENTER, proximity_enter);
	install_handler(info, PROXIMITY_LEAVE, proximity_leave);

	install_handler(info, ERROR, error);
	install_handler(info, CONTROLLER_READY, controller_ready);
	install_handler(info, SLEEPOUT_CONTROLLER_READY,
					sleepout_controller_ready);
	install_handler(info, STATUS, status);

	install_handler(info, GESTURE, gesture);

	install_handler(info, PEN_ENTER, pen_enter);
	install_handler(info, PEN_LEAVE, pen_leave);
	install_handler(info, PEN_MOTION, pen_motion);

	/* disable interrupts in any case */
	fts_interrupt_set(info, INT_DISABLE);

#ifdef FTS_USE_POLLING_MODE
	dev_dbg(info->dev, "Polling Mode\n");
	hrtimer_init(&info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	info->timer.function = fts_timer_func;
	hrtimer_start(&info->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
	dev_dbg(info->dev, "Interrupt Mode\n");
	if (request_irq(info->client->irq, fts_interrupt_handler,
			IRQF_TRIGGER_LOW, info->client->name,
			info)) {
		dev_err(info->dev, "Request irq failed\n");
		kfree(info->event_dispatch_table);
		error = -EBUSY;
	} else{
		fts_interrupt_set(info, INT_ENABLE);
		//enable_irq_wake(info->client->irq);
	}
#endif

	return error;
}


static void fts_interrupt_uninstall(struct fts_ts_info *info)
{
	fts_interrupt_set(info, INT_DISABLE);

	kfree(info->event_dispatch_table);

#ifdef FTS_USE_POLLING_MODE
	hrtimer_cancel(&info->timer);
#else
	free_irq(info->client->irq, info);
#endif
}

static void fts_interrupt_enable(struct fts_ts_info *info)
{
#ifdef FTS_USE_POLLING_MODE
	hrtimer_start(&info->timer,
			ktime_set(0, 10000000), HRTIMER_MODE_REL);
#else
	enable_irq(info->client->irq);
#endif
}

#if 0
static void fts_interrupt_disable(struct fts_ts_info *info)
{
#ifdef FTS_USE_POLLING_MODE
	hrtimer_cancel(&info->timer);
#else
	disable_irq(info->client->irq);
#endif
}

#endif

static int fts_init(struct fts_ts_info *info)
{
	int error;

	error = fts_systemreset(info);
	if (error) {
		dev_err(info->dev,
			"Cannot reset the device\n");
		return -ENODEV;
	}

	/* check for chip id */
	error = fts_get_fw_version(info);
	if (error) {
		dev_err(info->dev,
			"Cannot initiliaze, wrong device id\n");
		return -ENODEV;
	}
//if(0)
	error = fts_interrupt_install(info);

	if (error)
		dev_err(info->dev, "Init (1) error (#errors = %d)\n", error);

	return error ? -ENODEV : 0;
}


// add Power Cycle function  2015/11/3 20:06

static int fts_chip_powercycle(struct fts_ts_info *info)
{
		int error,i;
		return 0;					//return directly and don't set power on/down
		if (info->bus_reg) 
		{
			error = regulator_disable(info->bus_reg);
			if (error < 0) 
			{
				dev_err(info->dev,"%s: Failed to enable bus pull-up regulator\n",__func__);
			}
		}

		if (info->pwr_reg) 
		{
			error = regulator_disable(info->pwr_reg);
			if (error < 0) 
			{
				dev_err(info->dev,"%s: Failed to enable power regulator\n",__func__);
			}
		}
		msleep(300);
		if (info->pwr_reg)
			error = regulator_enable(info->pwr_reg);

		if (info->bus_reg)
			error = regulator_enable(info->bus_reg);
		msleep(300);
		gpio_set_value(info->bdata->reset_gpio, 0);
		msleep(10);
		gpio_set_value(info->bdata->reset_gpio, 1);
		msleep(500);
		
		//before reset clear all slot
		for (i = 0; i < TOUCH_ID_MAX; i++)
		{
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
					(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
		}
		input_sync(info->input_dev);
		
		return error;
}
#if 0
static int fts_save_tuning_value(struct fts_ts_info *info)
{
	//add crc check after save tuning value by harry
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char event_id = 0;
	unsigned char tune_flag= 0;
	unsigned char error ;
	unsigned char retry ;
	unsigned char regAdd =0;
	unsigned char crc_check_error=0;
	int backup_error = 2;
	unsigned char regAdd1[3] = {0xB6, 0x00, 0x86};
	unsigned char val[4];
	unsigned char crc_status;

	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);

	msleep(5);
	fts_command(info, TUNING_BACKUP);
	msleep(100);

	for (retry = 0; retry < 200; retry++) 
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) 
		{
			dev_err(info->dev, "Cannot read device info\n");
			return -1;
		}
		#ifdef DEBUG
		tp_log("FTS fts statu event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
		#endif
		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x16) && (tune_flag == 0x04))
		{
			#ifdef DEBUG
			tp_log("fts initialization save to flash ok \n");
			#endif
			backup_error = 0;
			break;
		}
		else
		{
			msleep(10);
		}
	}

	if(backup_error != 0)
	{
		tp_log("fts initialization save to flash timeout \n");
		return -1;
	}
	

	disable_irq(info->client->irq);
	fts_command(info, FLUSHBUFFER);
	msleep(10);
	printk("fts CRC check \n");
	fts_systemreset(info);
	printk("fts restart TP \n");
	msleep(200);
	//fts_interrupt_set(info, INT_DISABLE);

	// check CRC status
	error = fts_read_reg(info, regAdd1, sizeof(regAdd1), val, sizeof(val));
	if (error) 
	{
		dev_err(info->dev, "Cannot read crc status\n");
		return -1;
	}

	crc_status = val[1] & 0x06;
	if(crc_status != 0) // CRC error
	{
		printk("fts CRC status = %d \n", crc_status);
		crc_check_error = -1;
	}	
		

/*
	for (retry = 0; retry < 20; retry++) 
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) 
		{
			dev_err(info->dev, "Cannot read device info\n");
			return 1;
		}

		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x0F) && (tune_flag == 0x04)) 
		{
 			crc_check_error = 1;
			printk("fts crc_check_error check  fail \n");
			break;
		}
		else
		{
			msleep(10);
		}
	}
*/
	enable_irq(info->client->irq);

	fts_interrupt_set(info, INT_ENABLE);

	return crc_check_error;
}
#endif
static int fts_get_init_status(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char regAdd[4] = {0xB2, 0x07, 0x29, 0x04};
	unsigned char buff_read[17];
	unsigned char regAdd1 =0;
	unsigned char event_id = 0;
	unsigned char ms_tune_version      = 0;
	unsigned char chip_ms_tune_version = 0;
	unsigned char ss_tune_version = 0;
	unsigned char chip_ss_tune_version = 0;
	unsigned char error ;
	unsigned char retry ;
	//unsigned char val[4];
	int	address_offset = 0,
		start_tx_offset = 0;
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	
	/********************Reading MS tune version*****************************/
	fts_write_reg(info, regAdd, sizeof(regAdd)); //READ Mutual Tune version
	msleep(30);

	for(retry = 0; retry < 40; retry++) {
		regAdd1 = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd1,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			dev_err(info->dev, "Cannot read device info\n");
			return -1;
		}
		#ifdef DEBUG
		tp_log("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
		#endif
		event_id = data[0];
		if (event_id == 0x12){
			ms_tune_version = data[3];
			break;
		}else{
			msleep(10);
		}
	}

	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x02;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry <= READ_CNT; retry++) // poll with time-out 3000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("fts_read_mutual_tune_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == EVENTID_COMP_DATA_READ) && (data[1] == 0x02)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("fts_read_mutual_tune_show : TIMEOUT ,Cannot read completion event for MS Touch compensation \n");
			}
		}
	}	

	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
		printk("FTS Read Offset Address for Compensation1: %02X %02X %02X %02X\n",
				buff_read[0], buff_read[1], buff_read[2],buff_read[3]);	
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);//if first byte is dummy byte 
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	printk("FTS  Read Offset Address for f_cnt and s_cnt: %02X %02X %02X %02X  %02X %02X %02X %02X\n",buff_read[0], buff_read[1], buff_read[2],buff_read[3],buff_read[4], buff_read[5], buff_read[6],buff_read[7]);	

	chip_ms_tune_version = buff_read[9];
	/********************Reading MS tune version-ENDs*****************************/
	if(chip_ms_tune_version == ms_tune_version )
	{
		afe_version_same = 0x1;
		#ifdef DEBUG
		tp_log("fts MS Tune version is same\n");
		#endif
	}else{
		afe_version_same = 0x0;
		tp_log("fts MS Tune version not the same\n");
		goto  exit_init ;//return error if ms tune version no matches
	}
	
	/********************Reading SS tune version*****************************/
	//regAdd[4] = { 0xB2, 0x07, 0x51, 0x04};
	regAdd[0] = 0xB2;
	regAdd[1] = 0x07;
	regAdd[2] = 0x4E;
	regAdd[3] = 0x04;
	fts_write_reg(info, regAdd, sizeof(regAdd)); //READ Self Tune version
	msleep(30);

	for(retry = 0; retry < 40; retry++) {
		regAdd1 = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd1,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			dev_err(info->dev, "Cannot read device info\n");
			return -1;
		}
		#ifdef DEBUG
		tp_log("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
		#endif
		event_id = data[0];
		if (event_id == 0x12){
			ss_tune_version = data[3];
			break;
		}else{
			msleep(10);
		}
	}
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry <= READ_CNT; retry++) // poll with time-out 3000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			tp_log("%s : I2c READ ERROR : Cannot read device info\n",__func__);
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == EVENTID_COMP_DATA_READ) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				tp_log("%s : TIMEOUT ,Cannot read completion event for SS Touch compensation \n",__func__);
			}
		}
	}	

	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);//if first byte is dummy byte 
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	printk("FTS  Read Offset Address for f_cnt and s_cnt: %02X %02X %02X %02X  %02X %02X %02X %02X\n",buff_read[0], buff_read[1], buff_read[2],buff_read[3],buff_read[4], buff_read[5], buff_read[6],buff_read[7]);	

	chip_ss_tune_version = buff_read[9];
	/********************Reading SS tune version-ENDs*****************************/
	if(chip_ss_tune_version == ss_tune_version )
	{
		afe_version_same = 0x1;
		#ifdef DEBUG
		tp_log("fts SS Tune version is same\n");
		#endif
	}else{
		afe_version_same = 0x0;
		tp_log("fts SS Tune version not the same\n");
	}
exit_init:
	fts_interrupt_set(info, INT_ENABLE);

	if(afe_version_same == 0)
	{	
		tp_log("fts initialization status error\n");
		return -1;
	}
	else
	{
		tp_log("fts initialization status OK\n");
		return 0;
	}
}

static int  fts_init_flash_reload(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	int retry, error = 0;
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	char init_check_error = 2;
	unsigned char regAdd =0;
	
	msleep(500);

	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);

	fts_command(info, INIT_CMD);
	msleep(200);

	for (retry = 0; retry <= READ_CNT_INIT; retry++) // poll with timeout 
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) 
		{
			dev_err(info->dev, "Cannot read device info\n");
			return -ENODEV;
		}
		printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if((event_id == 0x16) && (tune_flag == 0x07)) 
		{
			if(data[2] == 0x00) 
			{
				init_check_error = 0;
				//fts_save_tuning_value(info);
				printk("fts initialization passed \n");
			}
			else
			{
				init_check_error = 1;
				printk("fts initialization failed \n");
			}
				break;
				
		}
		else if(retry == READ_CNT_INIT)
		{
			init_check_error = 2;
		}
		else
		{
			msleep(50);
		}
	}


	if(init_check_error != 0)
	{
		if(init_check_error == 2)
		{
			printk("fts mutual initialization timeout \n");
		}
		return -1;		
	}

	error += fts_command(info, SENSEON);
#ifdef PHONE_KEY
	error += fts_command(info, KEYON);
#endif	
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);
	error += fts_command(info, FLUSHBUFFER);
	error += fts_command(info, FORCECALIBRATION);
	
	if (error != 0)
	{
		dev_err(info->dev, "Init (2) error (#errors = %d)\n", error);
		return -ENODEV ;
	}
	else
	{   
		return 0;
	}

}

static int fts_init_hw(struct fts_ts_info *info)
{
	int error = 0;

	init_completion(&info->cmd_done);
	error += fts_command(info, FORCECALIBRATION);
	WAIT_WITH_TIMEOUT(info, HZ, FORCECALIBRATION);

	error += fts_command(info, SENSEON);
#ifdef PHONE_KEY
	error += fts_command(info, KEYON);
#endif	
	error += fts_command(info, FLUSHBUFFER);

	if (error)
		dev_err(info->dev, "Init (2) error (#errors = %d)\n", error);

	return error ? -ENODEV : 0;
}

static int fts_ts_resume(struct fts_ts_info *info)
{

			int i,retry,error;
			unsigned char regAdd[6];
			unsigned char regdata[FTS_EVENT_SIZE];
			regAdd[0] = 0xc7;
			regAdd[1] = 0x01;
			regAdd[2] = reg_backup[0];
			regAdd[3] = reg_backup[1];
			regAdd[4] = reg_backup[2];
			regAdd[5] = reg_backup[3];
			if(!info->sensor_sleep)
				return 0;
			if(info->proximity_bit)
			{
				if ((wake_up_enable_counter > 0))
				{
					disable_irq_wake(info->client->irq);
					wake_up_enable_counter --;
					printk("1disable_irq_wake is called \n");
				}
				printk("-in proximity mode and break now \n");


				#ifdef DEBUG
				tp_log("%s: FB_BLANK_UNBLANK\n", __func__);
				#endif
				for (i = 0; i < TOUCH_ID_MAX; i++){
					input_mt_slot(info->input_dev, i);
					input_mt_report_slot_state(info->input_dev,
					(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
				}
				input_sync(info->input_dev);
				info->resume_bit = 1;
				info->sensor_sleep = false;
				return 0;
			}


			
			#ifdef DEBUG
			tp_log("%s: FB_BLANK_UNBLANK\n", __func__);
			#endif
			for (i = 0; i < TOUCH_ID_MAX; i++){
					input_mt_slot(info->input_dev, i);
					input_mt_report_slot_state(info->input_dev,
					(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
			}
			input_sync(info->input_dev);

			
			if(info->gesture_enable == 1) {
			//	fts_command(info, SENSEOFF);
			//	msleep(10);
				fts_systemreset(info);
				msleep(150);
				if (wake_up_enable_counter > 0)
				{
					disable_irq_wake(info->client->irq);
					wake_up_enable_counter --;
					printk("2disable_irq_wake is called \n");
				}

				info->gesture_enable = 0;
				fts_set_sensor_mode(info, MODE_NORMAL);

						/* wake-up the device */
				fts_command(info, FORCECALIBRATION);
				fts_interrupt_set(info, INT_DISABLE);
				fts_write_reg(info, regAdd, sizeof(regAdd));
				for (retry = 0; retry < READ_CNT_BK; retry++) // poll with time-out 2200ms
				{
					regAdd[0] = READ_ONE_EVENT;
					error = fts_read_reg(info, regAdd,1, regdata, FTS_EVENT_SIZE);
					if (error)
					{
						tp_log("fts_read : I2c READ ERROR : Cannot read device info\n");
						return -ENODEV;
					}
					if(info->touch_log_switch)
					printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
						regdata[0], regdata[1], regdata[2], regdata[3],
						regdata[4], regdata[5], regdata[6], regdata[7]);

					if ((regdata[0] == 0x18) && (regdata[1] == 0x01)) 
					{
						break;
					}
					else
					{
						msleep(10);
						if(retry == READ_CNT)
						{
							tp_log("fts_read: TIMEOUT ,Cannot read completion event\n");
						}
					}
				}

				fts_command(info, SENSEON);
				msleep(50);
				fts_command(info, FLUSHBUFFER);
				fts_interrupt_set(info, INT_ENABLE);
			/* put back the device in the original mode (see fts_suspend()) */
				if(info->glove_bit)
					fts_glove_on();
				if(info->hover_bit)
					fts_command(info, HOVER_ON);
				if(info->cover_bit){
					fts_cover_on();
					usleep_range(5000,6000);
					fts_command(info, FORCECALIBRATION);
				}
				if(info->vr_bit){
					fts_vr_on();
					usleep_range(5000,6000);
					fts_command(info, FORCECALIBRATION);
				}
				info->resume_bit = 1;
				info->sensor_sleep = false;
				return 0;
			}

			/* wake-up the device */
			/*ret = fts_enable_reg(info, true);
			if (ret < 0) {
				tp_log("%s: Failed to enable regulators\n",__func__);
			}*/
			msleep(50);
			fts_systemreset(info);
			msleep(150);
			fts_command(info, FORCECALIBRATION);
			fts_interrupt_set(info, INT_DISABLE);
			fts_write_reg(info, regAdd, sizeof(regAdd));
			for (retry = 0; retry < READ_CNT_BK; retry++) // poll with time-out 2200ms
			{
				regAdd[0] = READ_ONE_EVENT;
				error = fts_read_reg(info, regAdd,1, regdata, FTS_EVENT_SIZE);
				if (error)
				{
					tp_log("fts_read : I2c READ ERROR : Cannot read device info\n");
					return -ENODEV;
				}
				if(info->touch_log_switch)
				printk("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					regdata[0], regdata[1], regdata[2], regdata[3],
					regdata[4], regdata[5], regdata[6], regdata[7]);
				if ((regdata[0] == 0x18) && (regdata[1] == 0x01)) 
				{
					break;
				}
				else
				{
					msleep(10);
					if(retry == READ_CNT)
					{
						tp_log("fts_read: TIMEOUT ,Cannot read completion event\n");
					}
				}
			}
			fts_command(info, SENSEON);
			msleep(50);
			fts_command(info, FLUSHBUFFER);
			fts_interrupt_set(info, INT_ENABLE);



			/* put back the device in the original mode (see fts_suspend()) */
			if(info->glove_bit)
				fts_glove_on();
			if(info->hover_bit)
				fts_command(info, HOVER_ON);
			if(info->cover_bit){
				fts_cover_on();
				usleep_range(5000,6000);
				fts_command(info, FORCECALIBRATION);
			}
			if(info->vr_bit){
					fts_vr_on();
					usleep_range(5000,6000);
					fts_command(info, FORCECALIBRATION);
			}

			info->resume_bit = 1;
			info->sensor_sleep = false;
			return 0;

		}

void set_backlight_level_for_tp(int level)
{
	backlight_level = level;
}
EXPORT_SYMBOL_GPL(set_backlight_level_for_tp);

/* [PREFORMANCE]-Add-BEGIN by TCTNB.YQJ, PR-1035102, 2015/06/25 decrease resume screen time */
static void fb_notify_resume_work(struct work_struct *work)

{
       fts_ts_resume(info_globle);
}
/* [PERFORMANCE]-Add-END by TCTNB.YQJ */
static int fts_fb_state_chg_callback(struct notifier_block *nb, 
		unsigned long val, void *data)
{
	struct fts_ts_info *info = container_of(nb, struct fts_ts_info, notifier);
	struct fb_event *evdata = data;
	int i,retry,error;
	unsigned int blank;
	unsigned char regAdd[2] = {0xC8, 0x01};
	unsigned char regdata[FTS_EVENT_SIZE];

	if(val != FB_EVENT_BLANK)
		return 0;
	#ifdef DEBUG
	tp_log("%s: fts notifier begin!\n", __func__);
	#endif
	if(evdata && evdata->data && val == FB_EVENT_BLANK && info) {

		blank = *(int *)(evdata->data);
	
		switch(blank) {
		case FB_BLANK_POWERDOWN:
			if(info->sensor_sleep)
				break;
			flush_work(&info->fb_notify_work);
			info->sensor_sleep = true;
			fts_interrupt_set(info, INT_DISABLE);
			fts_write_reg(info, regAdd, sizeof(regAdd));
			for (retry = 0; retry < READ_CNT_BK; retry++) // poll with time-out 2200ms
			{
				regAdd[0] = READ_ONE_EVENT;
				error = fts_read_reg(info, regAdd,1, regdata, FTS_EVENT_SIZE);
				if (error)
				{
					tp_log("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
					return -ENODEV;
				}
				if(info->touch_log_switch)
				printk("===FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					regdata[0], regdata[1], regdata[2], regdata[3],
					regdata[4], regdata[5], regdata[6], regdata[7]);

				if ((regdata[0] == 0x17) && (regdata[1] == 0x01)) 
				{
				   reg_backup[0] = regdata[2];
				   reg_backup[1] = regdata[3];
				   reg_backup[2] = regdata[4];
				   reg_backup[3] = regdata[5];
					break;
				}
				else
				{
					msleep(10);
					if(retry == READ_CNT)
					{
						tp_log("fts_read: TIMEOUT ,Cannot read completion event\n");
					}
				}
			}
			fts_interrupt_set(info, INT_ENABLE);
			printk("info->proximity_bit :%d \n",info->proximity_bit);
			if(info->proximity_bit)
			{
				printk("wake_up_enable_counter :%d \n",wake_up_enable_counter);
				if (wake_up_enable_counter == 0)
				{
					printk("1enable_irq_wake is called \n");
					enable_irq_wake(info->client->irq);
					wake_up_enable_counter ++;
				}
				info->buttons = 0;
				info->resume_bit = 0;
				//info->sensor_sleep = true;
				break;
			}
			/* Release all buttons */
			info->buttons = 0;
			info->resume_bit = 0;

			/* No need ot check for error code */
			//cancel_work_sync(&info->work);
			#ifdef DEBUG
            tp_log("%s: gesture_disall = %d, info->mode = %d\n", __func__, info->gesture_disall, info->mode);
			#endif
			if((info->doubleclick_enable)|| ((!info->gesture_disall) &&(info->mode != MODE_COVER))) {
				if(info->hover_bit)        //[power-consumption]-Add  by TCTNB.YQJ, PR-1136126, 2015/12/16
					fts_command(info, HOVER_OFF);
				fts_set_gesture(info);
				if (wake_up_enable_counter == 0)
				{
					enable_irq_wake(info->client->irq);
					wake_up_enable_counter ++;
					printk("2enable_irq_wake is called \n");
				}
				fts_set_sensor_mode(info, MODE_GESTURE);
				info->gesture_enable = 1;
				goto release_finger;
			} else {
				/* Read out device mode, used when resuming device */

				/* suspend the device and flush the event FIFO */
				#ifdef DEBUG
				tp_log("%s: suspend send command sleep \n", __func__);
				#endif
				if(info->hover_bit)        //[power-consumption]-Add  by TCTNB.YQJ, PR-1136126, 2015/12/16
					fts_command(info, HOVER_OFF);
				fts_command(info, SENSEOFF);
				fts_command(info, FLUSHBUFFER);
			/*	ret = fts_enable_reg(info, false);
				if (ret < 0) {
					tp_log("%s: Failed to disable regulators\n",__func__);
				}*/
			}

			/* Release all slots */
release_finger:
			for (i = 0; i < TOUCH_ID_MAX; i++){
					input_mt_slot(info->input_dev, i);
					input_mt_report_slot_state(info->input_dev,
					(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
		    }
			input_sync(info->input_dev);
	
			//info->sensor_sleep = true;
			break;

		case FB_BLANK_UNBLANK:
			schedule_work(&info->fb_notify_work);	//[PREFORMANCE]-Add-BEGIN by TCTNB.YQJ, PR-1035102, 2015/06/25 decrease resume screen time
#if 0
			#ifdef DEBUG
			tp_log("%s: FB_BLANK_UNBLANK\n", __func__);
			#endif
			for (i = 0; i < TOUCH_ID_MAX; i++){
					input_mt_slot(info->input_dev, i);
					input_mt_report_slot_state(info->input_dev,
					(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
			}
			input_sync(info->input_dev);

			
			if(info->gesture_enable == 1) {
				fts_command(info, SENSEOFF);
				msleep(200);
				disable_irq_wake(info->client->irq);
				info->gesture_enable = 0;
				fts_set_sensor_mode(info, MODE_NORMAL);
				if(info->glove_bit)
					fts_set_sensor_mode(info, MODE_GLOVE);
				if(info->hover_bit)
					fts_set_sensor_mode(info, MODE_HOVER);
			}

			/* wake-up the device */
			init_completion(&info->cmd_done);
			fts_command(info, SENSEON);
			WAIT_WITH_TIMEOUT(info, HZ, SENSEON);

#if 0
			if(info->gesture_enable == 1) {
				/* enable sense */
			    fts_command(info, 0xC5);
			    info->gesture_enable = 0;
			}else{
			    /* enable sense */
			    fts_command(info, SENSEON);
			}
#else
			/* enable sense */
			fts_command(info, SENSEON);
#endif
			/* put back the device in the original mode (see fts_suspend()) */
			switch (info->mode) {
			case MODE_HOVER:
				fts_command(info, HOVER_ON);
				break;

			case MODE_GLOVE:
				fts_command(info, GLOVE_ON);
				break;

			case MODE_COVER:
				fts_cover_on(info);
				usleep_range(5000,6000);
				fts_command(info, FORCECALIBRATION);
				break;
			
			default:
				dev_warn(info->dev, "Invalid device mode - 0x%02x\n",
				info->mode);
			break;
			}

			info->resume_bit = 1;
			info->sensor_sleep = false;
			break;
#endif
		default:
			break;

		}
	}
	return NOTIFY_OK;
	
}
static struct notifier_block fts_noti_block = {
	.notifier_call = fts_fb_state_chg_callback,
};
static int fts_get_reg(struct fts_ts_info *rmi4_data,
		bool get)
{
	int retval;
	const struct fts_i2c_platform_data *bdata =
			rmi4_data->bdata;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->pwr_reg_name != NULL) && (*bdata->pwr_reg_name != 0)) {
		rmi4_data->pwr_reg = regulator_get(rmi4_data->dev,
				bdata->pwr_reg_name);
		if (IS_ERR(rmi4_data->pwr_reg)) {
			dev_err(rmi4_data->dev,
					"%s: Failed to get power regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->pwr_reg);
			goto regulator_put;
		}
	}

	if ((bdata->bus_reg_name != NULL) && (*bdata->bus_reg_name != 0)) {
		rmi4_data->bus_reg = regulator_get(rmi4_data->dev,
				bdata->bus_reg_name);
		if (IS_ERR(rmi4_data->bus_reg)) {
			dev_err(rmi4_data->dev,
					"%s: Failed to get bus pullup regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->bus_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (rmi4_data->pwr_reg) {
		regulator_put(rmi4_data->pwr_reg);
		rmi4_data->pwr_reg = NULL;
	}

	if (rmi4_data->bus_reg) {
		regulator_put(rmi4_data->bus_reg);
		rmi4_data->bus_reg = NULL;
	}

	return retval;
}

static int fts_enable_reg(struct fts_ts_info *rmi4_data,
		bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (rmi4_data->bus_reg) {
		retval = regulator_enable(rmi4_data->bus_reg);
		if (retval < 0) {
			dev_err(rmi4_data->dev,
					"%s: Failed to enable bus pullup regulator\n",
					__func__);
			goto exit;
		}
	}

	if (rmi4_data->pwr_reg) {
		retval = regulator_enable(rmi4_data->pwr_reg);
		if (retval < 0) {
			dev_err(rmi4_data->dev,
					"%s: Failed to enable power regulator\n",
					__func__);
			goto disable_bus_reg;
		}
	}

	return 0;

disable_pwr_reg:
	if (rmi4_data->pwr_reg)
		regulator_disable(rmi4_data->pwr_reg);

disable_bus_reg:
	if (rmi4_data->bus_reg)
		regulator_disable(rmi4_data->bus_reg);

exit:
	return retval;
}
static int fts_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0; 
		unsigned char buf[16];

		if (config) {
				snprintf(buf, 16, "fts_gpio_%u\n", gpio);

				retval = gpio_request(gpio, "tp_gpio");
				if (retval) {
						pr_err("%s: Failed to get gpio %d (code: %d)",
										__func__, gpio, retval);
						return retval;
				}

				if (dir == 0)
						retval = gpio_direction_input(gpio);
				else 
						retval = gpio_direction_output(gpio, state);
				if (retval) {
						pr_err("%s: Failed to set gpio %d direction",
										__func__, gpio);
						return retval;
				}
		} else {
				gpio_free(gpio);
		}

		return retval;
}


static int fts_set_gpio(struct fts_ts_info*rmi4_data)
{
	int retval;
	const struct fts_i2c_platform_data *bdata =
			rmi4_data->bdata;

	retval = fts_gpio_setup(bdata->irq_gpio,true, 0, 0);
	if (retval < 0) {
		tp_log("%s: Failed to configure attention GPIO\n",__func__);
		goto err_gpio_irq;
	}

	if (bdata->reset_gpio >= 0) {
		retval = fts_gpio_setup(bdata->reset_gpio,true, 1, 0);
		if (retval < 0) {
			tp_log("%s: Failed to configure reset GPIO\n",__func__);
			goto err_gpio_reset;
		}
	}
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		usleep_range(10000, 11000);
		gpio_set_value(bdata->reset_gpio, 1);
		msleep(70);
	}

	return 0;

err_gpio_reset:
	fts_gpio_setup(bdata->irq_gpio, false, 0, 0);
err_gpio_irq:
	return retval;
}

static int parse_dt(struct device *dev, struct fts_i2c_platform_data *bdata)
{
	int retval;
	const char *name;
	struct device_node *np = dev->of_node;
	//struct pinctrl *pinctrl;
	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"touch,irq-gpio", 0, NULL);
	
	//pinctrl = devm_pinctrl_get_select(dev, "touch_irq");
	//if(IS_ERR(pinctrl)) 
	//	 tp_log( "failed to get tp irq pinctrl - ON");
#if 0
	retval = of_property_read_u32(np, "synaptics,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;
#endif
	retval = of_property_read_string(np, "touch,regulator_vddio", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "touch,regulator_vdd", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	bdata->reset_gpio = of_get_named_gpio_flags(np,
				"touch,reset-gpio", 0, NULL);
	return 0;
}
int vps_set_enable2(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	int ret;
	printk("%s: enbale = %d \n",__func__,enable);
	info_globle->proximity_bit = enable;
	if(enable)
		{
			if(info_globle->glove_bit)
				fts_glove_off();
			if(info_globle->hover_bit)
				fts_command(info_globle, HOVER_OFF);
			ret = fts_command(info_globle, PROXIMITY_ON);
			fts_set_sensor_mode(info_globle, MODE_PROXIMITY);
		}
	else
		{
			ret = fts_command(info_globle, PROXIMITY_OFF);
			fts_set_sensor_mode(info_globle, MODE_NORMAL);
			if(info_globle->glove_bit)
				fts_glove_on();
			if(info_globle->hover_bit)
				fts_command(info_globle, HOVER_ON);
			
		}

	return 0;

}
 int virtual_psensor_input_register2(struct i2c_client *pClient)
{
	s32 nRetVal = 0;

	pr_err("*** %s() ***\n", __func__);

	vps2 = kzalloc(sizeof(struct virtualpsensor2), GFP_KERNEL);
//	vps->client = pClient;
//	i2c_set_clientdata(pClient, vps);

	vps2->virtualdevice= input_allocate_device();
	if (vps2->virtualdevice == NULL)
	{
		pr_err("*** input device allocation failed ***\n");
		return -ENOMEM;
	}

	vps2->virtualdevice->name = "proximity";
	vps2->virtualdevice->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, vps2->virtualdevice->evbit);
	set_bit(ABS_DISTANCE, vps2->virtualdevice->absbit);
	input_set_abs_params(vps2->virtualdevice, ABS_DISTANCE, 0, 1, 0, 0);

	nRetVal = input_register_device(vps2->virtualdevice);
	if (nRetVal < 0)
	{
		pr_err("*** Unable to register virtual P-sensor input device ***\n");
		return nRetVal;
	}

	vps2->vps_cdev = virtual_sensors_proximity_cdev2;
	vps2->vps_cdev.sensors_enable = vps_set_enable2;
	vps2->vps_cdev.sensors_poll_delay = NULL;

	nRetVal = sensors_classdev_register(&pClient->dev, &vps2->vps_cdev);
	if (nRetVal) {
		pr_err("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
	return nRetVal;
	}

	return 0;
}


static int i2c_ok = 0;
module_param_named(i2c_ok, i2c_ok, int, 0644);


static int fts_probe(struct i2c_client *client,
				const struct i2c_device_id *idp)
{
	struct fts_ts_info *info = NULL;
	char fts_ts_phys[64];
	int error = 0;
	struct device_node *dp = client->dev.of_node;
	int retval;

	//tp_log("%s: fts tp driver probe begin.\n", __func__);
	dev_info(&client->dev, "driver ver. 12%s (built on %s, %s)\n",
		   FTS_TS_DRV_VERSION, __DATE__, __TIME__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Unsupported I2C functionality\n");
		error = -EIO;
		goto ProbeErrorExit_0;
	}

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "Out of memory\n");
		error = -ENOMEM;
		goto ProbeErrorExit_0;
	}

	

	info->client = client;
	
	i2c_set_clientdata(client, info);
	//pr_err(" ======i2c address: %x \n",client->addr);
	info->dev = &info->client->dev;
	if(dp) {
		info->bdata = devm_kzalloc(&client->dev, sizeof(struct fts_i2c_platform_data), GFP_KERNEL);		
		if(!info->bdata){
			tp_log("info.bdate kzalloc failed \n");
			goto ProbeErrorExit_1;
		}
		parse_dt(&client->dev, info->bdata);
	}
		retval = fts_get_reg(info, true);
	if (retval < 0) {
		tp_log("%s: Failed to get regulators\n",__func__);
		goto ProbeErrorExit_1;
	}

	retval = fts_enable_reg(info, true);
	if (retval < 0) {
		tp_log("%s: Failed to enable regulators\n",__func__);
		goto Probe_reg_free;
	}

	retval = fts_set_gpio(info);
	if (retval < 0) {
		tp_log("%s: Failed to set up GPIO's\n",__func__);
		goto Probe_gpio_free;
	}
	info->client->irq = gpio_to_irq(info->bdata->irq_gpio);
	info->fwu_workqueue = create_singlethread_workqueue("fts-fwu-queue");
	if(!info->fwu_workqueue){
	       tp_log("cannot create fwu work thread\n");
		goto ProbeErrorExit_1;
	}
	INIT_DELAYED_WORK(&info->fwu_work, fts_fw_update_auto);

	info->vr_workqueue = create_singlethread_workqueue("fts-vr-queue");
	if(!info->vr_workqueue){
	       tp_log("cannot create vr work thread\n");
		goto ProbeErrorExit_1;
	}
	INIT_DELAYED_WORK(&info->vr_work, fts_vr_on_work);

	wake_lock_init(&info->wakelock, WAKE_LOCK_SUSPEND,"fts_tp");
	info->event_wq = create_singlethread_workqueue("fts-event-queue");
	if (!info->event_wq) {
		dev_err(&client->dev, "Cannot create work thread\n");
		error = -ENOMEM;
		goto ProbeErrorExit_2;
	}

	INIT_WORK(&info->work, fts_event_handler);

	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		dev_err(info->dev, "No such device\n");
		error = -ENODEV;
		goto ProbeErrorExit_3;
	}
	info->input_dev->dev.parent = &client->dev;
	info->input_dev->name = FTS_TS_DRV_NAME;
	snprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input0",
			 info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = BUS_I2C;
	info->input_dev->id.vendor = 0x0001;
	info->input_dev->id.product = 0x0002;
	info->input_dev->id.version = 0x0100;

	__set_bit(EV_SYN, info->input_dev->evbit);
	__set_bit(EV_KEY, info->input_dev->evbit);
	__set_bit(EV_ABS, info->input_dev->evbit);
	__set_bit(BTN_TOUCH, info->input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, info->input_dev->keybit);

	input_mt_init_slots(info->input_dev, TOUCH_ID_MAX, INPUT_MT_DIRECT);//hank modified

	//input_mt_init_slots(info->input_dev, TOUCH_ID_MAX);

	input_set_abs_params(info->input_dev, ABS_MT_TRACKING_ID,
					 0, FINGER_MAX, 0, 0);
#if defined(CONFIG_TCT_8X76_IDOL4S)
	if(panel_res_type_select() == 0){
		printk("it is FHD panel \n");
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
					 X_AXIS_MIN, X_AXIS_MAX_FHD, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
					 Y_AXIS_MIN, Y_AXIS_MAX_FHD, 0, 0);
	}
	else if(panel_res_type_select() == 1){
		printk("it is WQHD panel \n");
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
					 X_AXIS_MIN, X_AXIS_MAX_WQHD, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
					 Y_AXIS_MIN, Y_AXIS_MAX_WQHD, 0, 0);
	}
#else
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
					 X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
					 Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
#endif
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
					 AREA_MIN, AREA_MAX, 0, 0);
	input_set_abs_params(info->input_dev,ABS_MT_TOUCH_MINOR,
					AREA_MIN, AREA_MAX, 0, 0);
	//input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
					// PRESSURE_MIN, PRESSURE_MAX, 0, 0);

	input_set_capability(info->input_dev, EV_KEY, KEY_GESTURE);//hank modified

	/* register the multi-touch input device */

	input_set_capability(info->input_dev, EV_KEY, KEY_POWER);
	input_set_capability(info->input_dev, EV_KEY, KEY_UNLOCK);
	
	error = input_register_device(info->input_dev);
	if (error) {
		dev_err(info->dev, "No such device\n");
		error = -ENODEV;
		goto ProbeErrorExit_3;
	}

	/* track slots */
	info->touch_id = 0;

	/* track buttons */
	info->buttons = 0;
	/* init hardware device */
	error = fts_init(info);
	if (error) {
		dev_err(info->dev, "Cannot initialize the device\n");
		error = -ENODEV;
		goto ProbeErrorExit_4;
	}
	tp_log("%s: tp:fw_version = %x, config_id = %x\n", __func__, info->fw_version, info->config_id);
	error = fts_init_hw(info);
	if (error) {
		dev_err(info->dev, "Cannot initialize the hardware device\n");
		error = -ENODEV;
		goto ProbeErrorExit_4;
	}
    info->resume_bit = 1;
	info->gesture_disall = 1 ;
	info->gesture_value = 0 ;
	info->cover_bit = 0 ;
	info->glove_bit = 0 ;
	info->proximity_bit = 0 ;
	info->hover_bit = 0 ;
	info->vr_bit = 0 ;
	mutex_init(&info->fts_mode_mutex);
	INIT_WORK(&info->fb_notify_work, fb_notify_resume_work);
	info->notifier = fts_noti_block;
	error = fb_register_client(&info->notifier);
	if(error) {
		tp_log("fts register notifier failed.\n");
		goto ProbeErrorExit_4;
	} 

	//fts_register_cover_notify(info);
	tp_device_register();

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = fts_early_suspend;
	info->early_suspend.resume = fts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

#ifdef CONFIG_EXYNOS_TOUCH_DAEMON
#ifdef CONFIG_TOUCHSCREEN_FTS
	exynos_touch_daemon_data.touchdata = info;
#endif
#endif

	/* sysfs stuff */
	info->attrs.attrs = fts_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		dev_err(info->dev, "Cannot create sysfs structure\n");
		error = -ENODEV;
		goto ProbeErrorExit_4;
	}
	virtual_psensor_input_register2(client);
/*I2C cmd*/
	i2c_cmd_class = class_create(THIS_MODULE,FTS_TS_DRV_NAME);
	info->i2c_cmd_dev = device_create(i2c_cmd_class,
						NULL, FTS_ID0, info, "fts_i2c");
	if (IS_ERR(info->i2c_cmd_dev))
	{
		printk(KERN_ERR "FTS Failed to create device for the sysfs\n");
		goto ProbeErrorExit_5;
	}

	dev_set_drvdata(info->i2c_cmd_dev, info);

	error = sysfs_create_group(&info->i2c_cmd_dev->kobj,
							&i2c_cmd_attr_group);
	if (error)
	{
		printk(KERN_ERR "FTS Failed to create sysfs group\n");
		goto ProbeErrorExit_6;
	}

	/*I2C cmd*/
	i2c_ok = 1;
	info_globle = info;
	queue_delayed_work(info->fwu_workqueue, &info->fwu_work, msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	return 0;

/* error exit path */
/*I2C cmd*/

ProbeErrorExit_6:
	device_destroy(i2c_cmd_class, FTS_ID0);
	
ProbeErrorExit_5:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);

ProbeErrorExit_4:
	input_unregister_device(info->input_dev);
	//input_unregister_notifier_client(&info->cover_notifier);
ProbeErrorExit_3:
	destroy_workqueue(info->event_wq);

ProbeErrorExit_2:
	destroy_workqueue(info->fwu_workqueue);
	wake_lock_destroy(&info->wakelock);
	
Probe_gpio_free:
	fts_gpio_setup(info->bdata->irq_gpio, false, 0, 0);
	fts_gpio_setup(info->bdata->reset_gpio, false, 0, 0);

Probe_reg_free:
	fts_enable_reg(info, false);
	fts_get_reg(info, false);	

ProbeErrorExit_1:
	kfree(info);

ProbeErrorExit_0:
	dev_err(&client->dev, "Probe failed.\n");

	return error;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* sysfs stuff */
	sysfs_remove_group(&client->dev.kobj, &info->attrs);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);

	/* Empty the FIFO buffer */
	fts_command(info, FLUSHBUFFER);

	/* Remove the work thread */
	destroy_workqueue(info->event_wq);
	destroy_workqueue(info->fwu_workqueue);
	
	//mx_remove_link(LINK_KOBJ_NAME);
	/*I2C cmd*/
	sysfs_remove_group(&info->i2c_cmd_dev->kobj,
			&i2c_cmd_attr_group);
	device_destroy(i2c_cmd_class, FTS_ID0);

	/* unregister the device */
	input_unregister_device(info->input_dev);
	//input_unregister_notifier_client(&info->cover_notifier);

	/* free all */
	kfree(info);

	return 0;
}


static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
#if 0

	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i;


	if(!st_tp || info->sensor_sleep)
		return 0;
	#ifdef DEBUG
	tp_log("%s: suspend begin!\n", __func__);
	#endif
	/* Release all buttons */
	info->buttons = 0;

	/* Release all slots */
	for (i = 0; i < TOUCH_ID_MAX; i++)
		if (__test_and_clear_bit(i, &info->touch_id)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
			(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
		}
	input_sync(info->input_dev);

	/* No need ot check for error code */
	cancel_work_sync(&info->work);

	if(info->gesture_enable) {
		fts_gesture_control(info);
		fts_command(info, ENTER_GESTURE_MODE);
		enable_irq_wake(info->client->irq);
	} else {
		fts_interrupt_disable(info);
		/* Read out device mode, used when resuming device */
		//fts_get_mode(info);

		/* suspend the device and flush the event FIFO */
		#ifdef DEBUG
		tp_log("%s: suspend send command sleep \n", __func__);
		#endif
		fts_command(info, SLEEPOFF);
		fts_command(info, FLUSHBUFFER);
	}
	info->sensor_sleep = true;
#if 0
	if ((info->power) && (info->power(FTS_POWER_OFF)))
		dev_warn(info->dev, "Cannot power-off device\n");
#endif
	/* ignore errors */
#endif
	return 0;
}


static int fts_resume(struct i2c_client *client)
{
#if 0
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int error;
#if 0
	/* Power-on the device */
	if ((info->power) && (info->power(FTS_POWER_ON))) {
		dev_err(&client->dev, "Cannot power-on device\n");
		return -ENODEV;
	}
#endif

	if(!st_tp || !info->sensor_sleep)
		return 0;
	#ifdef DEBUG
	tp_log("%s: start\n", __func__);
	#endif
	error = fts_systemreset(info);
	if (error) {
		dev_err(info->dev,
			"Cannot reset the device\n");
		return -ENODEV;
	}
	msleep(100);
	
	if(info->gesture_enable){
		disable_irq_wake(info->client->irq);
	}
	else {
		fts_interrupt_enable(info);
	/* enable interrupts */
	}

	/* wake-up the device */
	init_completion(&info->cmd_done);
	fts_command(info, SENSEON);
	WAIT_WITH_TIMEOUT(info, HZ, SENSEON);

	/* enable sense */
	fts_command(info, SENSEON);

	/* put back the device in the original mode (see fts_suspend()) */
	switch (info->mode) {
	case MODE_PROXIMITY:
		fts_command(info, PROXIMITY_ON);
		break;

	case MODE_HOVER:
		fts_command(info, HOVER_ON);
		break;

	case MODE_GESTURE:
		fts_command(info, GESTURE_ON);
		break;

	case MODE_HOVER_N_PROXIMITY:
		fts_command(info, HOVER_ON);
		fts_command(info, PROXIMITY_ON);
		break;

	case MODE_GESTURE_N_PROXIMITY:
		fts_command(info, GESTURE_ON);
		fts_command(info, PROXIMITY_ON);
		break;

	case MODE_GESTURE_N_PROXIMITY_N_HOVER:
		fts_command(info, HOVER_ON);
		fts_command(info, GESTURE_ON);
		fts_command(info, PROXIMITY_ON);
		break;

	default:
		dev_warn(info->dev, "Invalid device mode - 0x%02x\n",
				info->mode);
		break;
	}

	if(info->glove_bit) {
		error = fts_command(info, GLOVE_ON);
		if(error) {
			tp_log("%s: glove mode open failed!\n", __func__);
		}
	}
	info->sensor_sleep = false;
#endif
	return 0;
}

static struct of_device_id fts_of_match_table[] = {
	{
		.compatible = "st,fts",
	},
	{},
};
static const struct i2c_device_id fts_device_id[] = {
	{FTS_TS_DRV_NAME, 0},
	{}
};

static struct i2c_driver fts_i2c_driver = {
	.driver   = {
		.name = FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
	},
	.probe    = fts_probe,
	.remove   = fts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  = fts_suspend,
	.resume   = fts_resume,
#endif
	.id_table = fts_device_id,
};

static int __init fts_driver_init(void)
{
	return i2c_add_driver(&fts_i2c_driver);
}


static void __exit fts_driver_exit(void)
{
	i2c_del_driver(&fts_i2c_driver);
}


MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("JHJANG");
MODULE_AUTHOR("Giuseppe Di Giore <giuseppe.di-giore@st.com");
MODULE_LICENSE("GPL");

late_initcall(fts_driver_init);
module_exit(fts_driver_exit);
