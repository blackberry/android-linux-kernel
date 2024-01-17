/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*******************************************************************************
*
* File Name: focaltech_core.c
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-01-20
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include "focaltech_comm.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*File Version*/
#define FOCALTECH_CORE_INFO  "File Version of  focaltech_core.c:  V1.0.0 2016-03-09"

/*i2c client name for board file*/
#define FOCALTECH_TS_NAME_FOR_BOARD   "focaltech_ts"

/*i2c client name for DTS file*/
#define FOCALTECH_TS_NAME_FOR_DTS   "focaltech,focaltech_ts"


// I2C communication style
#define FTS_IIC_COMM_MODE      0//0: universal, 1: mda
enum enum_iic_comm_mode
{
	iic_comm_universal_mode = 0,
	iic_comm_dma_mode = 1,
};

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct i2c_client *fts_i2c_client = NULL;
struct fts_ts_platform_data *fts_platform_data = NULL;
#ifdef CONFIG_BBRY	//TCTNB,LCY,ADD,Task-3313186,20161104
extern int i2c_check_status_create(char *name,int value);
#endif
extern int get_board_version(void);
#define BOARD_PIO04 4
#define BOARD_PIO05 5
#define BAORD_OTHERS 0Xff

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_public_i2c_read(struct i2c_client *client, unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
static int fts_public_i2c_write(struct i2c_client *client, unsigned char *writebuf, int writelen);


#if defined(CONFIG_BBRY) || defined(BBRY_MINISW)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/*******************************************************************************
* functions body
*******************************************************************************/
/*******************************************************************************
* Name: fts_public_i2c_read
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int fts_public_i2c_read(struct i2c_client *client, unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
	int ret = 0;

	if(NULL == client)
	{
		FTS_COMMON_DBG("[FTS] %s. i2c client is empty!",  __FUNCTION__);
		return -1;
	}

	if(FTS_IIC_COMM_MODE == iic_comm_universal_mode)
	{
		ret = fts_i2c_read_universal(client,  writebuf,  writelen,  readbuf,  readlen);
	}
	else if(FTS_IIC_COMM_MODE == iic_comm_dma_mode)
	{
		ret = fts_i2c_read_dma(client,  writebuf,  writelen,  readbuf,  readlen);
	}
	else
	{
		FTS_COMMON_DBG("[FTS] the selected i2c communication mode is not supported!");
		ret = -1;
	}

	return ret;

}

/*******************************************************************************
* Name: fts_public_i2c_write
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int fts_public_i2c_write(struct i2c_client *client, unsigned char *writebuf, int writelen)
{
	int ret = 0;

	if(NULL == client)
	{
		FTS_COMMON_DBG("[FTS] %s. i2c client is empty!",  __FUNCTION__);
		return -1;
	}

	if(FTS_IIC_COMM_MODE == iic_comm_universal_mode)
	{
		ret = fts_i2c_write_universal(client,  writebuf,  writelen);
	}
	else if(FTS_IIC_COMM_MODE == iic_comm_dma_mode)
	{
		ret = fts_i2c_write_dma(client,  writebuf,  writelen);
	}
	else
	{
		FTS_COMMON_DBG("[FTS] the selected i2c communication mode is not supported!");
		ret = -1;
	}

	return ret;

}

/*******************************************************************************
* Name: fts_i2c_read
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_i2c_read(/*struct i2c_client *client,*/unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
	/*record the time of i2c read/write*/
	if(fts_auto_reset_record_time() < 0)
		return -1;

	return fts_public_i2c_read(fts_i2c_client,  writebuf,  writelen,  readbuf,  readlen);
}

/*******************************************************************************
* Name: fts_i2c_write
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_i2c_write(/*struct i2c_client *client, */unsigned char *writebuf, int writelen)
{
	/*record the time of i2c read/write*/
	if(fts_auto_reset_record_time() < 0)
		return -1;

	return fts_public_i2c_write(fts_i2c_client,  writebuf,  writelen);
}

/*******************************************************************************
* Name: fts_write_reg
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_write_reg(/*struct i2c_client *client,*/ unsigned char addr, const unsigned char val)
{
	unsigned char buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return fts_i2c_write( buf, sizeof(buf));
}

/*******************************************************************************
* Name: fts_read_reg
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_read_reg(/*struct i2c_client *client, */unsigned char addr, unsigned char *val)
{
	return fts_i2c_read(&addr, 1, val, 1);
}

/*******************************************************************************
* Name: fts_i2c_read_for_esd
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_i2c_read_for_esd(/*struct i2c_client *client,*/unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{

	return fts_public_i2c_read(fts_i2c_client,  writebuf,  writelen,  readbuf,  readlen);
}

/*******************************************************************************
* Name: fts_i2c_write_for_esd
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_i2c_write_for_esd(/*struct i2c_client *client, */unsigned char *writebuf, int writelen)
{

	return fts_public_i2c_write(fts_i2c_client,  writebuf,  writelen);
}

/*******************************************************************************
* Name: fts_write_reg_for_esd
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_write_reg_for_esd(/*struct i2c_client *client,*/ unsigned char addr, const unsigned char val)
{
	unsigned char buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return fts_i2c_write_for_esd( buf, sizeof(buf));
}

/*******************************************************************************
* Name: fts_read_reg_for_esd
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_read_reg_for_esd(/*struct i2c_client *client, */unsigned char addr, unsigned char *val)
{
	return fts_i2c_read_for_esd(&addr, 1, val, 1);
}


/* MODIFIED-BEGIN by Haojun Chen, 2016-12-08,BUG-3684946*/
/*******************************************************************************
* Name: fts_irq_enable
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_irq_enable(bool enable)
{
	if (enable) {
		if (fts_platform_data->fts_irq_enabled== 0) {
			enable_irq(fts_i2c_client->irq);
			fts_platform_data->fts_irq_enabled = 1;
		} else {
			//ERROR_COMMON_FTS("Unbalanced enable for IRQ enable");
		}
	} else {
		if (fts_platform_data->fts_irq_enabled == 1) {
			disable_irq(fts_i2c_client->irq);
			fts_platform_data->fts_irq_enabled = 0;
		} else {
			//ERROR_COMMON_FTS("Unbalanced enable for IRQ disable");
		}
	}
}
/* MODIFIED-END by Haojun Chen,BUG-3684946*/


#if defined(CONFIG_BBRY) || defined(BBRY_MINISW)
/*******************************************************************************
* Name: fb_notifier_callback
* Brief: Called from fb driver when there is an event (blank/unblank)
* Input:
* Output: None
* Return: None
*******************************************************************************/

extern bool bb_panel_is_off; // MODIFIED by Haojun Chen, 2016-11-15,BUG-3472476
extern void fts_release_all_touch(void); // MODIFIED by Haojun Chen, 2016-12-06,BUG-3668081
bool fts_ignore_irq = false;

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (fts_platform_data == NULL || fts_i2c_client == NULL) {
		ERROR_COMMON_FTS("initilization failed");
	}

	if (evdata && evdata->data) {
		blank = evdata->data;
		if (event == FB_EARLY_EVENT_BLANK && *blank == FB_BLANK_UNBLANK) {
			if (fts_platform_data->state == FTS_STATE_OFF) {
				INFO_COMMON_FTS("off --> on");
				//fts_report_suspend();
				fts_ignore_irq = true;
				enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_1p8, "disp_1p8", true);
				gpio_direction_output(fts_platform_data->disp_1p8_en_gpio, 1);
				usleep(1000);
				if (get_board_version() == BOARD_PIO05) {
					gpio_direction_output(fts_platform_data->disp_p5v_en_gpio,1);
					usleep(5000);
					gpio_direction_output(fts_platform_data->disp_n5v_en_gpio,1);
				} else {
					enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", true);
					usleep(5000);
					enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_n5v, "disp_n5v", true);
				}
				usleep(1000);
				gpio_direction_output(fts_platform_data->reset_gpio, 1);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
				usleep(1000);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 0);
				usleep(1000);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
				usleep(15000);
				//fts_report_resume();
				fts_ignore_irq = false;
				/* MODIFIED-BEGIN by Haojun Chen, 2016-12-26,BUG-3841357*/
				fts_irq_enable(true);
			} else if (fts_platform_data->state == FTS_STATE_GESTURE) {
				INFO_COMMON_FTS("gesture --> on");
				disable_irq_wake(fts_i2c_client->irq);
				/* MODIFIED-END by Haojun Chen,BUG-3841357*/
				fts_report_suspend();
				usleep(1000);
				gpio_direction_output(fts_platform_data->reset_gpio, 0);
				usleep(1000);
				gpio_direction_output(fts_platform_data->reset_gpio, 1);
				fts_report_resume();
			} else if (fts_platform_data->state == FTS_STATE_IGNORE_TOUCH) {
				INFO_COMMON_FTS("ignore touch --> on");
				//fts_report_resume();
				fts_irq_enable(true);
				fts_ignore_irq = false;
			} else {
				ERROR_COMMON_FTS("unexpected state %d --> on", fts_platform_data->state);
			}
			fts_platform_data->state = FTS_STATE_ON;
		} else if (event == FB_EVENT_BLANK && *blank == FB_BLANK_POWERDOWN) {
			if (fts_platform_data->state != FTS_STATE_ON && fts_platform_data->state != FTS_STATE_IGNORE_TOUCH) {
				ERROR_COMMON_FTS("unexpected state %d --> gesture/off", fts_platform_data->state);
			}
			/* MODIFIED-BEGIN by Haojun Chen, 2016-11-15,BUG-3472476*/
			if (!bb_panel_is_off) {
				printk("[mdss] bb_panel_is_off = %d", bb_panel_is_off);
				return 0;
			}
			/* MODIFIED-BEGIN by Haojun Chen, 2016-12-26,BUG-3841357*/
			fts_release_all_touch();
			/* MODIFIED-END by Haojun Chen,BUG-3472476*/
			if (fts_platform_data->tap_to_wake_enabled) {
				/* MODIFIED-BEGIN by Haojun Chen, 2016-11-24,BUG-3418596*/
				INFO_COMMON_FTS("on --> gesture");
				fts_report_suspend();
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
				usleep(1000);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 0);
				usleep(1000);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
				usleep(120000);
				fts_write_reg(0xD1, 0x10);  // enable double-tap
				fts_write_reg(0xD0, 0x01);  // enable gesture mode
				enable_irq_wake(fts_i2c_client->irq);
				/* MODIFIED-END by Haojun Chen,BUG-3841357*/
				fts_platform_data->state = FTS_STATE_GESTURE;
			} else {
				INFO_COMMON_FTS("on --> off");
				if (fts_platform_data->state == FTS_STATE_IGNORE_TOUCH || fts_platform_data->state == FTS_STATE_OFF) {
					INFO_COMMON_FTS(" IGNORE_TOUCH or OFF %d --> off", fts_platform_data->state);
				} else {
					//fts_irq_enable(false);
				}
				fts_ignore_irq = true;
				//fts_report_suspend();
				usleep(70000);
				gpio_direction_output(fts_platform_data->reset_gpio, 0);
				gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 0);
				if (get_board_version() == BOARD_PIO05) {
					gpio_direction_output(fts_platform_data->disp_n5v_en_gpio,0);
					usleep(7000);
					gpio_direction_output(fts_platform_data->disp_p5v_en_gpio,0);
				} else {
					enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_n5v, "disp_n5v", false);
					usleep(7000);
					enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", false);
				}
				usleep(5000);/*Mod by LCY,raise sleep time,defect-3671139,2016/12/06*/
				enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_1p8, "disp_1p8", false);
				gpio_direction_output(fts_platform_data->disp_1p8_en_gpio, 0);
				/* MODIFIED-END by Haojun Chen,BUG-3418596*/
				fts_platform_data->state = FTS_STATE_OFF;
				fts_ignore_irq = false;
			}
		} else if (event == FB_EARLY_EVENT_BLANK && *blank == FB_BLANK_NORMAL && !fts_platform_data->tap_to_wake_enabled) {
			if (fts_platform_data->state != FTS_STATE_OFF) {
				INFO_COMMON_FTS("Display is not off, ignore request for display on, ignore touches");
				return 0;
			}
			INFO_COMMON_FTS("off --> ignore touch");
			usleep(1000);
			//fts_report_suspend();
			fts_ignore_irq = true;
			enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_1p8, "disp_1p8", true);
			gpio_direction_output(fts_platform_data->disp_1p8_en_gpio, 1);
			usleep(1000);
			if (get_board_version() == BOARD_PIO05) {
				gpio_direction_output(fts_platform_data->disp_p5v_en_gpio,1);
				usleep(5000);
				gpio_direction_output(fts_platform_data->disp_n5v_en_gpio,1);
			} else {
				enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", true);
				usleep(5000);
				enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_n5v, "disp_n5v", true);
			}
			usleep(1000);
			gpio_direction_output(fts_platform_data->reset_gpio, 1);
			gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
			usleep(1000);
			gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 0);
			usleep(1000);
			gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 1);
			usleep(15000);
			//fts_report_resume();
			fts_platform_data->state = FTS_STATE_IGNORE_TOUCH;
		}
	}

	return 0;
}
#endif

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	/*TCTNB,LCY,Add begin,Task-3437054,20161110*/
	int ret = 0;
	u8 chip_id;
	/*TCTNB,LCY,Add end,Task-3437054,201611110*/

	FTS_COMMON_DBG();//default print: current function name and line number

	//check client->adapter's I2C function.
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_COMMON_DBG("i2c_check_functionality error. I2C not supported.");
		return -ENODEV;
	}

	/*set i2c client as global variable*/
	fts_i2c_client = client;

	/*init I2C communication */
	err = fts_i2c_communication_init();
	if(err < 0)
	{
		return err;
	}

	//	init platform data
	err = fts_platform_data_init(client);
	if(err < 0)
	{
		goto i2c_communication_exit;
	}

	// init report point module
	err = fts_report_init(client, fts_platform_data);
	if(err < 0)
	{
		goto platform_data_exit;
	}

	fts_platform_data->fts_irq_enabled = 1; // MODIFIED by Haojun Chen, 2016-12-08,BUG-3684946


	//	init gesture module which is used by report module.

	/*TCTNB,LCY,Add begin,Task-3437054,20161110*/
	ret = fts_read_reg(FTS_REG_CHIP_ID,&chip_id);
	if (ret<0)
	{
		i2c_check_status_create("touch_panel",0);
	}else{
		i2c_check_status_create("touch_panel",1);
	}
	/*TCTNB,LCY,Add end,Task-3437054,201611110*/
	if (get_board_version() == BOARD_PIO05) {
		FTS_COMMON_DBG("device is pio5, disable lab&ibb power supply");
		enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", false);
		enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", false);
	}

	// init flash TP IC firmware module
	err = fts_flash_init(client);
	if(err < 0)
	{
		goto report_exit;
	}

	// init APK communication module
	err = fts_apk_node_init();
	if(err < 0)
	{
		goto flash_exit;
	}

	//	init sysfs file node module
	err = fts_sysfs_init(fts_i2c_client);
	if(err < 0)
	{
		goto apk_node_exit;
	}

	//	init promximity sensor module
	err = fts_touch_proximity_init();
	if(err < 0)
	{
		goto sysfs_exit;
	}

	// init ESD protection module
	err = fts_auto_reset_init();
	if(err < 0)
	{
		goto touch_proximity_exit;
	}

	// init test module
	err = fts_test_init(fts_i2c_client);
	if(err < 0)
	{
		goto auto_reset_exit;
	}


#if defined(CONFIG_BBRY) || defined(BBRY_MINISW)
	fts_platform_data->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&fts_platform_data->fb_notif);

	if (err)
		ERROR_COMMON_FTS("Unable to register fb_notifier: %d", err);
#endif
	return 0;

	auto_reset_exit:
		fts_auto_reset_exit();
	touch_proximity_exit:
		fts_touch_proximity_exit();
	sysfs_exit:
		fts_sysfs_exit(fts_i2c_client);
	apk_node_exit:
		fts_apk_node_exit();
	flash_exit:
		fts_flash_exit();
	report_exit:
		fts_report_exit(fts_i2c_client, fts_platform_data);
	platform_data_exit:
		fts_platform_data_exit();
	i2c_communication_exit:
		fts_i2c_communication_exit();

	return err;

}

/*******************************************************************************
* Name: fts_remove
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int fts_remove(struct i2c_client *client)
{
	FTS_COMMON_DBG("");//default print: current function name and line number

	//	exit test module
	fts_test_exit(fts_i2c_client);

	// exit ESD protection module
	fts_auto_reset_exit();

	// exit proximity sensor module
	fts_touch_proximity_exit();

	//	exit sysfs file node module
	fts_sysfs_exit(fts_i2c_client);

	//	exit APK communication module
	fts_apk_node_exit();

	// exit flash firmware module
	fts_flash_exit();

	// exit report point module
	fts_report_exit(fts_i2c_client, fts_platform_data);

	//	exit platform data module which must exit later than other modules, because other modules maybe still still use platform data.
	fts_platform_data_exit();

	//	exit I2C communication module
	fts_i2c_communication_exit();

	// exit gesture module

	return 0;
}


/*******************************************************************************
* Name: fts_suspend
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
#if !defined(CONFIG_BBRY)
	FTS_COMMON_DBG("");//default print: current function name and line number
	/*enter suspend for report*/
	fts_report_suspend();

	fts_auto_reset_suspend();
#endif
	return 0;
}

/*******************************************************************************
* Name: fts_resume
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int fts_resume(struct i2c_client *client)
{
#if !defined(CONFIG_BBRY)
	FTS_COMMON_DBG("");//default print: current function name and line number
	/*enter resume for report*/
	fts_report_resume();

	fts_auto_reset_resume();
#endif
	return 0;
}

#if 0
static const struct i2c_device_id fts_id_table[] = {
	{ FOCALTECH_TS_NAME_FOR_BOARD, 0 },
	{ }//The last element is empty, which is used to mark the end of the table
};
MODULE_DEVICE_TABLE(i2c, fts_id_table);

static struct i2c_driver fts_driver = {
	.probe	=fts_probe,
	.remove	=fts_remove,
	.id_table	=fts_id_table,
	.driver	={
		.name = FOCALTECH_TS_NAME_FOR_BOARD,
		.owner = THIS_MODULE,
		},
	.suspend = fts_suspend,
	.resume	= fts_resume,
};
#else

/*Add begin by LCY,adjust LCD/TP power off sequence,defect-3671139,2016/12/06*/
static void fts_shutdown(struct i2c_client *client)
{
	FTS_COMMON_DBG("");//default print: current function name and line number
	/*enter resume for report*/

	gpio_direction_output(fts_platform_data->reset_gpio, 0);
	gpio_direction_output(fts_platform_data->disp_rst_n_gpio, 0);
	if (get_board_version() == BOARD_PIO05) {
		gpio_direction_output(fts_platform_data->disp_n5v_en_gpio,0);
		usleep(7000);
		gpio_direction_output(fts_platform_data->disp_p5v_en_gpio,0);
	} else {
		enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_n5v, "disp_n5v", false);
		usleep(7000);
		enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_p5v, "disp_p5v", false);
	}
	usleep(5000);
	enable_regulator(&fts_i2c_client->dev, fts_platform_data->disp_1p8, "disp_1p8", false);
	gpio_direction_output(fts_platform_data->disp_1p8_en_gpio, 0);
}
/*Add end by LCY,adjust LCD/TP power off sequence,defect-3671139,2016/12/06*/

static const struct i2c_device_id fts_id_table[] = {
	{ FOCALTECH_TS_NAME_FOR_BOARD, 0 },
	{ },//The last element is empty, which is used to mark the end of the table
};
MODULE_DEVICE_TABLE(i2c, fts_id_table);

static const struct of_device_id focaltech_of_match[] = {
		{.compatible = FOCALTECH_TS_NAME_FOR_DTS,},
		{},//The last element is empty, which is used to mark the end of the table
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);

static struct i2c_driver fts_driver = {
	.probe	=fts_probe,
	.remove	=fts_remove,
	.id_table	=fts_id_table,
	.driver	={
		.name = FOCALTECH_TS_NAME_FOR_BOARD,
		.owner = THIS_MODULE,
		.of_match_table = focaltech_of_match,
		},
	.suspend = fts_suspend,
	.resume	= fts_resume,
	.shutdown = fts_shutdown, /*Add by LCY,adjust LCD/TP power off sequence,defect-3671139,2016/12/06*/
};
#endif

static int __init fts_touch_driver_init(void)
{
	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_CORE_INFO);	//show version
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);
	return i2c_add_driver(&fts_driver);
}

static void __exit fts_touch_driver_exit(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);
	i2c_del_driver(&fts_driver);
}


MODULE_AUTHOR("Software Department, FocalTech");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");

module_init(fts_touch_driver_init);
module_exit(fts_touch_driver_exit);
