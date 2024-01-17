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
* File Name: Focaltech_sysfs.c
*
* Author:    Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-03-16
*   
* Modify: 
*
* Abstract:  create DEVICE_ATTR with sysfs interface that can be used by adb shell to look and operate.
* tp	sysfs path 	such as:	/sys/bus/i2c/devices/1-0038
*
*	
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_comm.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_SYSFS_INFO  "File Version of  focaltech_sysfs.c:  V1.0.0 2016-03-16"



/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/

/************************************************************************
* Name: fts_get_fw_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_get_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);  jacob use globle fts_wq_data data
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg( FTS_REG_FW_VER, &fwver) < 0)
	{
		FTS_COMMON_DBG( "Failed to get fw version!");
		return -1;
	}
	
	
	if (fwver == 255)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,"get tp fw version fail!");
		FTS_COMMON_DBG( "Failed to get fw version! Error value.");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "FW Version: %02X\n",  fwver);
		FTS_COMMON_DBG( "FW Version = 0x%02x.",  fwver);
	}
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_get_fw_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_get_fw_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}


/************************************************************************
* Name: fts_hw_reset_show
* Brief:  set hardware reset
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_hw_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	int iret = -1;
	FTS_COMMON_DBG("[FTS] Hardware Reset ! ");
	iret = fts_hardware_reset(0);
	msleep(20);
	iret = fts_hardware_reset(1);
	msleep(200);	
	
	if (iret < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE,"Failed to reset TP!\n");
		FTS_COMMON_DBG( "Failed to reset TP!");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "Reset TP succeeded.\n");
		FTS_COMMON_DBG( "Reset TP succeeded.");
	}
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_get_fw_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_hw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_read_reg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_read_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_read_reg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_read_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

/*int flag=0, x1=0, y1=0, x2=0, y2=0; 
sscanf(buf, "%d %d %d %d %d", &flag, &x1, &y1, &x2, &y2); 

FTS_COMMON_DBG( "%d %d %d %d %d\n\n", flag, x1, y1, x2, y2);

return count;*/


#if 1
	ssize_t num_read_chars = 0;
	int retval;
	
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		FTS_COMMON_DBG( "please input 2 character,  e.g: echo a6 > fts_read_reg");
		goto error_return;

	}

	memcpy(valbuf, buf, num_read_chars);
	retval = sstrtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		FTS_COMMON_DBG( "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"",  __FUNCTION__, buf);
		goto error_return;
	}
	
	/*read register*/
	regaddr = wmreg;
	FTS_COMMON_DBG("[focal]Read Reg : (Addr = 0x%02x)",  regaddr);
	if (fts_read_reg( regaddr, &regvalue) < 0)
		FTS_COMMON_DBG("[Focal] Read Reg : Could not read the register(0x%02x)",   regaddr);
	else
		FTS_COMMON_DBG("[Focal] Read Reg : Addr = 0x%02x, Value = 0x%02x",  regaddr, regvalue);


	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
#else
	ssize_t num_read_chars = 0;
	int retval;
	/*u32 wmreg=0;*/
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			FTS_COMMON_DBG( "please input 2 or 4 character");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		FTS_COMMON_DBG( "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"",  __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		FTS_COMMON_DBG("[focal][test](0x%02x)",  regaddr);
		if (fts_read_reg( regaddr, &regvalue) < 0)
			FTS_COMMON_DBG("[Focal] %s : Could not read the register(0x%02x)",  __func__, regaddr);
		else
			FTS_COMMON_DBG("[Focal] %s : the register(0x%02x) is 0x%02x",  __func__, regaddr, regvalue);
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg( regaddr, regvalue)<0)
			FTS_COMMON_DBG( "[Focal] %s : Could not write the register(0x%02x)",  __func__, regaddr);
		else
			FTS_COMMON_DBG( "[Focal] %s : Write 0x%02x into register(0x%02x) successful",  __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
#endif

}
/************************************************************************
* Name: fts_write_reg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_write_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_write_reg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_write_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	ssize_t num_read_chars = 0;
	int retval;
	/*u32 wmreg=0;*/
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;

	FTS_COMMON_DBG( "input buff = %s, size = %lu",  buf, count);
	
	if (num_read_chars != 4) 
	{
		FTS_COMMON_DBG( "please input 4 character, e.g: echo a601 > fts_write_reg");
		goto error_return;
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = sstrtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		FTS_COMMON_DBG( "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"",  __FUNCTION__, buf);
		goto error_return;
	}

	regaddr = wmreg>>8;
	regvalue = wmreg;
	if (fts_write_reg( regaddr, regvalue)<0)
		FTS_COMMON_DBG( "[Focal] Write Reg : Could not write the register(0x%02x)",  regaddr);
	else
		FTS_COMMON_DBG( "[Focal] Write Reg: Write 0x%02x into register(0x%02x) successfully",  regvalue, regaddr);

	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_upgrade_i_file_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_upgrade_i_file_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_upgrade_i_file_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_upgrade_i_file_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct fts_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	int i_ret;
	/* MODIFIED-BEGIN by Haojun Chen, 2016-12-08,BUG-3684946*/
	//struct i2c_client *client = fts_i2c_client;
	//data = (struct fts_ts_data *) i2c_get_clientdata();

	mutex_lock(&fts_input_dev->mutex);

	//disable_irq(client->irq);
	fts_irq_enable(false);
	/* MODIFIED-END by Haojun Chen,BUG-3684946*/

	i_ret = fts_flash_upgrade_with_i_file();
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_flash_get_i_file_version();
		FTS_COMMON_DBG( "%s [FTS] upgrade to new version 0x%x",  __func__, uc_host_fm_ver);
	}
	else
	{
		FTS_COMMON_DBG( "%s ERROR:[FTS] upgrade failed ret=%d.",  __func__, i_ret);
	}


	//fts_ctpm_auto_upgrade();
	/* MODIFIED-BEGIN by Haojun Chen, 2016-12-08,BUG-3684946*/
	//enable_irq(client->irq);
	fts_irq_enable(true);
	/* MODIFIED-END by Haojun Chen,BUG-3684946*/
	mutex_unlock(&fts_input_dev->mutex);

	return count;
}
/************************************************************************
* Name: fts_upgrade_bin_file_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_upgrade_bin_file_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_upgrade_bin_file_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_upgrade_bin_file_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	/* MODIFIED-BEGIN by Haojun Chen, 2016-12-08,BUG-3684946*/
	//struct i2c_client *client = fts_i2c_client;
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);

	//disable_irq(client->irq);
	fts_irq_enable(false);
	fts_flash_upgrade_with_bin_file( fwname);
	//enable_irq(client->irq);
	fts_irq_enable(true);
	/* MODIFIED-END by Haojun Chen,BUG-3684946*/

	mutex_unlock(&fts_input_dev->mutex);

	return count;
}

/************************************************************************
* Name: fts_wakeup_gesture_show
***********************************************************************/
static ssize_t fts_wakeup_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool enabled = true;
	enabled = fts_wakeup_gesture_read();
	return snprintf(buf, PAGE_SIZE, "fts_platform_data->tap_to_wake_enabled=%u\n", enabled);
}

/************************************************************************
* Name: fts_wakeup_gesture_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_wakeup_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool enable = true;
	unsigned int input;
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input) {
		enable = true;
	} else {
		enable = false;
	}
	fts_wakeup_gesture_set(enable);

	FTS_COMMON_DBG( "fts_wakeup_gesture_store:tap_to_wake_enabled=%u\n", enable);

	return count;
}
#if 0
/************************************************************************
* Name: fts_fts_get_project_code_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_get_project_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{/*
	//struct fts_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = fts_i2c_client;
	//data = (struct fts_ts_data *) i2c_get_clientdata();

	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);



	
	//fts_ctpm_auto_upgrade();
	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);
	*/
	return -EPERM;
}
/************************************************************************
* Name: fts_fts_get_project_code_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_get_project_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
#endif

/****************************************/
/* sysfs */
/* get the fw version
*   example:cat fts_get_fw_version
*/
static DEVICE_ATTR(fts_get_fw_version, S_IRUGO|S_IWUSR, fts_get_fw_version_show, fts_get_fw_version_store);
#if 0
/* get project code of fw
*   example:cat fts_get_project_code
*/
static DEVICE_ATTR(fts_get_project_code, S_IRUGO|S_IWUSR, fts_get_project_code_show, fts_get_project_code_store);
#endif

/* upgrade from *.i
*   example: echo 1.i > fts_upgrade_i_file
*/
static DEVICE_ATTR(fts_upgrade_i_file, S_IRUGO|S_IWUSR, fts_upgrade_i_file_show, fts_upgrade_i_file_store);

/*  upgrade from app.bin
*    example:echo "*_app.bin" > fts_upgrade_bin_file
*/
static DEVICE_ATTR(fts_upgrade_bin_file, S_IRUGO|S_IWUSR, fts_upgrade_bin_file_show, fts_upgrade_bin_file_store);

/* read and write register
*   read example: echo 88 > fts_rw_reg ---read register 0x88
*
*   note:the number of input must be 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(fts_read_reg, S_IRUGO|S_IWUSR, fts_read_reg_show, fts_read_reg_store);

/* read and write register
*   write example:echo 8807 > fts_rw_reg ---write 0x07 into register 0x88
*
*   note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(fts_write_reg, S_IRUGO|S_IWUSR, fts_write_reg_show, fts_write_reg_store);
/****************************************/
/* sysfs */
/* set hardware reset
*   example:cat fts_hw_reset
*/
static DEVICE_ATTR(fts_hw_reset, S_IRUGO|S_IWUSR, fts_hw_reset_show, fts_hw_reset_store);
/****************************************/
/* sysfs */
/* Wakeup Gesture
*/
static DEVICE_ATTR(fts_wakeup_gesture, S_IRUGO|S_IRGRP|S_IWUSR|S_IWGRP, fts_wakeup_gesture_show, fts_wakeup_gesture_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_fts_get_fw_version.attr,
	//&dev_attr_fts_get_project_code.attr,		
	&dev_attr_fts_upgrade_i_file.attr,
	&dev_attr_fts_upgrade_bin_file.attr,
	&dev_attr_fts_read_reg.attr,
	&dev_attr_fts_write_reg.attr,	
	&dev_attr_fts_hw_reset.attr,	
	&dev_attr_fts_wakeup_gesture.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_sysfs_init
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_sysfs_init(struct i2c_client *client)
{
	int err=0;
	
	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_SYSFS_INFO);	//show version
	FTS_COMMON_DBG("");//default print: current function name and line number
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		FTS_COMMON_DBG( "[focal] %s() - ERROR: sysfs_create_group() failed.",  __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		FTS_COMMON_DBG("[focal] %s() - sysfs_create_group() succeeded.", __func__);
	}
	//fts_protocol_windows_to_android(client);
	return err;
}
/************************************************************************
* Name: fts_sysfs_exit
* Brief:  remove sysfs
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_sysfs_exit(struct i2c_client *client)
{
	FTS_COMMON_DBG("");//default print: current function name and line number
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
