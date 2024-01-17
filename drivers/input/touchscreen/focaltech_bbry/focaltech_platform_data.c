/*
 *
 * FocalTech TouchScreen driver.
 *
 *Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
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
* File Name: focaltech_platform_data.c
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-03-04
*
* Modified:
*
*  Abstract:
*
*	Deal with data of platform
*
* Reference:
*
*******************************************************************************/
/* temporarily define this here until we have this porperly defined for the entire kernel */
#define CONFIG_BBRY 1

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

#include "focaltech_comm.h"
#include "focaltech_core.h"
#ifdef CONFIG_BBRY
#include <linux/regulator/consumer.h>
#endif
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*File Version*/
#define FOCALTECH_PLATFORM_DATA_INFO  "File Version of  focaltech_platform_data.c:  V1.0.0 2016-03-09"

/*Platform Type*/
#define PLATFORM_DATA_BOARD_TYPE   		0  /* 0:board file */
#define PLATFORM_DATA_DTS_TYPE     		1  /* 1:dts file */
#define PLATFORM_DATA_USERDEFINED_TYPE   	2  /* 2:user-defined */
#define PLATFORM_DATA_TYPE   			PLATFORM_DATA_DTS_TYPE

#if (PLATFORM_DATA_TYPE == PLATFORM_DATA_USERDEFINED_TYPE)
/*Platform Data*/
#define PLATFORM_DATA_IRQ_GPIO      EXYNOS4_GPX1(6)//
#define PLATFORM_DATA_IRQ_GPIO_FLAG    IRQF_TRIGGER_FALLING //
#define PLATFORM_DATA_RESET_GPIO      EXYNOS4_GPX1(7)//
#define PLATFORM_DATA_X_RESOLUTION    1080//
#define PLATFORM_DATA_Y_RESOLUTION    1620//
#define PLATFORM_DATA_PRESSURE_VALUE  255 //pressure_max
#endif

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum enum_Platform_Data_Type
{
	platform_data_from_board_file = 0,
	platform_data_from_dts_file = 1,
	platform_data_from_used_define = 2,
};

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
static int fts_platform_data_parse_dts_file(struct i2c_client *client, struct fts_ts_platform_data *pdata);
static int fts_platform_data_parse_board_file(struct i2c_client *client);
static int fts_platform_data_used_define(struct i2c_client *client, struct fts_ts_platform_data *pdata);

static int fts_platform_data_gpio_configure(struct i2c_client *client, struct fts_ts_platform_data *pdata);
static int fts_platform_data_gpio_release(struct fts_ts_platform_data *pdata);

/*******************************************************************************
* functions body
*******************************************************************************/
int fts_platform_data_init(struct i2c_client *client)
{
	int err = 0;

	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_PLATFORM_DATA_INFO);	//show version
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	/* malloc memory */
	fts_platform_data = kmalloc(sizeof(struct fts_ts_platform_data), GFP_ATOMIC);
	if (!fts_platform_data) {
		dev_err(&client->dev, "Failed to allocate memory");
		return -ENOMEM;
	}

	/*	init platform data */
#ifdef CONFIG_BBRY
	memset(fts_platform_data, 0, sizeof(struct fts_ts_platform_data));

	//because of LK splash, assume that we are initing in an ON state
	fts_platform_data->state = FTS_STATE_ON;
	fts_platform_data->tap_to_wake_enabled = false;
#else
	memset(fts_platform_data, sizeof(fts_platform_data), 0);
#endif

	/*	extract platform data from board file or DTS file */
	if(PLATFORM_DATA_TYPE == platform_data_from_board_file)
	{
		err = fts_platform_data_parse_board_file(client);
		if (err < 0 ) {
			dev_err(&client->dev, "Failed to parse board file!");
		}
	}
	else if(PLATFORM_DATA_TYPE == platform_data_from_dts_file)
	{
		err = fts_platform_data_parse_dts_file(client, fts_platform_data);
		if (err < 0 ) {
			dev_err(&client->dev, "Failed to parse dts file!");
		}
	}
	else if(PLATFORM_DATA_TYPE == platform_data_from_used_define)
	{
		fts_platform_data_used_define(client, fts_platform_data);
	}
	else
	{
		return -1;
	}

	//	set GPIOs that include interrupt pin and reset pin
	if(err>=0) fts_platform_data_gpio_configure(client, fts_platform_data);
	//client->irq = gpio_to_irq(fts_platform_data->irq_gpio);
	//client->addr = (0x72>>1);
	/*print platform data*/
	FTS_COMMON_DBG("[focal] platform data:  irq_gpio = %d, irq_gpio_flags = %d, reset_gpio = %d,\
	x_resolution_max = %d, y_resolution_max = %d, pressure_max = %d,  client->irq = %d, client->addr = %d",
		fts_platform_data->irq_gpio,
		fts_platform_data->irq_gpio_flags,
		fts_platform_data->reset_gpio,
		fts_platform_data->x_resolution_max,
		fts_platform_data->y_resolution_max,
		fts_platform_data->pressure_max,
		client->irq, client->addr
		);


	return err;
}
int fts_platform_data_exit(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);
	fts_platform_data_gpio_release(fts_platform_data);
	kfree(fts_platform_data);
	return 0;
}

#ifdef CONFIG_BBRY
static struct regulator* init_regulator(struct device *dev, char* name, int min_uV, int max_uV)
{
	struct regulator *vreg;
	int rc;
	vreg = regulator_get(dev, name);
	if (IS_ERR(vreg)) {
		rc = PTR_ERR(vreg);
		ERROR_COMMON_FTS("Regulator get failed %s rc=%d", name, rc);
		return NULL;
	}
	if (regulator_count_voltages(vreg) > 0) {
		rc = regulator_set_voltage(vreg, min_uV, max_uV);
		if (rc) {
			ERROR_COMMON_FTS("Regulator set_vtg failed %s rc=%d", name, rc);
		}
	}
	dev_err(dev, "Regulator get %s success\n", name);
	return vreg;
}

static u32 init_gpio(struct device *dev, char* name)
{
	int gpio, err;
	gpio = of_get_named_gpio(dev->of_node, name, 0);
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, name);
		if (err) {
			ERROR_COMMON_FTS("request gpio %s failed.", name);
		}
	} else {
		ERROR_COMMON_FTS("Unable to get %s", name);
	}
	FTS_COMMON_DBG("init gpio %s success\n", name);
	return gpio;
}

/* I got these numbers from the values used by the display driver */
#define VREG_ENABLE_LOAD 100000
#define VREG_DISABLE_LOAD 100
#define VREG_DISP_1P8_VOLTAGE 1800000
#define VREG_LABIBB_VOLTAGE_MIN 4600000
#define VREG_LABIBB_VOLTAGE_MAX 6000000

void enable_regulator(struct device *dev, struct regulator *vreg, const char* vreg_name, bool enable)
{
	int rc;
	dev_err(dev, "%s=%s\n", vreg_name, (enable)?"enable":"disable");
	if (enable) {
		rc = regulator_set_optimum_mode(vreg, VREG_ENABLE_LOAD);
		if (rc < 0) {
			ERROR_COMMON_FTS("failed to regulator_set_optimum_mode(%s), rc=%d", vreg_name, rc);
		}
		rc = regulator_enable(vreg);
		if (rc < 0) {
			ERROR_COMMON_FTS("failed to regulator_enable(%s), rc=%d", vreg_name, rc);
		}
	} else {
		rc = regulator_set_optimum_mode(vreg, VREG_DISABLE_LOAD);
		if (rc < 0) {
			ERROR_COMMON_FTS("failed to regulator_set_optimum_mode(%s) rc=%d", vreg_name, rc);
		}
		rc = regulator_disable(vreg);
		if (rc < 0) {
			ERROR_COMMON_FTS("failed to regulator_disable(%s), rc=%d", vreg_name, rc);
		}
	}
}
#endif

static int fts_platform_data_parse_dts_file(struct i2c_client *client, struct fts_ts_platform_data *pdata)
{
#if (PLATFORM_DATA_TYPE == PLATFORM_DATA_DTS_TYPE)
	struct device *dev = &client->dev;
	struct device_node *node;
	u32 temp_val;
	int rc = 0;

	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	node = dev->of_node;
	if(!node){
		dev_err(dev, "The of_node of device is NULL.");
		return -1;
	}

	//get int pin for irq*
	pdata->irq_gpio = of_get_named_gpio_flags(node, "focaltech,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
	{
		dev_err(dev, "Unable to get irq_gpio ");
	}

	//get reset pin*
#ifdef CONFIG_BBRY
	pdata->reset_gpio = of_get_named_gpio_flags(node, "focaltech,reset-gpio", 0, &pdata->reset_gpio_flags);
#else
	pdata->reset_gpio = of_get_named_gpio_flags(node, "focaltech,irq-gpio", 0, &pdata->reset_gpio_flags);
#endif
	if (pdata->reset_gpio < 0)
	{
		dev_err(dev, "Unable to get reset_gpio ");
	}

	//get x resolution*
	rc = of_property_read_u32(node, "focaltech,x-resolution", &temp_val);
	if (!rc)
		pdata->x_resolution_max= temp_val;
	else
		dev_err(dev, "Unable to get x-resolution ");

	//get y resolution*
	rc = of_property_read_u32(node, "focaltech,y-resolution", &temp_val);
	if (!rc)
		pdata->y_resolution_max= temp_val;
	else
		dev_err(dev, "Unable to get y-resolution ");
#ifdef CONFIG_BBRY
	pdata->disp_1p8 = init_regulator(dev, "disp_1p8", VREG_DISP_1P8_VOLTAGE, VREG_DISP_1P8_VOLTAGE);
	pdata->disp_p5v = init_regulator(dev, "disp_p5v", VREG_LABIBB_VOLTAGE_MIN, VREG_LABIBB_VOLTAGE_MAX);
	pdata->disp_n5v = init_regulator(dev, "disp_n5v", VREG_LABIBB_VOLTAGE_MIN, VREG_LABIBB_VOLTAGE_MAX);

	/* enable these at init - We assume that LK has turned the display on, and we want to
	 * explicitly vote the vregs on so that they do not get turned off in the kernel
	 */
	enable_regulator(dev, pdata->disp_1p8, "disp_1p8", true);
	enable_regulator(dev, pdata->disp_p5v, "disp_p5v", true);
	enable_regulator(dev, pdata->disp_n5v, "disp_n5v", true);

	pdata->disp_1p8_en_gpio = init_gpio(dev, "oem,disp-1p8-en-gpio");
	pdata->disp_p5v_en_gpio = init_gpio(dev, "oem,disp-p5v-en-gpio");
	pdata->disp_n5v_en_gpio = init_gpio(dev, "oem,disp-n5v-en-gpio");
	pdata->disp_rst_n_gpio = init_gpio(dev, "oem,disp-rst-n-gpio");
	/* MODIFIED-BEGIN by Haojun Chen, 2016-11-08,BUG-3403049*/
	/* MODIFIED-END by Haojun Chen,BUG-3403049*/
#endif
#endif

	return 0;

}
static int fts_platform_data_parse_board_file(struct i2c_client *client)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "failed to get platform data\n");
		return -1;
	}

	memcpy(fts_platform_data, client->dev.platform_data, sizeof(struct fts_ts_platform_data));

	//fts_platform_data = client->dev.platform_data;

	if(0 == fts_platform_data->irq_gpio || 0 == fts_platform_data->reset_gpio)
	{
		dev_err(&client->dev, "Error: irq_gpio or  reset_gpio equal 0!\n");
		return -1;
	}

/*	if(0 == pdata->irq_gpio)
		pdata->irq_gpio = PLATFORM_DATA_IRQ_GPIO;

	if(0 == pdata->irq_gpio_flags)
		pdata->irq_gpio_flags = PLATFORM_DATA_IRQ_GPIO_FLAG;

	if(0 == pdata->reset_gpio)
		pdata->reset_gpio = PLATFORM_DATA_RESET_GPIO;

	if(0 == pdata->x_resolution_max)
		pdata->x_resolution_max = PLATFORM_DATA_X_RESOLUTION;

	if(0 == pdata->y_resolution_max)
		pdata->y_resolution_max = PLATFORM_DATA_Y_RESOLUTION;

	if(0 == pdata->pressure_max)
		pdata->pressure_max = PLATFORM_DATA_PRESSURE_VALUE;
*/
	return 0;
}

static int fts_platform_data_used_define(struct i2c_client *client, struct fts_ts_platform_data *pdata)
{
#if (PLATFORM_DATA_TYPE == PLATFORM_DATA_USERDEFINED_TYPE)
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	pdata->irq_gpio = PLATFORM_DATA_IRQ_GPIO;
	pdata->irq_gpio_flags = PLATFORM_DATA_IRQ_GPIO_FLAG;
	pdata->reset_gpio = PLATFORM_DATA_RESET_GPIO;
	pdata->x_resolution_max = PLATFORM_DATA_X_RESOLUTION;
	pdata->y_resolution_max = PLATFORM_DATA_Y_RESOLUTION;
	pdata->pressure_max = PLATFORM_DATA_PRESSURE_VALUE;
#endif
	return 0;
}
/*******************************************************************************
*  Name: fts_platform_data_gpio_configure
*  Brief: set gpio, include irq pin and reset pin
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int fts_platform_data_gpio_configure(struct i2c_client *client, struct fts_ts_platform_data *pdata)
{
	int err = 0;

	// set IRQ as input GPIO
	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio,
						"fts_irq_gpio");
		if (err) {
			FTS_COMMON_DBG("irq gpio request failed.");
			goto err_irq_gpio_req;
		}

		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			FTS_COMMON_DBG("set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}else{
			FTS_COMMON_DBG("set irq_gpio as input gpio,  irq_gpio = %d", pdata->irq_gpio);
		}

		client->irq = gpio_to_irq(fts_platform_data->irq_gpio);

	}else{
		FTS_COMMON_DBG("invalid gpio of irq,  irq_gpio = %d", pdata->irq_gpio);
	}

	//	set RESET GPIO as output GPIO
	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio,
						"fts_reset_gpio");
		if (err) {
			FTS_COMMON_DBG("reset gpio request failed.");
			goto err_irq_gpio_dir;
		}
		/* set as output and high level. 	(0:low, 1:high) */
		err = gpio_direction_output(pdata->reset_gpio, 1);
		if (err) {
			FTS_COMMON_DBG("set_direction for reset gpio failed.");
				goto err_reset_gpio_dir;
		}
		else{
			FTS_COMMON_DBG("set reset_gpio as output gpio,  reset_gpio = %d", pdata->reset_gpio);
		}
		//msleep(pdata->hard_rst_dly);
		//gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}else{
		FTS_COMMON_DBG("invalid gpio of RESET,  reset_gpio = %d", pdata->reset_gpio);
	}

	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}
/*******************************************************************************
*  Name: fts_platform_data_gpio_release
*  Brief: free gpio
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int fts_platform_data_gpio_release(struct fts_ts_platform_data *pdata)
{
	//int err = 0;

	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

	if (gpio_is_valid(pdata->reset_gpio)) {
		/*
		 * This is intended to save leakage current
		 * only. Even if the call(gpio_direction_input)
		 * fails, only leakage current will be more but
		 * functionality will not be affected.
		 */
		//err = gpio_direction_input(pdata->reset_gpio);
		//if (err) {
		//	FTS_COMMON_DBG("unable to set direction for gpio [%d]", pdata->reset_gpio);
		//}
		gpio_free(pdata->reset_gpio);
	}
#ifdef CONFIG_BBRY
	if (gpio_is_valid(pdata->disp_1p8_en_gpio)) {
		gpio_free(pdata->disp_1p8_en_gpio);
	}
	if (gpio_is_valid(pdata->disp_rst_n_gpio)) {
		gpio_free(pdata->disp_rst_n_gpio);
	}
#endif

	return 0;

}
/*******************************************************************************
*  Name: fts_hardware_reset
*  Brief: set tp reset by gpio
*  Input: level == 0, set low level, level == 1, set high level
*  Output:
*  Return:
*******************************************************************************/
int fts_hardware_reset(bool level)
{
	//	chech whether gpio is valid
	if (!gpio_is_valid(fts_platform_data->reset_gpio)) {
		FTS_COMMON_DBG("invalid gpio of RESET,  reset_gpio = %d", fts_platform_data->reset_gpio);
		return -1;
	}


	//set reset pin high or low
	if(1 == level)
	{
		gpio_set_value(fts_platform_data->reset_gpio, 1);
		FTS_COMMON_DBG("Hardware reset, set HIGH level.  reset_gpio = %d, ", fts_platform_data->reset_gpio);
	}
	else
	{
		gpio_set_value(fts_platform_data->reset_gpio, 0);
		FTS_COMMON_DBG("Hardware reset, set LOW level.  reset_gpio = %d, ", fts_platform_data->reset_gpio);
	}

	return 0;

}

/*******************************************************************************
*  Name: fts_wakeup_gesture_set
*  Brief: set gesture mode
*  Input: enabled == true, disabled == false
*  Output:
*  Return:
*******************************************************************************/
void fts_wakeup_gesture_set(bool enabled)
{
	fts_platform_data->tap_to_wake_enabled = enabled;
	FTS_COMMON_DBG( "set fts_platform_data->tap_to_wake_enabled=%u\n", fts_platform_data->tap_to_wake_enabled);
}
/*******************************************************************************
*  Name: fts_wakeup_gesture_read
*  Brief: read gesture mode
*  Input: enabled == true, disabled == false
*  Output:
*  Return:
*******************************************************************************/
bool fts_wakeup_gesture_read(void)
{
	return fts_platform_data->tap_to_wake_enabled;
}
