/*
 * Copyright (C) 2014 BlackBerry Limited
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#ifdef CONFIG_BBRY
#include <linux/switch.h>
#include "include/linux/platform_data/slimport_device.h"
#else
#include <linux/platform_data/slimport_device.h>
#endif
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include "slimport_tx_drv.h"
#ifdef CONFIG_BBRY
#include "slimport_private.h"
#else
#include "slimport.h"
#endif

#ifdef CONFIG_BBRY
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <mdss_hdmi_slimport.h>

#define STALE_STATE_WAIT_SEC (15)
#define MAIN_PROC_ACTIVE_DELAY (50)
#define STALE_STATE_LIMIT (STALE_STATE_WAIT_SEC*1000 / MAIN_PROC_ACTIVE_DELAY)
static int playback_main_proc_delay = MAIN_PROC_ACTIVE_DELAY;

#define PLAY_BACK_EASE_TIME_SEC (5)
#define PLAY_BACK_ACTIVE_STATE_LIMIT \
		(PLAY_BACK_EASE_TIME_SEC*1000 / MAIN_PROC_ACTIVE_DELAY)
#define MAIN_PROC_RELAX_DELAY (300)
#endif

#ifdef CONFIG_SLIMPORT_CEC
#include "cec_lib/slimport_tx_cec.h"
#endif

struct i2c_client *anx7808_client;
int hdcp_en;

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
int have_custom_sw_pre = 0;
int custom_sw_pre = 0;
static int lock_usb;
static int cbl_det;
static struct class *slimport_class;
static struct device *slimport_class_dev;
#endif

#ifdef CONFIG_BBRY
/* Module parameter */
int phy_tuning[NUM_TUNING_VALS] = {0};

struct completion init_aux_ch_completion;
static uint32_t sp_tx_chg_current_ma = NORMAL_CHG_I_MA;

static int hdmi_dongle_resets;
#define MAX_DONGLE_RESETS (2)
#endif

struct anx7808_data {
#ifdef CONFIG_BBRY
	struct anx7808_platform_data data;
	bool initialized;
	bool update_chg_type;
	struct msm_hdmi_slimport_ops hdmi_ops;
#endif
	struct anx7808_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock slimport_lock;
	int cab_irq;
};

#ifdef CONFIG_BBRY
static void slimport_publish_online(struct anx7808_data *anx7808, int online);
#endif

static bool hdcp_enable = 1;

/*sysfs read interface*/
static ssize_t hdcp_ctrl_show(
	struct device *dev, struct device_attribute *attr,
	 char *buf)
{
	return sprintf(buf, "%d\n", hdcp_en);
}

/*sysfs write interface*/
static ssize_t hdcp_ctrl_store(
	struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	hdcp_en = val;
	DEV_DBG("hdcp_en set to %ld", val);
	return count;
}

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
/*sysfs read interface*/
static ssize_t cbl_det_show(
	struct device *dev, struct device_attribute *attr,
	 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cbl_det);
}

/* Write to file in the following format:
 * <swing><single space><preemphasis>
 * Valid swing values are in [0,3]
 * Valid pre-emphasis values are in [0,4-swing-1]
 * If preemphasis is omitted it will be assumed 0.
 * If both are omitted then both are assumed 0*/
static ssize_t sw_pre_store(
	struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	long val = 0;
	char *endp = 0;
	unchar swing = 0, preemph = 0;

	val = simple_strtol(buf, &endp, 10);
	swing = val;

	if (endp) {
		buf = endp+1;
		val = simple_strtol(buf, &endp, 10);
		preemph = val;
	}

	if (swing >= 4) {
		DEV_ERR("Custom Swing Error:(%d) >= 4", swing);
		return -EINVAL;
	}
	if (preemph >= (4 - swing)) {
		DEV_ERR("Custom PreEmph Error:(%d) > 4-(%d)", preemph, swing);
		return -EINVAL;
	}

	have_custom_sw_pre = 1;
	custom_sw_pre = preemph << 3 | swing;

	DEV_DBG("Custom swing = %d preemph = %d", swing, preemph);

	return count;
}

/*sysfs read interface*/
static ssize_t lock_usb_show(
	struct device *dev, struct device_attribute *attr,
	 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lock_usb);
}

/*sysfs write interface*/
static ssize_t lock_usb_store(
	struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	int ret;
	long val;
	ret = strict_strtol(buf, 10, &val);
	if (ret)
		return ret;
	lock_usb = val;
	DEV_DBG("lock_usb set to %ld", val);
	return count;
}

/* for control from user space */
static struct device_attribute slimport_class_attrs[] = {
	__ATTR(lock_usb, S_IRUGO | S_IWUSR, lock_usb_show, lock_usb_store),
	__ATTR(sw_preemph, S_IWUSR, NULL, sw_pre_store),
	__ATTR(cbl_det, S_IRUGO, cbl_det_show, NULL),
	__ATTR_NULL
};
#endif /* CONFIG_BBRY_MFG or CONFIG_BBRY_DEBUG */

#ifdef CONFIG_BBRY
bool is_phy_tuning_set()
{
	int i;
	static bool phy_tuning_set;

	if (phy_tuning_set)
		return true;

	for (i = 0; i < NUM_TUNING_VALS; i++)
		if (phy_tuning[i]) {
			phy_tuning_set = true;
			return true;
		}

	return false;
}
#endif

/* for hdcp control from user space */
static struct device_attribute slimport_device_attrs[] = {
	__ATTR(hdcp_switch, S_IRUGO | S_IWUSR, hdcp_ctrl_show, hdcp_ctrl_store),
};

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

#ifdef CONFIG_BBRY
	if (sp_tx_pd_mode)
		return -EIO;
#endif

	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7808_client, offset);
	if (ret < 0) {
		DEV_ERR("%s: failed to read i2c addr=%x\n",
			__func__, slave_addr);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

#ifdef CONFIG_BBRY
	if (sp_tx_pd_mode)
		return -EIO;
#endif

	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx7808_client, offset, value);
	if (ret < 0) {
		DEV_ERR("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);
	}
	return ret;
}

void sp_tx_hardware_poweron(void)
{
#ifdef CONFIG_BBRY
	struct anx7808_data *anx7808 = dev_get_drvdata((const struct device *)&anx7808_client->dev);
	struct anx7808_platform_data *pdata = anx7808->pdata;
#else
	struct anx7808_platform_data *pdata =
		anx7808_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	msleep(1);
	gpio_set_value(pdata->gpio_p_dwn, 0);
	msleep(2);
	gpio_set_value(pdata->gpio_v10_ctrl, 1);
#ifdef CONFIG_BBRY
	gpio_set_value(pdata->gpio_v33_ctrl, 1);
#endif
	msleep(20);
	gpio_set_value(pdata->gpio_reset, 1);

	DEV_DBG("%s: anx7808 power on\n", __func__);
}

void sp_tx_hardware_powerdown(void)
{
#ifdef CONFIG_BBRY
	struct anx7808_data *anx7808 = dev_get_drvdata((const struct device *)&anx7808_client->dev);
	struct anx7808_platform_data *pdata = anx7808->pdata;

	slimport_publish_online(anx7808, 0);
	hdmi_rx_set_hpd(0);
	msleep(2);
#else
	struct anx7808_platform_data *pdata =
		anx7808_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	msleep(1);
	gpio_set_value(pdata->gpio_v10_ctrl, 0);
#ifdef CONFIG_BBRY
	gpio_set_value(pdata->gpio_v33_ctrl, 0);
#endif
	msleep(5);
	gpio_set_value(pdata->gpio_p_dwn, 1);
	msleep(1);

	DEV_DBG("%s: anx7808 power down\n", __func__);
}

#ifdef CONFIG_BBRY
static void reset_hdmi_dongle(void)
{
	uint8_t data = 0;

	/* confirm connected to 7730 (TODO - maybe unecessary) */
	if (!i2c_master_read_reg(0, 0x03, &data) || data != 0x77)
		return;

	if (!i2c_master_read_reg(0, 0x02, &data) || data != 0x30)
		return;

	if (i2c_master_read_reg(0, 0x06, &data)) {
		data = 0xA0;	/* logic and AUX sw reset on 7730 */

		i2c_master_write_reg(0, 0x06, data);

		/* delay to allow reset to complete */
		msleep(200);

		/* read back reset reg - should be 0 */
		if (i2c_master_read_reg(0, 0x06, &data))
			DEV_DBG("7730 Reset Success? %d", (data == 0));
	}
}
#endif

static void slimport_cable_plug_proc(struct anx7808_data *anx7808)
{
	if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det)) {
		msleep(50);
		if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det)) {
			if (sp_tx_pd_mode) {
				sp_tx_pd_mode = 0;
				sp_tx_hardware_poweron();
				sp_tx_power_on(SP_TX_PWR_REG);
				sp_tx_power_on(SP_TX_PWR_TOTAL);
				hdmi_rx_initialization();
				sp_tx_initialization();
				slimport_driver_version();
				sp_tx_vbus_poweron();
				/*msleep(200);*/
				if (!sp_tx_get_cable_type(1)) {
					DEV_ERR("%s:AUX ERR\n", __func__);
					sp_tx_vbus_powerdown();
					sp_tx_power_down(SP_TX_PWR_REG);
					sp_tx_power_down(SP_TX_PWR_TOTAL);
					sp_tx_hardware_powerdown();
					sp_tx_pd_mode = 1;
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_enable = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_rx_type = RX_NULL;
					sp_tx_rx_type_backup = RX_NULL;
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					return;
				}
				sp_tx_aux_polling_enable(1);
				sp_tx_rx_type_backup = sp_tx_rx_type;
			}
			switch (sp_tx_rx_type) {
			case RX_HDMI:
				if (sp_tx_get_hdmi_connection())
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				break;
			case RX_DP:
				if (sp_tx_get_dp_connection())
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				break;
			case RX_VGA_GEN:
				if (sp_tx_get_vga_connection())
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				break;
			case RX_VGA_9832:
				if (sp_tx_get_vga_connection()) {
					sp_tx_send_message(MSG_CLEAR_IRQ);
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				}
				break;
			case RX_NULL:
			default:
				break;
			}
		}
	} else if (sp_tx_pd_mode == 0) {
		sp_tx_vbus_powerdown();
		sp_tx_power_down(SP_TX_PWR_REG);
		sp_tx_power_down(SP_TX_PWR_TOTAL);
		sp_tx_hardware_powerdown();
		sp_tx_pd_mode = 1;
		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		sp_tx_rx_type = RX_NULL;
		sp_tx_rx_type_backup = RX_NULL;
		sp_tx_set_sys_state(STATE_CABLE_PLUG);
	}
}

static void slimport_edid_proc(void)
{
#ifdef CONFIG_SLIMPORT_CEC
	/*Make sure CEC operation only
	happened after cec path is setup.*/
	if (sp_tx_rx_type == RX_HDMI) {
		if (sp_tx_get_hdmi_connection()) {
			cec_init();
			DEV_ERR("cec initialed!\n");
		} else {
			DEV_ERR("hdmi connection is not stable,\
cec not initialed!\n");
			return;
		}
	}

#endif
	sp_tx_edid_read();

#ifdef CONFIG_BBRY
	if (bedid_break) {
		DEV_ERR("EDID corruption!");

		if (sp_tx_rx_type == RX_HDMI &&
				hdmi_dongle_resets < MAX_DONGLE_RESETS) {
			DEV_ERR("Try to recover ANX7730");
			sp_tx_aux_polling_enable(0);
			reset_hdmi_dongle();
			hdmi_dongle_resets++;

			sp_tx_rx_type = RX_NULL;
			sp_tx_rx_type_backup = -1; /* Force SP reset */
			sp_tx_set_sys_state(STATE_CABLE_PLUG);
			return;
		}
	}
	hdmi_dongle_resets = 0;
#else
	if (bedid_break)
		DEV_ERR("%s: EDID corruption!\n", __func__);
#endif

	hdmi_rx_set_hpd(1);
	hdmi_rx_set_termination(1);
	sp_tx_set_sys_state(STATE_LINK_TRAINING);

}

#ifdef CONFIG_BBRY
static void sp_tx_limit_upstream_pclk(void)
{
	struct anx7808_data *anx7808 =
		dev_get_drvdata((const struct device *)&anx7808_client->dev);

	if (anx7808->hdmi_ops.out_set_slimport_max_pclk) {
		int max_pclk;
		switch (slimport_link_bw) {
		case BW_162G:
			max_pclk = 53000;
			break;
		case BW_27G:
			max_pclk = 89000;
			break;
		case BW_54G:
		default:
			max_pclk = 180000;
			break;
		}
		anx7808->hdmi_ops.out_set_slimport_max_pclk(
					anx7808->pdata->hdmi_pdev, max_pclk);
	}
}
#endif

int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	DEV_ERR("Slimport_read_edid_block: %d", block);
	if (block == 0) {
#ifdef CONFIG_BBRY
		sp_tx_limit_upstream_pclk();
#endif
		memcpy(edid_buf, bedid_firstblock, sizeof(bedid_firstblock));
	} else if (block == 1) {
		memcpy(edid_buf, bedid_extblock, sizeof(bedid_extblock));
	} else {
		pr_err("%s: block number %d is invalid\n", __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

enum SP_LINK_BW slimport_get_link_bw(void)
{
	return slimport_link_bw;
}
EXPORT_SYMBOL(slimport_get_link_bw);

enum RX_CBL_TYPE sp_get_ds_cable_type(void)
{
	return sp_tx_rx_type;
}
EXPORT_SYMBOL(sp_get_ds_cable_type);

#ifdef CONFIG_SLIMPORT_FAST_CHARGE
enum CHARGING_STATUS sp_get_ds_charge_type(void)
{
	return downstream_charging_status;
}
EXPORT_SYMBOL(sp_get_ds_charge_type);
#endif

#ifdef CONFIG_BBRY
bool slimport_is_connected(void)
{
	struct anx7808_data *anx7808;
	bool result = false;

	if (!anx7808_client) {
		DEV_NOTICE("%s: device not initialized", __func__);
		return false;
	}

	anx7808 = dev_get_drvdata((const struct device *)&anx7808_client->dev);

	if (!anx7808 || !anx7808->pdata || !anx7808->initialized)
		return false;

	/* Note that gpio_cbl_det doesn't go low until
	 * the chip powers down out of slimport mode so this function
	 * is not entirely reliable if called around the time that the
	 * cable is unplugged
	 */
	if (gpio_get_value(anx7808->pdata->gpio_cbl_det)) {
		mdelay(10);
		if (gpio_get_value(anx7808->pdata->gpio_cbl_det)) {
			pr_info("slimport cable is detected\n");
			result = true;
		}
	}

	return result;
}
EXPORT_SYMBOL(slimport_is_connected);

uint32_t slimport_get_chg_current(void)
{
	struct anx7808_data *anx7808;
	int ret;

	if (!anx7808_client) {
		DEV_NOTICE("%s: device not initialized", __func__);
		return false;
	}

	anx7808 = dev_get_drvdata((const struct device *)&anx7808_client->dev);

	if (!anx7808 || !anx7808->initialized)
		return 0;

	INIT_COMPLETION(init_aux_ch_completion);
	anx7808->update_chg_type = true;

	ret = wait_for_completion_timeout(&init_aux_ch_completion,
					msecs_to_jiffies(2000));
	if (!ret) {
		DEV_ERR("failed to access charger type\n");
		return NORMAL_CHG_I_MA;
	}

	return sp_tx_chg_current_ma;
}
EXPORT_SYMBOL(slimport_get_chg_current);
#endif

static void slimport_config_output(void)
{
	sp_tx_clean_hdcp();
	sp_tx_set_colorspace();
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
	sp_tx_enable_video_input(1);
	sp_tx_set_sys_state(STATE_HDCP_AUTH);
}

static void slimport_playback_proc(void)
{
	if ((sp_tx_rx_type == RX_VGA_9832)
		|| (sp_tx_rx_type == RX_VGA_GEN)) {
		if ((sp_tx_hw_hdcp_en == 0) && (hdcp_en == 1)) {
			sp_tx_video_mute(1);
			sp_tx_set_sys_state(STATE_HDCP_AUTH);
		} else if ((sp_tx_hw_hdcp_en == 1) && (hdcp_en == 0))
			sp_tx_disable_slimport_hdcp();
	}
}

static void slimport_cable_monitor(struct anx7808_data *anx7808)
{
	if ((gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det))
		&& (!sp_tx_pd_mode)) {
		sp_tx_get_downstream_type();
		if (sp_tx_rx_type_backup != sp_tx_rx_type) {
			DEV_DBG("cable changed!\n");
			sp_tx_vbus_powerdown();
			sp_tx_power_down(SP_TX_PWR_REG);
			sp_tx_power_down(SP_TX_PWR_TOTAL);
			sp_tx_hardware_powerdown();
			sp_tx_pd_mode = 1;
			sp_tx_link_config_done = 0;
			sp_tx_hw_lt_enable = 0;
			sp_tx_hw_lt_done = 0;
			sp_tx_rx_type = RX_NULL;
			sp_tx_rx_type_backup = RX_NULL;
			sp_tx_set_sys_state(STATE_CABLE_PLUG);
		}
	}
}

#ifdef CONFIG_BBRY
static void slimport_publish_hdcp_cap(struct anx7808_data *anx7808)
{
	static int first = 1;
	int switch_val;

	switch (sp_get_ds_cable_type()) {
	case RX_HDMI:
	case RX_DP:
		switch_val = 1;
		break;
	default:
		switch_val = 0;
	}
	if (first || switch_val != anx7808->pdata->hdcp_cap_sdev.state) {
		switch_set_state(&anx7808->pdata->hdcp_cap_sdev, switch_val);
		DEV_DBG("%s: hdcp cap switch = %d", __func__, switch_val);
	}
	first = 0;
}

static void slimport_publish_online(struct anx7808_data *anx7808, int online)
{
	if (online != anx7808->pdata->online_sdev.state) {
		switch_set_state(&anx7808->pdata->online_sdev, online);
		DEV_DBG("%s: online switch = %d", __func__, online);
	}
}
#endif

static void slimport_main_proc(struct anx7808_data *anx7808)
{
#ifdef CONFIG_BBRY
	static int state_count;
#endif

	mutex_lock(&anx7808->lock);

	if (!sp_tx_pd_mode) {
		sp_tx_int_irq_handler();
		hdmi_rx_int_irq_handler();
	}

#ifdef CONFIG_BBRY
	if (sp_tx_system_state != STATE_PLAY_BACK &&
			sp_tx_system_state != STATE_CABLE_PLUG) {
		if (++state_count >= STALE_STATE_LIMIT) {
			DEV_ERR("stuck in state(%d)", sp_tx_system_state);
			sp_tx_vbus_powerdown();
			sp_tx_power_down(SP_TX_PWR_REG);
			sp_tx_power_down(SP_TX_PWR_TOTAL);
			sp_tx_hardware_powerdown();
			sp_tx_pd_mode = 1;
			sp_tx_link_config_done = 0;
			sp_tx_hw_lt_enable = 0;
			sp_tx_hw_lt_done = 0;
			sp_tx_rx_type = RX_NULL;
			sp_tx_rx_type_backup = RX_NULL;
			sp_tx_set_sys_state(STATE_CABLE_PLUG);
			state_count = 0;
		}
	} else if (sp_tx_system_state == STATE_PLAY_BACK) {
		if (++state_count >= PLAY_BACK_ACTIVE_STATE_LIMIT &&
			playback_main_proc_delay != MAIN_PROC_RELAX_DELAY) {
				DEV_DBG("Relaxing main process delay for playback");
				playback_main_proc_delay = MAIN_PROC_RELAX_DELAY;
		}
	}
#endif

	if (sp_tx_system_state == STATE_CABLE_PLUG)
		slimport_cable_plug_proc(anx7808);

	if (sp_tx_system_state == STATE_PARSE_EDID)
#ifndef CONFIG_BBRY
		slimport_edid_proc();
#else
	{
		slimport_publish_hdcp_cap(anx7808);
		slimport_edid_proc();
		if (sp_tx_system_state == STATE_LINK_TRAINING)
			slimport_publish_online(anx7808, 1);
	}

	if (anx7808->update_chg_type && !sp_tx_pd_mode) {
		sp_tx_chg_current_ma = sp_tx_get_chg_current();
		anx7808->update_chg_type = false;
		complete_all(&init_aux_ch_completion);
	}
#endif

	if (sp_tx_system_state == STATE_CONFIG_HDMI)
		sp_tx_config_hdmi_input();

	if (sp_tx_system_state == STATE_LINK_TRAINING) {
		if (!sp_tx_lt_pre_config())
			sp_tx_hw_link_training();
	}

	if (sp_tx_system_state == STATE_CONFIG_OUTPUT)
		slimport_config_output();

	if (sp_tx_system_state == STATE_HDCP_AUTH) {
		if (hdcp_enable) {
			sp_tx_hdcp_process();
		} else {
			sp_tx_power_down(SP_TX_PWR_HDCP);
			sp_tx_video_mute(0);
			hdmi_rx_show_video_info();
			sp_tx_show_infomation();
			sp_tx_set_sys_state(STATE_PLAY_BACK);
		}
	}

	if (sp_tx_system_state == STATE_PLAY_BACK)
		slimport_playback_proc();

	if (sp_tx_system_state_bak != sp_tx_system_state) {
		slimport_cable_monitor(anx7808);
		DEV_DBG("slimport_cable_monitor()...\n");
		sp_tx_system_state_bak = sp_tx_system_state;
#ifdef CONFIG_BBRY
		state_count = 0;
		playback_main_proc_delay = MAIN_PROC_ACTIVE_DELAY;
#endif
	}


	mutex_unlock(&anx7808->lock);
}

static uint8_t anx7808_chip_detect(void)
{
	return sp_tx_chip_located();
}

static void anx7808_chip_initial(void)
{
#ifdef EYE_TEST
	sp_tx_eye_diagram_test();
#else
	sp_tx_variable_init();
	sp_tx_vbus_powerdown();
	sp_tx_hardware_powerdown();
	sp_tx_set_sys_state(STATE_CABLE_PLUG);
	sp_tx_system_state_bak = STATE_INIT;

#endif
}

static void anx7808_free_gpio(struct anx7808_data *anx7808)
{
	gpio_free(anx7808->pdata->gpio_v10_ctrl);
	gpio_free(anx7808->pdata->gpio_cbl_det);
	gpio_free(anx7808->pdata->gpio_int);
	gpio_free(anx7808->pdata->gpio_reset);
	gpio_free(anx7808->pdata->gpio_p_dwn);
}
static int anx7808_init_gpio(struct anx7808_data *anx7808)
{
	int ret = 0;

	DEV_DBG("anx7808 init gpio\n");

	ret = gpio_request(anx7808->pdata->gpio_p_dwn, "anx_p_dwn_ctl");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_p_dwn);
		goto err0;
	}
	gpio_direction_output(anx7808->pdata->gpio_p_dwn, 1);

	ret = gpio_request(anx7808->pdata->gpio_reset, "anx7808_reset_n");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(anx7808->pdata->gpio_reset, 0);

	ret = gpio_request(anx7808->pdata->gpio_int, "anx7808_int_n");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_int);
		goto err2;
	}
	gpio_direction_input(anx7808->pdata->gpio_int);

	ret = gpio_request(anx7808->pdata->gpio_cbl_det, "anx7808_cbl_det");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_cbl_det);
		goto err3;
	}
	gpio_direction_input(anx7808->pdata->gpio_cbl_det);

	ret = gpio_request(anx7808->pdata->gpio_v10_ctrl, "anx7808_v10_ctrl");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_v10_ctrl);
		goto err4;
	}
	gpio_direction_output(anx7808->pdata->gpio_v10_ctrl, 0);

#ifdef CONFIG_BBRY
	ret = gpio_request(anx7808->pdata->gpio_v33_ctrl, "anx7808_v33_ctrl");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_v33_ctrl);
		goto err5;
	}
	gpio_direction_output(anx7808->pdata->gpio_v33_ctrl, 0);
#endif


	gpio_set_value(anx7808->pdata->gpio_v10_ctrl, 0);
#ifdef CONFIG_BBRY
	gpio_set_value(anx7808->pdata->gpio_v33_ctrl, 0);
#endif
	gpio_set_value(anx7808->pdata->gpio_reset, 0);
	gpio_set_value(anx7808->pdata->gpio_p_dwn, 1);
	goto out;

#ifdef CONFIG_BBRY
err5:
	gpio_free(anx7808->pdata->gpio_v33_ctrl);
#endif
err4:
	gpio_free(anx7808->pdata->gpio_v10_ctrl);
err3:
	gpio_free(anx7808->pdata->gpio_cbl_det);
err2:
	gpio_free(anx7808->pdata->gpio_int);
err1:
	gpio_free(anx7808->pdata->gpio_reset);
err0:
	gpio_free(anx7808->pdata->gpio_p_dwn);
out:
	return ret;
}

static int anx7808_system_init(void)
{
	int ret = 0;

	ret = anx7808_chip_detect();
	if (ret == 0) {
		DEV_ERR("%s : failed to detect anx7808\n", __func__);
		return -ENODEV;
	}

	anx7808_chip_initial();
	return 0;
}

static irqreturn_t anx7808_cbl_det_isr(int irq, void *data)
{
	struct anx7808_data *anx7808 = (struct anx7808_data *)data;
	int status;


	if (gpio_get_value(anx7808->pdata->gpio_cbl_det)) {
		wake_lock(&anx7808->slimport_lock);
		DEV_DBG("%s : detect cable insertion\n", __func__);
#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
		cbl_det = 1;
		if (!lock_usb)
#endif
		queue_delayed_work(anx7808->workqueue, &anx7808->work, 0);
	} else {
		DEV_DBG("%s : detect cable removal\n", __func__);
#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
		cbl_det = 0;
#endif
		status = cancel_delayed_work_sync(&anx7808->work);
		if (status == 0)
			flush_workqueue(anx7808->workqueue);
		wake_unlock(&anx7808->slimport_lock);
#if defined(CONFIG_BBRY)
		wake_lock_timeout(&anx7808->slimport_lock, 300);
#endif
	}

	return IRQ_HANDLED;
}

static void anx7808_work_func(struct work_struct *work)
{
#ifndef EYE_TEST
	struct anx7808_data *td = container_of(work, struct anx7808_data,
								work.work);

	slimport_main_proc(td);
#ifdef CONFIG_BBRY
	if (sp_tx_system_state > STATE_CABLE_PLUG &&
			sp_tx_system_state < STATE_PLAY_BACK) {
		queue_delayed_work(td->workqueue, &td->work,
				msecs_to_jiffies(MAIN_PROC_ACTIVE_DELAY));
	}
	else if (sp_tx_system_state == STATE_PLAY_BACK) {
		queue_delayed_work(td->workqueue, &td->work,
				msecs_to_jiffies(playback_main_proc_delay));
	} else {
#endif
	queue_delayed_work(td->workqueue, &td->work,
			msecs_to_jiffies(300));
#ifdef CONFIG_BBRY
	}
#endif

#endif
}

#ifdef CONFIG_BBRY
int anx7808_regulator_configure(
	struct device *dev, struct anx7808_platform_data *pdata)
{
	int rc = 0;
/* To do : regulator control after H/W change */
	return rc;

	pdata->avdd_10 = regulator_get(dev, "analogix,vdd_ana");

	if (IS_ERR(pdata->avdd_10)) {
		rc = PTR_ERR(pdata->avdd_10);
		pr_err("%s : Regulator get failed avdd_10 rc=%d\n",
			   __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->avdd_10) > 0) {
		rc = regulator_set_voltage(pdata->avdd_10, 1000000,
							1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
				   __func__, rc);
			goto error_set_vtg_avdd_10;
		}
	}

	pdata->dvdd_10 = regulator_get(dev, "analogix,vdd_dig");
	if (IS_ERR(pdata->dvdd_10)) {
		rc = PTR_ERR(pdata->dvdd_10);
		pr_err("%s : Regulator get failed dvdd_10 rc=%d\n",
			   __func__, rc);
		goto error_set_vtg_avdd_10;
	}

	if (regulator_count_voltages(pdata->dvdd_10) > 0) {
		rc = regulator_set_voltage(pdata->dvdd_10, 1000000,
							1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
				   __func__, rc);
			goto error_set_vtg_dvdd_10;
		}
	}

	return 0;

error_set_vtg_dvdd_10:
	regulator_put(pdata->dvdd_10);
error_set_vtg_avdd_10:
	regulator_put(pdata->avdd_10);

	return rc;
}

static void anx7808_regulator_deconfigure(
		struct device *dev, struct anx7808_platform_data *pdata)
{
	regulator_put(pdata->dvdd_10);
	regulator_put(pdata->avdd_10);
}

static int anx7808_parse_dt(
	struct device *dev, struct anx7808_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	struct platform_device *hdmi_pdev = NULL;
	struct device_node *hdmi_tx_node = NULL;

	pdata->gpio_p_dwn = of_get_named_gpio_flags(
		np, "analogix,p-dwn-gpio", 0, NULL);

	pdata->gpio_reset = of_get_named_gpio_flags(
		np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det = of_get_named_gpio_flags(
		np, "analogix,cbl-det-gpio", 0, NULL);

	pdata->gpio_int = of_get_named_gpio_flags(
			np, "analogix,int-gpio", 0, NULL);

	printk(KERN_INFO
			"%s gpio p_dwn : %d, reset : %d,  gpio_cbl_det %d\n",
			DEV_TAG, pdata->gpio_p_dwn,
			pdata->gpio_reset,
			pdata->gpio_cbl_det);
	/*
	 * if "external-ldo-control" property is not exist, we
	 * assume that it is used in board.
	 * if don't use external ldo control,
	 * please use "external-ldo-control=<0>" in dtsi
	 */
	rc = of_property_read_u32(np, "analogix,external-ldo-control",
		&pdata->external_ldo_control);
	if (rc == -EINVAL)
		pdata->external_ldo_control = 1;

	if (pdata->external_ldo_control) {
		pdata->gpio_v10_ctrl = of_get_named_gpio_flags(
			np, "analogix,v10-ctrl-gpio", 0, NULL);

		pdata->gpio_v33_ctrl = of_get_named_gpio_flags(
			np, "analogix,v33-ctrl-gpio", 0, NULL);

		printk(KERN_INFO "%s gpio_v10_ctrl %d avdd33-en-gpio %d\n",
				DEV_TAG, pdata->gpio_v10_ctrl, pdata->gpio_v33_ctrl);
	}

	/* parse phandle for hdmi tx */
	hdmi_tx_node = of_parse_phandle(np, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		return -EINVAL;
	}

	hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: hdmi_pdev [0X%x] to pdata->pdev\n",
	       __func__, (unsigned int)hdmi_pdev);

	pdata->hdmi_pdev = hdmi_pdev;

	if (!is_phy_tuning_set()) {
		struct property *prop;
		int len;
		prop = of_find_property(np, "oem,phy_tuning", NULL);
		if (!prop) {
			pr_err("Unable to find oem,phy_tuning");
			return 0;
		}

		len = prop->length / sizeof(int);
		if (len != NUM_TUNING_VALS) {
			pr_err("Invalid size for phy_tuning: %d (actual) != %d",
					len, NUM_TUNING_VALS);
			return 0;
		}

		rc = of_property_read_u32_array(np, "oem,phy_tuning",
				(u32 *) phy_tuning, prop->length / sizeof(u32));

		if (rc)
			pr_err("Unable to read oem,phy_tuning");
	}

	return 0;
}
#endif

static int anx7808_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct anx7808_data *anx7808;
	int ret = 0;
	int i;
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK)) {
		DEV_ERR("%s: i2c bus does not support the anx7808\n",
			__func__);
		ret = -ENODEV;
		goto exit;
	}

	anx7808 = kzalloc(sizeof(struct anx7808_data), GFP_KERNEL);
	if (!anx7808) {
		DEV_ERR("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

#ifdef CONFIG_BBRY
	anx7808->pdata = &anx7808->data;
#else
	anx7808->pdata = client->dev.platform_data;
#endif
	i2c_set_clientdata(client, anx7808);
	memcpy(&anx7808_client, &client, sizeof(client));

	mutex_init(&anx7808->lock);

#if CONFIG_BBRY
	ret = anx7808_parse_dt(&client->dev, anx7808->pdata);
	if(ret) {
		DEV_ERR("%s: failed to parse dt\n", __func__);
		goto err0;
	}

	if(anx7808->pdata->hdmi_pdev) {
		struct msm_hdmi_slimport_ops *ops = &anx7808->hdmi_ops;
		ops->in_read_edid_block = slimport_read_edid_block;
		ret = msm_hdmi_register_slimport(anx7808->pdata->hdmi_pdev,
			ops, NULL);
		if(ret) {
			ret = -EPROBE_DEFER;
			DEV_ERR("Failed to register with msm hdmi: %d", ret);
			goto err0;
		}
		DEV_ERR("Registered with msm hdmi");

		if (ops->out_set_slimport_max_pclk) {
			/* Max pclk of 180MHz chosen based on logic
			 * in sp_tx_bw_lc_sel which determines that
			 * we are over supported bandwidth if
			 * the input pclk is greater than 180MHz
			 */
			ops->out_set_slimport_max_pclk(
					anx7808->pdata->hdmi_pdev, 180000);
		}
	}

	anx7808->pdata->hdcp_cap_sdev.name = "hdmi_bridge_secure";
	ret = switch_dev_register(&anx7808->pdata->hdcp_cap_sdev);
	if (ret < 0) {
		DEV_ERR("secure switch registration failed %d\n", ret);
		goto err0;
	}

	anx7808->pdata->online_sdev.name = "hdmi_bridge";
	ret = switch_dev_register(&anx7808->pdata->online_sdev);
	if (ret < 0) {
		DEV_ERR("online switch registration failed %d\n", ret);
		goto err0_4;
	}

	ret = anx7808_regulator_configure(&client->dev, anx7808->pdata);
	if (ret) {
		pr_err("%s %s: parsing dt for anx7808 is failed.\n",
				DEV_TAG, __func__);
		goto err0_5;
	}

	init_completion(&init_aux_ch_completion);
#else
	if (!anx7808->pdata) {
		ret = -EINVAL;
		goto err0;
	}
#endif

	ret = anx7808_init_gpio(anx7808);
	if (ret) {
		DEV_ERR("%s: failed to initialize gpio\n", __func__);
		goto err1;
	}

	INIT_DELAYED_WORK(&anx7808->work, anx7808_work_func);

	anx7808->workqueue = create_singlethread_workqueue("anx7808_work");
	if (anx7808->workqueue == NULL) {
		DEV_ERR("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err2;
	}

	ret = anx7808_system_init();
	if (ret) {
		DEV_ERR("%s: failed to initialize anx7808\n", __func__);
		goto err2;
	}

	anx7808->cab_irq = gpio_to_irq(anx7808->pdata->gpio_cbl_det);
	if (anx7808->cab_irq < 0) {
		DEV_ERR("%s : failed to get gpio irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(anx7808->cab_irq, NULL, anx7808_cbl_det_isr,
					IRQF_TRIGGER_RISING
					| IRQF_TRIGGER_FALLING,
					"anx7808_cabel_det", anx7808);
	if (ret < 0) {
		DEV_ERR("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = enable_irq_wake(anx7808->cab_irq);
	if (ret < 0) {
		DEV_ERR("%s : Enable irq for cable detect", __func__);
		DEV_ERR("interrupt wake enable fail\n");
		goto err3;
	}

	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++) {
		ret = device_create_file(
			&client->dev, &slimport_device_attrs[i]);
		if (ret) {
			DEV_ERR("%s :anx7808 sysfs register failed\n",
				__func__);
			goto err4;
		}
	}

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
	if (slimport_class) {
		slimport_class_dev = device_create(slimport_class, &client->dev,
								0, NULL,
								"anx78xx");
		if (slimport_class_dev == NULL)
			DEV_ERR("Failed to create slimport class device");
	}
#endif

	wake_lock_init(&anx7808->slimport_lock,
		WAKE_LOCK_SUSPEND, "slimport_wake_lock");

#ifdef CONFIG_BBRY
	anx7808->initialized = true;
#endif

	goto exit;

err4:
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		device_remove_file(&client->dev, &slimport_device_attrs[i]);
err3:
	free_irq(anx7808->cab_irq, anx7808);
err2:
	destroy_workqueue(anx7808->workqueue);
err1:
	anx7808_free_gpio(anx7808);
#ifdef CONFIG_BBRY
	anx7808_regulator_deconfigure(&client->dev, anx7808->pdata);
err0_5:
	switch_dev_unregister(&anx7808->pdata->online_sdev);
err0_4:
	switch_dev_unregister(&anx7808->pdata->hdcp_cap_sdev);
#endif
err0:
	kfree(anx7808);
#ifdef CONFIG_BBRY
	anx7808_client = NULL;
#endif
exit:
	return ret;
}

static int anx7808_i2c_remove(struct i2c_client *client)
{
	struct anx7808_data *anx7808 = i2c_get_clientdata(client);
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		device_remove_file(&client->dev, &slimport_device_attrs[i]);
	free_irq(anx7808->cab_irq, anx7808);
#ifdef CONFIG_BBRY
#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
	device_destroy(slimport_class, 0);
#endif
	anx7808_regulator_deconfigure(&client->dev, anx7808->pdata);
	switch_dev_unregister(&anx7808->pdata->hdcp_cap_sdev);
	switch_dev_unregister(&anx7808->pdata->online_sdev);
#endif
	anx7808_free_gpio(anx7808);
	destroy_workqueue(anx7808->workqueue);
	wake_lock_destroy(&anx7808->slimport_lock);
	kfree(anx7808);
	return 0;
}

#ifdef CONFIG_BBRY
#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
    { .compatible = "analogix,anx7808",},
    { },
};
#endif
#endif

static const struct i2c_device_id anx7808_id[] = {
	{ "anx7808", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, anx7808_id);

static struct i2c_driver anx7808_driver = {
	.driver = {
		.name = "anx7808",
		.owner = THIS_MODULE,
#ifdef CONFIG_BBRY
#ifdef CONFIG_OF
		.of_match_table = anx_match_table,
#endif
#endif
	},
	.probe = anx7808_i2c_probe,
	.remove = anx7808_i2c_remove,
	.id_table = anx7808_id,
};

static int __init anx7808_init(void)
{
	int ret = 0;

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
	slimport_class = class_create(THIS_MODULE, "slimport");
	if (IS_ERR(slimport_class))
		return PTR_ERR(slimport_class);

	slimport_class->suspend = NULL;
	slimport_class->resume = NULL;
	slimport_class->dev_attrs = slimport_class_attrs;
#endif

	ret = i2c_add_driver(&anx7808_driver);
	if (ret < 0)
		DEV_ERR("%s: failed to register anx7808 i2c drivern",
			__func__);
	return ret;
}

static void __exit anx7808_exit(void)
{
	i2c_del_driver(&anx7808_driver);
}

module_init(anx7808_init);
module_exit(anx7808_exit);


MODULE_DESCRIPTION("Slimport  transmitter ANX7808 driver");
MODULE_AUTHOR("FeiWang <fwang@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("V0.8");

#if defined(CONFIG_BBRY_DEBUG)
module_param_array(phy_tuning, int, NULL, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(phy_tuning, "Alter 20-elem phy tuning array");
#endif

