/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
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
#include "anx_ohio_driver.h"
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

static int create_sysfs_interfaces(struct device *dev);

/* to access global platform data */
static struct ohio_platform_data *g_pdata;
/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/22, PR-1918620 */
static u8 FW_VERSION;
static u8 IC_R02;
static u8 IC_R03;
static bool first_time = true;
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/22 */

#define DONGLE_CABLE_INSERT  1

struct i2c_client *ohio_client;

struct ohio_platform_data {
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
#ifdef SUP_OHIO_INT_VECTOR
	int gpio_intr_comm;
#endif
#ifdef SUP_VBUS_CTL
	int gpio_vbus_ctrl;
#endif
	spinlock_t lock;
};

struct ohio_data {
	struct ohio_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock ohio_lock;
};

/* ohio power status, sync with interface and cable detection thread */

inline unsigned char OhioReadReg(unsigned char RegAddr)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_byte_data(ohio_client, RegAddr);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
	return (uint8_t) ret;

}

inline int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}


inline int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}

inline void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;
	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_byte_data(ohio_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
}

void ohio_power_standby(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);
	gpio_set_value(pdata->gpio_p_on, 0);
	mdelay(1);

	pr_info("%s %s: Ohio power down\n", LOG_TAG, __func__);
}

void ohio_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	int retry_count, i;

	pr_info("%s %s: Ohio power on\n", LOG_TAG, __func__);

	for (retry_count = 0; retry_count < 3; retry_count++) {
		#ifdef OHIO_DEBUG
		pr_info("%s %s: Ohio check ocm loading...\n", LOG_TAG, __func__);
		#endif
		/*power on pin enable */
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(10);

		/*power reset pin enable */
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(1);

		/* load delay T3 : eeprom 3.2s,  OTP 20ms*/
		for (i = 0; i < OHIO_OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if (OhioReadReg(0x16) == 0x80) {
				#ifdef OHIO_DEBUG
				pr_info("%s %s: interface initialization\n", LOG_TAG, __func__);
				#endif

				chip_register_init();
				interface_init();
				send_initialized_setting();

				#ifdef OHIO_DEBUG
				if (OhioReadReg(0x7F) == 0x01)
					pr_info("%s %s: OTP chip is power on! firmware version is 0x%x\n", LOG_TAG, __func__, OhioReadReg(0x44));
				else
					pr_info("%s %s: EEPROM chip is power on! firmware version is 0x%x\n", LOG_TAG, __func__, OhioReadReg(0x44));
				#endif

				/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/22, PR-1918620 */
				if( first_time ){
					FW_VERSION = OhioReadReg(0x44);
					IC_R02 = OhioReadReg(0x02);
					IC_R03 = OhioReadReg(0x03);

					first_time = false;
				}
				/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/22 */

				return;
			}
			mdelay(1);
			printk(".");
		}
		ohio_power_standby();
		mdelay(10);
	}

}


/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/06/02, PR-2221001, read FW interface */
ssize_t ohio_readFW(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	pr_info("%s: firmware version is 0x%x\n", __func__, FW_VERSION);

	return snprintf(buf, sizeof(char) * 8, "%02x\n", FW_VERSION);

}
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/06/02 */


/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/16, PR-1804050, USB type C status detect interface */
ssize_t ohio_readid(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	pr_info("%s: firmware version is 0x%x, ID02=%x, ID03=%x\n", __func__, FW_VERSION, IC_R02, IC_R03);

	return snprintf(buf, sizeof(char) * 8, "%02x%02x\n", IC_R03, IC_R02);

}
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/16 */


/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/16, PR-1804050, USB type C status detect interface */
ssize_t ohio_detect(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char reg;
	u8 cc_status;
	//cc_status read from Register 0x2A
	//0xf1, 0xf4, read from Register 0x48

	cc_status = get_cc_status();

	if (cc_status == CC1_EN) {
		reg = 0xf1;
	}else if(cc_status == CC2_EN) {
		reg = 0xf4;
	}else{
		reg = 0xff;
	}
	pr_info("%s: reg=<0x%x>, cc_status=<0x%x>\n", __func__, reg, cc_status);

	return snprintf(buf, sizeof(char) * 8, "%02x\n", reg);
}
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/16 */


void ohio_vbus_control(bool value)
{
#ifdef SUP_VBUS_CTL

#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	if(value)
		gpio_set_value(pdata->gpio_vbus_ctrl, 1);
	else
		gpio_set_value(pdata->gpio_vbus_ctrl, 0);
#endif

}

static void ohio_free_gpio(struct ohio_data *ohio)
{
	gpio_free(ohio->pdata->gpio_cbl_det);
	gpio_free(ohio->pdata->gpio_reset);
	gpio_free(ohio->pdata->gpio_p_on);
#ifdef SUP_OHIO_INT_VECTOR
	gpio_free(ohio->pdata->gpio_intr_comm);
#endif
#ifdef SUP_VBUS_CTL
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
}

static int ohio_init_gpio(struct ohio_data *ohio)
{
	int ret = 0;

	pr_info("%s %s: ohio init gpio\n", LOG_TAG, __func__);
	/*  gpio for chip power down  */
	ret = gpio_request(ohio->pdata->gpio_p_on, "ohio_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_p_on, 0);
	/*  gpio for chip reset  */
	ret = gpio_request(ohio->pdata->gpio_reset, "ohio_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_reset, 0);

	/*  gpio for ohio cable detect  */
	ret = gpio_request(ohio->pdata->gpio_cbl_det, "ohio_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(ohio->pdata->gpio_cbl_det);

	#ifdef SUP_OHIO_INT_VECTOR
	/*  gpio for chip interface communaction */
	ret = gpio_request(ohio->pdata->gpio_intr_comm, "ohio_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(ohio->pdata->gpio_intr_comm);
	#endif

	#ifdef SUP_VBUS_CTL
	/*  gpio for vbus control  */
	ret = gpio_request(ohio->pdata->gpio_vbus_ctrl, "ohio_vbus_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_vbus_ctrl);
		goto err4;
	}
	gpio_direction_output(ohio->pdata->gpio_vbus_ctrl, 0);
	#endif

	goto out;

#ifdef SUP_VBUS_CTL
err4:
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
#ifdef SUP_OHIO_INT_VECTOR
err3:
	gpio_free(ohio->pdata->gpio_intr_comm);
#endif
err2:
	gpio_free(ohio->pdata->gpio_cbl_det);
err1:
	gpio_free(ohio->pdata->gpio_reset);
err0:
	gpio_free(ohio->pdata->gpio_p_on);

	return 1;
out:
	return 0;
}

void ohio_main_process(void)
{
	/* do main loop, do what you want to do */
}

#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
	struct ohio_data *anxohio = data;
	unsigned int count = 9;
	unsigned int cable_det_count = 0;
	u8 val = 0;

	do {
		val = gpio_get_value(anxohio->pdata->gpio_cbl_det);
		if (DONGLE_CABLE_INSERT == val)
			cable_det_count++;
		mdelay(1);;
	} while (count--);

	if (cable_det_count > 7)
		return 1;
	else if (cable_det_count < 3)
		return 0;
	else
		return atomic_read(&ohio_power_status);
}
#endif

static irqreturn_t ohio_cbl_det_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;
	int cable_connected = 0;

	#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det((void *)ohio);
	#else
	cable_connected = gpio_get_value(ohio->pdata->gpio_cbl_det);
	#endif

	#ifdef OHIO_DEBUG
	pr_info("%s %s : cable plug pin status %d\n", LOG_TAG, __func__, cable_connected);
	#endif

	if (cable_connected == DONGLE_CABLE_INSERT) {
		if (atomic_read(&ohio_power_status) == 1) {
			#ifdef CABLE_DET_PIN_HAS_GLITCH
			mdelay(2);
			ohio_power_standby();
			#else
			return IRQ_HANDLED;
			#endif
		}
		atomic_set(&ohio_power_status, 1);
		ohio_hardware_poweron();

	} else {
		atomic_set(&ohio_power_status, 0);
		#ifdef SUP_VBUS_CTL
		gpio_set_value(ohio->pdata->gpio_vbus_ctrl, 0);
		#endif

		ohio_power_standby();
	}

	return IRQ_HANDLED;
}

#ifdef SUP_OHIO_INT_VECTOR
static irqreturn_t ohio_intr_comm_isr(int irq, void *data)
{
	if (atomic_read(&ohio_power_status) != 1)
		return IRQ_NONE;

	if (is_soft_reset_intr()) {
#ifdef OHIO_DEBUG
		pr_info("%s %s : ======I=====\n", LOG_TAG, __func__);
#endif

	handle_intr_vector();
	}
	return IRQ_HANDLED;
}
#endif

#if 0
static void ohio_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					    work.work);
	int workqueu_timer = 0;
	workqueu_timer = 1;
	mutex_lock(&td->lock);
	ohio_main_process();
	mutex_unlock(&td->lock);
	queue_delayed_work(td->workqueue, &td->work,
			   msecs_to_jiffies(workqueu_timer));
}
#endif

#ifdef CONFIG_OF
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_p_on =
	    of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det =
	    of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VBUS_CTL
	pdata->gpio_vbus_ctrl =
	    of_get_named_gpio_flags(np, "analogix,vbus-ctrl-gpio", 0, NULL);
#endif
#ifdef SUP_OHIO_INT_VECTOR
	pdata->gpio_intr_comm =
	    of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);
#endif
	pr_info("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d\n",
		LOG_TAG, pdata->gpio_p_on,
		pdata->gpio_reset, pdata->gpio_cbl_det);

	return 0;
}
#else
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	return -ENODEV;
}
#endif
static int ohio_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{

	struct ohio_data *ohio;
	struct ohio_platform_data *pdata;
	int ret = 0;
	int cbl_det_irq = 0;

	/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/22, PR-1918620, resolve not recognize OTG when power on */
	int cbl_det_during_power = 0;
	/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/22 */

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:ohio's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ohio = kzalloc(sizeof(struct ohio_data), GFP_KERNEL);
	if (!ohio) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ohio_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = ohio_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		ohio->pdata = pdata;
	} else {
		ohio->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = ohio->pdata;
	ohio_client = client;
	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);

	atomic_set(&ohio_power_status, 0);

	mutex_init(&ohio->lock);

	if (!ohio->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = ohio_init_gpio(ohio);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	#if 0
	INIT_DELAYED_WORK(&ohio->work, ohio_work_func);
	#endif

	ohio->workqueue = create_singlethread_workqueue("ohio_work");
	if (ohio->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	cbl_det_irq = gpio_to_irq(ohio->pdata->gpio_cbl_det);
	if (cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err1;
	}

	wake_lock_init(&ohio->ohio_lock, WAKE_LOCK_SUSPEND, "ohio_wake_lock");

	ret = request_threaded_irq(cbl_det_irq, NULL, ohio_cbl_det_isr,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
				   | IRQF_ONESHOT, "ohio-cbl-det", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = irq_set_irq_wake(cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}
#ifdef SUP_OHIO_INT_VECTOR
	client->irq = gpio_to_irq(ohio->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, ohio_intr_comm_isr,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ohio-intr-comm", ohio);

	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		goto err4;
	}
#endif
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}

	/*when probe ohio device, enter standy mode */
	ohio_power_standby();

	/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/22, PR-1918620, resolve not recognize OTG when power on */
	#ifdef CABLE_DET_PIN_HAS_GLITCH
		cbl_det_during_power = confirmed_cable_det((void *)ohio);
	#else
		cbl_det_during_power = gpio_get_value(ohio->pdata->gpio_cbl_det);
	#endif

	pr_info("%s, cbl_det_during_power=<%d>\n", __func__, cbl_det_during_power);

	if(cbl_det_during_power){
		ohio_hardware_poweron();
	}
	/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/22 */

	pr_info("ohio_i2c_probe successfully %s %s end\n", LOG_TAG, __func__);
	goto exit;

err4:
	free_irq(client->irq, ohio);
err3:
	free_irq(cbl_det_irq, ohio);
err1:
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
err0:
	ohio_client = NULL;
	kfree(ohio);
exit:
	return ret;
}

static int ohio_i2c_remove(struct i2c_client *client)
{
	struct ohio_data *ohio = i2c_get_clientdata(client);
	printk("ohio_i2c_remove\n");
	free_irq(client->irq, ohio);
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
	wake_lock_destroy(&ohio->ohio_lock);
	kfree(ohio);
	return 0;
}

static int ohio_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ohio_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ohio_id[] = {
	{"ohio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ohio_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,ohio",},
	{},
};
#endif

static struct i2c_driver ohio_driver = {
	.driver = {
		   .name = "ohio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = ohio_i2c_probe,
	.remove = ohio_i2c_remove,
	.suspend = ohio_i2c_suspend,
	.resume = ohio_i2c_resume,
	.id_table = ohio_id,
};

static void __init ohio_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&ohio_driver);
	if (ret < 0)
		pr_err("%s: failed to register ohio i2c drivern", __func__);
}

static int __init ohio_init(void)
{
	async_schedule(ohio_init_async, NULL);
	return 0;
}

static void __exit ohio_exit(void)
{
	i2c_del_driver(&ohio_driver);
}

#ifdef OHIO_DEBUG
void dump_reg(void)
{
	int i = 0;
	u8 val = 0;

	printk("dump registerad:\n");
	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		val = OhioReadReg(i);

		if ((i) % 0x10 == 0x00)
			printk("\n[%x]:%02x ", i, val);
		else
			printk("%02x ", val);

	}
	printk("\n");
}

ssize_t anx_ohio_send_pd_cmd(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;

	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}


ssize_t anx_ohio_send_pswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_power_swap());
}

ssize_t anx_ohio_send_dswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_data_swap());
}

ssize_t anx_ohio_try_source(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_source());
}

ssize_t anx_ohio_try_sink(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_sink());
}

ssize_t anx_ohio_get_data_role(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx_ohio_get_power_role(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx_ohio_rd_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	printk("reg[%x] = %x\n", cmd, OhioReadReg(cmd));

	return count;

}

ssize_t anx_ohio_wr_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%d  %d", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	OhioWriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));
	return count;
}

ssize_t anx_ohio_dump_register(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i = 0;
	for (i = 0; i < 256; i++) {
		printk("%x", OhioReadReg(i));
		if (i % 0x10 == 0)
			pr_info("\n");

		snprintf(&buf[i], sizeof(u8), "%d", OhioReadReg(i));
	}

	printk("\n");

	return i;
}

ssize_t anx_ohio_select_rdo_index(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int cmd;
	cmd = sscanf(buf, "%d", &cmd);
	if (cmd <= 0)
		return 0;

	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}

/* for debugging */
static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(pdcmd, S_IRUGO | S_IWUSR, NULL,
	       anx_ohio_send_pd_cmd),
	__ATTR(rdreg, S_IRUGO | S_IWUSR, NULL,
	       anx_ohio_rd_reg),
	__ATTR(wrreg, S_IRUGO | S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(rdoidx, S_IRUGO | S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(dumpreg, S_IRUGO | S_IWUSR, anx_ohio_dump_register,
	       NULL),
	__ATTR(prole, S_IRUGO | S_IWUSR, anx_ohio_get_power_role,
	       NULL),
	__ATTR(drole, S_IRUGO | S_IWUSR, anx_ohio_get_data_role,
	       NULL),
	__ATTR(trysrc, S_IRUGO | S_IWUSR, anx_ohio_try_source,
	       NULL),
	__ATTR(trysink, S_IRUGO | S_IWUSR, anx_ohio_try_sink,
	       NULL),
	__ATTR(pswap, S_IRUGO | S_IWUSR, anx_ohio_send_pswap,
	       NULL),
	__ATTR(dswap, S_IRUGO | S_IWUSR, anx_ohio_send_dswap,
	       NULL),
	/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/16, PR-1804050, USB type C status detect interface */
	__ATTR(rdinfo, S_IRUGO, ohio_readid,
			   NULL),
	__ATTR(detect, S_IRUGO, ohio_detect,
			   NULL),
	/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/16 */
/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/06/02, PR-2221001, read FW interface */
	__ATTR(rdfw, S_IRUGO, ohio_readFW,
		   NULL),
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/06/02 */
};

#else
/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/04/16, PR-1804050, USB type C status detect interface */
static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(rdinfo, S_IRUGO, ohio_readid,
	       NULL),
	__ATTR(detect, S_IRUGO, ohio_detect,
		   NULL),
/* [PLATFORM]-Add-BEGIN by TCTNB.WPL, 2016/06/02, PR-2221001, read FW interface */
	__ATTR(rdfw, S_IRUGO, ohio_readFW,
		   NULL),
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/06/02 */
};
/* [PLATFORM]-Add-END by TCTNB.WPL, 2016/04/16 */
#endif

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	pr_info("ohio create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx_ohio_device_attrs); i++)
		if (device_create_file(dev, &anx_ohio_device_attrs[i]))
			goto error;
	pr_info("success\n");
	return 0;
error:

	for (; i >= 0; i--)
		device_remove_file(dev, &anx_ohio_device_attrs[i]);
	pr_err("%s %s: ohio Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}

module_init(ohio_init);
module_exit(ohio_exit);

MODULE_DESCRIPTION("USB PD Ohio driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.7");
