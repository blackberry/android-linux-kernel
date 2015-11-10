/*
 * Copyright (C) 2015 BlackBerry Limited
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
#include "sensor_extended.h"
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/sensors.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#define M4_HUB_DEV_NAME "m4_hub"
#define DEFAULT_BOOT0 (110)
#define DEFAULT_BOOT1 (107)
#define CMD_GET_VERSION (0x01)
#define WAKELOCK_TIMEOUT(n) ((n * HZ)/1000) /* Wakelock timeout in milliseconds */

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32 min_uV;
	u32 max_uV;
};

struct m4_config_t {
	int boot0;
	int boot1;
	int hub_rst;
	int hub_int;
	int msm_int;
	int msm_sleep;
	struct wake_lock hub_wake_lock;
};

static struct sensor_regulator m4_hub_vreg[] = {
	{NULL, "vcc",	  1800000, 1800000},
	{NULL, "vcc_i2c", 1800000, 1800000},
	{NULL, "vcc_sensors", 2850000, 2850000},
};

static struct m4_config_t _m4_config;

static int m4_hub_config_regulator(struct i2c_client *client, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(m4_hub_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			m4_hub_vreg[i].vreg =
				regulator_get(&client->dev,
				m4_hub_vreg[i].name);
			if (IS_ERR(m4_hub_vreg[i].vreg)) {
				rc = PTR_ERR(m4_hub_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n", __func__, rc);
				m4_hub_vreg[i].vreg = NULL;
				goto error_vdd;
			}

			pr_info("Got m4 vreg\n");
			if (regulator_count_voltages(
				m4_hub_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					m4_hub_vreg[i].vreg,
					m4_hub_vreg[i].min_uV,
					m4_hub_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(m4_hub_vreg[i].vreg);
					m4_hub_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			pr_info("m4 hub regulator enable %d\n", on);
			rc = regulator_enable(m4_hub_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(
					m4_hub_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						m4_hub_vreg[i].vreg, 0,
						m4_hub_vreg[i].max_uV);
				}
				regulator_put(m4_hub_vreg[i].vreg);
				m4_hub_vreg[i].vreg = NULL;
				goto error_vdd;
			}
			pr_info("m4 hub regulator enable success\n");
		}
		msleep(50);
		return rc;
	} else
		i = num_reg;

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(m4_hub_vreg[i].vreg)) {
			if (0 < regulator_count_voltages(m4_hub_vreg[i].vreg))
				regulator_set_voltage(m4_hub_vreg[i].vreg, 0, m4_hub_vreg[i].max_uV);

			regulator_disable(m4_hub_vreg[i].vreg);
			regulator_put(m4_hub_vreg[i].vreg);
			m4_hub_vreg[i].vreg = NULL;
		}
	}
	return rc;
}

static struct sensors_classdev sensors_cdev = {
	.name = "m4_hub",
	.vendor = "Unknown",
	.version = 1,
	.handle = SENSOR_TYPE_ACCELEROMETER,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "0",
	.resolution = "0",
	.sensor_power = "0",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static irqreturn_t m4_hub_wake_irq(int irq, void *dev_id)
{
	struct m4_config_t *m4_config = (struct m4_config_t*) dev_id;
	/* Hold wakelock for 200 msec
	 * According to google: (https://source.android.com/devices/sensors/suspend-mode.html)
	 * the driver must hold a "timeout wake lock" for 200 milliseconds
	 * each time an event is being reported. That is, the SoC should not
	 * be allowed to go back to sleep in the 200 milliseconds following
	 * a wake-up interrupt.
	 */
	wake_lock_timeout(&m4_config->hub_wake_lock, WAKELOCK_TIMEOUT(200));
	return IRQ_HANDLED;
}

static int m4_hub_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc;
	struct device_node *np = client->dev.of_node;
	struct m4_config_t *m4_config = &_m4_config;
	memset(m4_config, 0, sizeof(struct m4_config_t));

	dev_info(&client->dev, "m4_hub probe\n");
	/* power it on now */
	rc = m4_hub_config_regulator(client, true);
	if (rc)
		goto fail0;

	/* read device tree for boot pins*/
	m4_config->boot0 = of_get_named_gpio(np, "hub_boot0", 0);
	m4_config->boot1 = of_get_named_gpio(np, "hub_boot1", 0);
	m4_config->hub_rst = of_get_named_gpio(np, "hub_rst", 0);
	m4_config->hub_int = of_get_named_gpio(np, "hub_int", 0);
	m4_config->msm_int = of_get_named_gpio(np, "msm_int_hub_wake_n", 0);
	m4_config->msm_sleep = of_get_named_gpio(np, "msm_sleep", 0);

	rc = expose_output(&client->dev, m4_config->boot0, "hub_boot0", 0, "hub_boot0");
	if (rc) {
		dev_err(&client->dev, "failed to expose hub_boot0: %d", rc);
		goto fail0;
	}

	rc = expose_output(&client->dev, m4_config->boot1, "hub_boot1", 0, "hub_boot1");
	if (rc) {
		dev_err(&client->dev, "failed to expose hub_boot1: %d", rc);
		goto fail1;
	}

	rc = expose_output(&client->dev, m4_config->hub_rst, "hub_rst", 1, "hub_rst");
	if (rc) {
		dev_err(&client->dev, "failed to expose hub_rst: %d", rc);
		goto fail2;
	}

	rc = expose_interrupt(&client->dev, m4_config->hub_int, "hub_int");
	if (rc) {
		dev_err(&client->dev, "failed to expose hub_int: %d", rc);
		goto fail3;
	}

	rc = sensors_classdev_register(&client->dev, &sensors_cdev);
	if (rc) {
		dev_err(&client->dev, "failed to register class");
		goto fail4;
	}

	rc = expose_output(&client->dev, m4_config->msm_int, "msm_int", 0, "msm_int");
	if (rc) {
		dev_err(&client->dev, "failed to expose msm_int: %d", rc);
		goto fail5;
	}
	if( !m4_config->msm_sleep ){
		dev_err(&client->dev, "failed to get msm_sleep");
		goto fail5;
	}
	if (0 > (rc = gpio_request(m4_config->msm_sleep, "msm_sleep"))) {
		dev_err(&client->dev, "failed to request msm_sleep: %d", rc);
		goto fail5;
	}

	if (0 != (rc = gpio_direction_output(m4_config->msm_sleep, 0))) {
		dev_err(&client->dev, "failed to set msm_sleep: %d", rc);
		goto fail6;
	}

	wake_lock_init(&m4_config->hub_wake_lock, WAKE_LOCK_SUSPEND, "m4_hub_wake_lock");

	enable_irq_wake(gpio_to_irq(m4_config->hub_int));

	device_init_wakeup(&client->dev, true);

	/* Extend the data available through the class interface*/
	sensor_create_extension(&client->dev);
	i2c_set_clientdata(client, m4_config);
	return 0;

fail6:
	gpio_free(m4_config->msm_sleep);
fail5:
	gpio_unexport(m4_config->msm_int);
	gpio_free(m4_config->msm_int);
fail4:
	gpio_unexport(m4_config->hub_int);
	gpio_free(m4_config->hub_int);
fail3:
	gpio_unexport(m4_config->hub_rst);
	gpio_free(m4_config->hub_rst);
fail2:
	gpio_unexport(m4_config->boot1);
	gpio_free(m4_config->boot1);
fail1:
	gpio_unexport(m4_config->boot0);
	gpio_free(m4_config->boot0);
fail0:
	return rc;
}

static int m4_hub_remove(struct i2c_client *client)
{
	struct m4_config_t *m4_config = (struct m4_config_t *) i2c_get_clientdata(client);
	dev_info(&client->dev, "m4_hub remove\n");
	if (m4_config) {
		gpio_unexport(m4_config->hub_rst);
		gpio_unexport(m4_config->boot1);
		gpio_unexport(m4_config->boot0);
		gpio_free(m4_config->hub_rst);
		gpio_free(m4_config->boot1);
		gpio_free(m4_config->boot0);
		gpio_free(m4_config->msm_sleep);
		gpio_unexport(m4_config->msm_int);
		device_init_wakeup(&client->dev, false);
		wake_lock_destroy(&m4_config->hub_wake_lock);
	}
	return 0;
}

#ifdef CONFIG_PM
static int m4_hub_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;
	struct m4_config_t *m4_config = (struct m4_config_t *) i2c_get_clientdata(client);

	if (0 != (rc = gpio_direction_output(m4_config->msm_sleep, 1))) {
		dev_err(&client->dev, "failed to set msm_sleep for suspend: %d", rc);
	}
	if (0 != (rc = request_irq(gpio_to_irq(m4_config->hub_int), m4_hub_wake_irq,
								IRQF_TRIGGER_RISING | IRQF_SHARED, "hub_wake_int", m4_config))){
		dev_err(&client->dev, "Could not request irq\n");
	}

	return 0;
}

static int m4_hub_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;
	struct m4_config_t *m4_config = (struct m4_config_t *) i2c_get_clientdata(client);

	if (0 != (rc = gpio_direction_output(m4_config->msm_sleep, 0))) {
		dev_err(&client->dev, "failed to set msm_sleep for resume: %d", rc);
	}
	free_irq(gpio_to_irq(m4_config->hub_int), m4_config);

	return 0;
}

static SIMPLE_DEV_PM_OPS(m4_hub_pm_ops, m4_hub_suspend, m4_hub_resume);
#endif

#ifdef CONFIG_OF
static struct of_device_id m4_hub_table[] = {
	{ .compatible = "stm,m4_hub",},
	{ },
};
#else
#define m4_hub_table NULL
#endif

static const struct i2c_device_id m4_hub_id[]
	= { { M4_HUB_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, m4_hub_id);

static struct i2c_driver m4_hub_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = M4_HUB_DEV_NAME,
#ifdef CONFIG_PM
		.pm	= &m4_hub_pm_ops,
#endif
		.of_match_table = m4_hub_table,
	},
	.probe = m4_hub_probe,
	.remove = m4_hub_remove,
	.id_table = m4_hub_id,
};

module_i2c_driver(m4_hub_driver);
MODULE_DESCRIPTION("I2C M4 Hub Communication");
MODULE_AUTHOR("BlackBerry Limited");
MODULE_LICENSE("GPL v2");
