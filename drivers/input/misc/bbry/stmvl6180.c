/*
 * Copyright (C) 2014 BlackBerry Limited
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
 * Description: Lightweight Linux driver for the vl6180
 * The purpose of this lightweight driver is to power up the sensor, and
 * expose the GPIO to userspace via sysfs. Most of the code is
 * boilerplate linux driver code, with the exception of the functions:
 * - vl6180_config_regulator, for power cycling the sensor
 * - vl6180_probe, which kicks off the power cycle, and exposes the GPIO
 */
#include "stmvl6180.h"
#include "sensor_extended.h"

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#define VL6180_DEFAULT_GPIO (74)

#define ONESHOT_THRESHOLD (40)

typedef struct  {
	struct regulator *vreg;
	const char *name;
	u32 min_uV;
	u32 max_uV;
} sensor_regulator;

struct stmvl6180_config_t {
	int reset;
	int vl6180_int;
};

static struct stmvl6180_config_t vl6180_config;

sensor_regulator vl6180_vreg[] = {
	{NULL, "vcc",    2850000, 2850000},
	{NULL, "vcc_i2c", 0, 0},
};

static struct sensors_classdev sensors_cdev = {
	.name = "vl6180",
	.vendor = "Unknown",
	.version = 1,
	.handle = SENSOR_TYPE_PROXIMITY,
	.type = SENSOR_TYPE_PROXIMITY,
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

static struct workqueue_struct *workqueue;
static struct work_struct oneshot_work;

static int proxi_oneshot(struct i2c_client *client);
struct proxi_struct {
	void (*complete)(int object_detected, void *proxi_data);
	void *data;
};

static struct proxi_struct proxi;

/*
 * vl6180_config_regulator is used to properly configure the regulator
 * for the vl6180 part. This is necessary for the part to be powered
 * up/down correctly.
 */
static int vl6180_config_regulator(struct i2c_client *client, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(vl6180_vreg) / sizeof(sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			vl6180_vreg[i].vreg =
				regulator_get(&client->dev,
				vl6180_vreg[i].name);
			if (IS_ERR(vl6180_vreg[i].vreg)) {
				rc = PTR_ERR(vl6180_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n", __func__, rc);
				vl6180_vreg[i].vreg = NULL;
				goto error_vdd;
			}

			if (regulator_count_voltages(
				vl6180_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					vl6180_vreg[i].vreg,
					vl6180_vreg[i].min_uV,
					vl6180_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(vl6180_vreg[i].vreg);
					vl6180_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			rc = regulator_enable(vl6180_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(
					vl6180_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						vl6180_vreg[i].vreg, 0,
						vl6180_vreg[i].max_uV);
				}
				regulator_put(vl6180_vreg[i].vreg);
				vl6180_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		msleep(50);
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(vl6180_vreg[i].vreg)) {
			if (0 < regulator_count_voltages(vl6180_vreg[i].vreg))
				regulator_set_voltage(vl6180_vreg[i].vreg, 0, vl6180_vreg[i].max_uV);

			regulator_disable(vl6180_vreg[i].vreg);
			regulator_put(vl6180_vreg[i].vreg);
			vl6180_vreg[i].vreg = NULL;
		}
	}
	return rc;
}

static int vl6180_device_power_on(struct i2c_client *client)
{
	return vl6180_config_regulator(client, true);
}

static int vl6180_device_power_off(struct i2c_client *client)
{
	return vl6180_config_regulator(client, false);
}

static void synaptics_rmi4_oneshot_work(struct work_struct *work)
{
	int retval;
	struct i2c_client *client = to_i2c_client(sensors_cdev.dev->parent);

	retval = proxi_oneshot(client);

	if ((retval > 0) && (retval <= ONESHOT_THRESHOLD))
		proxi.complete(1, proxi.data);
	else if (retval > ONESHOT_THRESHOLD)
		proxi.complete(0, proxi.data);
	else
		proxi.complete(retval, proxi.data);
}

static int vl6180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	struct device_node *np = client->dev.of_node;
	memset(&vl6180_config, 0, sizeof(struct stmvl6180_config_t));
	pr_info("vl6180 probe\n");
	if (0 > (err = vl6180_device_power_on(client))) {
		pr_err("power on failed: %d\n", err);
		goto fail0;
	}

	err = of_get_named_gpio(np, "reset", 0);
	if (err < 0) {
		dev_err(&client->dev, "failed to get reset property");
		goto fail0;
	}
	vl6180_config.reset = err;

	err = expose_output(&client->dev, vl6180_config.reset, "reset", 1, "reset");
	if (err) {
		dev_err(&client->dev, "failed to expose reset: %d", err);
		goto fail1;
	}
	/*read gpio from device tree*/
	err = of_get_named_gpio(np, "interrupt", 0);
	if (err < 0) {
		dev_err(&client->dev, "failed to get interrupt property");
		goto fail1;
	}
	vl6180_config.vl6180_int = err;
	pr_info("vl6180 got interrupt %d\n", vl6180_config.vl6180_int);

	if (0 != expose_interrupt(&client->dev, vl6180_config.vl6180_int, "vl6180_interrupt")) {
		dev_err(&client->dev, "failed to expose gpio %d", vl6180_config.vl6180_int);
		goto fail2;
	}

	/* create sensor class entry for sensor */
	err = sensors_classdev_register(&client->dev, &sensors_cdev);
	if (0 != err) {
		dev_err(&client->dev, "failed to register class");
		goto fail2;
	}


	workqueue = alloc_workqueue("vl6180_wq",
				(WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM), 1);

	INIT_WORK(&oneshot_work,
			synaptics_rmi4_oneshot_work);

	/* Extend the data available through the class interface*/
	sensor_create_extension(&client->dev);
	i2c_set_clientdata(client, &vl6180_config);
	return 0;

fail2:
	gpio_unexport(vl6180_config.vl6180_int);
	gpio_free(vl6180_config.vl6180_int);

fail1:
	gpio_unexport(vl6180_config.reset);
	gpio_free(vl6180_config.reset);

fail0:
	vl6180_device_power_off(client);
	pr_err("%s: Driver initialization failed\n", VL6180_DEV_NAME);

	return err;
}

static int vl6180_remove(struct i2c_client *client)
{
	struct stmvl6180_config_t *config =
		(struct stmvl6180_config_t *) i2c_get_clientdata(client);
	dev_info(&client->dev, "vl6180_remove\n");
	if (config) {
		gpio_unexport(config->reset);
		gpio_free(config->reset);
		gpio_unexport(config->vl6180_int);
		gpio_free(config->vl6180_int);
	}
	return 0;
}

static const struct i2c_device_id vl6180_id[]
					= { { VL6180_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, vl6180_id);

static struct i2c_driver vl6180_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = VL6180_DEV_NAME,
	},
	.probe = vl6180_probe,
	.remove = vl6180_remove,
	.id_table = vl6180_id,
};

static int __init vl6180_init(void)
{
	int ret;
	pr_info("%s driver: init\n", VL6180_DEV_NAME);
	ret =  i2c_add_driver(&vl6180_driver);
	pr_info("vl6180 i2c_add_driver is %d\n", ret);
	return ret;
}

static void __exit vl6180_exit(void)
{
	pr_info("%s driver exit\n", VL6180_DEV_NAME);
	i2c_del_driver(&vl6180_driver);
}

#define SYN_I2C_RETRY_TIMES 2

static int write_bytes(struct i2c_client *client,
		uint16_t addr, unsigned short length, unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	uint8_t *addr_ptr = (uint8_t *) &addr;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(uint8_t) * length + 2,
			.buf = buf,
		}
	};

	buf[0] = addr_ptr[1];
	buf[1] = addr_ptr[0];
	memcpy(&buf[2], &data[0], sizeof(uint8_t) * length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&client->dev,
			"%s: I2C retry %d\n",
			__func__, retry + 1);
		if (retry == 0)
			dump_stack();
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&client->dev,
			"%s: I2C write over retry limit\n",
			__func__);
		retval = -EIO;
	}

	return retval;
}

static int write_byte(struct i2c_client *client,
		uint16_t addr, unsigned char data)
{
	return write_bytes(client, addr, 1, &data);
}

static int read_bytes(struct i2c_client *client,
		uint16_t addr, unsigned short length, unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[2];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	uint8_t *addr_ptr = (uint8_t *) &addr;
	buf[0] = addr_ptr[1]; /* msb first */
	buf[1] = addr_ptr[0];


	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(&client->dev,
			"%s: I2C retry %d\n",
			__func__, retry + 1);
		if (retry == 0)
			dump_stack();
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&client->dev,
			"%s: I2C read over retry limit\n",
			__func__);
		retval = -EIO;
	}

	return retval;
}

static int read_byte(struct i2c_client *client,
		uint16_t addr, unsigned char *data)
{
	return read_bytes(client, addr, 1, data);
}

static int proxi_oneshot(struct i2c_client *client)
{
	int retval;
	uint8_t data;
	int count = 0;


	retval = read_byte(client,
		RESULT_RANGE_STATUS, &data);

	if (retval < 0) {
		dev_err(&client->dev,
			"Failed to read RESULT_RANGE_STATUS (%d)\n", retval);
		return -EIO;
	}

	if (0 == (data & 0x1)) {
		dev_err(&client->dev,
			"vl6180_oneshot: prox not ready (%d)\n", data);
		return -EAGAIN;
	} else {
		dev_dbg(&client->dev,
			"vl6180_oneshot: prox is ready\n");
	}

	retval = write_byte(client,
		RANGE_START,
		(SYSALS_MODE_SELECT_SINGLE | SYSALS_STARTSTOP_START));

	if (retval < 0) {
		dev_err(&client->dev,
			"Failed to write RANGE_START (%d)\n", retval);
		return -EIO;
	}

	while (count < 1000) {
		retval = read_byte(client,
			RESULT_RANGE_STATUS, &data);

		if (retval < 0) {
			dev_err(&client->dev,
				"Failed to read RESULT_RANGE_STATUS(%d)\n", retval);
			return -EIO;
		}


		dev_dbg(&client->dev,
				"RESULT_RANGE_STATUS=0x%x\n",
				data);

		if (0 != (data & 0x1)) {
			retval = read_byte(client,
				RESULT_RANGE_VAL, &data);

			if (retval < 0) {
				dev_err(&client->dev,
					"Failed to read (%d)\n", retval);
				return -EIO;
			}

			dev_err(&client->dev,
				"vl6180_oneshot: range=%d, retry count=%d\n",
				data, count);

			return data;
		}

		count += 1;
		usleep(1000);
	}

	dev_err(&client->dev,
		"vl6180_oneshot: timeout\n");
	return -ETIMEDOUT;
}

void get_proxi_value_async(
		void (*complete_callback)(int object_detected, void *data),
		void *data)
{
	proxi.data = data;
	proxi.complete = complete_callback;
	queue_work(workqueue,
			&oneshot_work);
}
EXPORT_SYMBOL(get_proxi_value_async);

/*
 * Perform a late_initcall to ensure that the hub driver has initialized first
 */
/* module_init(vl6180_init); */
late_initcall(vl6180_init);
module_exit(vl6180_exit);
MODULE_DESCRIPTION("STMVL6180 Input Subsystem Interface");
MODULE_AUTHOR("BlackBerry Limited");
MODULE_LICENSE("GPL v2");
