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
#include <linux/delay.h>
#include <linux/sensors.h>
#include <linux/of.h>

#define I2C_RETRY_DELAY 5
#define I2C_RETRIES 5
#define SENSOR_HUB_DEV_NAME "sensor_hub"
#define GO_CMD (0x47)
#define DEFAULT_INT_PIN (67)
#define DEFAULT_RST_PIN (69)

static struct sensors_classdev sensors_cdev = {
	.name = "sensor_hub",
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

static int get_interrupt(struct device * dev, int * gpio)
{
	u32 val;
	int rc = get_u32_property(dev, &val, "interrupt");
	*gpio = val;
	return rc;
}

static int get_reset_pin(struct device * dev, int * gpio)
{
	u32 val;
	int rc = get_u32_property(dev, &val, "reset-pin");
	*gpio = val;
	return rc;
}

static bool get_hub_go(struct device * dev)
{
	bool rc = false;
	struct device_node *np = dev->of_node;
	if ( np != NULL ) {
		rc = of_property_read_bool(np, "hub-go-cmd");
	}
	return rc;
}

static int sensor_hub_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc;
	int tries = 0;
	int int_pin;
	int rst_pin;
	u8 buf[] = {
		GO_CMD,
	};
	dev_info(&client->dev, "sensor_hub probe\n");

	if ( get_hub_go(&client->dev) == true ) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = client->flags,
				.len = sizeof(buf),
				.buf = buf,
			},
		};

		do {
			rc = i2c_transfer(client->adapter, msgs, 1);
			if (rc != 1)
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((rc != 1) && (++tries < I2C_RETRIES));

		if (rc != 1) {
			dev_err(&client->dev, "write transfer error\n");
			return -EIO;
		}
	}
	if (0 != get_interrupt(&client->dev, &int_pin))
		int_pin = DEFAULT_INT_PIN;
	if (0 != get_reset_pin(&client->dev, &rst_pin))
		rst_pin = DEFAULT_RST_PIN;

	if (0 != (rc = expose_interrupt(&client->dev, int_pin,
					"hub_interrupt"))) {
		dev_err(&client->dev, "failed to expose interrupt");
		return -1;
	}

	if (0 != (rc = expose_reset(&client->dev, rst_pin, "hub_reset", 1))) {
		dev_err(&client->dev, "failed to expose reset pin");
		return -1;
	}

	/* create sensor class entry for sensor */
	rc = sensors_classdev_register(&client->dev, &sensors_cdev);
	if (0 != rc) {
		dev_err(&client->dev, "failed to register class");
		return -1;
	}
	/* Extend the data available through the class interface*/
	sensor_create_extension(&client->dev);
	return 0;
}

static int sensor_hub_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "sensor_hub remove\n");
	return 0;
}

static const struct i2c_device_id sensor_hub_id[]
	= { { SENSOR_HUB_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, sensor_hub_id);

static struct i2c_driver sensor_hub_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_HUB_DEV_NAME,
	},
	.probe = sensor_hub_probe,
	.remove = sensor_hub_remove,
	.id_table = sensor_hub_id,
};
static int __init sensor_hub_init(void)
{
	int rc;
	rc = i2c_add_driver(&sensor_hub_driver);
	return rc;
}

static void __exit sensor_hub_exit(void)
{
    i2c_del_driver(&sensor_hub_driver);
}

module_init(sensor_hub_init);
module_exit(sensor_hub_exit);
MODULE_DESCRIPTION("I2C Sensor Hub Communication");
MODULE_AUTHOR("BlackBerry Limited");
MODULE_LICENSE("GPL v2");
