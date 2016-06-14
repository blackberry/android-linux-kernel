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

#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

# define IDTP9028_VRECT_MSB 0x40
# define IDTP9028_VRECT_LSB 0x41
# define IDTP9028_IOUT_MSB 0x42
# define IDTP9028_IOUT_LSB 0x43
# define IDTP9028_FREQ_MSB 0x44
# define IDTP9028_FREQ_LSB 0x45
# define IDTP9028_MISC_REG 0x48
# define IDTP9028_PMA_RXID_OFFSET 0x5B
# define IDTP9028_WPC_RXID_OFFSET 0x54
# define IDTP9028_FOD_SECTOR1_GAIN_OFFSET 0x49
# define IDTP9028_TX_TYPE_MASK 0x01

# define IDTP9028_PMA_RXID_LENGTH 11
# define IDTP9028_PMA_RXID_STR_LENGTH ((2 * IDTP9028_PMA_RXID_LENGTH) + 1)
# define IDTP9028_WPC_RXID_LENGTH 7
# define IDTP9028_FOD_PARAM_LENGTH 11

#define IDTP9028_NAME "idt-p9028"
#define IDTP9028_MAX_READ_RETRY 6

static char *model_name[] = {
	"WPC",
	"PMA",
	"Unknown"
};

static enum power_supply_property idtp9028_props[] = {
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

struct idtp9028_device {
	struct i2c_client *client;
	char pma_rxid[IDTP9028_PMA_RXID_STR_LENGTH];
	char *tx_type;
	struct power_supply wlc_psy;
	int chg_en_gpio_prop;
	unsigned chg_en;
};

static char *idtp9028_supplied_from[] = {
	"dc",
};

static struct idtp9028_device *idtp9028_dev;

static int idtp9028_read(uint16_t addr, uint8_t *data)
{
	int ret;
	uint8_t reg = (addr & 0xFF);
	uint8_t i;

	struct i2c_msg msgs[] = {
		{
			.addr = idtp9028_dev->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = idtp9028_dev->client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	*data = 0;

	if (idtp9028_dev->client) {
		for (i = 0; i < IDTP9028_MAX_READ_RETRY; i++) {
			ret = i2c_transfer(idtp9028_dev->client->adapter, msgs, 2);
			if (ret != 2) {
				pr_err("%s: i2c_transfer() trial_%u failure! "
				"addr=0x%x, ret=%d\n", __func__, i+1, addr, ret);
				msleep(10);
			} else {
				break;
			}
		}

		if (i == IDTP9028_MAX_READ_RETRY) {
				pr_err("%s: i2c_transfer() failure! "
				"addr=0x%x, ret=%d\n", __func__, addr, ret);
				return -ENODATA;
		}
	} else {
		pr_err("%s: idtp9028_dev->client is NULL!\n", __func__);
		return -ENODATA;
	}
	return 0;
}

static char *idtp9028_get_tx_type(void)
{
	int ret;
	int tx_type = 2;
	uint8_t data;

	ret = idtp9028_read(IDTP9028_MISC_REG, &data);
	if (ret == 0) {
		tx_type = (data & IDTP9028_TX_TYPE_MASK);
	} else if (ret < 0) {
		pr_err("%s: idtp9028_read() failed!\n", __func__);
	}
	return model_name[tx_type];
}

static unsigned idtp9028_get_current_ua(void)
{
	int ret;
	uint16_t iout;
	uint8_t data;

	ret = idtp9028_read(IDTP9028_IOUT_MSB, &data);
	if (ret < 0) {
		pr_err("%s: idtp9028_read() failed!\n", __func__);
		return 0;
	}
	iout = data << 8;

	ret = idtp9028_read(IDTP9028_IOUT_LSB, &data);
	if (ret < 0) {
		pr_err("%s: idtp9028_read() failed!\n", __func__);
		return 0;
	}
	iout |= data >> 4;

	return (iout * (18000000 / 4096))/100;
}

static int idtp9028_get_dc_presence(void)
{
	union power_supply_propval dc_present = {0,};
	struct power_supply *dc_psy = power_supply_get_by_name("dc");

	if (dc_psy == NULL) {
		pr_err("%s: Failed to get dc-psy!\n", __func__);
		return 0;
	}

	if (dc_psy->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT,
		&dc_present)) {
		pr_err("%s: Failed to read dc-presence!\n", __func__);
		return 0;
	}

	if (dc_present.intval < 0)
		dc_present.intval = 0;

	return dc_present.intval;
}

static int idtp9028_expose_gpio_output(struct device *dev, int gpio,
					const char *name, const int default_val,
							const char *entry_name)
{
	int err;

	if (0 == gpio_is_valid(gpio)) {
		pr_err("%s: Requested gpio_prop [%d] unavailable!\n", __func__,
									gpio);
		return -EINVAL;
	}

	err = gpio_request(gpio, name);
	if (0 > err) {
		pr_err("%s: Can't request gpio_prop [%d]: %d\n", __func__, gpio,
									err);
		return err;
	}

	err = gpio_export(gpio, false);
	if (0 != err) {
		pr_err("%s: Can't export gpio_prop [%d]: %d\n", __func__, gpio,
									err);
		goto fail;
	}

	err = gpio_direction_output(gpio, default_val);
	if (0 != err)
		pr_err("%s: Failed to set direction for gpio_prop [%d]\n",
								__func__, gpio);

fail:
	gpio_unexport(gpio);
	gpio_free(gpio);

	return err;
}

static int idtp9028_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = idtp9028_dev->pma_rxid;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = idtp9028_dev->tx_type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (idtp9028_get_dc_presence())
			val->intval = idtp9028_get_current_ua();
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = idtp9028_dev->chg_en;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int idtp9028_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (idtp9028_expose_gpio_output(&idtp9028_dev->client->dev,
			idtp9028_dev->chg_en_gpio_prop, "wlc_chg_en_n",
					(val->intval == 0), "wlc_chg_en_n")) {
			pr_err("%s: Failed to expose wlc_chg_en_n signal!\n",
								__func__);
		} else {
			idtp9028_dev->chg_en = (val->intval > 0);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int idtp9028_read_status(void)
{
	int i;
	int ret;
	static bool read_pma_rxid;
	char byte_str[3];
	u8 PmaRxId[IDTP9028_PMA_RXID_LENGTH];
	char PmaRxIds[IDTP9028_PMA_RXID_STR_LENGTH];

	if (idtp9028_get_dc_presence()) {
		if (!read_pma_rxid) {
			for (i = 0; i < IDTP9028_PMA_RXID_LENGTH; i++) {
				ret = idtp9028_read(i+IDTP9028_PMA_RXID_OFFSET,
								&(PmaRxId[i]));
				if (ret < 0) {
					pr_err("%s: idtp9028_read() failed!\n",
								__func__);
					strlcpy(idtp9028_dev->pma_rxid,
					"Error", IDTP9028_PMA_RXID_STR_LENGTH);
					break;
				}
			}

			if (!ret) {
				memset(PmaRxIds, 0,
						IDTP9028_PMA_RXID_STR_LENGTH);
				for (i = (IDTP9028_PMA_RXID_LENGTH - 1);
								i >= 0; i--) {
					snprintf(byte_str, 3, "%02x",
								PmaRxId[i]);
					strlcat(PmaRxIds, byte_str,
						IDTP9028_PMA_RXID_STR_LENGTH);
				}
				strlcpy(idtp9028_dev->pma_rxid, PmaRxIds,
						IDTP9028_PMA_RXID_STR_LENGTH);
				read_pma_rxid = true;
			}
		}
		idtp9028_dev->tx_type = idtp9028_get_tx_type();
	} else {
		idtp9028_dev->tx_type = "Unknown";
	}
	return 0;
}

static void idtp9028_external_power_changed(struct power_supply *psy)
{
	pr_info("%s: wlc event update!\n", __func__);

	idtp9028_read_status();

	power_supply_changed(&idtp9028_dev->wlc_psy);
}

static int idtp9028_register_psy(struct idtp9028_device *idtp9028_dev)
{
	int ret = 0;

	idtp9028_dev->wlc_psy.name = "wlc";
	idtp9028_dev->wlc_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
	idtp9028_dev->wlc_psy.num_supplicants = 0;
	idtp9028_dev->wlc_psy.properties = idtp9028_props;
	idtp9028_dev->wlc_psy.num_properties = ARRAY_SIZE(idtp9028_props);
	idtp9028_dev->wlc_psy.get_property = idtp9028_get_property;
	idtp9028_dev->wlc_psy.set_property = idtp9028_set_property;
	idtp9028_dev->wlc_psy.supplied_from = idtp9028_supplied_from;
	idtp9028_dev->wlc_psy.num_supplies =
					ARRAY_SIZE(idtp9028_supplied_from);
	idtp9028_dev->wlc_psy.external_power_changed =
					idtp9028_external_power_changed;

	ret = power_supply_register(&idtp9028_dev->client->dev,
						&idtp9028_dev->wlc_psy);
	if (ret)
		pr_err("%s: failed to register power_supply! ret=%d\n",
								__func__, ret);

	return ret;
}

static int idtp9028_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = 0;

	if (client->dev.of_node == NULL) {
		pr_err("%s: Device tree node doesn't exist!\n", __func__);
		return -ENODEV;
	}

	idtp9028_dev = kzalloc(sizeof(*idtp9028_dev), GFP_KERNEL);
	if (!idtp9028_dev) {
		pr_err("%s: alloc failed!\n", __func__);
		return -ENOMEM;
	}

	idtp9028_dev->chg_en_gpio_prop = of_get_named_gpio(client->dev.of_node,
							"wlc_chg_en_n", 0);
	if (idtp9028_dev->chg_en_gpio_prop < 0) {
		pr_err("%s: Failed to get wlc_chg_en_n GPIO property!\n",
							__func__);
	}
	strlcpy(idtp9028_dev->pma_rxid, "Unknown",
						IDTP9028_PMA_RXID_STR_LENGTH);
	idtp9028_dev->tx_type = "Unknown";
	idtp9028_dev->client = client;
	idtp9028_dev->chg_en = 1;
	i2c_set_clientdata(client, idtp9028_dev);

	ret = idtp9028_register_psy(idtp9028_dev);
	if (ret) {
		pr_err("%s: idtp9028_register_psy failed!\n", __func__);
		goto err_register_psy;
	}

	idtp9028_read_status();

	pr_info("%s: %s is ready!\n", __func__, IDTP9028_NAME);

	return 0;

err_register_psy:
	kfree(idtp9028_dev);
	idtp9028_dev = NULL;

	pr_err("%s: %s init failed!\n", __func__, IDTP9028_NAME);

	return ret;
}

static int idtp9028_remove(struct i2c_client *client)
{
	struct idtp9028_device *idtp9028_dev = i2c_get_clientdata(client);

	power_supply_unregister(&idtp9028_dev->wlc_psy);
	kfree(idtp9028_dev);
	idtp9028_dev = NULL;

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id idtp9028_table[] = {
	{.compatible = "idt,idt-p9028",},
	{},
};
#else
#define idtp9028_table NULL
#endif

static const struct i2c_device_id idtp9028_device_id[] = {
	{IDTP9028_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, idtp9028_device_id);

static struct i2c_driver idtp9028_driver = {
	.driver	= {
		.name	= IDTP9028_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = idtp9028_table,
	},
	.probe		= idtp9028_probe,
	.remove		= idtp9028_remove,
	.id_table	= idtp9028_device_id,
};
module_i2c_driver(idtp9028_driver);

MODULE_DESCRIPTION("IDT-P9028 wireless power chip driver");
MODULE_AUTHOR("BlackBerry Limited");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" IDTP9028_NAME);
