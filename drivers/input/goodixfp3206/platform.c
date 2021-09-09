/* Copyright (C) 2018 Tcl Corporation Limited */
/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

    /*get pwr resource*/
    gf_dev->pwr_gpio = of_get_named_gpio(np,"fp-gpio_pwr",0);
    if(gf_dev->pwr_gpio < 0) {
        pr_err("falied to get pwr gpio!\n");
        return gf_dev->pwr_gpio;
    }
    pr_info("gf:pwr_gpio:%d\n", gf_dev->pwr_gpio);
    rc = devm_gpio_request(dev, gf_dev->pwr_gpio, "goodix_pwr");
    if(rc) {
        pr_err("failed to request pwr gpio, rc = %d\n", rc);
        goto error_pwr;
    }


	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0); // MODIFIED by Ji.Chen, 2018-01-19,BUG-5888909

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	/* MODIFIED-BEGIN by Ji.Chen, 2018-05-22,BUG-6257514*/
	gpio_direction_output(gf_dev->pwr_gpio, 0);
	//power on
	//gpio_direction_output(gf_dev->pwr_gpio, 1);
	//msleep(10);
	//gpio_direction_output(gf_dev->reset_gpio, 1);
	/* MODIFIED-END by Ji.Chen,BUG-6257514*/

err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);

error_pwr:
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	/* TODO: add your power control here */
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
        gpio_set_value(gf_dev->pwr_gpio, 1);
    }
    msleep(10);
	pr_info("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	/* TODO: add your power control here */
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
        gpio_set_value(gf_dev->pwr_gpio, 0);
        gpio_set_value(gf_dev->reset_gpio, 0);
    }
	pr_info("---- power off ----\n");

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0); // MODIFIED by Ji.Chen, 2018-05-22,BUG-6257514
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

