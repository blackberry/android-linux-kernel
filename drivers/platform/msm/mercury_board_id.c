/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

/* ------------------------------------------------------------------------- */

static int board_id_gpio1;
static int board_id_gpio2;
static int board_id_gpio3;
static int gpio1_status;
static int gpio2_status;
static int gpio3_status;
#define BOARD_PIO04 4
#define BOARD_PIO05 5
#define BAORD_OTHERS 0Xff


static int board_id_data_parse_dts_file(struct platform_device *pdev)
{
	struct device_node *node;
	struct pinctrl *pinctrl;
	struct pinctrl_state *set_state;
	int retval;
	//int ret = 0;

	printk("%s \n",  __func__);

	node = pdev->dev.of_node;
	if(!node){
		printk("The of_node of device is NULL.\n");
		return -1;
	}
	
	board_id_gpio1 = of_get_named_gpio(node, "qcom,board-id-gpio1", 0);
	if (!gpio_is_valid(board_id_gpio1)) {
		printk("%s board_id_gpio1 is invalid\n", __func__);
		return -EINVAL;
	}

	board_id_gpio2 = of_get_named_gpio(node, "qcom,board-id-gpio2", 0);
	if (!gpio_is_valid(board_id_gpio3)) {
		printk("%s board_id_gpio2 is invalid\n", __func__);
		return -EINVAL;
	}

	board_id_gpio3 = of_get_named_gpio(node, "qcom,board-id-gpio3", 0);
	if (!gpio_is_valid(board_id_gpio3)) {
		printk("%s board_id_gpio3 is invalid\n", __func__);
		return -EINVAL;
	}
	
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_dbg(&pdev->dev, "%s: Unable to get pinctrl handle\n",
			__func__);
		return -EINVAL;
	}

	set_state = pinctrl_lookup_state(pinctrl,
						"id_default");
	if (IS_ERR(set_state)) {
			dev_err(&pdev->dev,
				"cannot get ts pinctrl active state\n");
			return -EINVAL;
	}
	retval = pinctrl_select_state(pinctrl, set_state);
	if (retval) {
		dev_err(&pdev->dev,
				"cannot set ts pinctrl active state\n");
		return retval;
	}
	printk("%s: board id gpio nu = %d %d %d \n", __func__, board_id_gpio1, board_id_gpio2, board_id_gpio3);
	
	return 0;
}

static int board_id_gpio_config(void)
{
	int ret;
	
	ret = gpio_request(board_id_gpio1, "board_id_gpio1");
	if (ret < 0) {
		printk("Failed to request board_id_gpio1 %d, err %d\n",
				board_id_gpio1, ret);
		return -1;
	}
	
	ret = gpio_direction_input(board_id_gpio1);
	if (ret) {
		printk("%s Failed to set gpio direction gpio = %d \n", __func__, board_id_gpio1);
		gpio_free(board_id_gpio1);
		return -1;
	}

	ret = gpio_request(board_id_gpio2, "board_id_gpio2");
	if (ret < 0) {
		printk("Failed to request board_id_gpio2 %d, err %d\n",
				board_id_gpio2, ret);
		return -1;
	}

	ret = gpio_direction_input(board_id_gpio2);
	if (ret) {
		printk("%s Failed to set gpio direction gpio = %d \n", __func__, board_id_gpio2);
		gpio_free(board_id_gpio2);
		return -1;
	}

	ret = gpio_request(board_id_gpio3, "board_id_gpio3");
	if (ret < 0) {
		printk("Failed to request board_id_gpio3 %d, err %d\n",
				board_id_gpio3, ret);
		return -1;
	}

	ret = gpio_direction_input(board_id_gpio3);
	if (ret) {
		printk("%s Failed to set gpio direction gpio = %d \n", __func__, board_id_gpio3);
		gpio_free(board_id_gpio3);
		return -1;
	}
	gpio1_status = gpio_get_value(board_id_gpio1);
	gpio2_status = gpio_get_value(board_id_gpio2);
	gpio3_status = gpio_get_value(board_id_gpio3);

	printk("%s: gpio status = %d %d %d\n", __func__, gpio1_status, gpio2_status, gpio3_status);

	//gpio_free(board_id_gpio1);
	//gpio_free(board_id_gpio2);
	//gpio_free(board_id_gpio3);

	return 0;
}

int get_board_version(void)
{
	printk("%s %d %d %d\n", __func__, gpio1_status, gpio2_status, gpio3_status);
	if (gpio1_status == 1 && gpio2_status ==0 && gpio3_status == 0)
		return BOARD_PIO04;
	else if (gpio1_status == 0 && gpio2_status ==1 && gpio3_status == 0)
		return BOARD_PIO05;
	else
		return BAORD_OTHERS;
}

EXPORT_SYMBOL(get_board_version);



static int board_id_probe(struct platform_device *pdev)
{
	int ret;

	ret = board_id_data_parse_dts_file(pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "%s: Error in get dts data \n", __func__);
		return ret;
	}

	ret = board_id_gpio_config();
	if (ret != 0) {
		dev_err(&pdev->dev, "%s: Failed to config gpio \n", __func__);
		return ret;
	}

	return 0;
}

static int board_id_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mercury_board_id_dt_match[] = {
	{.compatible = "qcom,mercury-board-id"},
	{},
};
MODULE_DEVICE_TABLE(of, mercury_board_id_dt_match);

static struct platform_driver board_id_driver = {
	.driver = {
		.name = "mercury-board-id",
		.owner = THIS_MODULE,
		.of_match_table = mercury_board_id_dt_match,
	},
	.probe = board_id_probe,
	.remove = board_id_remove,
};

static int __init mercury_board_id_init(void)
{
	int rc;

	printk("mercury_board_id_init\n");
	rc = platform_driver_register(&board_id_driver);
	if (rc) {
		pr_err("%s: Failed to register mercury_board_id driver\n",
			__func__);
		return rc;
	}

	return 0;
}

arch_initcall(mercury_board_id_init);

