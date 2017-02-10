#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/qpnp/pin.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pwm.h>
#define FRONT_LED_LOG_TAG "front_led"
#define front_led_info(fmt, ...) \
            printk(KERN_ALERT FRONT_LED_LOG_TAG ": " fmt "\n", ##__VA_ARGS__)

enum led_mode
{
	LED_FLASH,
	LED_TORCH
};
struct led_data{
	struct led_classdev	cdev;
    //struct gpio_config_data	*gpio_cfg;
	struct platform_device *dev;
	int enable_gpio;
	int flash_gpio;
	enum led_mode mode;
};
static void enable_gpio_set(struct led_data *led)
{
	int err = 0;
	front_led_info("led->enable_gpio = %d\n",led->enable_gpio);
	if(led->mode ==LED_FLASH)
	{
		err = gpio_request_one(led->flash_gpio,0,"flash_gpio");
		if(err)
			front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
		gpio_set_value(led->flash_gpio, 1);
        err = gpio_request_one(led->enable_gpio,0,"enable_gpio");
		if(err)
			front_led_info("enable_gpio %d request fail\n",led->enable_gpio);
		gpio_set_value(led->enable_gpio, 1);
	}
	else if(led->mode ==LED_TORCH)
	{
		err = gpio_request_one(led->flash_gpio,0,"flash_gpio");
		if(err)
			front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
		gpio_set_value(led->flash_gpio, 0);
        err = gpio_request_one(led->enable_gpio,0,"enable_gpio");
		if(err)
			front_led_info("enable_gpio %d request fail\n",led->enable_gpio);
		gpio_set_value(led->enable_gpio, 1);
	}
}

static void disable_gpio_set(struct led_data *led)
{
	gpio_set_value(led->flash_gpio, 0);
	gpio_free(led->flash_gpio);
	gpio_set_value(led->enable_gpio, 0);
	gpio_free(led->enable_gpio);
}

static void flash_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
    struct led_data *led;
    led = container_of(led_cdev, struct led_data, cdev);
    led->cdev.brightness = value;
    if(value){
    front_led_info("enter flash_led_set");
    	enable_gpio_set(led);
    }else{
	disable_gpio_set(led);
    }
}
static enum led_brightness flash_led_get(struct led_classdev *led_cdev)
{
	struct led_data *led;

	led = container_of(led_cdev, struct led_data, cdev);

	return led->cdev.brightness;
}

static int parse_dts(struct led_data *led,
		struct device_node *node)
{
    int rc;
    rc = of_property_read_string(node, "linux,name",
			&led->cdev.name);
    front_led_info("name = %s\n",led->cdev.name);
    if (rc < 0) {
	dev_err(&led->dev->dev,"Failure reading led name, rc = %d\n", rc);
    }
    rc = of_property_read_u32(node, "led-mode",
			&led->mode);
    front_led_info("mode = %d\n",led->mode);

    led->enable_gpio = of_get_named_gpio(node, "qcom,enable-gpio", 0);

    front_led_info("enable_gpio = %d\n",led->enable_gpio);

    led->flash_gpio = of_get_named_gpio(node, "qcom,flash-gpio", 0);

    front_led_info("flash_gpio = %d\n",led->flash_gpio);
    rc = of_property_read_string(node, "qcom,default-led-trigger",
				&led->cdev.default_trigger);
    return rc;
}


static int front_flash_led_probe(struct platform_device *pdev)
{
	    struct led_data *led;
	    int rc;
	    struct device_node *node;
	    node = pdev->dev.of_node;
	    led = devm_kzalloc(&pdev->dev,sizeof(struct led_data), GFP_KERNEL);
	    if (!led) {
		   dev_err(&pdev->dev, "Unable to allocate memory\n");
		   return -ENOMEM;
	    }
	    led->dev = pdev;
	    rc = parse_dts(led,node);
	    if(rc<0)
	    {
			dev_err(&pdev->dev, "unable to parse dts, rc=%d\n",rc);
			return rc;
	    }
	    led->cdev.brightness_set    = flash_led_set;
	    led->cdev.brightness_get    = flash_led_get;
	    rc = led_classdev_register(&pdev->dev, &led->cdev);
	    if (rc) {
			dev_err(&pdev->dev, "unable to register led, rc=%d\n",rc);
			return rc;
	    }
	    dev_set_drvdata(&pdev->dev, led);
	    return 0;
}

static int front_flash_led_remove(struct platform_device *pdev)
{
	struct led_data *led;
	led = dev_get_drvdata(&pdev->dev);
	led_classdev_unregister(&led->cdev);

	return 0;
}


static struct of_device_id front_flash_led_match_table[] = {
	{ .compatible = "tcl,front_flash_led",},
	{ },
};

static struct platform_driver front_flash_led_driver = {
	.probe = front_flash_led_probe,
	.remove = front_flash_led_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "front_flash_led",
		.of_match_table = front_flash_led_match_table,
	},
};

module_platform_driver(front_flash_led_driver);
MODULE_LICENSE("GPL v2");
