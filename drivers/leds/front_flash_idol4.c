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

/*struct gpio_config_data {
	u8	source_sel;
	u8	mode_ctrl;
	u8	vin_ctrl;
	u8  invert;
	u8  pull;
	//bool	enable;
};*/
   
struct qpnp_pin_cfg gpio_en_config =
{
//insert your configuration for your purpose
.mode = QPNP_PIN_MODE_DIG_OUT,
.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT, //chooses DTES1 as src select
.invert = QPNP_PIN_INVERT_ENABLE,
.pull = QPNP_PIN_GPIO_PULL_NO,
.vin_sel = QPNP_PIN_VIN0,
.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
.output_type = QPNP_PIN_OUT_BUF_CMOS,
.master_en = 1, // to enable the GPIO1
};
struct qpnp_pin_cfg gpio_dis_config =
{
//insert your configuration for your purpose
.mode = QPNP_PIN_MODE_DIG_OUT,
.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT, 
.invert = QPNP_PIN_MASTER_DISABLE,
.pull = QPNP_PIN_GPIO_PULL_DN,
.vin_sel = QPNP_PIN_VIN0,
.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
.output_type = QPNP_PIN_OUT_BUF_CMOS,
.master_en = 0,  
};

enum led_mode
{
	LED_PWM,
	LED_FLASH,
	LED_TORCH
};

struct qpnp_pin_cfg pwm_en_config =
{
//insert your configuration for your purpose
.mode = QPNP_PIN_MODE_DIG_OUT,
.src_sel = QPNP_PIN_LV_MV_SEL_DTEST1, 
.pull = QPNP_PIN_GPIO_PULL_NO,
.vin_sel = QPNP_PIN_VIN0,
.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
.output_type = QPNP_PIN_OUT_BUF_CMOS,
.master_en = 1, 
};
/*
struct qpnp_pin_cfg pwm_dis_config =
{
//insert your configuration for your purpose
.mode = QPNP_PIN_MODE_DIG_IN,
.src_sel = QPNP_PIN_LV_MV_SEL_DTEST1, 
.pull = QPNP_PIN_GPIO_PULL_DN,
.vin_sel = QPNP_PIN_VIN0,
.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
.output_type = QPNP_PIN_OUT_BUF_CMOS,
.master_en = 0, 
};
*/
struct led_data{
	struct led_classdev	cdev;
    //struct gpio_config_data	*gpio_cfg;
	struct platform_device *dev;
	int enable_gpio;
	int flash_gpio;
	int pwm_gpio;
	enum led_mode mode;
	struct pwm_device	*pwm_dev;
};

static void enable_gpio_set(struct led_data *led)
{
	//struct qpnp_pin_cfg test_en_config;
	/*test_en_config.mode = 1;
	test_en_config.src_sel = 0;
	test_en_config.vin_sel = 0;
	test_en_config.pull = 5;
	test_en_config.invert = 1;
	test_en_config.master_en = 1;
	test_en_config.output_type = 0;*/
	int err = 0;
	front_led_info("led->enable_gpio = %d\n",led->enable_gpio);
	if(led->mode ==LED_PWM)
	{
		err = gpio_request_one(led->flash_gpio,0,"flash_gpio");
		if(err)
			front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
		gpio_set_value(led->flash_gpio, 0);
                gpio_request_one(led->pwm_gpio,0,"pwm_gpio");
		if(err)
			front_led_info("pwm_gpio %d request fail\n",led->pwm_gpio);
		gpio_set_value(led->pwm_gpio, 0);
		qpnp_pin_config(led->enable_gpio, &gpio_en_config);		
	}
	else if(led->mode ==LED_FLASH)
	{
		err = gpio_request_one(led->flash_gpio,0,"flash_gpio");
		if(err)
			front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
		gpio_set_value(led->flash_gpio, 0);//change for new flash IC by xiaoming.hu@tcl.com
                gpio_request_one(led->pwm_gpio,0,"pwm_gpio");
		if(err)
			front_led_info("pwm_gpio %d request fail\n",led->pwm_gpio);
		gpio_set_value(led->pwm_gpio, 0);//change for new flash IC by xiaoming.hu@tcl.com
		qpnp_pin_config(led->enable_gpio, &gpio_en_config);
	}
	else if(led->mode ==LED_TORCH)
	{
		err = gpio_request_one(led->flash_gpio,0,"flash_gpio");
		if(err)
			front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
		gpio_set_value(led->flash_gpio, 1);//change for new flash IC by xiaoming.hu@tcl.com
                gpio_request_one(led->pwm_gpio,0,"pwm_gpio");
		if(err)
			front_led_info("pwm_gpio %d request fail\n",led->pwm_gpio);//change for new flash IC by xiaoming.hu@tcl.com
		gpio_set_value(led->pwm_gpio, 0);
		qpnp_pin_config(led->enable_gpio, &gpio_dis_config);
	}
}

static void disable_gpio_set(struct led_data *led)
{
/*	struct qpnp_pin_cfg test_dis_config;
	test_dis_config.mode = 0;
	test_dis_config.src_sel = 0;
	test_dis_config.vin_sel = 0;
	test_dis_config.pull = 4;
	test_dis_config.master_en = 0;
	test_dis_config.output_type = 1;*/
	gpio_set_value(led->flash_gpio, 0);
	gpio_free(led->flash_gpio);
	gpio_set_value(led->pwm_gpio, 0);
	gpio_free(led->pwm_gpio);
	qpnp_pin_config(led->enable_gpio, &gpio_dis_config);
}

static void enable_pwm_set(struct led_data *led)
{
	qpnp_pin_config(led->enable_gpio, &pwm_en_config);
}
/*
static void disable_pwm_set(struct led_data *led)
{
	struct qpnp_pin_cfg test_dis_config;
	test_dis_config.mode = 0;
	test_dis_config.src_sel = 0;
	test_dis_config.vin_sel = 0;
	test_dis_config.pull = 4;
	test_dis_config.master_en = 0;
	qpnp_pin_config(led->enable_gpio, &pwm_dis_config);	
}*/


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


static void led_blink(struct led_data *led,int duty_us, int period_us)
{
	int rc;
	rc = pwm_config_us(led->pwm_dev,duty_us,period_us);
	if (rc < 0) {
		dev_err(&led->dev->dev, "Failed to " \
					"configure pwm for new values\n");			
			}
}

static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
{
	struct led_data *led;
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct led_data, cdev);
        ret = gpio_request_one(led->flash_gpio,0,"flash_gpio");
	if(ret)
	    front_led_info("flash_gpio %d request fail\n",led->flash_gpio);
        gpio_set_value(led->flash_gpio, 0);
        ret = gpio_request_one(led->pwm_gpio,0,"pwm_gpio");
	if(ret)
	     front_led_info("pwm_gpio %d request fail\n",led->pwm_gpio);
	gpio_set_value(led->pwm_gpio, 0);
        enable_pwm_set(led);
	pwm_enable(led->pwm_dev);
	led_blink(led,2000000,4000000);

    return count;	
}
static DEVICE_ATTR(blink, 0664, NULL, store_blink);

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

    led->pwm_gpio = of_get_named_gpio(node, "qcom,pwm-gpio", 0);

    front_led_info("pwm_gpio = %d\n",led->pwm_gpio);
    if(led->mode ==LED_PWM)//sun zhangyang add for pwm
	{
           led->pwm_dev = of_pwm_get(node, NULL);     
        }


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
			goto fail_id_check;
	    }
	    led->cdev.brightness_set    = flash_led_set;
	    led->cdev.brightness_get    = flash_led_get;
	    rc = led_classdev_register(&pdev->dev, &led->cdev);
	    if (rc) {
			dev_err(&pdev->dev, "unable to register led, rc=%d\n",rc);
			goto fail_id_check;
	    }
		rc = sysfs_create_file(&led->cdev.dev->kobj, &dev_attr_blink.attr);
		if(rc)
			goto fail_id_check;
	    dev_set_drvdata(&pdev->dev, led);
	    return 0;
	
    fail_id_check:
         led_classdev_unregister(&led->cdev);
    return rc;
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
