#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
/*[Defect]-ADD-BEGIN by TCTSH.Cedar, 1595246, 2016/2/16, Add debounce timer for vr*/
#include <linux/jiffies.h>
#include <linux/timer.h>
/*[Defect]-ADD-END   by TCTSH.Cedar, 1595246, 2016/2/16, Add debounce timer for vr*/
#include "hall.h"
#include <linux/interrupt.h>
/*[FEATURE]-Modified-BEGIN by TCTSH.xingchen.wang for task 1238223, 2015/12/25, add TP cover and vr funtion*/
#ifdef CONFIG_TCT_8X76_IDOL4
#define TP_FT5436_LEATHER_COVER
#define TP_FT5436_VR_GLASS
#endif
#ifdef CONFIG_TOUCHSCREEN_GT5688_IDOL4
/*[PLATFORM]-Add-BEGIN by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
#define TP_GT5688_VR_GLASS
/*[PLATFORM]-Add-END by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
#endif
#ifdef TP_FT5436_LEATHER_COVER
extern void ft5x06_disable_leather_cover(void);
extern void ft5x06_enable_leather_cover(void);
#endif

#ifdef TP_FT5436_VR_GLASS
extern void ft5x06_disable_vr(void);
extern void ft5x06_enable_vr(void);
#endif
/*[PLATFORM]-Add-BEGIN by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
#ifdef TP_GT5688_VR_GLASS
extern void gt5688_disable_vr(void);
extern void gt5688_enable_vr(void);
#endif
/*[PLATFORM]-Add-END by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
/*[FEATURE]-Modified-END by TCTSH.xingchen.wang for task 1238223, 2015/12/25*/

struct input_dev *hall_input = NULL;
struct hall_data *hall = NULL;
static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;

/*[FEATURE]-Modified-BEGIN by TCTNB.QW,2015/11/04, Add a parameter for hall module*/
static unsigned hall_key = 250;
module_param(hall_key, uint, S_IRUGO);
/*[FEATURE]-Modified-END by TCTNB.QW,2015/11/04, Add a parameter for hall module*/

/*[FEATURE]-Modified-BEGIN by TCTSH.Cedar, 526254, 2015/10/22, re write codes here to match idol4*/
void report_hall_boot_state(void)
{
	unsigned int gpio_code = 0;

	gpio_code = gpio_get_value(hall->irq_gpio_cover)? KEY_UNLOCK_COVER : KEY_LOCK_LED_COVER;
	hall->hall_cover_state = gpio_code;
	hall_key = gpio_code;//Add a parameter for hall module
	input_report_key(hall_input, gpio_code, 1);
	input_sync(hall_input);
	input_report_key(hall_input, gpio_code, 0);
	input_sync(hall_input);

	gpio_code = gpio_get_value(hall->irq_gpio_vr)? KEY_VR_REMOVED : KEY_VR_DETECTED;
	hall->hall_vr_state = gpio_code;
	input_report_key(hall_input, gpio_code, 1);
	input_sync(hall_input);
	input_report_key(hall_input, gpio_code, 0);
	input_sync(hall_input);
}
/*[FEATURE]-Add-END   by TCTSH.Cedar, 526254, 2015/10/22, re write codes here to match idol4*/

static int hall_power_set(struct hall_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->pdev->dev,"Regulator vdd disable failed rc=%d\n", rc);
			goto out;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->pdev->dev,"Regulator vdd enable failed rc=%d\n", rc);
			goto out;
		}

		data->power_enabled = true;
		usleep(50000);
		return rc;
	} else {
		dev_warn(&data->pdev->dev,"Power on=%d. enabled=%d\n",on, data->power_enabled);
		return rc;
	}

out:
	return rc;
}

static int hall_power_init(struct hall_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				HALL_VDD_MAX_UV);

		regulator_put(data->vdd);
	} else {
		data->vdd = regulator_get(&data->pdev->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->pdev->dev,"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				HALL_VDD_MIN_UV, HALL_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->pdev->dev,"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static void hall_cover_function(struct work_struct *work)
{
	unsigned int gpio_code = 0;
	gpio_code = gpio_get_value(hall->irq_gpio_cover)? KEY_UNLOCK_COVER : KEY_LOCK_LED_COVER;
	hall->hall_cover_state = gpio_code;
	hall_key = gpio_code;			// Add a parameter for hall module
	input_report_key(hall_input, gpio_code, 1);
	input_sync(hall_input);
	input_report_key(hall_input, gpio_code, 0);
	input_sync(hall_input);
/*[FEATURE]-Modified-BEGIN by TCTSH.xingchen.wang for task 1238223, 2015/12/25, add TP cover funtion*/
#ifdef TP_FT5436_LEATHER_COVER
	if (gpio_code == KEY_UNLOCK_COVER)
		ft5x06_disable_leather_cover();
	if (gpio_code == KEY_LOCK_LED_COVER)
		ft5x06_enable_leather_cover();
#endif
/*[FEATURE]-Modified-END by TCTSH.xingchen.wang for task 1238223, 2015/12/25*/
}

/*[Defect]-ADD-BEGIN by TCTSH.Cedar, 1595246, 2016/2/16, change irq type and add debounce for vr*/
static void hall_vr_debounce_timeout_handle(unsigned long data)
{
	/*[FEATURE]-Modified-BEGIN by TCTSH.Cedar, 526254, 2015/10/22, for VR device detection*/
	unsigned int gpio_code = gpio_get_value(hall->irq_gpio_vr)? KEY_VR_REMOVED : KEY_VR_DETECTED;
	/*[FEATURE]-Modified-END   by TCTSH.Cedar, 526254, 2015/10/22, for VR device detection*/

	printk("%s hall->hall_vr_state=%d; gpio_code=%d\n",__func__, hall->hall_vr_state, gpio_code);
	//del_timer(&hall->vr_debounce_timer);

	if(hall->hall_vr_state != gpio_code)
	{
		hall->hall_vr_state = gpio_code;
		input_report_key(hall_input, gpio_code, 1);
		input_sync(hall_input);
		input_report_key(hall_input, gpio_code, 0);
		input_sync(hall_input);
		/*[FEATURE]-Modified-BEGIN by TCTSH.xingchen.wang for task 1238223, 2015/12/25, add TP cover funtion*/
		#ifdef TP_FT5436_VR_GLASS
		if (gpio_code == KEY_VR_REMOVED)
			ft5x06_disable_vr();
		if (gpio_code == KEY_VR_DETECTED)
			ft5x06_enable_vr();
		#endif
/*[PLATFORM]-Add-BEGIN by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
                #ifdef TP_GT5688_VR_GLASS
               if (gpio_code == KEY_VR_REMOVED)
		{	gt5688_disable_vr();
                   printk("\n\n -----%s,%d,disable vr\n",__func__,__LINE__);  
                }   
		if (gpio_code == KEY_VR_DETECTED)
		{	gt5688_enable_vr();
                   printk("\n\n -----%s,%d,enable vr\n",__func__,__LINE__); 
                }
               #endif
/*[PLATFORM]-Add-END by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
		/*[FEATURE]-Modified-END by TCTSH.xingchen.wang for task 1238223, 2015/12/25*/
	}
}

static void hall_vr_function(struct work_struct *work)
{
	printk("%s start debounce timer\n",__func__);
	//del_timer(&hall->vr_debounce_timer);
	mod_timer(&hall->vr_debounce_timer, jiffies + msecs_to_jiffies(HALL_VR_DEBOUNCE_PERIOD));
}

irqreturn_t interrupt_hall_irq(int irq, void *dev)
{
	unsigned int irq_type = IRQ_TYPE_LEVEL_LOW;
	printk("\n\n -----%s,%d,irq_gpio_cover = %d\n\n",__func__,__LINE__,gpio_get_value(hall->irq_gpio_cover));

	if(gpio_get_value(hall->irq_gpio_cover))
	{
		irq_type = IRQ_TYPE_LEVEL_LOW;
	}
	else
	{
		irq_type = IRQ_TYPE_LEVEL_HIGH;
	}
	irq_set_irq_type(gpio_to_irq(hall->irq_gpio_cover), irq_type);

	queue_work(hall->hall_wq, &hall->hall_cover_work);

	return IRQ_HANDLED;
}
irqreturn_t interrupt_hall_irq_sub(int irq, void *dev)
{
	unsigned int irq_type = IRQ_TYPE_LEVEL_LOW;
	printk("\n\n -----%s,%d,irq_gpio_vr = %d\n\n",__func__,__LINE__,gpio_get_value(hall->irq_gpio_vr));

	if(gpio_get_value(hall->irq_gpio_vr))
	{
		irq_type = IRQ_TYPE_LEVEL_LOW;
	}
	else
	{
		irq_type = IRQ_TYPE_LEVEL_HIGH;
	}
	irq_set_irq_type(gpio_to_irq(hall->irq_gpio_vr), irq_type);

	queue_work(hall->hall_wq, &hall->hall_vr_work);
	return IRQ_HANDLED;
}
/*[Defect]-Modified-END   by TCTSH.Cedar, 1595246, 2016/2/16, change irq type and add debounce for vr*/

static int hall_pinctrl_init(struct hall_data *hall, struct platform_device *pdev)
{
	hall->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(hall->pinctrl)) {
		dev_err(&pdev->dev, "Failed to get pinctrl\n");
		return PTR_ERR(hall->pinctrl);
	}
	hall->pin_default = pinctrl_lookup_state(hall->pinctrl, "default");
	if (IS_ERR_OR_NULL(hall->pin_default)) {
		dev_err(&pdev->dev, "Failed to look up default state\n");
		return PTR_ERR(hall->pin_default);
	}
	return 0;
}

static ssize_t hall_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf ( buf, "hall_cover_state:%d hall_vr_state:%d \n",hall->hall_cover_state,hall->hall_vr_state);
}

static DEVICE_ATTR(hall_state, 0444, hall_state_show, NULL);

int hall_probe(struct platform_device *pdev)
{
	int rc = 0;
	unsigned int irq_type = IRQ_TYPE_LEVEL_LOW;
	struct device_node *node = pdev->dev.of_node;

	hall = devm_kzalloc(&pdev->dev, sizeof(struct hall_data),
				 GFP_KERNEL);
	if (hall == NULL)
	{
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	if (!hall_pinctrl_init(hall, pdev)) {
		rc = pinctrl_select_state(hall->pinctrl, hall->pin_default);
		if (rc) {
			dev_err(&pdev->dev, "Can't select pinctrl state\n");
			goto error;
		}
	}

	hall->irq_gpio_cover = of_get_named_gpio(node, "interrupt-gpios", 0);
	hall->irq_gpio_vr = of_get_named_gpio(node, "interrupt-gpios", 1);
	
	if (hall->irq_gpio_cover < 0)
	{
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"interrupt-gpios", node->full_name, hall->irq_gpio_cover);
		goto error;
	}
	else
	{
		rc = gpio_request(hall->irq_gpio_cover, "HALL_INTERRUPT");
		if (rc)
		{
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, hall->irq_gpio_cover, rc);

			goto error;
		}
		rc = gpio_direction_input(hall->irq_gpio_cover);
		if (rc)
		{
			dev_err(&pdev->dev, "Unable to set direction for irq gpio [%d]\n",
				hall->irq_gpio_cover);
			gpio_free(hall->irq_gpio_cover);
			goto error;
		}
	}

	if (hall->irq_gpio_vr > 0)
	{
		rc = gpio_request(hall->irq_gpio_vr, "HALL_INTERRUPT_SUB");
		if (rc)
		{
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, hall->irq_gpio_vr, rc);
			gpio_free(hall->irq_gpio_cover);
			goto error;
		}
		rc = gpio_direction_input(hall->irq_gpio_vr);
		if (rc)
		{
			dev_err(&pdev->dev, "Unable to set direction for irq gpio [%d]\n",
				hall->irq_gpio_vr);
			goto exit_free_gpio;
		}
	}

	hall->pdev = pdev;
	dev_set_drvdata(&pdev->dev, hall);

	hall_power_init(hall, 1);
	hall_power_set(hall, 1);
	hall_input = input_allocate_device();
	if (!hall_input)
	{
		dev_err(&pdev->dev, "hall.c: Not enough memory\n");
		return -ENOMEM;
	}

	hall_input->name = "hall";
	input_set_capability(hall_input, EV_KEY, KEY_UNLOCK_COVER);
	input_set_capability(hall_input, EV_KEY, KEY_LOCK_LED_COVER);
	/*[FEATURE]-Modified-BEGIN by TCTSH.Cedar, 526254, 2015/10/22, for VR device detection*/
	if (hall->irq_gpio_vr > 0)
	{
	input_set_capability(hall_input, EV_KEY, KEY_VR_REMOVED);
	input_set_capability(hall_input, EV_KEY, KEY_VR_DETECTED);
  }
	/*[FEATURE]-Modified-END   by TCTSH.Cedar, 526254, 2015/10/22, for VR device detection*/

	rc = input_register_device(hall_input);
	if (rc)
	{
		dev_err(&pdev->dev, "hall.c: Failed to register device\n");
		return rc;
	}
	hall->hall_wq = create_singlethread_workqueue("hall_wq");
	INIT_WORK(&hall->hall_cover_work, hall_cover_function);
//	hall->hall_wq = create_singlethread_workqueue("hall_wq_sub");
	device_init_wakeup(&pdev->dev,1);	//Add by TCTNB.qw for enable device as a wakeup device.
	enable_irq_wake(gpio_to_irq(hall->irq_gpio_cover));
	if (hall->irq_gpio_vr > 0)
	{
		INIT_WORK(&hall->hall_vr_work, hall_vr_function);
		enable_irq_wake(gpio_to_irq(hall->irq_gpio_vr));
	}
	/* Modify  BEGIN by TCTNB.qw  after resource get ready then request irq   2015/11/18*/
	/*[Defect]-Modified-BEGIN   by TCTSH.Cedar, 1595246, 2016/2/16, change irq type and add debounce for vr*/
	report_hall_boot_state();

	if(gpio_get_value(hall->irq_gpio_cover))
	{
		irq_type = IRQ_TYPE_LEVEL_LOW;
	}
	else
	{
		irq_type = IRQ_TYPE_LEVEL_HIGH;
	}
	rc = request_irq(gpio_to_irq(hall->irq_gpio_cover), interrupt_hall_irq, irq_type, "hall_cover", &pdev->dev);
	if(rc < 0)
	{
		dev_err(&pdev->dev, "%s:request_irq failed\n",__func__);
		goto exit_free_gpio;
	}

	if (hall->irq_gpio_vr > 0)
	{
		if(gpio_get_value(hall->irq_gpio_vr))
		{
			irq_type = IRQ_TYPE_LEVEL_LOW;
		}
		else
		{
			irq_type = IRQ_TYPE_LEVEL_HIGH;
		}
		rc = request_irq(gpio_to_irq(hall->irq_gpio_vr), interrupt_hall_irq_sub, irq_type, "hall_vr", &pdev->dev);
		if(rc < 0)
		{
			goto exit_free_irq;
		}
		setup_timer(&hall->vr_debounce_timer, hall_vr_debounce_timeout_handle, (unsigned long)pdev);
	}
	/*[Defect]-Modified-END   by TCTSH.Cedar, 1595246, 2016/2/16, change irq type and add debounce for vr*/
	/* Modify  END by TCTNB.qw  after resource get ready then request irq   2015/11/18*/
	hall_class= class_create(THIS_MODULE, "hall");
	    if (IS_ERR(hall_class))
        dev_err(&pdev->dev,"Failed to create class(hall_class)!\n");
	hall_dev = device_create(hall_class, NULL, 0, NULL, "hall_device");
	    if (IS_ERR(hall_dev))
        dev_err(&pdev->dev, "Failed to create device(hall_dev)!\n");
		if (device_create_file(hall_dev, &dev_attr_hall_state) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_hall_state.attr.name);
	printk("\n\n ***##xx##*** hall probe completed \n\n");
	return 0;
exit_free_irq:
	free_irq(gpio_to_irq(hall->irq_gpio_cover),NULL);
exit_free_gpio:
	gpio_free(hall->irq_gpio_cover);
	gpio_free(hall->irq_gpio_vr);

error:
	devm_kfree(&pdev->dev, hall);

	return rc;
}

int hall_remove(struct platform_device *pdev)
{
	hall = platform_get_drvdata(pdev);
	/*[Defect]-ADD-BEGIN by TCTSH.Cedar, 1595246, 2016/2/16, Add debounce timer for vr*/
	del_timer_sync(&hall->vr_debounce_timer);
	/*[Defect]-ADD-END   by TCTSH.Cedar, 1595246, 2016/2/16, Add debounce timer for vr*/

	free_irq(gpio_to_irq(hall->irq_gpio_cover),NULL);
	free_irq(gpio_to_irq(hall->irq_gpio_vr),NULL);
	input_unregister_device(hall_input);
	if (hall_input)
	{
		input_free_device(hall_input);
		hall_input = NULL;
	}
	if (gpio_is_valid(hall->irq_gpio_cover))
		gpio_free(hall->irq_gpio_cover);
	if (gpio_is_valid(hall->irq_gpio_vr))
		gpio_free(hall->irq_gpio_vr);
	return 0;
}

static struct of_device_id hall_of_match[] = {
	{.compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		   .name = "qcom,hall",
		   .owner = THIS_MODULE,
		   .of_match_table = hall_of_match,
	}
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __init hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");

