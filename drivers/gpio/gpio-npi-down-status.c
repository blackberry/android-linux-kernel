#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
//#include <linux/device.h>


//npi_down_gpio		902+69 //VDF

/*  add npi down gpio in dtsi file and use function "of_get_named_gpio" get the gpio number, can't use gpio index directly ,
because in kener vision 3.10 gpio_number=base(8916 8936 8939 is 902)+gpio_index , the base value we can see in "gpio_lib.conf" file .  TCTNB  ZXZ add in 2014/12/13 */

static int npi_down_gpio;

static char npi_down_status[4]={0};

int npi_down_status_detect(void)
{
	int status;

/* remove gpio_request and gpio_free,GPIO69 is used by usb_otg and npi down status,
so gpio_request will fail here, we not request and get gpio status directly*/
	#if 0
	int rc = 0;
	rc = gpio_request(npi_down_gpio, "npi down status");

		if (rc) {
			pr_err("npi down status request gpio=%d failed, rc=%d\n", npi_down_gpio,rc);
			goto err_gpio;
		}

		rc = gpio_direction_input(npi_down_gpio);
		if (rc) {
			pr_err("set_direction for gpio=%d failed, rc=%d\n",npi_down_gpio,rc);
			goto err_gpio;
		}
	#endif

		status=gpio_get_value(npi_down_gpio);

	#if 0
		gpio_free(npi_down_gpio);
	#endif

		if(status)
		strcpy(npi_down_status,"1");
		else
		strcpy(npi_down_status,"0");

		return 0;


	#if 0
	err_gpio:
			gpio_free(npi_down_gpio);
		return  -ENODEV;
	#endif

}


static ssize_t npi_down_status_read(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int err;

	err=npi_down_status_detect();

	if(err == -ENODEV) {
		pr_err("npi down status read error!\n");
	}
	return sprintf(buf, "%s\n", npi_down_status);
}


/* npi_down_status value attribute (/<sysfs>/class/npi_down_status/status) */
static struct class_attribute npi_down_status_value =
	__ATTR(status, 0444, npi_down_status_read, NULL);


static int npi_down_status_creat_file(void)
{
	int ret;
	struct class *npi_down_class;

	/* npi_down_status create (/<sysfs>/class/npi_down_status) */
	npi_down_class = class_create(THIS_MODULE, "npi_down_status");
	if (IS_ERR(npi_down_class)) {
		ret = PTR_ERR(npi_down_class);
		printk(KERN_ERR "npi_down_class: couldn't create npi_down_status\n");
	}
	ret = class_create_file(npi_down_class, &npi_down_status_value);
	if (ret) {
		printk(KERN_ERR "npi_down_status: couldn't create npi_down_status_value\n");
	}

	return 0;

}


static int npi_down_status_probe(struct platform_device *pdev)
{
	int rc = 0;

	npi_down_gpio=of_get_named_gpio(pdev->dev.of_node,
		"qcom,npi-down-gpio", 0);
	
	rc=npi_down_status_creat_file();

	return rc;
}


static int npi_down_status_remove(struct platform_device *pdev)
{
	#if 0
	gpio_free(npi_down_gpio);
	#endif

	return 0;
}


static const struct of_device_id npi_down_dt_match[] = {
	{.compatible = "qcom,npi-down-status"},
	{}
};

MODULE_DEVICE_TABLE(of, npi_down_dt_match);

static struct platform_driver npi_down_status_driver = {
	.probe = npi_down_status_probe,
	.remove = npi_down_status_remove,
	.shutdown = NULL,
	.driver = {
		.name = "npi_down_status",
		.of_match_table = npi_down_dt_match,
	},
};

static int npi_down_register_driver(void)
{
	return platform_driver_register(&npi_down_status_driver);
}



static int __init npi_down_status_init(void)
{

	int ret;

	ret = npi_down_register_driver();
	if (ret) {
		pr_err("npi_down_register_driver() failed!\n");
		return ret;
	}

	return ret;

}

module_init(npi_down_status_init);

static void __exit npi_down_status_exit(void)
{
}
module_exit(npi_down_status_exit);

MODULE_DESCRIPTION("Get NPI down status");
MODULE_LICENSE("GPL v2");

