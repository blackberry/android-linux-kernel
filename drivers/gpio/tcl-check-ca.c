#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

int Is_ca;
bool proto1_check = 0;

struct  tcl_ca_data{
     int board_id0;
     int board_id1;
     int board_id2;
     struct pinctrl *ca_pinctrl;
     struct pinctrl_state *gpio_state_config;
};

static int get_id_value(int id)
{
   int tmp = 0; 
   tmp = gpio_get_value(id);
   return tmp;
}

static bool is_proto1(struct tcl_ca_data  *data)
{
        volatile int id0_value=0;
        volatile int id1_value=0;
        volatile int id2_value=0;
        int tmp;
	id0_value = get_id_value(data->board_id0);
	id1_value = get_id_value(data->board_id1);
	id2_value = get_id_value(data->board_id2);

        tmp = (id0_value<<2) | (id1_value<<1) | (id2_value);
        return (tmp < 3) ? 1 : 0;
}

static int is_ca(struct tcl_ca_data  *data)
{
        volatile int id0_value=0;
        volatile int id1_value=0;
        volatile int id2_value=0;
	id0_value = get_id_value(data->board_id0);
	id1_value = get_id_value(data->board_id1);
	id2_value = get_id_value(data->board_id2);

	if(((id0_value == 0) && (id1_value == 1) && (id2_value == 0)) || 
		((id0_value == 1) && (id1_value == 0) && (id2_value == 1)))
		 return 1;
	else
		 return 0;
}

static ssize_t tcl_ca_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", Is_ca);
}
static ssize_t tcl_ca_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}
static DEVICE_ATTR(tcl_ca, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,tcl_ca_show, tcl_ca_store);


static int tcl_create_sysfs(struct device *dev)
{
    int ret;
    ret = device_create_file(dev, &dev_attr_tcl_ca);
    if (ret){
	 device_remove_file(dev, &dev_attr_tcl_ca);
	 return ret;
    }
    return 0;
}

static void tcl_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_tcl_ca);
}



static int tcl_check_ca_probe(struct platform_device *pdev)
{
	int ret; 
	struct tcl_ca_data  *data;
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	data->ca_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(data->ca_pinctrl)) {
		dev_err(&pdev->dev, "Unable to get pinctrl.\n");
		ret = PTR_ERR(data->ca_pinctrl);
		data->ca_pinctrl = NULL;
		return ret;
	}
	data->gpio_state_config = pinctrl_lookup_state(data->ca_pinctrl,"boardid_config");
	ret = pinctrl_select_state(data->ca_pinctrl, data->gpio_state_config);
	if(ret){
        dev_err(&pdev->dev, "Unable to set pinctrl.\n");
        data->gpio_state_config = NULL;
        return ret;
	}

	data->board_id0 = of_get_named_gpio(pdev->dev.of_node, "tcl,board_id0", 0);
	data->board_id1 = of_get_named_gpio(pdev->dev.of_node, "tcl,board_id1", 0);
	data->board_id2 = of_get_named_gpio(pdev->dev.of_node, "tcl,board_id2", 0);

	Is_ca =  is_ca(data);
	ret = tcl_create_sysfs(&pdev->dev);
	if(ret)
         return ret;
        proto1_check = is_proto1(data);
    return 0;
}

static int tcl_check_ca_remove(struct platform_device *pdev)
{

    tcl_remove_sysfs(&pdev->dev);
	return 0;
}


static struct of_device_id tcl_check_ca_match_table[] = {
	{ .compatible = "tcl,tcl_check_ca",},
	{ },
};

static struct platform_driver tcl_check_ca_driver = {
	.probe = tcl_check_ca_probe,
	.remove = tcl_check_ca_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tcl_check_ca",
		.of_match_table = tcl_check_ca_match_table,
	},
};

module_platform_driver(tcl_check_ca_driver);
MODULE_LICENSE("GPL v2");
