/* Copyright (C) 2017 Tcl Corporation Limited */
/*
    i2c-check.c - i2c-bus driver, char device interface


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

/* ------------------------------------------------------------------------- */

static struct class * i2c_check_class;
static struct device * i2c_dev[10];
static int status[10];
static int index = 0;
#define DEVICE_ATTR0(_name, _mode, _show, _store) \
	struct device_attribute dev_attr0_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR1(_name, _mode, _show, _store) \
	struct device_attribute dev_attr1_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR2(_name, _mode, _show, _store) \
	struct device_attribute dev_attr2_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR3(_name, _mode, _show, _store) \
	struct device_attribute dev_attr3_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR4(_name, _mode, _show, _store) \
	struct device_attribute dev_attr4_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR5(_name, _mode, _show, _store) \
	struct device_attribute dev_attr5_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR6(_name, _mode, _show, _store) \
	struct device_attribute dev_attr6_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR7(_name, _mode, _show, _store) \
	struct device_attribute dev_attr7_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR8(_name, _mode, _show, _store) \
	struct device_attribute dev_attr8_##_name = __ATTR(_name, _mode, _show, _store)
#define DEVICE_ATTR9(_name, _mode, _show, _store) \
	struct device_attribute dev_attr9_##_name = __ATTR(_name, _mode, _show, _store)

static ssize_t i2c_dev0_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[0]);
	return ret;
}

static ssize_t  i2c_dev0_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev1_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[1]);
	return ret;
}

static ssize_t  i2c_dev1_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev2_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[2]);
	return ret;
}

static ssize_t  i2c_dev2_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev3_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[3]);
	return ret;
}

static ssize_t  i2c_dev3_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev4_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[4]);
	return ret;
}

static ssize_t  i2c_dev4_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev5_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[5]);
	return ret;
}

static ssize_t  i2c_dev5_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev6_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[6]);
	return ret;
}

static ssize_t  i2c_dev6_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev7_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[7]);
	return ret;
}

static ssize_t  i2c_dev7_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev8_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[8]);
	return ret;
}

static ssize_t  i2c_dev8_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static ssize_t i2c_dev9_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 50, "%d\n", status[9]);
	return ret;
}

static ssize_t  i2c_dev9_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	unsigned long val = 0;
	ret = kstrtoul(buf, 10, &val);
	return size;
}

static DEVICE_ATTR0(status, 0444, i2c_dev0_status_show, i2c_dev0_status_store);
static DEVICE_ATTR1(status, 0444, i2c_dev1_status_show, i2c_dev1_status_store);
static DEVICE_ATTR2(status, 0444, i2c_dev2_status_show, i2c_dev2_status_store);
static DEVICE_ATTR3(status, 0444, i2c_dev3_status_show, i2c_dev3_status_store);
static DEVICE_ATTR4(status, 0444, i2c_dev4_status_show, i2c_dev4_status_store);
static DEVICE_ATTR5(status, 0444, i2c_dev5_status_show, i2c_dev5_status_store);
static DEVICE_ATTR6(status, 0444, i2c_dev6_status_show, i2c_dev6_status_store);
static DEVICE_ATTR7(status, 0444, i2c_dev7_status_show, i2c_dev7_status_store);
static DEVICE_ATTR8(status, 0444, i2c_dev8_status_show, i2c_dev8_status_store);
static DEVICE_ATTR9(status, 0444, i2c_dev9_status_show, i2c_dev9_status_store);

int i2c_check_status_create(const char *name,int value)
{
	int rc = 0;
	if( name != NULL && index < 10 ){
		if(index == 0){
			i2c_dev[0] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[0]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[0], &dev_attr0_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr0_status.attr.name);
  			status[0] = value;
	}else if(index == 1){
			i2c_dev[1] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[1]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[1], &dev_attr1_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr1_status.attr.name);
  			status[1] = value;
  	}else if(index == 2){
			i2c_dev[2] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[2]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[2], &dev_attr2_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr2_status.attr.name);
  			status[2] = value;
  	}else if(index == 3){
			i2c_dev[3] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[3]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[3], &dev_attr3_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr3_status.attr.name);
  			status[3] = value;
  	}else if(index == 4){
			i2c_dev[4] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[4]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[4], &dev_attr4_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr4_status.attr.name);
  			status[4] = value;
  	}else if(index == 5){
			i2c_dev[5] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[5]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[5], &dev_attr5_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr5_status.attr.name);
  			status[5] = value;
  	}else if(index == 6){
			i2c_dev[6] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[6]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[6], &dev_attr6_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr6_status.attr.name);
  			status[6] = value;
  	}else if(index == 7){
			i2c_dev[7] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[7]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[7], &dev_attr7_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr7_status.attr.name);
  			status[7] = value;
  	}else if(index == 8){
			i2c_dev[8] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[8]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[8], &dev_attr8_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr8_status.attr.name);
  			status[8] = value;
  	}else if(index == 9){
			i2c_dev[9] = device_create(i2c_check_class, NULL, 0, NULL, name);
			if (IS_ERR(i2c_dev[9]))
				pr_err("Failed to create device!\n");
			rc = device_create_file(i2c_dev[9], &dev_attr9_status);
			if ( rc < 0)
				pr_err("Failed to create device file(%s)!\n", dev_attr9_status.attr.name);
  			status[9] = value;
  	}
	index++;
  }
  return 0;
}
EXPORT_SYMBOL(i2c_check_status_create);

void i2c_check_device_register ( void )
{
	i2c_check_class = class_create(THIS_MODULE, "i2c_check");
	if (IS_ERR(i2c_check_class))
		pr_err("Failed to create class(i2c_check_class)!\n");
}

/*
 * module load/unload record keeping
 */
static int __init i2c_check_init(void)
{
	//int res;

	printk(KERN_INFO "i2c_check driver\n");
	i2c_check_device_register();
	return 0;
}

arch_initcall(i2c_check_init);

