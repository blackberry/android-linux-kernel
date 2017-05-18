/*! @file vfsSpiDrv.c
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright (C) 2011-2013 Validity Sensors, Inc.
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation; either version 2
**  of the License, or (at your option) any later version.
**  
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**  
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**  
*/

#include <vfsSpiDrv.h>

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/compat.h>

#include <asm-generic/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

//su_debug
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
//su_debug

#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/types.h>

#include <linux/platform_device.h>

#define VALIDITY_PART_NAME "validity_fingerprint"
#define VFSSPI_WAKE_TIME   (5 * HZ)


/* This orientation is based on the position of the sensor 
 * starting from the Top Direction */
#define VFS_SENSOR_ORIENTATION_URDL	1 /* This is the usual configuation when the sensor is mounted Frontside */ 
#define VFS_SENSOR_ORIENTATION_LURD	2
#define VFS_SENSOR_ORIENTATION_DLUR	3
#define VFS_SENSOR_ORIENTATION_RDLU	4
#define VFS_SENSOR_ORIENTATION_ULDR	5  /* This is the usual configuation when the sensor is mounted backside */
#define VFS_SENSOR_ORIENTATION_LDRU	6
#define VFS_SENSOR_ORIENTATION_DRUL	7
#define VFS_SENSOR_ORIENTATION_RULD	8


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_mutex);
static struct class *vfsspi_device_class;
static int gpio_irq;

#ifdef CONFIG_OF
static struct of_device_id validity_metallica_table[] = {
	{ .compatible = "vfsspi,vfs4300",},
	{ },
};
#else
#define validity_metallica_table NULL
#endif


/*
 * vfsspi_devData - The spi driver private structure
 * @devt:Device ID
 * @vfs_spi_lock:The lock for the spi device
 * @spi:The spi device
 * @device_entry:Device entry list
 * @buffer_mutex:The lock for the transfer buffer
 * @is_opened:Indicates that driver is opened
 * @buffer:buffer for transmitting data
 * @null_buffer:buffer for transmitting zeros
 * @stream_buffer:buffer for transmitting data stream
 * @stream_buffer_size:streaming buffer size
 * @drdy_pin:DRDY GPIO pin number
 * @sleep_pin:Sleep GPIO pin number
 * @user_pid:User process ID, to which the kernel signal
 *	indicating DRDY event is to be sent
 * @signal_id:Signal ID which kernel uses to indicating
 *	user mode driver that DRDY is asserted
 * @current_spi_speed:Current baud rate of SPI master clock
 */
struct vfsspi_device_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfs_spi_lock;
	struct platform_device *spi;
	struct list_head device_entry;
	struct mutex buffer_mutex;
	unsigned int is_opened;
	unsigned char *buffer;
	unsigned char *null_buffer;
	unsigned char *stream_buffer;
	size_t stream_buffer_size;
	unsigned int drdy_pin;
	unsigned int sleep_pin;
#if DO_CHIP_SELECT
	unsigned int cs_pin;
#endif
	int user_pid;
	int signal_id;
	unsigned int current_spi_speed;
	unsigned int is_drdy_irq_enabled;
	unsigned int drdy_ntf_type;
	struct mutex kernel_lock;

//su_debug 
	struct regulator *vdd;
	unsigned int vdd18_en; /* Ldo 1.8V enable GPIO pin number */
	unsigned int vdd33_en; /* Ldo 3.3V enable GPIO pin number */	
//su_debug


    /* wake lock to ensoure fingerprint handled */
    struct wake_lock         wake_lock;
    int                      wake_lock_acquired;
    struct timer_list        wake_unlock_timer;

    /* work queue & worker */
    struct work_struct       irq_worker;
};

#ifdef VFSSPI_32BIT
/*
 * Used by IOCTL compat command:
 *         VFSSPI_IOCTL_RW_SPI_MESSAGE
 *
 * @rx_buffer:pointer to retrieved data
 * @tx_buffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rx_buffer;
	compat_uptr_t tx_buffer;
	unsigned int len;
};
#endif

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_enableIrq(struct vfsspi_device_data *vfsspi_device);

static void vfsspi_wake_unlock(struct vfsspi_device_data *data)
{
    pr_debug("%s: enter\n", __func__);

    if (data->wake_lock_acquired) {
        wake_unlock(&data->wake_lock);
        data->wake_lock_acquired = 0;
    }
}

static void vfsspi_wake_unlock_timer_handler(unsigned long ptr)
{
    struct vfsspi_device_data *data = (struct vfsspi_device_data*)ptr;

    pr_debug("%s: enter\n", __func__);
    vfsspi_enableIrq(data);
    vfsspi_wake_unlock(data);
}

static void vfsspi_wake_lock_delayed_unlock(struct vfsspi_device_data *data)
{
    pr_debug("%s: enter\n", __func__);

    if (!data->wake_lock_acquired) {
        wake_lock(&data->wake_lock);
        data->wake_lock_acquired = 1;
    }
    mod_timer(&data->wake_unlock_timer, jiffies + VFSSPI_WAKE_TIME);
}

static void vfsspi_irq_worker(struct work_struct *arg)
{
    struct vfsspi_device_data *vfsspi_device = container_of(arg, struct vfsspi_device_data, irq_worker);

    vfsspi_wake_lock_delayed_unlock(vfsspi_device);
    vfsspi_sendDrdyNotify(vfsspi_device);
    printk("%s: exit\n", __func__);
}

static int vfsspi_send_drdy_signal(struct vfsspi_device_data *vfsspi_device)
{
	struct task_struct *t;
	int ret = 0;

	pr_debug("vfsspi_send_drdy_signal\n");
	pr_info("vfs Debug : %s ******vfsspi_device->user_pid = %d\n",__func__,vfsspi_device->user_pid);


	if (vfsspi_device->user_pid != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		pr_debug("Searching task with PID=%08x\n",
			vfsspi_device->user_pid);
		t = pid_task(find_pid_ns(vfsspi_device->user_pid, &init_pid_ns),
			     PIDTYPE_PID);
		if (t == NULL) {
			pr_debug("No such pid\n");
			rcu_read_unlock();
			return -ENODEV;
		}
		rcu_read_unlock();
		/* notify DRDY signal to user process */
		ret = send_sig_info(vfsspi_device->signal_id,
				    (struct siginfo *)1, t);
		if (ret < 0)
			pr_err("Error sending signal\n");

	} else {
		pr_err("pid not received yet\n");
	}

	return ret;
}

static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *fPos)
{
	ssize_t               status = 0;
	pr_debug("vfsspi_write\n");
	return status;
}

static ssize_t vfsspi_read(struct file *filp, char __user *buf,
			size_t count, loff_t *fPos)
{
	ssize_t                status    = 0;
	pr_debug("vfsspi_read\n");
	return status;
}

#if 0
static int vfsspi_xfer(struct vfsspi_device_data *vfsspi_device,
			struct vfsspi_ioctl_transfer *tr)
{
	int status = 0;
	pr_debug("vfsspi_xfer\n");
	return status;

} /* vfsspi_xfer */
#endif
static int vfsspi_rw_spi_message(struct vfsspi_device_data *vfsspi_device,
				 unsigned long arg)
{
	pr_debug("vfsspi_rw_spi_message\n");
	return 0;
}

static int vfsspi_set_clk(struct vfsspi_device_data *vfsspi_device,
			  unsigned long arg)
{
	pr_debug("vfsspi_set_clk\n");
	return 0;
}

static int vfsspi_register_drdy_signal(struct vfsspi_device_data *vfsspi_device,
				       unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usr_signal;
	if (copy_from_user(&usr_signal, (void __user *)arg, sizeof(usr_signal)) != 0) {
		pr_err("Failed copy from user.\n");
		return -EFAULT;
	} else {
		vfsspi_device->user_pid = usr_signal.user_pid;
		vfsspi_device->signal_id = usr_signal.signal_id;
	}
	return 0;
}

static irqreturn_t vfsspi_irq(int irq, void *context)
{
	struct vfsspi_device_data *vfsspi_device = context;

	/* Linux kernel is designed so that when you disable
	an edge-triggered interrupt, and the edge happens while
	the interrupt is disabled, the system will re-play the
	interrupt at enable time.
	Therefore, we are checking DRDY GPIO pin state to make sure
	if the interrupt handler has been called actually by DRDY
	interrupt and it's not a previous interrupt re-play */
	if (gpio_get_value(vfsspi_device->drdy_pin) == DRDY_ACTIVE_STATUS) {
		schedule_work(&vfsspi_device->irq_worker);
	}

	return IRQ_HANDLED;
}

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev)
{
    struct task_struct *t;
    struct file *efd_file = NULL;
    struct eventfd_ctx *efd_ctx = NULL;	int ret = 0;

    pr_debug("vfsspi_sendDrdyEventFd\n");

    if (vfsSpiDev->user_pid != 0) {
        rcu_read_lock();
        /* find the task_struct associated with userpid */
        pr_debug("Searching task with PID=%08x\n", vfsSpiDev->user_pid);
        t = pid_task(find_pid_ns(vfsSpiDev->user_pid, &init_pid_ns),
            PIDTYPE_PID);
        if (t == NULL) {
            pr_debug("No such pid\n");
            rcu_read_unlock();
            return -ENODEV;
        }
        efd_file = fcheck_files(t->files, vfsSpiDev->signal_id);
        rcu_read_unlock();

        if (efd_file == NULL) {
            pr_debug("No such efd_file\n");
            return -ENODEV;
        }
        
        efd_ctx = eventfd_ctx_fileget(efd_file);
        if (efd_ctx == NULL) {
            pr_debug("eventfd_ctx_fileget is failed\n");
            return -ENODEV;
        }

        /* notify DRDY eventfd to user process */
        eventfd_signal(efd_ctx, 1);

        /* Release eventfd context */
        eventfd_ctx_put(efd_ctx);
    }

    return ret;
}

static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev)
{
    int ret = 0;
    pr_info("vfs Debug : %s ***1***vfsSpiDev->drdy_ntf_type = %d\n",__func__,vfsSpiDev->drdy_ntf_type);	
    if (vfsSpiDev->drdy_ntf_type == VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD) {
        ret = vfsspi_sendDrdyEventFd(vfsSpiDev);
    } else {
        ret = vfsspi_send_drdy_signal(vfsSpiDev);
    }

    return ret;
}

static int vfsspi_enableIrq(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("vfsspi_enableIrq\n");

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_ENABLE) {
		pr_debug("DRDY irq already enabled\n");
		return -EINVAL;
	}

	enable_irq(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;

        if (enable_irq_wake(gpio_irq)) {
        	pr_err("fail to enable_irq_wake\n");
        	return -3;
        }

	return 0;
}

static int vfsspi_disableIrq(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("vfsspi_disableIrq\n");

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_DISABLE) {
		pr_debug("DRDY irq already disabled\n");
		return -EINVAL;
	}
	disable_irq_nosync(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;

	if (disable_irq_wake(gpio_irq)) {
		pr_err("fail to disable_irq_wake\n");
		return -3;
	}

	return 0;
}
static int vfsspi_set_drdy_int(struct vfsspi_device_data *vfsspi_device,
			       unsigned long arg)
{
	unsigned short drdy_enable_flag;
	if (copy_from_user(&drdy_enable_flag, (void __user *)arg,
			   sizeof(drdy_enable_flag)) != 0) {
		pr_err("Failed copy from user.\n");
		return -EFAULT;
	}

	if (drdy_enable_flag == 0){

			vfsspi_wake_lock_delayed_unlock(vfsspi_device);
			vfsspi_disableIrq(vfsspi_device);
        }
	else {
			vfsspi_enableIrq(vfsspi_device);
			/* Workaround the issue where the system
			  misses DRDY notification to host when
			  DRDY pin was asserted before enabling
			  device.*/
			if (gpio_get_value(vfsspi_device->drdy_pin) ==
				DRDY_ACTIVE_STATUS) {
				vfsspi_sendDrdyNotify(vfsspi_device);
			}
	}
	return 0;
}

static void vfsspi_hardReset(struct vfsspi_device_data *vfsspi_device)
{
	pr_debug("vfsspi_hardReset\n");
	pr_info("vfs Debug : %s *****\n",__func__);
	if (vfsspi_device != NULL) {
		gpio_set_value(vfsspi_device->vdd18_en,1);
		gpio_set_value(vfsspi_device->vdd33_en,1);
                mdelay(5);
		pr_info("vfsspi_hardReset turn on vdd18 and vdd33\n");
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		mdelay(1);
		gpio_set_value(vfsspi_device->sleep_pin, 1);
		mdelay(5);
	}
}

static void vfsspi_suspend(struct vfsspi_device_data *vfsspi_device)
{
	pr_info("vfsspi_suspend\n");

	if (vfsspi_device != NULL) {
		spin_lock(&vfsspi_device->vfs_spi_lock);
		gpio_set_value(vfsspi_device->sleep_pin, 0);
		gpio_set_value(vfsspi_device->vdd18_en,0);
		gpio_set_value(vfsspi_device->vdd33_en,0);
		pr_info("vfsspi_suspend turn off vdd18 and vdd33\n");
		spin_unlock(&vfsspi_device->vfs_spi_lock);
	}
}

static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int ret_val = 0;
	struct vfsspi_device_data *vfsspi_device = NULL;

	pr_debug("vfsspi_ioctl\n");

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		pr_err("invalid magic. cmd=0x%X Received=0x%X Expected=0x%X\n",
			cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		return -ENOTTY;
	}

	vfsspi_device = filp->private_data;
	mutex_lock(&vfsspi_device->buffer_mutex);
	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_RESET:
		pr_debug("VFSSPI_IOCTL_DEVICE_RESET:\n");
		vfsspi_hardReset(vfsspi_device);
		break;
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		pr_debug("VFSSPI_IOCTL_DEVICE_SUSPEND:\n");
		vfsspi_suspend(vfsspi_device);
		break;
	}		
	case VFSSPI_IOCTL_RW_SPI_MESSAGE:
		pr_debug("VFSSPI_IOCTL_RW_SPI_MESSAGE");
		ret_val = vfsspi_rw_spi_message(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_CLK:
		pr_debug("VFSSPI_IOCTL_SET_CLK\n");
		ret_val = vfsspi_set_clk(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
		pr_debug("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL\n");
		ret_val = vfsspi_register_drdy_signal(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_DRDY_INT:
		pr_debug("VFSSPI_IOCTL_SET_DRDY_INT\n");
		ret_val = vfsspi_set_drdy_int(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
        {
            vfsspi_iocSelectDrdyNtfType_t drdyTypes;

            pr_debug("VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE\n");

            if (copy_from_user(&drdyTypes, (void __user *)arg,
                sizeof(vfsspi_iocSelectDrdyNtfType_t)) != 0) {
                    pr_debug("copy from user failed.\n");
                    ret_val = -EFAULT;
            } else {
                if (0 != (drdyTypes.supportedTypes & VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD)) {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
                } else {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
                }
                drdyTypes.selectedType = vfsspi_device->drdy_ntf_type;
                if (copy_to_user((void __user *)arg, &(drdyTypes),
                    sizeof(vfsspi_iocSelectDrdyNtfType_t)) == 0) {
                        ret_val = 0;
                } else {
                    pr_debug("copy to user failed\n");
                }
            }
            break;
        }
	case VFSSPI_IOCTL_GET_SENSOR_ORIENTATION:
	{
		pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION\n");
		if ((void __user *)arg != NULL)
		{
			/* Returning appropriate value based on the current position */
			*((unsigned int *) arg) = VFS_SENSOR_ORIENTATION_LURD;
			ret_val = 0;
		}
		else
		{
			pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION failed\n");
			*((unsigned int *)arg) = 0;
			ret_val = -EFAULT;
		}
		break;
		
	}
	default:
		ret_val = -EFAULT;
		break;
	}

	mutex_unlock(&vfsspi_device->buffer_mutex);
	return ret_val;
}

static int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int status = -ENXIO;

	pr_debug("vfsspi_open\n");

	mutex_lock(&device_list_mutex);

	list_for_each_entry(vfsspi_device, &device_list, device_entry) {
		if (vfsspi_device->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		mutex_lock(&vfsspi_device->kernel_lock);
		if (vfsspi_device->is_opened != 0) {
			status = -EBUSY;
			pr_err("vfsspi_open: is_opened != 0, -EBUSY");
			goto vfsspi_open_out;
		}
		vfsspi_device->user_pid = 0;
        vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
		if (vfsspi_device->buffer != NULL) {
			pr_err("vfsspi_open: buffer != NULL");
			goto vfsspi_open_out;
		}
		vfsspi_device->null_buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->null_buffer == NULL) {
			status = -ENOMEM;
			pr_err("vfsspi_open: null_buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->buffer == NULL) {
			status = -ENOMEM;
			kfree(vfsspi_device->null_buffer);
			pr_err("vfsspi_open: buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->is_opened = 1;
		filp->private_data = vfsspi_device;
		nonseekable_open(inode, filp);

vfsspi_open_out:
		mutex_unlock(&vfsspi_device->kernel_lock);
	}
	mutex_unlock(&device_list_mutex);
	return status;
}


static int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int                   status     = 0;

	pr_debug("vfsspi_release\n");

	mutex_lock(&device_list_mutex);
	vfsspi_device = filp->private_data;
	filp->private_data = NULL;
	vfsspi_device->is_opened = 0;
	if (vfsspi_device->buffer != NULL) {
		kfree(vfsspi_device->buffer);
		vfsspi_device->buffer = NULL;
	}

	if (vfsspi_device->null_buffer != NULL) {
		kfree(vfsspi_device->null_buffer);
		vfsspi_device->null_buffer = NULL;
	}

	mutex_unlock(&device_list_mutex);
	return status;
}

/* file operations associated with device */
static const struct file_operations vfsspi_fops = {
	.owner   = THIS_MODULE,
	.write   = vfsspi_write,
	.read    = vfsspi_read,
	.unlocked_ioctl   = vfsspi_ioctl,
	.open    = vfsspi_open,
	.release = vfsspi_release,
};

static void vfsspi_init_gpio(struct vfsspi_device_data* _vfsspi_device_data){
	struct gpio all_gpio [] = {
		{_vfsspi_device_data->drdy_pin , GPIOF_DIR_IN , "syna_irq_gpio"},
		{_vfsspi_device_data->sleep_pin ,GPIOF_OUT_INIT_HIGH , "syna_reset_gpio"},
		{_vfsspi_device_data->vdd18_en ,GPIOF_OUT_INIT_HIGH  , "syna_vdd18_en"},
		{_vfsspi_device_data->vdd33_en , GPIOF_OUT_INIT_HIGH , "syna_vdd33_en"},
	}; 

	gpio_request_array(all_gpio , ARRAY_SIZE(all_gpio));
}

static void vfsspi_regular(struct vfsspi_device_data *data,bool enable){
	gpio_set_value(data->vdd18_en,1);
	gpio_set_value(data->vdd33_en,1);
	pr_info("vfs Debug : %s ****\n",__func__);					
}
//su_debug

static int vfsspi_parse_dt(struct device *dev,
	struct vfsspi_device_data *data)
{
	struct device_node *np = dev->of_node;
	int errorno = 0;
	int gpio;
	gpio = of_get_named_gpio(np, "vfsspi,vfsspi-sleepPin", 0);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->sleep_pin = gpio;
		printk("%s: sleepPin=%d\n",
			__func__, data->sleep_pin);
	}
	gpio = of_get_named_gpio(np, "vfsspi,vfsspi-drdyPin", 0);
	if (gpio < 0) {
		gpio = errorno;
		goto dt_exit;
	} else {
		data->drdy_pin = gpio;
		printk("%s: drdyPin=%d\n",
			__func__, data->drdy_pin);
	}

	gpio = of_get_named_gpio(np, "vfsspi,vfsspi-vdd18_en", 0);
	if (gpio < 0) {
		gpio = errorno;
		goto dt_exit;
	} else {
		data->vdd18_en = gpio;
		printk("%s: vdd18_en=%d\n",
			__func__, data->vdd18_en);
	}

	gpio = of_get_named_gpio(np, "vfsspi,vfsspi-vdd33_en", 0);
	if (gpio < 0) {
		gpio = errorno;
		goto dt_exit;
	} else {
		data->vdd33_en = gpio;
		printk("%s: vdd33_en=%d\n",
			__func__, data->vdd33_en);
	}

#if 0 
   data->vdd = regulator_get(dev, "vdd_ana");
    printk("data->vdd is %p\n", data->vdd);
	pr_info("vfs Debug : %s ******4****\n",__func__);	
    if (regulator_count_voltages(data->vdd) > 0) {
        errorno = regulator_set_voltage(data->vdd, 1800000,    1800000);
        if (errorno) {
            printk("%s: ----regulator set_vtg vdd_reg failed rc=%d\n", __func__, errorno);
            regulator_put(data->vdd);
            return errorno ;
        }
    }
    errorno = regulator_set_optimum_mode(data->vdd, 6000);
    if (errorno < 0) {
        printk("%s: set_optimum_mode vdd_reg failed, rc=%d\n", __func__, errorno);
        return errorno;
    }
    errorno = regulator_enable(data->vdd);
    if (errorno) {
        printk("%s: ----Regulator vdd_reg enable failed rc=%d\n", __func__, errorno);
        return errorno;
    }
	gpio = of_get_named_gpio(np, "vfsspi-ldoPin", 0);
	if (gpio < 0) {
		data->ldo_pin = 0;
		printk("%s: fail to get ldo_pin\n", __func__);
	} else {
		data->ldo_pin = gpio;
		printk("%s: ldo_pin=%d\n",
			__func__, data->ldo_pin);
	}
		
	printk("VFS Debug : %s: ldo_pin=%d\n",
			__func__, data->ldo_pin);
#endif

#ifdef ENABLE_SENSORS_FPRINT_SECURE
	data->tz_mode = true;
#endif
dt_exit:
	return errorno;
}
//su_debug

static int vfsspi_probe(struct platform_device *spi)
{
	int status = 0;
	struct vfsspi_device_data *vfsspi_device;
	struct device *dev;

	pr_info("vfsspi_probe\n");

	vfsspi_device = kzalloc(sizeof(*vfsspi_device), GFP_KERNEL);

	if (vfsspi_device == NULL)
		return -ENOMEM;
#if 1
	if (spi->dev.of_node) {
		status = vfsspi_parse_dt(&spi->dev, vfsspi_device);
		if (status) {
			printk("%s - Failed to parse DT\n", __func__);
			goto vfsspi_probe_drdy_failed;
		}
	}
#endif
//	vfsspi_device->drdy_pin  = VFSSPI_DRDY_PIN + 879;
//	vfsspi_device->sleep_pin = VFSSPI_SLEEP_PIN + 879;
//	vfsspi_device->vdd18_en = 1001 ;
//	vfsspi_device->vdd33_en = 894 ;
	pr_info("VFS Debug : %s ***vfsspi_device->drdy_pin = %d ,vfsspi_device->sleep_pin = %d,gpio vdd18_en = %d,gpio vdd33_en = %d ***\n",__func__,vfsspi_device->drdy_pin,vfsspi_device->sleep_pin,vfsspi_device->vdd18_en,vfsspi_device->vdd33_en);
	if((!gpio_is_valid(vfsspi_device->drdy_pin)))
		pr_info("VFS Debug : gpio drdy_pin is invald");
	if((!gpio_is_valid(vfsspi_device->sleep_pin)))
		pr_info("VFS Debug : gpio drdy_pin is invald");
	if((!gpio_is_valid(vfsspi_device->vdd18_en)))
		pr_info("VFS Debug : gpio vdd18_en is invald");	
	if((!gpio_is_valid(vfsspi_device->vdd33_en)))
		pr_info("VFS Debug : gpio vdd33_en is invald");					
#if 0
	if (gpio_request(vfsspi_device->drdy_pin, "vfsspi_drdy") < 0) {
		status = -EBUSY;
		goto vfsspi_probe_drdy_failed;
	}
	pr_info("VFS Debug : %s ***2***\n",__func__);
	if (gpio_request(vfsspi_device->sleep_pin, "vfsspi_sleep")) {
		status = -EBUSY;
		goto vfsspi_probe_sleep_failed;
	}
#else
	vfsspi_init_gpio(vfsspi_device);
#endif
	gpio_direction_output(vfsspi_device->vdd18_en,1);
	gpio_direction_output(vfsspi_device->vdd33_en,1);

#if DO_CHIP_SELECT
	pr_debug("HANDLING CHIP SELECT");
	vfsspi_device->cs_pin  = VFSSPI_CS_PIN;
	if (gpio_request(vfsspi_device->cs_pin, "vfsspi_cs") < 0) {
		status = -EBUSY;
		goto vfsspi_probe_cs_failed;
	}
	status = gpio_direction_output(vfsspi_device->cs_pin, 1);
	if (status < 0) {
		pr_err("gpio_direction_input CS failed\n");
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}
	gpio_set_value(vfsspi_device->cs_pin, 1);
#endif
#if 1
	status = gpio_direction_output(vfsspi_device->sleep_pin, 1);
	if (status < 0) {
		pr_err("gpio_direction_output SLEEP failed\n");
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}
#endif
	status = gpio_direction_input(vfsspi_device->drdy_pin);
	if (status < 0) {
		pr_err("gpio_direction_input DRDY failed\n");
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}

	gpio_irq = gpio_to_irq(vfsspi_device->drdy_pin);

	if (gpio_irq < 0) {
		pr_err("gpio_to_irq failed\n");
		status = -EBUSY;
		goto vfsspi_probe_gpio_init_failed;
	}

	if (request_irq(gpio_irq, vfsspi_irq, IRQF_TRIGGER_RISING,
			"vfsspi_irq", vfsspi_device) < 0) {
		pr_err("request_irq failed\n");
		status = -EBUSY;
		goto vfsspi_probe_irq_failed;
	}

//power_on 
	vfsspi_regular(vfsspi_device,1);

	/* Initialize driver data. */
	vfsspi_device->current_spi_speed = SLOW_BAUD_RATE;
	vfsspi_device->spi = spi;

	spin_lock_init(&vfsspi_device->vfs_spi_lock);
	mutex_init(&vfsspi_device->buffer_mutex);
	mutex_init(&vfsspi_device->kernel_lock);

	INIT_LIST_HEAD(&vfsspi_device->device_entry);

    /* init wake lock */
    vfsspi_device->wake_lock_acquired = 0;
    wake_lock_init(&vfsspi_device->wake_lock, WAKE_LOCK_SUSPEND, "fingerprint_wakelock");

    /* init work queue for interrupt */
    INIT_WORK(&vfsspi_device->irq_worker, vfsspi_irq_worker);
    init_timer(&vfsspi_device->wake_unlock_timer);
    vfsspi_device->wake_unlock_timer.expires = jiffies - 1;
    vfsspi_device->wake_unlock_timer.function = vfsspi_wake_unlock_timer_handler;
    vfsspi_device->wake_unlock_timer.data = (unsigned long)vfsspi_device;
    add_timer(&vfsspi_device->wake_unlock_timer);

    vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;

	if (status != 0)
		goto vfsspi_probe_failed;
	mutex_lock(&device_list_mutex);
	/* Create device node */
	/* register major number for character device */
	status = alloc_chrdev_region(&(vfsspi_device->devt),
				     0, 1, VALIDITY_PART_NAME);
	if (status < 0) {
		pr_err("alloc_chrdev_region failed\n");
		goto vfsspi_probe_alloc_chardev_failed;
	}

	cdev_init(&(vfsspi_device->cdev), &vfsspi_fops);
	vfsspi_device->cdev.owner = THIS_MODULE;
	status = cdev_add(&(vfsspi_device->cdev), vfsspi_device->devt, 1);
	if (status < 0) {
		pr_err("cdev_add failed\n");
		unregister_chrdev_region(vfsspi_device->devt, 1);
		goto vfsspi_probe_cdev_add_failed;
	}

	vfsspi_device_class = class_create(THIS_MODULE, "validity_fingerprint");

	if (IS_ERR(vfsspi_device_class)) {
		pr_err("vfsspi_init: class_create() is failed - unregister chrdev.\n");
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);
		status = PTR_ERR(vfsspi_device_class);
		goto vfsspi_probe_class_create_failed;
	}

	dev = device_create(vfsspi_device_class, &spi->dev,
			    vfsspi_device->devt, vfsspi_device, "vfsspi");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0)
		list_add(&vfsspi_device->device_entry, &device_list);
	mutex_unlock(&device_list_mutex);
	if (status != 0)
		goto vfsspi_probe_failed;

	//spi_set_drvdata(spi, vfsspi_device);
	vfsspi_hardReset(vfsspi_device);
	//test(spi);

	pr_info("vfs Debug : %s vfsspi_device->is_opened = %d******\n",__func__,vfsspi_device->is_opened);
	pr_info("vfsspi_probe successful");

	return 0;

vfsspi_probe_failed:
vfsspi_probe_class_create_failed:
	cdev_del(&(vfsspi_device->cdev));
vfsspi_probe_cdev_add_failed:
	unregister_chrdev_region(vfsspi_device->devt, 1);
vfsspi_probe_alloc_chardev_failed:
vfsspi_probe_irq_failed:
	free_irq(gpio_irq, vfsspi_device);
vfsspi_probe_gpio_init_failed:
#if DO_CHIP_SELECT
		gpio_free(vfsspi_device->cs_pin);
vfsspi_probe_cs_failed:
#endif
	gpio_free(vfsspi_device->sleep_pin);
//vfsspi_probe_sleep_failed:
//	gpio_free(vfsspi_device->drdy_pin);
vfsspi_probe_drdy_failed:
	kfree(vfsspi_device);
	mutex_destroy(&vfsspi_device->buffer_mutex);
	mutex_destroy(&vfsspi_device->kernel_lock);
	pr_err("vfsspi_probe failed!!\n");
	return status;
}

static int vfsspi_remove(struct platform_device *spi)
{
	int status = 0;

	struct vfsspi_device_data *vfsspi_device = NULL;

	pr_debug("vfsspi_remove\n");

	vfsspi_device = platform_get_drvdata(spi);

	if (vfsspi_device != NULL) {
		spin_lock_irq(&vfsspi_device->vfs_spi_lock);
		vfsspi_device->spi = NULL;
		//spi_set_drvdata(spi, NULL);
		spin_unlock_irq(&vfsspi_device->vfs_spi_lock);

		mutex_lock(&device_list_mutex);

		free_irq(gpio_irq, vfsspi_device);
#if DO_CHIP_SELECT
		gpio_free(vfsspi_device->cs_pin);
#endif
		gpio_free(vfsspi_device->sleep_pin);
		gpio_free(vfsspi_device->drdy_pin);

		/* Remove device entry. */
		list_del(&vfsspi_device->device_entry);
		device_destroy(vfsspi_device_class, vfsspi_device->devt);
		class_destroy(vfsspi_device_class);
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);

		mutex_destroy(&vfsspi_device->buffer_mutex);
		mutex_destroy(&vfsspi_device->kernel_lock);

		kfree(vfsspi_device);
		mutex_unlock(&device_list_mutex);
	}

	return status;
}

struct platform_driver validity_metallica_driver = {
	.driver = {
		.name  = VALIDITY_PART_NAME,
		.owner = THIS_MODULE,
		.of_match_table = validity_metallica_table,
	},
		.probe  = vfsspi_probe,
		.remove =/*__devexit_p*/(vfsspi_remove),
};

static int __init vfsspi_init(void)
{
	int status = 0;


	pr_debug("vfsspi_init\n");

	status = platform_driver_register(&validity_metallica_driver);
	if (status < 0) {
		pr_err("vfsspi_init: platform_driver_register() is failed - unregister chrdev.\n");
		return status;
	}
	pr_debug("init is successful\n");

	return status;
}

static void __exit vfsspi_exit(void)
{
	pr_debug("vfsspi_exit\n");
	platform_driver_unregister(&validity_metallica_driver);
}

module_init(vfsspi_init);
module_exit(vfsspi_exit);

MODULE_DESCRIPTION("Validity FPS sensor");
MODULE_LICENSE("GPL");
