/*
 * Copyright (C) 2014 BlackBerry Limited
 * Copyright (C) 2012 Broadcom Corporation.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/async.h>
#include <linux/regulator/consumer.h>
#include "bcm2079x.h"

/* do not change below */
#define MAX_BUFFER_SIZE         780

/* Read data */
#define PACKET_HEADER_SIZE_NCI  (4)
#define PACKET_HEADER_SIZE_HCI  (3)
#define PACKET_TYPE_NCI         (16)
#define PACKET_TYPE_HCIEV       (4)
#define MAX_PACKET_SIZE         (PACKET_HEADER_SIZE_NCI + 255)

typedef struct {
	wait_queue_head_t   read_wq;
	struct mutex        read_mutex;
	int                 read_mutex_ready;
	struct i2c_client  *client;
	int                 client_irq_ready;
	struct miscdevice   device;
	int                 device_ready;
	struct clk         *clk_src;
	int                 clk_src_ready;
	int                 irq_gpio;           /* NFC_INT */
	int                 irq_gpio_ready;
	enum of_gpio_flags  irq_flag;
	bool                irq_enabled;
	spinlock_t          irq_enabled_lock;
	unsigned int        irq_count;
	int                 ena_gpio;           /* NFC_RESET */
	int                 ena_gpio_ready;
	enum of_gpio_flags  ena_flag;
	int                 wak_gpio;           /* NFC_WAKE_N */
	int                 wak_gpio_ready;
	enum of_gpio_flags  wak_flag;
	struct regulator   *vddio;              /* VDDIO regulator handle */
	int                 vddio_ready;
	struct wake_lock    wake_lock;
	int                 wake_lock_ready;
	bool                device_opened;
} bcm2079x_device;

static void bcm2079x_init_stat(bcm2079x_device *bcm2079x_dev)
{
	bcm2079x_dev->irq_count = 0;
}

static void bcm2079x_enable_irq(bcm2079x_device *bcm2079x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (!bcm2079x_dev->irq_enabled) {
		bcm2079x_dev->irq_enabled = true;
		enable_irq(bcm2079x_dev->client->irq);
		enable_irq_wake(bcm2079x_dev->client->irq);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

static void bcm2079x_disable_irq(bcm2079x_device *bcm2079x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_enabled) {
		disable_irq_nosync(bcm2079x_dev->client->irq);
		bcm2079x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

/*
 * The alias address 0x79, when sent as a 7-bit address from the host processor
 * will match the first byte (highest 2 bits) of the default client address
 * (0x1FA) that is programmed in bcm20791.
 * When used together with the first byte (0xFA) of the byte sequence below,
 * it can be used to address the bcm20791 in a system that does not support
 * 10-bit address and change the default address to 0x38.
 * the new address can be changed by changing the CLIENT_ADDRESS below if 0x38
 * conflicts with other device on the same i2c bus.
 *
 */
static int change_client_addr(bcm2079x_device *bcm2079x_dev, int addr)
{
	int ret = 0;
	char addr_data[] = {
		0xFA, 0xF2, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2A
	};

	/*
	struct i2c_client *client;
	int i;
	#define ALIAS_ADDRESS     0x79

	client = bcm2079x_dev->client;
	client->addr = ALIAS_ADDRESS;
	client->flags &= ~I2C_CLIENT_TEN;

	addr_data[5] = addr & 0xFF;
	ret = 0;
	for (i = 1; i < sizeof(addr_data) - 1; ++i)
		ret += addr_data[i];
	addr_data[sizeof(addr_data) - 1] = (ret & 0xFF);
	dev_info(&client->dev,
		 "Change client device from (0x%04X) flag = "\
		 "%04x, addr_data[%d] = %02x\n",
		 client->addr, client->flags, sizeof(addr_data) - 1,
		 addr_data[sizeof(addr_data) - 1]);
	ret = i2c_master_send(client, addr_data, sizeof(addr_data));
	if (ret != sizeof(addr_data)) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		dev_info(&client->dev,
			 "Change client device from (0x%04X) flag = "\
			 "%04x, addr_data[%d] = %02x\n",
			 client->addr, client->flags, sizeof(addr_data) - 1,
			 addr_data[sizeof(addr_data) - 1]);
		ret = i2c_master_send(client, addr_data, sizeof(addr_data));
	}
	client->addr = addr_data[5];

	dev_info(&client->dev,
		 "Change client device changed to (0x%04X) flag = %04x, "
		 "ret = %d\n",
		 client->addr, client->flags, ret);
	*/

	return (ret == sizeof(addr_data) ? 0 : -EIO);
}

static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *dev_id)
{
	bcm2079x_device *bcm2079x_dev = dev_id;
	unsigned long flags;

	bcm2079x_disable_irq(bcm2079x_dev);
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if ((bcm2079x_dev->irq_count == 0) && bcm2079x_dev->device_opened)
		wake_lock(&bcm2079x_dev->wake_lock);
	bcm2079x_dev->irq_count++;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	/* Since disable_irq_nosync() is used to disable interrupt, it's
	 * possible to get here after bcm2079x_disable_irq() call, so we only
	 * wake up read_wq while bcm2079x_dev->device_opened is true and
	 * abandon all unexpected interrupts
	 */
	if (bcm2079x_dev->device_opened)
		wake_up(&bcm2079x_dev->read_wq);
	else
		dev_warn(&bcm2079x_dev->client->dev,
			 "abandon unexpected interrupt\n");

	return IRQ_HANDLED;
}

static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);
	unsigned long flags;

	poll_wait(filp, &bcm2079x_dev->read_wq, wait);

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_count > 0)
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
				 size_t count, loff_t *offset)
{
	int ret;
	unsigned char tmp[MAX_BUFFER_SIZE];
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);
	int total;
	int len;

	total = 0;
	len = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bcm2079x_dev->read_mutex);

	if (bcm2079x_dev->irq_count > 0) {
		unsigned long flags;

		/** Read the first 4 bytes to include the length of the NCI or
		*** HCI packet.
		*** NOTE: i2c_master_recv() just returns byte count requested if
		***       no error, but valid byte count of data may be less
		***       than the byte count requested.
		**/
		total = i2c_master_recv(bcm2079x_dev->client, tmp, 4);
		if (total == 4) {
			/** First byte is the packet type
			**/
			switch (tmp[0]) {
			case PACKET_TYPE_NCI:
				len = tmp[PACKET_HEADER_SIZE_NCI-1];
			break;

			case PACKET_TYPE_HCIEV:
				len = tmp[PACKET_HEADER_SIZE_HCI-1];
				if (len == 0)
					total--;/*Since payload is 0, decrement
						  total size (from 4 to 3) */
				else
					len--;/*First byte of payload is in
						tmp[3] already */
			break;

			default:
				len = 0;/*Unknown packet byte */
				dev_warn(&bcm2079x_dev->client->dev,
					 "unknown packet type: 0x%02X\n",
					 tmp[0]);
			break;
			} /* switch*/

			/** make sure full packet fits in the buffer
			**/
			if (len > 0 && (len + total) <= count) {
				/** read the remainder of the packet.
				**/
				ret = i2c_master_recv(bcm2079x_dev->client,
						      tmp+total, len);
				if (ret == len)
					total += len;
			} /* if */
		} /* if */

		spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
		bcm2079x_dev->irq_count--;
		if ((bcm2079x_dev->irq_count == 0) &&
		    wake_lock_active(&bcm2079x_dev->wake_lock))
			wake_unlock(&bcm2079x_dev->wake_lock);
		spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

		bcm2079x_enable_irq(bcm2079x_dev);
	}

	mutex_unlock(&bcm2079x_dev->read_mutex);

	/*
	 * allow call with buf == NULL for cleaning up I2C buffer and releasing
	 * wake up locker in bcm2079x_dev_release() in case NFC irq just handled
	 * by irq routine but data has not been readed yet.
	 *
	 */
	if (buf && (total > 0)) {
		if (total > count) {
			dev_err(&bcm2079x_dev->client->dev,
				"not enough user space (%d byte(s)) for"
				" data (%d byte(s))\n",
				(int)count, total);
			total = -EFAULT;
		} else if (copy_to_user(buf, tmp, total)) {
			dev_err(&bcm2079x_dev->client->dev,
				"failed to copy to user space for total"
				" %d byte(s)\n",
				total);
			total = -EFAULT;
		}
	}

	return total;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *offset)
{
	int ret;
	char tmp[MAX_BUFFER_SIZE];
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&bcm2079x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}

	mutex_lock(&bcm2079x_dev->read_mutex);
	/* Write data */

	ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to write with rc = %d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&bcm2079x_dev->read_mutex);

	return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);

	dev_info(&bcm2079x_dev->client->dev,
		 "Opening '/dev/%s' (%d,%d)\n",
		 filp->f_path.dentry->d_iname, imajor(inode),
		 iminor(inode));

	bcm2079x_init_stat(bcm2079x_dev);

	/* enable interrupt */
	bcm2079x_enable_irq(bcm2079x_dev);
	/* set device_opened to true */
	bcm2079x_dev->device_opened = true;
	/* power on NFC chip */
	gpio_set_value(bcm2079x_dev->ena_gpio, 1);

	dev_info(&bcm2079x_dev->client->dev,
		 "Opened '/dev/%s' (%d,%d)\n",
		 filp->f_path.dentry->d_iname, imajor(inode),
		 iminor(inode));

	return ret;
}

static int bcm2079x_dev_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);
	unsigned long flags;

	dev_info(&bcm2079x_dev->client->dev,
		 "Closing '/dev/%s' (%d,%d)\n",
		 filp->f_path.dentry->d_iname, imajor(inode),
		 iminor(inode));

	/* power off NFC chip to stop interrupt */
	gpio_set_value(bcm2079x_dev->ena_gpio, 0);
	/* set device_opened to false to stop handling interrupt */
	bcm2079x_dev->device_opened = false;

	/* wipe out I2C data from NFC chip and release wake up locker before
	 * close */
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	while (bcm2079x_dev->irq_count > 0) {
		int irq_count = bcm2079x_dev->irq_count;
		spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

		dev_info(&bcm2079x_dev->client->dev,
			 "clean I2C data from NFC chip before closure (%d)\n",
			 irq_count);
		bcm2079x_dev_read(filp, NULL, MAX_BUFFER_SIZE, 0);
		/* wait for possible unread data */
		msleep(100);

		spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	/* disable interrupt */
	bcm2079x_disable_irq(bcm2079x_dev);

	dev_info(&bcm2079x_dev->client->dev,
		 "Closed '/dev/%s' (%d,%d)\n",
		 filp->f_path.dentry->d_iname, imajor(inode),
		 iminor(inode));

	return ret;
}

static long bcm2079x_dev_ioctl(struct file *filp,
			       unsigned int cmd, unsigned long arg)
{
	bcm2079x_device *bcm2079x_dev = container_of(filp->private_data,
						     bcm2079x_device, device);

	switch (cmd) {
	case BCMNFC_READ_FULL_PACKET:
		break;
	case BCMNFC_READ_MULTI_PACKETS:
		break;
	case BCMNFC_CHANGE_ADDR:
		dev_info(&bcm2079x_dev->client->dev,
			 "%s, BCMNFC_CHANGE_ADDR (%x, %lx):\n",
			 __func__, cmd, arg);
		return change_client_addr(bcm2079x_dev, arg);
	case BCMNFC_POWER_CTL:
		gpio_set_value(bcm2079x_dev->ena_gpio, arg);
		break;
	case BCMNFC_WAKE_CTL:
		gpio_set_value(bcm2079x_dev->wak_gpio, arg);
		break;
	default:
		dev_err(&bcm2079x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	return 0;
}

static const struct file_operations bcm2079x_dev_fops = {
	.owner          = THIS_MODULE,
	.open           = bcm2079x_dev_open,
	.release        = bcm2079x_dev_release,
	.llseek         = no_llseek,
	.poll           = bcm2079x_dev_poll,
	.read           = bcm2079x_dev_read,
	.write          = bcm2079x_dev_write,
	.unlocked_ioctl = bcm2079x_dev_ioctl,
	.compat_ioctl   = bcm2079x_dev_ioctl
};

static struct clk *bcm2079x_clock_source(struct device *dev,
					 const char *clk_src_name)
{
	struct clk *ret = NULL;

	if (!strcmp(clk_src_name, "RFCLK1"))
		ret  = clk_get(dev, "ref_clk_rf");
	else if (!strcmp(clk_src_name, "BBCLK2"))
		ret  = clk_get(dev, "ref_clk");

	return ret;
}

static void bcm2079x_free(bcm2079x_device *bcm2079x_dev)
{
	if (bcm2079x_dev->client_irq_ready) {
		if (bcm2079x_dev->irq_count > 0) {
			/* shouldn't be here since the function is called only
			** from prob and remove. When in prob, chip is NOT
			** powered on yet. The remove function is called only
			** when the driver is used as a module, not built in
			** kernel. Still add it here in case of using as module
			** for debug
			**/
			wake_unlock(&bcm2079x_dev->wake_lock);
		}
		free_irq(bcm2079x_dev->client->irq, bcm2079x_dev);
		bcm2079x_dev->client_irq_ready = 0;
	}

	if (bcm2079x_dev->device_ready) {
		misc_deregister(&bcm2079x_dev->device);
		bcm2079x_dev->device_ready = 0;
	}

	if (bcm2079x_dev->wake_lock_ready) {
		wake_lock_destroy(&bcm2079x_dev->wake_lock);
		bcm2079x_dev->wake_lock_ready = 0;
	}

	if (bcm2079x_dev->read_mutex_ready) {
		mutex_destroy(&bcm2079x_dev->read_mutex);
		bcm2079x_dev->read_mutex_ready = 0;
	}

	if (bcm2079x_dev->vddio_ready) {
		regulator_disable(bcm2079x_dev->vddio);
		bcm2079x_dev->vddio_ready = 0;
	}

	if (bcm2079x_dev->vddio) {
		regulator_put(bcm2079x_dev->vddio);
		bcm2079x_dev->vddio = NULL;
	}

	if (bcm2079x_dev->clk_src_ready) {
		clk_disable_unprepare(bcm2079x_dev->clk_src);
		bcm2079x_dev->clk_src_ready = 0;
	}

	if (bcm2079x_dev->clk_src) {
		clk_put(bcm2079x_dev->clk_src);
		bcm2079x_dev->clk_src = NULL;
	}

	if (bcm2079x_dev->wak_gpio_ready) {
		gpio_set_value(bcm2079x_dev->wak_gpio, 0);
		gpio_free(bcm2079x_dev->wak_gpio);
		bcm2079x_dev->wak_gpio_ready = 0;
	}

	if (bcm2079x_dev->ena_gpio_ready) {
		gpio_set_value(bcm2079x_dev->ena_gpio, 0);
		gpio_free(bcm2079x_dev->ena_gpio);
		bcm2079x_dev->ena_gpio_ready = 0;
	}

	if (bcm2079x_dev->irq_gpio_ready) {
		gpio_free(bcm2079x_dev->irq_gpio);
		bcm2079x_dev->irq_gpio_ready = 0;
	}
}

static int bcm2079x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret;
	static bcm2079x_device _bcm2079x_dev;
	bcm2079x_device *bcm2079x_dev = &_bcm2079x_dev;
	const char *clk_src_name = NULL;

	memset(bcm2079x_dev, 0, sizeof(*bcm2079x_dev));

	bcm2079x_dev->client = client;

	if (client->dev.of_node) {
		bcm2079x_dev->irq_gpio =
			of_get_named_gpio_flags(client->dev.of_node,
						"bcm,irq-gpio", 0,
						&bcm2079x_dev->irq_flag);
		if (!gpio_is_valid(bcm2079x_dev->irq_gpio)) {
			dev_err(&client->dev, "invalid 'bcm,irq-gpio'\n");
			return -EINVAL;
		}

		bcm2079x_dev->ena_gpio =
			of_get_named_gpio_flags(client->dev.of_node,
						"bcm,ena-gpio", 0,
						&bcm2079x_dev->ena_flag);
		if (!gpio_is_valid(bcm2079x_dev->ena_gpio)) {
			dev_err(&client->dev, "invalid 'bcm,ena-gpio'\n");
			return -EINVAL;
		}

		bcm2079x_dev->wak_gpio =
			of_get_named_gpio_flags(client->dev.of_node,
						"bcm,wak-gpio", 0,
						&bcm2079x_dev->wak_flag);
		if (!gpio_is_valid(bcm2079x_dev->wak_gpio)) {
			dev_err(&client->dev, "invalid 'bcm,wak-gpio'\n");
			return -EINVAL;
		}

		if (of_property_read_string(client->dev.of_node,
					    "bcm,clk-src",
					    &clk_src_name)) {
			clk_src_name = NULL;
			dev_info(&client->dev, "no 'bcm,clk-src' configured\n");
		}

		/* Get the regulator handle */
		bcm2079x_dev->vddio = regulator_get(&client->dev,
						    "bcm,nfc-vddio");
		if (IS_ERR(bcm2079x_dev->vddio)) {
			dev_err(&client->dev,
				"failed to regulator_get('bcm,nfc-vddio'). "
				"rc=%ld\n",
				PTR_ERR(bcm2079x_dev->vddio));
			bcm2079x_dev->vddio = NULL;
			return -EINVAL;
		}
	} else {
		dev_err(&client->dev, "nfc probe of_node failed\n");
		return -ENODEV;
	}

	ret = gpio_request_one(bcm2079x_dev->irq_gpio, GPIOF_IN, "nfc_int");
	if (ret) {
		dev_err(&client->dev, "failed to configure 'bcm,irq-gpio'\n");
		goto err_exit;
	}
	bcm2079x_dev->irq_gpio_ready = 1;

	ret = gpio_request_one(bcm2079x_dev->ena_gpio, GPIOF_OUT_INIT_LOW,
			       "nfc_ena");
	if (ret) {
		dev_err(&client->dev, "failed to configure 'bcm,ena-gpio'\n");
		goto err_exit;
	}
	bcm2079x_dev->ena_gpio_ready = 1;

	ret = gpio_request_one(bcm2079x_dev->wak_gpio, GPIOF_OUT_INIT_LOW,
			       "nfc_wak");
	if (ret) {
		dev_err(&client->dev, "failed to configure 'bcm,wak-gpio'\n");
		goto err_exit;
	}
	bcm2079x_dev->wak_gpio_ready = 1;

	if (clk_src_name) {
		bcm2079x_dev->clk_src =
			bcm2079x_clock_source(&client->dev, clk_src_name);
		if (IS_ERR_OR_NULL(bcm2079x_dev->clk_src)) {
			dev_err(&client->dev,
				"failed to get clock 'bcm,clk-src'\n");
			bcm2079x_dev->clk_src = NULL;
			goto err_exit;
		}

		ret = clk_prepare_enable(bcm2079x_dev->clk_src);
		if (ret) {
			dev_err(&client->dev,
				"failed to enable clock 'bcm,clk-src'\n");
			goto err_exit;
		}
		bcm2079x_dev->clk_src_ready = 1;
	}

	ret = regulator_enable(bcm2079x_dev->vddio);
	if (ret) {
		dev_err(&client->dev,
			"failed to enable vddio 'bcm,nfc-vddio'\n");
		goto err_exit;
	}
	bcm2079x_dev->vddio_ready = 1;

	gpio_set_value(bcm2079x_dev->ena_gpio, 1);
	msleep(50);
	gpio_set_value(bcm2079x_dev->ena_gpio, 0);
	gpio_set_value(bcm2079x_dev->wak_gpio, 0);

	/* init mutex and queues */
	init_waitqueue_head(&bcm2079x_dev->read_wq);
	mutex_init(&bcm2079x_dev->read_mutex);
	bcm2079x_dev->read_mutex_ready = 1;
	wake_lock_init(&bcm2079x_dev->wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");
	bcm2079x_dev->wake_lock_ready = 1;

	bcm2079x_dev->device.minor = MISC_DYNAMIC_MINOR;
	bcm2079x_dev->device.name = "bcm2079x";
	bcm2079x_dev->device.fops = &bcm2079x_dev_fops;
	ret = misc_register(&bcm2079x_dev->device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_exit;
	}
	bcm2079x_dev->device_ready = 1;

	/* set device_opened to false to abandon any unexpected interrupt until
	** it's opened
	**/
	bcm2079x_dev->device_opened = false;
	spin_lock_init(&bcm2079x_dev->irq_enabled_lock);
	bcm2079x_dev->irq_count = 0;
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
	bcm2079x_dev->irq_enabled = true;
	ret = request_irq(client->irq, bcm2079x_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, bcm2079x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_exit;
	}
	bcm2079x_dev->client_irq_ready = 1;
	bcm2079x_disable_irq(bcm2079x_dev);

	/* set up i2c client data for receiving i2c data from i2c framework */
	i2c_set_clientdata(client, bcm2079x_dev);

	dev_info(&client->dev,
		 "%s, probing bcm2079x driver exited successfully\n", __func__);

	return 0;

err_exit:
	bcm2079x_free(bcm2079x_dev);

	return ret;
}

static int bcm2079x_remove(struct i2c_client *client)
{
	bcm2079x_device *bcm2079x_dev;

	bcm2079x_dev = i2c_get_clientdata(client);
	bcm2079x_free(bcm2079x_dev);

	return 0;
}

static const struct i2c_device_id bcm2079x_id[] = {
	{"bcm2079x", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bcm2079x_id);

static struct of_device_id bcm_match_table[] = {
	{.compatible = "bcm,bcm2079x"},
	{}
};
MODULE_DEVICE_TABLE(of, bcm_match_table);

static struct i2c_driver bcm2079x_driver = {
	.id_table = bcm2079x_id,
	.probe    = bcm2079x_probe,
	.remove   = bcm2079x_remove,
	.driver   = {
		.name           = "bcm2079x",
		.of_match_table = bcm_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init bcm2079x_dev_init(void)
{
	int ret;

	ret = i2c_add_driver(&bcm2079x_driver);

	return ret;
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
	i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
