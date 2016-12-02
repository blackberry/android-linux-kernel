/*
 * Copyright (C) 2015 BlackBerry Limited
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/version.h>

#define BBAUTH_BULK_BUFFER_SIZE           (16 * 1024)

struct bbauth_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	atomic_t online;
	atomic_t error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	wait_queue_head_t io_wq;
	struct usb_request *out_req;
	struct usb_request *in_req;
	int rx_done;
	int tx_done;
};

static struct bbauth_dev *_bbauth_dev;

/*-------------------------------------------------------------------------*/

/* utility functions */

static struct usb_request *bbauth_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void bbauth_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int bbauth_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1)
		return 0;
	else {
		atomic_dec(excl);
		return -EBUSY;
	}
}

static inline void bbauth_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/*-------------------------------------------------------------------------*/

/* for dev node i/o operation */

static ssize_t bbauth_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct bbauth_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	pr_debug("bbauth_read(%d)\n", (int)count);
	if (!_bbauth_dev)
		return -ENODEV;

	if (count > BBAUTH_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (bbauth_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		pr_debug("bbauth_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->io_wq,
			(atomic_read(&dev->online) ||
			atomic_read(&dev->error)));
		if (ret < 0) {
			bbauth_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (atomic_read(&dev->error)) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->out_req;
	req->length = BBAUTH_BULK_BUFFER_SIZE;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		pr_debug("bbauth_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		atomic_set(&dev->error, 1);
		goto done;
	} else
		pr_debug("rx %p queue\n", req);

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->io_wq, dev->rx_done ||
				atomic_read(&dev->error));
	if (ret < 0) {
		if (ret != -ERESTARTSYS)
			atomic_set(&dev->error, 1);
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (!atomic_read(&dev->error)) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		pr_debug("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		r = xfer;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;

	} else
		r = -EIO;

done:

	bbauth_unlock(&dev->read_excl);
	pr_debug("bbauth_read returning %d\n", r);
	return r;
}

static ssize_t bbauth_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct bbauth_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	if (!_bbauth_dev)
		return -ENODEV;
	pr_debug("bbauth_write(%d)\n", (int)count);

	if (bbauth_lock(&dev->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (atomic_read(&dev->error)) {
			pr_debug("bbauth_write dev->error\n");
			r = -EIO;
			break;
		}

		req = dev->in_req;

		if (req != 0) {
			if (count > BBAUTH_BULK_BUFFER_SIZE)
				xfer = BBAUTH_BULK_BUFFER_SIZE;
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			dev->tx_done = 0;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_debug("bbauth_write: xfer error %d\n", ret);
				atomic_set(&dev->error, 1);
				r = -EIO;
				break;
			}

			ret = wait_event_interruptible(dev->io_wq, dev->tx_done ||
				atomic_read(&dev->error));
			if (ret < 0) {
				if (ret != -ERESTARTSYS)
					atomic_set(&dev->error, 1);
				r = ret;
				usb_ep_dequeue(dev->ep_out, req);
				break;
			}

			buf += xfer;
			count -= xfer;
		}
	}

	bbauth_unlock(&dev->write_excl);
	pr_debug("bbauth_write returning %d\n", r);
	return r;
}

static int bbauth_open(struct inode *ip, struct file *fp)
{
	pr_info("bbauth_open\n");

	if (!_bbauth_dev)
		return -ENODEV;

	if (bbauth_lock(&_bbauth_dev->open_excl))
		return -EBUSY;

	fp->private_data = _bbauth_dev;

	/* clear the error latch */
	atomic_set(&_bbauth_dev->error, 0);

	return 0;
}

static int bbauth_release(struct inode *ip, struct file *fp)
{
	pr_info("bbauth_release\n");

	bbauth_unlock(&_bbauth_dev->open_excl);
	return 0;
}

static const struct file_operations bbauth_fops = {
	.owner = THIS_MODULE,
	.read = bbauth_read,
	.write = bbauth_write,
	.open = bbauth_open,
	.release = bbauth_release,
};

static const char bbauth_shortname[] = "usb_bbauth";

static struct miscdevice bbauth_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = bbauth_shortname,
	.fops = &bbauth_fops,
};

/*-------------------------------------------------------------------------*/

/* callbacks */

static void bbauth_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct bbauth_dev *dev = _bbauth_dev;

	dev->tx_done = 1;
	if (req->status != 0)
		atomic_set(&dev->error, 1);

	wake_up(&dev->io_wq);
}

static void bbauth_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct bbauth_dev *dev = _bbauth_dev;

	dev->rx_done = 1;
	if (req->status != 0 && req->status != -ECONNRESET)
		atomic_set(&dev->error, 1);

	wake_up(&dev->io_wq);
}

/*-------------------------------------------------------------------------*/

/* function driver operation */

static struct usb_interface_descriptor bbauth_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x01,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor bbauth_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_ss_ep_comp_descriptor bbauth_superspeed_in_comp_desc = {
	.bLength =		sizeof bbauth_superspeed_in_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor bbauth_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_ss_ep_comp_descriptor bbauth_superspeed_out_comp_desc = {
	.bLength =		sizeof bbauth_superspeed_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor bbauth_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor bbauth_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor bbauth_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor bbauth_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_bbauth_descs[] = {
	(struct usb_descriptor_header *) &bbauth_interface_desc,
	(struct usb_descriptor_header *) &bbauth_fullspeed_in_desc,
	(struct usb_descriptor_header *) &bbauth_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_bbauth_descs[] = {
	(struct usb_descriptor_header *) &bbauth_interface_desc,
	(struct usb_descriptor_header *) &bbauth_highspeed_in_desc,
	(struct usb_descriptor_header *) &bbauth_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_bbauth_descs[] = {
	(struct usb_descriptor_header *) &bbauth_interface_desc,
	(struct usb_descriptor_header *) &bbauth_superspeed_in_desc,
	(struct usb_descriptor_header *) &bbauth_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &bbauth_superspeed_out_desc,
	(struct usb_descriptor_header *) &bbauth_superspeed_out_comp_desc,
	NULL,
};


static int bbauth_create_bulk_endpoints(struct bbauth_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for bb ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	req = bbauth_request_new(dev->ep_out, BBAUTH_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = bbauth_complete_out;
	dev->out_req = req;

	req = bbauth_request_new(dev->ep_in, BBAUTH_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = bbauth_complete_in;
	dev->in_req = req;

	return 0;

fail:
	pr_err("bbauth_create_bulk_endpoints() could not allocate requests\n");
	return -ENOMEM;
}

static int
bbauth_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct bbauth_dev	*dev = container_of(f, struct bbauth_dev, function);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "bbauth_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	bbauth_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = bbauth_create_bulk_endpoints(dev, &bbauth_fullspeed_in_desc,
			&bbauth_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		bbauth_highspeed_in_desc.bEndpointAddress =
			bbauth_fullspeed_in_desc.bEndpointAddress;
		bbauth_highspeed_out_desc.bEndpointAddress =
			bbauth_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		bbauth_superspeed_in_desc.bEndpointAddress =
			bbauth_fullspeed_in_desc.bEndpointAddress;
		bbauth_superspeed_out_desc.bEndpointAddress =
			bbauth_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
bbauth_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct bbauth_dev	*dev = container_of(f, struct bbauth_dev, function);

	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);

	wake_up(&dev->io_wq);

	bbauth_request_free(dev->out_req, dev->ep_out);
	bbauth_request_free(dev->in_req, dev->ep_in);
}

static int bbauth_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct bbauth_dev	*dev = container_of(f, struct bbauth_dev, function);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "bbauth_function_set_alt intf: %d alt: %d\n", intf, alt);

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	atomic_set(&dev->online, 1);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->io_wq);
	return 0;
}

static void bbauth_function_disable(struct usb_function *f)
{
	struct bbauth_dev	*dev = container_of(f, struct bbauth_dev, function);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "bbauth_function_disable cdev %p\n", cdev);
	/*
	 * Bus reset happened or cable disconnected.  No
	 * need to disable the configuration now.  We will
	 * set noify_close to true when device file is re-opened.
	 */
	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->io_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int bbauth_bind_config(struct usb_configuration *c)
{
	struct bbauth_dev *dev = _bbauth_dev;

	pr_debug("bbauth_bind_config\n");

	dev->cdev = c->cdev;
	dev->function.name = "bbauth";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	dev->function.fs_descriptors = fs_bbauth_descs;
#else
	dev->function.descriptors = fs_bbauth_descs;
#endif
	dev->function.hs_descriptors = hs_bbauth_descs;
	if (gadget_is_superspeed(c->cdev->gadget))
		dev->function.ss_descriptors = ss_bbauth_descs;
	dev->function.bind = bbauth_function_bind;
	dev->function.unbind = bbauth_function_unbind;
	dev->function.set_alt = bbauth_function_set_alt;
	dev->function.disable = bbauth_function_disable;

	return usb_add_function(c, &dev->function);
}

static int bbauth_setup(void)
{
	struct bbauth_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	init_waitqueue_head(&dev->io_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	_bbauth_dev = dev;

	ret = misc_register(&bbauth_device);
	if (ret)
		goto err;

	return 0;

err:
	kfree(dev);
	_bbauth_dev = NULL;
	pr_err("bb gadget driver failed to initialize\n");
	return ret;
}

static void bbauth_cleanup(void)
{
	misc_deregister(&bbauth_device);

	kfree(_bbauth_dev);
	_bbauth_dev = NULL;
}
