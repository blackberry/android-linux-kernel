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

#define BB_BULK_BUFFER_SIZE           (16 * 1024)

struct bb_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	atomic_t online;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	wait_queue_head_t io_wq;
	struct usb_request *out_req;
	struct usb_request *in_req;
	int rx_done;
	int rx_error;
	int tx_done;
	int tx_error;
};

static struct bb_dev *_bb_dev;

/*-------------------------------------------------------------------------*/

/* utility functions */

static struct usb_request *bb_request_new(struct usb_ep *ep, int buffer_size)
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

static void bb_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int bb_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1)
		return 0;
	else {
		atomic_dec(excl);
		return -EBUSY;
	}
}

static inline void bb_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/*-------------------------------------------------------------------------*/

/* for dev node i/o operation */

static ssize_t bb_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct bb_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count;
	int ret;
	bool read_zlp = false;

	pr_debug("bb_read(%d)\n", (int)count);
	if (!_bb_dev)
		return -ENODEV;

	if (count > BB_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (!atomic_read(&dev->online))
		return -EPIPE;

	if (bb_lock(&dev->read_excl))
		return -EBUSY;

	req = dev->out_req;
	if (req != 0) {
		do {
			req->length = count;
			dev->rx_done = 0;
			dev->rx_error = 0;
			ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_err("bb_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				goto done;
			} else
				pr_debug("rx %p queue\n", req);

			/* wait for a request to complete */
			ret = wait_event_interruptible(dev->io_wq, dev->rx_done ||
					dev->rx_error);
			if (ret < 0) {
				r = ret;
				usb_ep_dequeue(dev->ep_out, req);
				goto done;
			}
			if (!dev->rx_error) {
				if (!read_zlp) {
					pr_debug("rx %p %d\n", req, req->actual);
					r = req->actual;
					if (copy_to_user(buf, req->buf, req->actual))
						r = -EFAULT;
				}
			} else
				r = -EIO;

			if (read_zlp) {
				read_zlp = false;
			} else if ((req->actual == count) &&
					(req->actual % dev->ep_out->maxpacket == 0)) {
				/* A ZLP will be coming, so queue another read to consume it */
				/* Even though we only need to read a ZLP, there's some hardware
				 that requires a multiple of max packet size for a read */
				count = dev->ep_out->maxpacket;
				read_zlp = true;
			}
		} while (read_zlp);
	} else
		r = -ENODEV;

done:

	bb_unlock(&dev->read_excl);
	pr_debug("bb_read returning %d\n", r);
	return r;
}

static ssize_t bb_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct bb_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count;
	int ret;
	bool send_zlp = false;

	pr_debug("bb_write(%d)\n", (int)count);
	if (!_bb_dev)
		return -ENODEV;

	if (count > BB_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (!atomic_read(&dev->online))
		return -EPIPE;

	if (bb_lock(&dev->write_excl))
		return -EBUSY;

	req = dev->in_req;
	if (req != 0) {
		if (copy_from_user(req->buf, buf, count)) {
			r = -EFAULT;
			goto done;
		}

		do {
			req->length = count;
			dev->tx_done = 0;
			dev->tx_error = 0;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_err("bb_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				goto done;
			}

			ret = wait_event_interruptible(dev->io_wq, dev->tx_done ||
					dev->tx_error);
			if (ret < 0) {
				r = ret;
				usb_ep_dequeue(dev->ep_out, req);
				goto done;
			}
			if (!dev->tx_error) {
				if (!send_zlp) {
					pr_debug("tx %p %d\n", req, req->actual);
					r = req->actual;
				}
			} else
				r = -EIO;

			if (send_zlp) {
				send_zlp = false;
			} else if (req->actual % dev->ep_out->maxpacket == 0) {
				count = 0;
				send_zlp = true;
			}
		} while (send_zlp);
	} else
		r = -ENODEV;

done:

	bb_unlock(&dev->write_excl);
	pr_debug("bb_write returning %d\n", r);
	return r;
}

static int bb_open(struct inode *ip, struct file *fp)
{
	pr_info("bb_open\n");

	if (!_bb_dev)
		return -ENODEV;

	if (bb_lock(&_bb_dev->open_excl))
		return -EBUSY;

	fp->private_data = _bb_dev;

	return 0;
}

static int bb_release(struct inode *ip, struct file *fp)
{
	pr_info("bb_release\n");

	bb_unlock(&_bb_dev->open_excl);
	return 0;
}

static const struct file_operations bb_fops = {
	.owner = THIS_MODULE,
	.read = bb_read,
	.write = bb_write,
	.open = bb_open,
	.release = bb_release,
};

static const char bb_shortname[] = "usb_blackberry";

static struct miscdevice bb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = bb_shortname,
	.fops = &bb_fops,
};

/*-------------------------------------------------------------------------*/

/* callbacks */

static void bb_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct bb_dev *dev = _bb_dev;

	dev->tx_done = 1;
	if (req->status != 0)
		dev->tx_error = 1;

	wake_up(&dev->io_wq);
}

static void bb_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct bb_dev *dev = _bb_dev;

	dev->rx_done = 1;
	if (req->status != 0 && req->status != -ECONNRESET)
		dev->rx_error = 1;

	wake_up(&dev->io_wq);
}

/*-------------------------------------------------------------------------*/

/* function driver operation */

static struct usb_interface_descriptor bb_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x02,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor bb_superspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_ss_ep_comp_descriptor bb_superspeed_in_comp_desc = {
	.bLength =		sizeof bb_superspeed_in_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor bb_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_ss_ep_comp_descriptor bb_superspeed_out_comp_desc = {
	.bLength =		sizeof bb_superspeed_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor bb_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor bb_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor bb_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor bb_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_bb_descs[] = {
	(struct usb_descriptor_header *) &bb_interface_desc,
	(struct usb_descriptor_header *) &bb_fullspeed_in_desc,
	(struct usb_descriptor_header *) &bb_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_bb_descs[] = {
	(struct usb_descriptor_header *) &bb_interface_desc,
	(struct usb_descriptor_header *) &bb_highspeed_in_desc,
	(struct usb_descriptor_header *) &bb_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_bb_descs[] = {
	(struct usb_descriptor_header *) &bb_interface_desc,
	(struct usb_descriptor_header *) &bb_superspeed_in_desc,
	(struct usb_descriptor_header *) &bb_superspeed_in_comp_desc,
	(struct usb_descriptor_header *) &bb_superspeed_out_desc,
	(struct usb_descriptor_header *) &bb_superspeed_out_comp_desc,
	NULL,
};

#define INTERFACE_STRING_INDEX 0

static struct usb_string bb_string_defs[] = {
	[INTERFACE_STRING_INDEX].s	= "BlackBerry",
	{  },	/* end of list */
};

static struct usb_gadget_strings bb_string_table = {
	.language		= 0x0409,	/* en-US */
	.strings		= bb_string_defs,
};

static struct usb_gadget_strings *bb_strings[] = {
	&bb_string_table,
	NULL,
};


static int bb_create_bulk_endpoints(struct bb_dev *dev,
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
	req = bb_request_new(dev->ep_out, BB_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = bb_complete_out;
	dev->out_req = req;

	req = bb_request_new(dev->ep_in, BB_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = bb_complete_in;
	dev->in_req = req;

	return 0;

fail:
	pr_err("bb_create_bulk_endpoints() could not allocate requests\n");
	return -ENOMEM;
}

static int
bb_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct bb_dev	*dev = container_of(f, struct bb_dev, function);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "bb_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	bb_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = bb_create_bulk_endpoints(dev, &bb_fullspeed_in_desc,
			&bb_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		bb_highspeed_in_desc.bEndpointAddress =
			bb_fullspeed_in_desc.bEndpointAddress;
		bb_highspeed_out_desc.bEndpointAddress =
			bb_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		bb_superspeed_in_desc.bEndpointAddress =
			bb_fullspeed_in_desc.bEndpointAddress;
		bb_superspeed_out_desc.bEndpointAddress =
			bb_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
bb_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct bb_dev	*dev = container_of(f, struct bb_dev, function);

	atomic_set(&dev->online, 0);

	wake_up(&dev->io_wq);

	bb_request_free(dev->out_req, dev->ep_out);
	bb_request_free(dev->in_req, dev->ep_in);
}

static int bb_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct bb_dev	*dev = container_of(f, struct bb_dev, function);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "bb_function_set_alt intf: %d alt: %d\n", intf, alt);

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

static void bb_function_disable(struct usb_function *f)
{
	struct bb_dev	*dev = container_of(f, struct bb_dev, function);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "bb_function_disable cdev %p\n", cdev);
	/*
	 * Bus reset happened or cable disconnected.  No
	 * need to disable the configuration now.  We will
	 * set noify_close to true when device file is re-opened.
	 */
	atomic_set(&dev->online, 0);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->io_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int bb_bind_config(struct usb_configuration *c)
{
	struct bb_dev *dev = _bb_dev;
	int ret = 0;

	pr_debug("bb_bind_config\n");

	/* allocate a string ID for our interface */
	if (bb_string_defs[INTERFACE_STRING_INDEX].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		bb_string_defs[INTERFACE_STRING_INDEX].id = ret;
		bb_interface_desc.iInterface = ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = "blackberry";
	dev->function.strings = bb_strings;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	dev->function.fs_descriptors = fs_bb_descs;
#else
	dev->function.descriptors = fs_bb_descs;
#endif
	dev->function.hs_descriptors = hs_bb_descs;
	if (gadget_is_superspeed(c->cdev->gadget))
		dev->function.ss_descriptors = ss_bb_descs;
	dev->function.bind = bb_function_bind;
	dev->function.unbind = bb_function_unbind;
	dev->function.set_alt = bb_function_set_alt;
	dev->function.disable = bb_function_disable;

	return usb_add_function(c, &dev->function);
}

static int bb_setup(void)
{
	struct bb_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	init_waitqueue_head(&dev->io_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	_bb_dev = dev;

	ret = misc_register(&bb_device);
	if (ret)
		goto err;

	return 0;

err:
	kfree(dev);
	_bb_dev = NULL;
	pr_err("bb gadget driver failed to initialize\n");
	return ret;
}

static void bb_cleanup(void)
{
	misc_deregister(&bb_device);

	kfree(_bb_dev);
	_bb_dev = NULL;
}
