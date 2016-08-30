/* 
 *[FEATURE]-ADD by TCTSH.lijuan.wu, mini log, task-1488899
 * 
 */

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/elf.h>
#include <linux/wait.h>
#include <soc/qcom/ssrminilog.h>


#define WAIT_MSECS	120000
#define MAX_SSR_REASON_LEN	81U
#define SSR_NAME_LEN  10U
char  g_ssr_reason[MAX_SSR_REASON_LEN+SSR_NAME_LEN];


static int enable_ssrminilog;
module_param(enable_ssrminilog, int, S_IRUGO | S_IWUSR);

struct ssrminilog_device {
	char name[256];

	unsigned int data_ready;
	unsigned int consumer_present;
	int ssrminilog_status;

	struct completion ssrminilog_complete;
	struct miscdevice device;

	wait_queue_head_t rec_wait_q;
};
static void *ssr_log_dev;

static int ssrminilog_open(struct inode *inode, struct file *filep)
{
	struct ssrminilog_device *rd_dev =  container_of(filep->private_data,
				struct ssrminilog_device, device);
	rd_dev->consumer_present = 1;
	rd_dev->ssrminilog_status = 0;
	return 0;
}

static int ssrminilog_release(struct inode *inode, struct file *filep)
{
	struct ssrminilog_device *rd_dev = container_of(filep->private_data,
				struct ssrminilog_device, device);
	rd_dev->consumer_present = 0;
	rd_dev->data_ready = 0;
	complete(&rd_dev->ssrminilog_complete);
	return 0;
}

static ssize_t ssrminilog_read(struct file *filep, char __user *buf, size_t count,
			loff_t *pos)
{
	struct ssrminilog_device *rd_dev = container_of(filep->private_data,
				struct ssrminilog_device, device);

	int ret = 0;
	if ((filep->f_flags & O_NONBLOCK) && !rd_dev->data_ready)
		return -EAGAIN;

	ret = wait_event_interruptible(rd_dev->rec_wait_q, rd_dev->data_ready);
	pr_debug("ssrminilog_read  wait_event_interruptible ret=%d", ret);
	if (ret)
		return ret;

	rd_dev->ssrminilog_status = 0;
	rd_dev->data_ready = 0;
	*pos = 0;
	ret= simple_read_from_buffer(buf, count, pos, g_ssr_reason, strlen(g_ssr_reason));
	//pr_info("ssrminilog_read complete");//log_temp;
	complete(&rd_dev->ssrminilog_complete);
	memset(g_ssr_reason,0,MAX_SSR_REASON_LEN+SSR_NAME_LEN);
	return ret;
}

static unsigned int ssrminilog_poll(struct file *filep,
					struct poll_table_struct *wait)
{
	struct ssrminilog_device *rd_dev = container_of(filep->private_data,
				struct ssrminilog_device, device);
	unsigned int mask = 0;

	if (rd_dev->data_ready)
		mask |= (POLLIN | POLLRDNORM);

	poll_wait(filep, &rd_dev->rec_wait_q, wait);
	return mask;
}

static const struct file_operations ssrminilog_file_ops = {
	.open = ssrminilog_open,
	.release = ssrminilog_release,
	.read = ssrminilog_read,
	.poll = ssrminilog_poll
};

static void *create_ssrminilog_device(const char *dev_name, struct device *parent)
{
	int ret;
	struct ssrminilog_device *rd_dev;

	if (!dev_name) {
		pr_err("%s: Invalid device name.\n", __func__);
		return NULL;
	}

	rd_dev = kzalloc(sizeof(struct ssrminilog_device), GFP_KERNEL);

	if (!rd_dev) {
		pr_err("%s: Couldn't alloc space for ssrminilog device!",
			__func__);
		return NULL;
	}

	snprintf(rd_dev->name, ARRAY_SIZE(rd_dev->name), "%s", dev_name);

	init_completion(&rd_dev->ssrminilog_complete);

	rd_dev->device.minor = MISC_DYNAMIC_MINOR;
	rd_dev->device.name = rd_dev->name;
	rd_dev->device.fops = &ssrminilog_file_ops;
	rd_dev->device.parent = parent;

	init_waitqueue_head(&rd_dev->rec_wait_q);

	ret = misc_register(&rd_dev->device);

	if (ret) {
		pr_err("%s: misc_register failed for %s (%d)", __func__,
				dev_name, ret);
		kfree(rd_dev);
		return NULL;
	}

	return (void *)rd_dev;
}

void destroy_ssr_check_device(void )
{
	struct ssrminilog_device *rd_dev = ssr_log_dev;

	if (IS_ERR_OR_NULL(rd_dev))
		return;

	misc_deregister(&rd_dev->device);
	kfree(rd_dev);
}
module_exit(destroy_ssr_check_device);

static int _do_ssrminilog(void *handle)
{
	int ret;
	struct ssrminilog_device *rd_dev = (struct ssrminilog_device *)handle;

	if (!rd_dev->consumer_present) {
		pr_err("ssrminilog(%s): No consumers. Aborting..\n", rd_dev->name);
		return -EPIPE;
	}

	rd_dev->data_ready = 1;
	rd_dev->ssrminilog_status = -1;

	INIT_COMPLETION(rd_dev->ssrminilog_complete);

	/* Tell userspace that the data is ready */
	wake_up(&rd_dev->rec_wait_q);

	/* Wait (with a timeout) to let the ssr minilog complete */
	ret = wait_for_completion_timeout(&rd_dev->ssrminilog_complete,
			msecs_to_jiffies(WAIT_MSECS));

	if (!ret) {
		pr_err("ssrminilog(%s): Timed out waiting for userspace.\n",
			rd_dev->name);
		ret = -EPIPE;
	} else
		ret = (rd_dev->ssrminilog_status == 0) ? 0 : -EPIPE;

	rd_dev->data_ready = 0;
	//pr_info("_do_ssrminilog  return ret=%d", ret);
	return ret;
}

int do_ssrminilog(void)
{
	int ret;

	if(enable_ssrminilog == 1)
	{
		if(ssr_log_dev==NULL)
			return 0;
		ret = _do_ssrminilog(ssr_log_dev);
		if (ret < 0)
			pr_err("unable to do ssr minilog %d\n", ret);
	}
	return ret;
}
EXPORT_SYMBOL(do_ssrminilog);

static __init int ssrminilog_init(void)
{

	ssr_log_dev = create_ssrminilog_device("ssr_log", NULL);

	if (IS_ERR_OR_NULL(ssr_log_dev)) {
		pr_err("Unable to create ssr log device.\n");
		ssr_log_dev = NULL;
	}

	return 0;
}
late_initcall(ssrminilog_init);

void ssr_check(const char*name, char * reason)
{
	memset(g_ssr_reason,0,MAX_SSR_REASON_LEN+SSR_NAME_LEN);
	snprintf(g_ssr_reason,sizeof(g_ssr_reason), "%s#%s",name,reason);
	//pr_info(" g_ssr_reason is %s",g_ssr_reason);

}
EXPORT_SYMBOL(ssr_check);

