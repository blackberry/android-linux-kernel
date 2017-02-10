/* Copyright (C) 2016 Tcl Corporation Limited */
/*
 * (C) Copyright 2010
 * MediaTek <www.MediaTek.com>
 *
 * Android Exception Device
 *
 */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include "aed.h"
#include <linux/pid.h>



static int aee_mode = AEE_MODE_CUSTOMER_USER;

extern void aee_kernel_RT_Monitor_api(int lParam);
#define AEEIOCTL_RT_MON_Kick _IOR('p', 0x0A, int)

/******************************************************************************
 * FUNCTION PROTOTYPES
 *****************************************************************************/
static long monitor_hang_ioctl(struct file *file, unsigned int cmd, unsigned long arg);



/******************************************************************************
 * hang detect File operations
 *****************************************************************************/
static int monitor_hang_open(struct inode *inode, struct file *filp)
{
	LOGD("%s\n", __func__);
//	aee_kernel_RT_Monitor_api (600) ;
	return 0;
}

static int monitor_hang_release(struct inode *inode, struct file *filp)
{
	LOGD("%s\n", __func__);
	return 0;
}

static unsigned int monitor_hang_poll(struct file *file, struct poll_table_struct *ptable)
{
	LOGD("%s\n", __func__);
	return 0;
}

static ssize_t monitor_hang_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	LOGD("%s\n", __func__);
	return 0;
}

static ssize_t monitor_hang_write(struct file *filp, const char __user *buf, size_t count,
				  loff_t *f_pos)
{

	LOGD("%s\n", __func__);
	return 0;
}


/* QHQ RT Monitor */
/* QHQ RT Monitor    end */




/*
 * aed process daemon and other command line may access me
 * concurrently
 */
static long monitor_hang_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	static long long monitor_status;
	if (cmd == AEEIOCTL_WDT_KICK_POWERKEY) {

		return ret;

	}
/* QHQ RT Monitor */
	if (cmd == AEEIOCTL_RT_MON_Kick) {
		LOGE("AEEIOCTL_RT_MON_Kick ( %d)\n", (int)arg);
		aee_kernel_RT_Monitor_api((int)arg);
		return ret;
	}
//	LOGE("AEEIOCTL_RT_MON_Kick unknown cmd :(%d)( %d)\n",(int)cmd, (int)arg);
//	LOGE("AEEIOCTL_RT_MON_Kick known cmd :(%d)( %d)\n",(int)AEEIOCTL_WDT_KICK_POWERKEY, (int)AEEIOCTL_RT_MON_Kick);
/* QHQ RT Monitor end */

	if (cmd == AEEIOCTL_SET_SF_STATE) {
		if (copy_from_user(&monitor_status, (void __user *)arg, sizeof(long long)))
			ret = -1;
		LOGE("AEE_MONITOR_SET[status]: 0x%llx", monitor_status);
		return ret;
	} else if (cmd == AEEIOCTL_GET_SF_STATE) {
		if (copy_to_user((void __user *)arg, &monitor_status, sizeof(long long)))
			ret = -1;
		return ret;
	}

	return -1;
}




/* QHQ RT Monitor */
static struct file_operations aed_wdt_RT_Monitor_fops = {
	.owner = THIS_MODULE,
	.open = monitor_hang_open,
	.release = monitor_hang_release,
	.poll = monitor_hang_poll,
	.read = monitor_hang_read,
	.write = monitor_hang_write,
	.unlocked_ioctl = monitor_hang_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = monitor_hang_ioctl,
#endif	
};






static struct miscdevice aed_wdt_RT_Monitor_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "RT_Monitor",
	.fops = &aed_wdt_RT_Monitor_fops,
};



/* bleow code is added for monitor_hang_init */
static int monitor_hang_init(void);

static int hang_detect_init(void);
/* bleow code is added for hang detect */



static int __init monitor_hang_init(void)
{
	int err = 0;

/* bleow code is added by QHQ  for hang detect */
	err = misc_register(&aed_wdt_RT_Monitor_dev);
	if (unlikely(err)) {
		LOGE("aee: failed to register aed_wdt_RT_Monitor_dev device!\n");
		return err;
	}

	hang_detect_init();
/* bleow code is added by QHQ  for hang detect */
/* end */

	return err;
}

static void __exit monitor_hang_exit(void)
{
	int err;
	err = misc_deregister(&aed_wdt_RT_Monitor_dev);
	if (unlikely(err))
		LOGE("failed to unregister RT_Monitor device!\n");
}




/* bleow code is added by QHQ  for hang detect */
/* For the condition, where kernel is still alive, but system server is not scheduled. */

#define HD_PROC "hang_detect"

/* static DEFINE_SPINLOCK(hd_locked_up); */
#define HD_INTER 30

static int hd_detect_enabled;
static int hd_timeout = 0x7fffffff;
static int hang_detect_counter = 0x7fffffff;
int InDumpAllStack=0;
static int system_server_pid=0;
static int surfaceflinger_pid=0;
static int init_pid=0;

static int FindTaskByName(char *name)
{
	struct task_struct *task;
	for_each_process(task) {
		
		if (task && (strcmp(task->comm, "init") == 0)) {
			init_pid=task->pid;
			LOGE("[Hang_Detect] %s found pid:%d.\n", task->comm, task->pid);
		}
		
		if (task && (strcmp(task->comm, "surfaceflinger") == 0)) {
			surfaceflinger_pid=task->pid;
			LOGE("[Hang_Detect] %s found pid:%d.\n", task->comm, task->pid);
		}
		if (task && (strcmp(task->comm, name) == 0)) {
			system_server_pid=task->pid;
			LOGE("[Hang_Detect] %s found pid:%d.\n", task->comm, task->pid);
			return task->pid;
		}
	}
	LOGE("[Hang_Detect] system_server not found!\n");
	return -1;
}

//extern void show_stack(struct task_struct *tsk, unsigned long *sp);
void sched_show_task_local(struct task_struct *p)
{
	unsigned long free = 0;
	int ppid;
	unsigned state;
    char stat_nam[] = TASK_STATE_TO_CHAR_STR;
	state = p->state ? __ffs(p->state) + 1 : 0;
	printk(KERN_INFO "%-15.15s %c", p->comm,
		state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
#if BITS_PER_LONG == 32
	if (state == TASK_RUNNING)
		printk(KERN_CONT " running  ");
	else
		printk(KERN_CONT " %08lx ", thread_saved_pc(p));
#else
	if (state == TASK_RUNNING)
		printk(KERN_CONT "  running task    ");
	else
		printk(KERN_CONT " %016lx ", thread_saved_pc(p));
#endif
#ifdef CONFIG_DEBUG_STACK_USAGE
	free = stack_not_used(p);
#endif
	rcu_read_lock();
	ppid = task_pid_nr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	printk(KERN_CONT "%5lu %5d %6d 0x%08lx\n", free,
		task_pid_nr(p), ppid,
		(unsigned long)task_thread_info(p)->flags);

//	print_worker_info(KERN_INFO, p);
//	show_stack(p, NULL);
}

void show_state_filter_local(unsigned long state_filter)
{
	struct task_struct *g, *p;

#if BITS_PER_LONG == 32
	printk(KERN_INFO
		"  task                PC stack   pid father\n");
#else
	printk(KERN_INFO
		"  task                        PC stack   pid father\n");
#endif
	do_each_thread(g, p) {
		/*
		 * reset the NMI-timeout, listing all files on a slow
		 * console might take a lot of time:
		 */
//		if (!state_filter || (p->state & state_filter))
//			sched_show_task(p);
	} while_each_thread(g, p);
}



static void ShowStatus(void)
{

	struct task_struct *t,*p;
	struct pid *pid;
	int count=0;
	InDumpAllStack=1;
	
	//show all kbt in init
	LOGE("[Hang_Detect] dump init all thread bt \n");
	if(init_pid)
	{	pid=find_get_pid(init_pid);
		t=p=get_pid_task(pid,PIDTYPE_PID);
		do 
		{
			sched_show_task_local(t);
		} while_each_thread(p, t);
	}	
	
	//show all kbt in surfaceflinger
	LOGE("[Hang_Detect] dump surfaceflinger all thread bt \n");
	if(surfaceflinger_pid)
	{	pid=find_get_pid(surfaceflinger_pid);
		t=p=get_pid_task(pid,PIDTYPE_PID);
		count=0;
		do 
		{
			sched_show_task_local(t);
			if((++count)%5==4)
				msleep(20);
		} while_each_thread(p, t);
	}	
	msleep(100);
	//show all kbt in system_server
	LOGE("[Hang_Detect] dump system_server all thread bt \n");
	if(system_server_pid)
	{	pid=find_get_pid(system_server_pid);
		t=p=get_pid_task(pid,PIDTYPE_PID);
		count=0;
		do 
		{
			sched_show_task_local(t);
			if((++count)%5==4)
				msleep(20);
		} while_each_thread(p, t);
	}	
	msleep(100);
	//show all D state thread kbt
	LOGE("[Hang_Detect] dump all D thread bt \n");
	show_state_filter_local(TASK_UNINTERRUPTIBLE); 
	debug_show_all_locks();
	system_server_pid=0;
	surfaceflinger_pid=0;
	init_pid=0;
	InDumpAllStack=0;
	msleep(10);
}

static int hang_detect_thread(void *arg)
{

	/* unsigned long flags; */
//	struct sched_param param = {.sched_priority = RTPM_PRIO_WDT };

	LOGE("[Hang_Detect] hang_detect thread starts.\n");

//	sched_setscheduler(current, SCHED_FIFO, &param);

	while (1) {
		if ((1 == hd_detect_enabled) && (FindTaskByName("system_server") != -1)) {
			LOGE("[Hang_Detect] hang_detect thread counts down %d:%d.\n",
			     hang_detect_counter, hd_timeout);


			if (hang_detect_counter <= 0) {
				ShowStatus();
			}

		
			if (hang_detect_counter == 0)
			{
				LOGE("[Hang_Detect] we should triger	HWT	...\n");
				if(aee_mode!=AEE_MODE_CUSTOMER_USER) 
				{
//					aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP, "\nCRDISPATCH_KEY:SS Hang\n","we triger HWT ");
					msleep(30 * 1000);
				}
				else //only Customer user load  trigger HWT
				{
//					aee_kernel_exception_api(__FILE__, __LINE__, DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP, "\nCRDISPATCH_KEY:SS Hang\n","we triger HWT ");
					msleep(30 * 1000);
					local_irq_disable();
					while (1);
					BUG();
				}	
			}

			hang_detect_counter--;
		} else {
			/* incase of system_server restart, we give 2 mins more.(4*HD_INTER) */
			if (1 == hd_detect_enabled) {
				hang_detect_counter = hd_timeout + 4;
				hd_detect_enabled = 0;
			}
			LOGE("[Hang_Detect] hang_detect disabled.\n");
		}

		msleep((HD_INTER) * 1000);
	}
	return 0;
}

void hd_test(void)
{
	hang_detect_counter = 0;
	hd_timeout = 0;
}

void aee_kernel_RT_Monitor_api(int lParam)
{
	if (0 == lParam) {
		hd_detect_enabled = 0;
		hang_detect_counter = hd_timeout;
		LOGE("[Hang_Detect] hang_detect disabled\n");
	} else if (lParam > 0) {
		hd_detect_enabled = 1;
		hang_detect_counter = hd_timeout = ((long)lParam + HD_INTER - 1) / (HD_INTER);
		LOGE("[Hang_Detect] hang_detect enabled %d\n", hd_timeout);
	}
}

#if 0
static int hd_proc_cmd_read(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{

	/* with a read of the /proc/hang_detect, we reset the counter. */
	int len = 0;
	LOGE("[Hang_Detect] read proc	%d\n", count);

	len = sprintf(buf, "%d:%d\n", hang_detect_counter, hd_timeout);

	hang_detect_counter = hd_timeout;

	return len;
}

static int hd_proc_cmd_write(struct file *file, const char *buf, unsigned long count, void *data)
{

	/* with a write function , we set the time out, in seconds. */
	/* with a '0' argument, we set it to max int */
	/* with negative number, we will triger a timeout, or for extention functions (future use) */

	int counter = 0;
	int retval = 0;

	retval = sscanf(buf, "%d", &counter);

	LOGE("[Hang_Detect] write	proc %d, original %d: %d\n", counter, hang_detect_counter,
	     hd_timeout);

	if (counter > 0) {
		if (counter % HD_INTER != 0)
			hd_timeout = 1;
		else
			hd_timeout = 0;

		counter = counter / HD_INTER;
		hd_timeout += counter;
	} else if (counter == 0)
		hd_timeout = 0x7fffffff;
	else if (counter == -1)
		hd_test();
	else
		return count;

	hang_detect_counter = hd_timeout;

	return count;
}
#endif

int hang_detect_init(void)
{

	struct task_struct *hd_thread;
	/* struct proc_dir_entry *de = create_proc_entry(HD_PROC, 0664, 0); */

	unsigned char *name = "hang_detect";

	LOGD("[Hang_Detect] Initialize proc\n");

	/* de->read_proc = hd_proc_cmd_read; */
	/* de->write_proc = hd_proc_cmd_write; */

	LOGD("[Hang_Detect] create hang_detect thread\n");

	hd_thread = kthread_create(hang_detect_thread, NULL, name);
	wake_up_process(hd_thread);

	return 0;
}

/* added by QHQ  for hang detect */
/* end */


module_init(monitor_hang_init);
module_exit(monitor_hang_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek AED Driver");
MODULE_AUTHOR("MediaTek Inc.");

