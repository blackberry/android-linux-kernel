/*
 * Copyright (C) 2018 BlackBerry Limited
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

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/************************************************************************
 * Macros
 ************************************************************************/
/*#define DEBUG*/
#ifdef DEBUG
#define LOGD(x, ...) logDebug(x, ##__VA_ARGS__)
#else
#define LOGD(x, ...)
#endif

/************************************************************************
 * Types
 ************************************************************************/
struct pinfo_node {
	int pid;

	struct process_info __rcu *pinfo;
#ifdef BID_USER_DEBUG
	void *kobj;
#endif

	struct list_head list;
	struct rcu_head rcu;

	struct pinfo_node *parent;
	struct list_head children;
	struct list_head siblings;
};

struct process_info_update_ctx {
	struct pinfo_node *node;
	struct process_info *old_pinfo;
	struct process_info *new_pinfo;
	unsigned long flags;

	/*
	 * Used when an update is actually creating a new pinfo
	 * to hold the created pinfo. That way I can tell what
	 * buffers are changed and free them if necessary.
	 */
	struct process_info dummy;
};

/************************************************************************
 * Forward declarations
 ************************************************************************/

/************************************************************************
 * Globals
 ************************************************************************/
static DEFINE_SPINLOCK(ListLock);
static DEFINE_SPINLOCK(AllocLock);
static struct pinfo_node ProcList;
static struct process_info *FreePInfo;

/************************************************************************
 * Inline functions
 ************************************************************************/
/*
 * A function to fetch a valid process_info pointer
 * from a pinfo_node.
 *
 * This function assumes you already hold an rcu_read_lock
 *
 * @param node	The pinfo node
 *
 * @return A pointer to the associated process_info struct
 */
static inline struct process_info* node_to_pinfo(const struct pinfo_node *node) {
	return rcu_dereference(node->pinfo);
}

#ifdef BID_USER_DEBUG
/************************************************************************
 * Debugging/sysfs code
 ************************************************************************/
struct proc_kobj {
	struct kobject kobj;
	struct pinfo_node *node;
	struct attribute_group attr_group;
	char name[16];
};

static ssize_t pinfo_attr_name_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	struct process_info *pinfo;
	struct proc_kobj *pkobj = container_of(kobj, struct proc_kobj, kobj);
	int rc;

	rcu_read_lock();
	pinfo = node_to_pinfo(pkobj->node);
	if (pinfo != NULL)
		rc = snprintf(buf, PAGE_SIZE, "%s\n", pinfo->name);
	else
		rc = snprintf(buf, PAGE_SIZE, "--dead--\n");
	rcu_read_unlock();

	return rc;
}

static ssize_t pinfo_attr_flags_show(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	struct process_info *pinfo;
	struct proc_kobj *pkobj = container_of(kobj, struct proc_kobj, kobj);
	int rc;

	rcu_read_lock();
	pinfo = node_to_pinfo(pkobj->node);
	if (pinfo != NULL)
		rc = snprintf(buf, PAGE_SIZE, "0x%08x\n", pinfo->flags);
	else
		rc = snprintf(buf, PAGE_SIZE, "--dead--\n");
	rcu_read_unlock();

	return rc;
}

static ssize_t pinfo_attr_parent_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct process_info *pinfo;
	struct proc_kobj *pkobj = container_of(kobj, struct proc_kobj, kobj);
	int rc;

	rcu_read_lock();
	if (pkobj->node->parent != NULL) {
		pinfo = node_to_pinfo(pkobj->node->parent);
		rc = snprintf(buf, PAGE_SIZE, "%d%s\n",
			      pinfo->pid,
			      (pinfo->flags & BIDE_PROC_FLAG_DEAD) ? " (dead)" :
			      "");
	} else {
		rc = snprintf(buf, PAGE_SIZE, "--no parent--\n");
	}
	rcu_read_unlock();

	return rc;
}

static ssize_t pinfo_attr_children_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct pinfo_node *node;
	struct process_info *pinfo;
	struct proc_kobj *pkobj = container_of(kobj, struct proc_kobj, kobj);
	int rc;
	char *ptr = buf;

	rcu_read_lock();
	list_for_each_entry(node, &pkobj->node->children, siblings) {
		pinfo = node_to_pinfo(node);
		/* -1 to leave space for a newline at the end */
		rc = snprintf(ptr, PAGE_SIZE-(ptr-buf)-1,
			      (ptr == buf) ? "%d%s" : " %d%s",
			      node->pid,
			      (pinfo->flags & BIDE_PROC_FLAG_DEAD) ?
				" (dead)" : "");
		if (rc < 0) {
			/* Error */
			rc = snprintf(buf, PAGE_SIZE, "Error %d", rc);
			ptr = buf + rc;
			break;
		} else if (rc >= PAGE_SIZE-(ptr-buf)-1) {
			/* Ran out of space. Indicate truncation */
			buf[PAGE_SIZE-5] = '.';
			buf[PAGE_SIZE-4] = '.';
			buf[PAGE_SIZE-3] = '.';
			buf[PAGE_SIZE-2] = '\0';
			ptr = &buf[PAGE_SIZE-2];
			break;
		}
		ptr += rc;
	}
	*ptr++ = '\n';
	*ptr = '\0';

	rcu_read_unlock();
	return ptr-buf;
}

static struct kobj_attribute pinfo_attr_name =
	__ATTR(name, 0600, pinfo_attr_name_show, NULL);

static struct kobj_attribute pinfo_attr_flags =
	__ATTR(flags, 0600, pinfo_attr_flags_show, NULL);

static struct kobj_attribute pinfo_attr_parent =
	__ATTR(parent, 0600, pinfo_attr_parent_show, NULL);

static struct kobj_attribute pinfo_attr_children =
	__ATTR(children, 0600, pinfo_attr_children_show, NULL);

static struct attribute *PinfoAttrs[] = {
	&pinfo_attr_name.attr,
	&pinfo_attr_flags.attr,
	&pinfo_attr_parent.attr,
	&pinfo_attr_children.attr,
	NULL
};

static ssize_t sysfs_show(struct kobject *kobj,
			struct attribute *attr,
			char *buf)
{
	struct kobj_attribute *kattr;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		return kattr->show(kobj, kattr, buf);

	return -EIO;
}

static void proc_kobj_free(struct kobject *kobj)
{
	struct proc_kobj *pkobj = container_of(kobj, struct proc_kobj, kobj);
	kfree(pkobj);
}

struct sysfs_ops PinfoKobjSysfsOps = {
	.show = sysfs_show
};

struct kobj_type PinfoKobjType = {
	.sysfs_ops = &PinfoKobjSysfsOps,
	.default_attrs = PinfoAttrs,
	.release = proc_kobj_free
};

static struct kobject *BideRootKObj;
static struct kset *PinfoKset;

/*
 * Sysfs interactions can't happen within a critical section
 * or an interrupt context. So use the workqueue mechanism
 * to make registration/release from sysfs happen in
 * a process context, outside of a critical section
 */
struct proc_kobj_work {
	struct work_struct work;
	struct proc_kobj *pkobj;
	int pid;
};

static void proc_kobj_work_register(struct work_struct *work)
{
	struct proc_kobj_work *pkobj_work;
	struct proc_kobj *pkobj;
	int rc = 0;

	pkobj_work = container_of(work, struct proc_kobj_work, work);
	pkobj = pkobj_work->pkobj;

	kobject_init(&pkobj->kobj, &PinfoKobjType);
	rc = kobject_add(&pkobj->kobj, NULL, "%d", pkobj_work->pid);
	if (rc) {
		logWarn("Unable to init/add kobj for pinfo %d. %d",
			pkobj_work->pid, rc);
		/*
		 * Don't release the kobject at this point. It is dead
		 * but since the associated pinfo may still be around
		 * I need to hang onto it.
		 */
	}
	kfree(pkobj_work);
}

static void proc_kobj_work_release(struct work_struct *work)
{
	struct proc_kobj_work *pkobj_work;
	pkobj_work = container_of(work, struct proc_kobj_work, work);
	kobject_del(&pkobj_work->pkobj->kobj);
	kfree(pkobj_work);
}

static void proc_kobj_work_free(struct work_struct *work)
{
	struct proc_kobj_work *pkobj_work;
	pkobj_work = container_of(work, struct proc_kobj_work, work);
	kobject_put(&pkobj_work->pkobj->kobj);
	kfree(pkobj_work);
}

static int proc_schedule_kobj_work(struct proc_kobj *pkobj,
				    void(*work_func)(struct work_struct *work))
{
	struct proc_kobj_work *work;

	work = kzalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		INIT_WORK(&work->work, work_func);
		work->pkobj = pkobj;
		schedule_work(&work->work);
		return 0;
	} else {
		return ENOMEM;
	}
}

static void sysfs_proc_init(void)
{
	/*
	BideRootKObj = kobject_create_and_add("bide", kernel_kobj);
	if (BideRootKObj) {
		PinfoKset = kset_create_and_add("processes", NULL, BideRootKObj);
		if (!PinfoKset) {
			logWarn("Unable to create processes kset");
			kobject_put(BideRootKObj);
		}
	} else {
		logWarn("Unable to create bide root kobject");
	}
	*/
}

static void sysfs_proc_exit(void)
{
	if (PinfoKset)
		kset_unregister(PinfoKset);
	if (BideRootKObj)
		kobject_put(BideRootKObj);
}

static void sysfs_pinfo_init(struct pinfo_node *node)
{
	struct process_info *pinfo = node_to_pinfo(node);
	struct proc_kobj *pkobj = NULL;
	struct proc_kobj_work *work = NULL;

	if (!PinfoKset)
		return;

	pkobj = kzalloc(sizeof(*pkobj), GFP_ATOMIC);
	if (!pkobj) {
		logWarn("Unable to allocate kobj for pinfo %d (%s)",
			   pinfo->pid, pinfo->name);
		return;
	}
	work = kzalloc(sizeof(*work), GFP_ATOMIC);
	if (!work) {
		logWarn("Unable to allocate kobj reg work for pinfo %d (%s)",
			pinfo->pid, pinfo->name);
		/*
		 * Ok to just bail. It means this pinfo won't have an
		 * associated kobj so it won't show up in sysfs
		 */
		kfree(pkobj);
		return;
	}

	pkobj->node = node;
	pkobj->kobj.kset = PinfoKset;
	node->kobj = pkobj;


	INIT_WORK(&work->work, proc_kobj_work_register);
	work->pkobj = pkobj;
	work->pid = pinfo->pid;
	schedule_work(&work->work);
}

static inline void sysfs_pinfo_release(struct pinfo_node *node) {
	struct proc_kobj *pkobj;

	pkobj = node->kobj;
	if (pkobj != NULL) {
		if (proc_schedule_kobj_work(pkobj, proc_kobj_work_release)) {
			logWarn("Unable to allocate kobj release work for pinfo %d (%s)",
				   node->pid, node->pinfo->name);
			/* May not be too bad. When the pkobj is actually
			 * free'ed it will automatically be removed from the
			 * filesystem
			 */
		}
	}
}

static inline void sysfs_pinfo_free(struct pinfo_node *node) {
	struct proc_kobj *pkobj;

	pkobj = node->kobj;
	if (pkobj != NULL) {
		if (proc_schedule_kobj_work(pkobj, proc_kobj_work_free)) {
			logWarn("Unable to allocate kobj free work for pinfo %d (%s)",
				   node->pid, node->pinfo->name);
			/* Guess we are going to have a memory leak :( */
		}
	}
}

#else
#define sysfs_proc_init(...)
#define sysfs_proc_exit(...)
#define sysfs_pinfo_init(...)
#define sysfs_pinfo_release(...)
#define sysfs_pinfo_free(...)
#endif

/************************************************************************
 * Internal API
 ************************************************************************/
static struct process_info* pinfo_alloc(void)
{
	struct process_info *pinfo;
	unsigned long flags;

	spin_lock_irqsave(&AllocLock, flags);
	if (FreePInfo) {
		pinfo = FreePInfo;
		FreePInfo = NULL;
		memset(pinfo, 0, sizeof(*pinfo));
	} else {
		pinfo = kzalloc(sizeof(*pinfo), GFP_ATOMIC);
	}
	spin_unlock_irqrestore(&AllocLock, flags);

	return pinfo;
}

/*
 * Indicates that a new pinfo will replace an old one.
 * This causes any buffers in the old pinfo that are
 * NOT shared between them to be free'ed
 */
static void pinfo_replaced(struct process_info *old_pinfo,
			   struct process_info *new_pinfo)
{
	if (old_pinfo->name != new_pinfo->name)
		kfree(old_pinfo->name);
}

/*
 * Indicates a pinfo is available for reuse.
 *
 * This may cause the pinfo's underlying memory
 * to be free'ed if the system decides it doesn't
 * want to save the pinfo for reuse.
 */
static inline void pinfo_reuse(struct process_info *pinfo)
{
	unsigned long flags;

	spin_lock_irqsave(&AllocLock, flags);
	if (!FreePInfo)
		FreePInfo = pinfo;
	else
		kfree(pinfo);
	spin_unlock_irqrestore(&AllocLock, flags);
}

/*
 * Frees a pinfo and all its internal buffers
 *
 * Once the internal items are free'ed the pinfo
 * will be scheduled for reuse.
 */
static void pinfo_free(struct pinfo_node *node)
{
	struct process_info *pinfo = node_to_pinfo(node);

#ifdef BID_USER_DEBUG
	if (node->kobj != NULL)
		sysfs_pinfo_free(node);
#endif
	kfree(pinfo->name);

	pinfo_reuse(pinfo);

	kfree(node);
}

static void pinfo_free_rcu(struct rcu_head *rcu_node)
{
	struct pinfo_node *node = container_of(rcu_node, struct pinfo_node, rcu);
	pinfo_free(node);
}

static int pinfo_init(struct pinfo_node *node,
			int pid,
			const struct pinfo_node *pnode)
{
	const char *name;
	struct process_info *pinfo = node_to_pinfo(node);

	/* Clone the parent's entry if there is one */
	if (pnode != NULL) {
		const struct process_info *ppinfo = node_to_pinfo(pnode);
		*pinfo = *ppinfo;
		name = pinfo->name;
	} else {
		/*
		 * Everything should have a parent except for init.
	         * Assume this is init if possible, if not just
		 * use a dummy value for name for now.
		 */
		if (pid == INIT_PID) {
			name = "init";
		} else if (pid == KTHREAD_PID) {
			name ="kthread";
		} else {
			logWarn("Unable to find parent for pid %d", pid);
			name = "__unknown__";
		}
		/* Setup any non-zero defaults in pinfo */
	}
	pinfo->node = node;
	pinfo->pid = pid;
	pinfo->flags = 0;

	pinfo->name = kmalloc(strlen(name)+1, GFP_ATOMIC);
	if (!pinfo->name) {
		logError("Unable to allocate buffer for name '%s'", name);
		return -1;
	}
	memcpy(pinfo->name, name, strlen(name)+1);

	/*
	 * We track whether the process was started by init and zygote
	 * as it causes us to defer security checks until after
	 * they finish their initialization steps.
	 */
	if (!strcmp("/init", pinfo->name)) {
		pinfo->flags |=
			BIDE_PROC_FLAG_INIT_SPAWNED |
			BIDE_PROC_FLAG_INITIALIZING;
	} else if (!strcmp("/system/bin/app_process32", pinfo->name) ||
		   !strcmp("/system/bin/app_process64", pinfo->name))
	{
		pinfo->flags |=
			BIDE_PROC_FLAG_ZYGOTE_SPAWNED |
			BIDE_PROC_FLAG_INITIALIZING;
	}

	if (!ctl_snapshot_complete())
		pinfo->flags |= BIDE_PROC_FLAG_PRE_SNAPSHOT;

	sysfs_pinfo_init(node);

	LOGD("Initializing pinfo for %d with parent %d (%s)", pid,
		pnode != NULL ? pnode->pid : -1, name);

	return 0;
}

static struct pinfo_node* pinfo_create(int pid, int ppid)
{
	struct pinfo_node *node;
	struct process_info *pinfo;
	struct pinfo_node *pnode = NULL;


	/*
	 * At this point we should be in an rcu read-side
	 * critical section AND hold the ListLock
	 *
	 * List may have changed by the time I get here,
	 * so I need to search it again to make sure the
	 * PID I'm interested in still isn't present.
	 *
	 * I also need the PID's parent's entry so search
	 * for that while I'm here.
	 */
	list_for_each_entry(node, &ProcList.list, list) {
		if (node->pid == pid) {
			/*
			 * Desired entry is now in the list.
			 * Just return it.
			 */
			return node;
		} else if (node->pid == ppid) {
			pnode = node;
		}
	}

	if (pnode == NULL) {
		logWarn("Couldn't find parent %d for pid %d", ppid, pid);
	}

	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node) {
		logError("Failed to allocate node for pid %d", pid);
		return NULL;
	}
	pinfo = pinfo_alloc();
	if (!pinfo) {
		logError("Failed to allocate pinfo for pid %d", pid);
		kfree(node);
		return NULL;
	}
	node->pid = pid;
	node->pinfo = pinfo;
	INIT_LIST_HEAD(&node->children);
	node->parent = pnode;

	if (pinfo_init(node, pid, pnode)) {
		pinfo_free(node);
		return NULL;
	}

	return node;
}

static inline struct pinfo_node* proclist_search_rcu(int pid)
{
	struct pinfo_node *node;

	list_for_each_entry_rcu(node, &ProcList.list, list) {
		if (node->pid == pid)
			return node;
	}

	return NULL;
}

static inline struct pinfo_node* proclist_search(int pid)
{
	struct pinfo_node *node;

	list_for_each_entry(node, &ProcList.list, list) {
		if (node->pid == pid)
			return node;
	}

	return NULL;
}

static void proclist_clear(void)
{
	struct pinfo_node list;
	struct pinfo_node *node;
	struct list_head *pos;
	struct list_head *next;
	unsigned long flags = 0;

	/*
	 * Save the list and then wipe it out
	 */
	spin_lock_irqsave(&ListLock, flags);
	list = ProcList;
	INIT_LIST_HEAD(&ProcList.list);
	spin_unlock_irqrestore(&ListLock, flags);

	/*
	 * After the synchronize completes it is now
	 * safe to actually delete the nodes
	 */
	synchronize_rcu();
	list_for_each_safe(pos, next, &ProcList.list) {
		node = list_entry(pos, struct pinfo_node, list);
		list_del(pos);
		pinfo_free(node);
	}
}

static struct process_info* pinfo_begin_create(int pid, int ppid,
							 struct process_info_update_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));

	spin_lock_irqsave(&ListLock, ctx->flags);
	ctx->node = proclist_search(pid);
	if (!ctx->node) {
		/* Create a new pinfo */
		ctx->node = pinfo_create(pid, ppid);
		if (!ctx->node) {
			logError("Unable to create data for pid %d", pid);
			goto failure;
		}
		ctx->new_pinfo = ctx->node->pinfo;

		/*
		 * Setup a dummy old_pinfo so we can tell if any
		 * buffers get changed by the caller before
		 * it commits the new pinfo.
		 */
		ctx->old_pinfo = &ctx->dummy;
		*ctx->old_pinfo = *ctx->new_pinfo;

	} else {
		logError("pinfo_begin_create: Node already exists for pid: %d, ppid: %d", pid, ppid);
		goto failure;
	}

	return ctx->new_pinfo;

failure:
	spin_unlock_irqrestore(&ListLock, ctx->flags);
	return NULL;
}

static struct process_info* pinfo_begin_update(int pid,
					       struct process_info_update_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));

	spin_lock_irqsave(&ListLock, ctx->flags);
	ctx->node = proclist_search(pid);
	if (!ctx->node)
		goto failure;
	ctx->old_pinfo = node_to_pinfo(ctx->node);

	ctx->new_pinfo = pinfo_alloc();
	if (!ctx->new_pinfo)
		goto failure;

	*ctx->new_pinfo = *ctx->old_pinfo;
	return ctx->new_pinfo;

failure:
	spin_unlock_irqrestore(&ListLock, ctx->flags);
	return NULL;
}

static void pinfo_reclaim_process_info(struct rcu_head *rp)
{
	/* Get a hold of the structure that contains the rcu element. */
	struct process_info *fp = container_of(rp, struct process_info, rcu);

	/* Only clean the name if we were told that no one else is using it. */
	if (fp->flags & BIDE_PROC_FREE_NAME)
		kfree(fp->name);

	fp->name = NULL;
	pinfo_reuse(fp);
}

static inline void pinfo_finish_update(struct process_info_update_ctx *ctx)
{
	if (ctx->old_pinfo != &ctx->dummy) {
		/* It is an update */
		rcu_assign_pointer(ctx->node->pinfo, ctx->new_pinfo);
	} else {
		/* A new entry was created */
		list_add_tail_rcu(&ctx->node->list, &ProcList.list);
		if (ctx->node->parent)
			list_add_tail_rcu(&ctx->node->siblings,
					  &ctx->node->parent->children);

	}

	spin_unlock_irqrestore(&ListLock, ctx->flags);

	if (ctx->old_pinfo != &ctx->dummy) {
		/*
		 * The new pinfo may not have a new name, it may just be
		 * referencing the name that the old pinfo structure had.
		 * In that case the name should not be released when the
		 * pinfo structure is getting released.
		 */
		if (ctx->old_pinfo->name != ctx->new_pinfo->name)
			ctx->old_pinfo->flags |= BIDE_PROC_FREE_NAME;

		/*
		 * Schedule the reclamation of the pinfo struct when
		 * all the threads are done using it.
		 */
		call_rcu(&ctx->old_pinfo->rcu, pinfo_reclaim_process_info);

	} else {
		/*
		 * For a new pinfo I need to clear out any buffers
		 * that may have been replaced during creation.
		 */
		pinfo_replaced(ctx->old_pinfo, ctx->new_pinfo);
	}
}

static inline void pinfo_cancel_update(struct process_info_update_ctx *ctx)
{
	/*
	 * The caller is responsible for free'ing any new
	 * buffers. So I don't need to worry about differences
	 * between old and new pinfos
	 */
	if (ctx->old_pinfo != &ctx->dummy) {
		/*
		 * It was an update. Buffers are shared
		 * with the original so I don't want
		 * to delete them. Just schedule the pinfo
		 * for reuse and ignore its internals.
		 */
		pinfo_reuse(ctx->new_pinfo);
	} else {
		/*
		 * It was creating a new node. A full
		 * pinfo_free is required so any buffers
		 * alloc'ed during creation are properly removed
		 */
		pinfo_free(ctx->node);
	}
	spin_unlock_irqrestore(&ListLock, ctx->flags);
}

/************************************************************************
 * Public API
 ************************************************************************/
int __init proc_debug_init(void)
{
	sysfs_proc_init();
	logInfo("Proc debug initialized");
	return 0;
}

int __init proc_init(void)
{
	INIT_LIST_HEAD(&ProcList.list);
	sysfs_proc_init();
	logInfo("Proc initialized");
	return 0;
}

int __exit proc_exit(void)
{
	unsigned long flags = 0;

	sysfs_proc_exit();

	proclist_clear();

	spin_lock_irqsave(&AllocLock, flags);
	if (FreePInfo)
		kfree(FreePInfo);
	spin_unlock_irqrestore(&AllocLock, flags);

	return 0;
}

/*
 * A function to get, and create if necessary, a process_info struct
 * for a given pid.
 *
 * If a mapping from pid->process_info does not yet exist
 * one is created by cloning the parent's.
 *
 * @param   pid             The PID of the process we are interested in
 *
 * @return  A pointer to the process_info struct or NULL if one can't
 *          be found and created
 */
const struct process_info* __proc_get_info(int pid)
{
	struct pinfo_node *node;
	struct process_info *pinfo;

	node = proclist_search_rcu(pid);
	if (node == NULL) {
		logError("Node does not exist for pid: %d", pid);
	} else {
		pinfo = node_to_pinfo(node);
	}

	return pinfo;
}

/*
 * A function to create a process_info struct
 * for a given pid.
 *
 * If a mapping from pid->process_info does not yet exist
 * one is created by cloning the parent's.
 *
 * @param   pid             The PID of the process we are interested in
 *
 * @param   ppid             The PID of the parent process we are interested in
 *
 * @return  A pointer to the process_info struct or NULL if one can't
 *          be found and created
 */
const struct process_info* __proc_create_info(int pid, int ppid)
{
	struct pinfo_node *node;
	struct process_info *pinfo;

	node = proclist_search_rcu(pid);
	if (node == NULL) {
		struct process_info_update_ctx ctx;
		pinfo = pinfo_begin_create(pid, ppid, &ctx);
		if (pinfo)
			pinfo_finish_update(&ctx);
	} else {
		logError("proc_create_info: Node already exists for pid: %d, ppid: %d", pid, ppid);
		return NULL;
	}

	return pinfo;
}

/*
 * Helper function that retrieves the name of a given process
 * into the indicated buffer.
 *
 * This API is guaranteed to produce a NULL-terminated string
 * in buffer even if it is unable to find the process.
 *
 * @param pid           The PID of the process
 * @param buffer        Buffer to store the processes name
 * @param size          The size of the buffer in bytes.
 */
void proc_task_get_name(int pid, char *buffer, size_t size)
{
	const struct process_info *pinfo;

	pinfo = proc_get_info(pid);
	if (pinfo != NULL) {
		strlcpy(buffer, pinfo->name, size);
	} else {
		snprintf(buffer, size, "Unknown PID (%d)", pid);
	}
	proc_put_info(pinfo);
}

/*
 * Helper function that indicates if a given process is
 * in the 'initializing' state.
 *
 * A process is in the initializing state if it is being
 * forked from either init or zygote and the jump into
 * the new program's code hasn't happened yet.
 *
 * @param pid   The pid of the process
 *
 * @returnu     Non-zero if the process is in the initializing state,
 *              zero otherwise. If the process isn't found it is
 *              considered to NOT be in the initializing state.
 */
int proc_is_pid_initializing(int pid)
{
	const struct process_info *pinfo;
	int result = 0;

	pinfo = proc_get_info(pid);
	if (pinfo != NULL)
		result = ((pinfo->flags & BIDE_PROC_FLAG_INITIALIZING) != 0);
	proc_put_info(pinfo);

	return result;
}


/*
 * Returns the process_info for the specified process's parent.
 *
 * The information is for the processes original parent, the
 * one that spawned/forked it. Unlike the normal process hierarchy
 * a process is not re-parented if its parent dies. Instead we
 * remember the last state of the parent to be returned by
 * this API.
 *
 * On success, this API implicitly starts an RCU read-side critical section.
 * To complete/exit the section call proc_put_info.
 *
 * @note The parent process may no longer be present in the system,
 *       nor is its PID guaranteed to be unique. It may have been
 *       reused.
 *
 * @param pinfo	        The process_info, probably retrieved by
 *                      proc_get_info, whos parent is being requested.
 *
 * @returns             The parent's process_info, or NULL if the process
 *                      has no parent. This should only happen for the /init
 *                      process.
 */
const struct process_info *proc_get_parent(const struct process_info *pinfo) {
	if (pinfo->node->parent) {
		rcu_read_lock();
		return node_to_pinfo(pinfo->node->parent);
	} else {
		return NULL;
	}
}

/*
 * A function to indicate a process has 'exec' into
 * a new program.
 *
 * This API should only be called once the process has
 * moved into the new security domain. This is because
 * any pending security checks are completed at
 * this time.
 *
 * @param pid   The PID of the process being exec'ed
 * @param name  The name of the binary/process being exec'ed into
 */
void proc_task_exec(int pid, const char* name)
{
	struct process_info *pinfo;
	char *new_name = NULL;
	struct process_info_update_ctx ctx;

	pinfo = pinfo_begin_update(pid, &ctx);
	if (!pinfo)
		return;

	LOGD("Handling exec for pid %d: %s -> %s", pid, pinfo->name, name);

	new_name = kmalloc(strlen(name)+1, GFP_ATOMIC);
	if (!new_name) {
		logError("Unable to allocate buffer for name '%s'", name);
		goto cancel;
	}
	memcpy(new_name, name, strlen(name)+1);
	pinfo->name = new_name;
	pinfo_finish_update(&ctx);

	return;

cancel:
	pinfo_cancel_update(&ctx);
	if (new_name)
		kfree(new_name);
}

/*
 * A function to indicate a task has died.
 *
 * @param pid The PID of the process that died.
 */
void proc_task_free(int pid)
{
	struct pinfo_node *node;
	struct process_info *pinfo;
	unsigned long flags = 0;

	spin_lock_irqsave(&ListLock, flags);

	node = proclist_search(pid);
	if (node != NULL) {
		pinfo = node_to_pinfo(node);
		LOGD("proc %d (%s) is dying.", pinfo->pid, pinfo->name);
		list_del_rcu(&node->list);
		/*
		 * Node is now dead. Mark it such.
		 * I don't think I need to worry about doing the
		 * full rcu-update procedure since the only
		 * thing that cares about this flag are updaters
		 * and they don't happen concurrently.
		 */
		pinfo->flags |= BIDE_PROC_FLAG_DEAD;

		/*
		 * If the node has no children, it can be removed
		 * otherwise, I need to keep it around so that
		 * we have proper lineage information. If I do
		 * remove the node then I need to do the same
		 * check with its parent.
		 */
		if (list_empty(&node->children)) {
			while (node &&
			       list_empty(&node->children) &&
			       (pinfo->flags & BIDE_PROC_FLAG_DEAD))
			{
				LOGD("Marking proc %d (%s) for deletion.", pinfo->pid, pinfo->name);
				if (node->parent)
					list_del_rcu(&node->siblings);
				call_rcu(&node->rcu, pinfo_free_rcu);
				node = node->parent;
				if (node != NULL)
					pinfo = node_to_pinfo(node);
			}
		}
#ifdef BIDE_USER_DEBUG
		else if (node->kobj != NULL) {
			/* I can't delete the node yet, but I want to
			 * remove it from sysfs. So schedule some
			 * work to do that.
			 */
			sysfs_pinfo_release(node);
		}
#endif
	} else {
		LOGD("Unable to find dying proc %d", pid);
	}

	spin_unlock_irqrestore(&ListLock, flags);
}

/*
 * A function indicating that a processes' initialization
 * phase has completed.
 *
 * That means control is being passed to the application's
 * code and should no longer be truested.
 *
 * This API should only be called for processes spawned
 * from either init or zygote. It should also only be
 * called once the process is in the right security
 * domain as any pending security checks are completed
 * during this function call.
 *
 * @param pid   The PID of the process
 * @param sid   The SID of the processes' security domain
 */
void proc_task_init_complete(int pid, u32 sid)
{
	struct pinfo_node *node;
	struct process_info *pinfo;
	struct process_info_update_ctx ctx;
	int caps_ok;
	const struct cred *cred;
	kernel_cap_t cap_inheritable;
	kernel_cap_t cap_permitted;
	kernel_cap_t cap_effective;
	kernel_cap_t cap_denied;

	rcu_read_lock();
	node = proclist_search_rcu(pid);
	if (!node) {
		logWarn("Unexpected init complete call for unknown pid %d", pid);
		goto exit_from_rcu;
	}
	pinfo = node_to_pinfo(node);
	LOGD("Initialization complete for %s (%d)",
	     pinfo->name, pid);
	if (!(pinfo->flags & BIDE_PROC_FLAG_INITIALIZING)) {
		/*
		 * This is okay. It means an exec completed but not
		 * one that was started by /init. In that case it
		 * was never in the INITIALIZING state and
		 * security checks were not delayed.
		 */
		goto exit_from_rcu;
	}

	/* Verify security state at this point */
	cred = rcu_dereference(current->cred);
	caps_ok = caps_verify(pid, sid, &cred->cap_effective,
			      &cred->cap_inheritable, &cred->cap_permitted,
			      &cap_denied);
	if (!caps_ok) {
		/* Need to stash these so I can use them outside of rcu */
		cap_effective = cred->cap_effective;
		cap_inheritable = cred->cap_inheritable;
		cap_permitted = cred->cap_permitted;
	}
	/* Security state verification DONE by this point */

	rcu_read_unlock();

	/* Process any security failures at this point */
	if (!caps_ok)
		caps_gen_report(pid, &cap_effective, &cap_inheritable,
				&cap_permitted, &cap_denied);

	pinfo = pinfo_begin_update(pid, &ctx);
	if (pinfo != NULL) {
		pinfo->flags &= ~BIDE_PROC_FLAG_INITIALIZING;
		pinfo_finish_update(&ctx);
	}

	return;

exit_from_rcu:
	rcu_read_unlock();
}
