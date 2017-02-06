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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/selinux_netlink.h>
#include <linux/version.h>
#include <net/sock.h>

/* Access to security_context_to_sid selinux internal */
#include <security.h>

#include "pathtrust_private.h"

/*
 * stores a list of banned sid values
 */
struct pathtrust_selctx {
	u32 sid;
	struct list_head lhead;
};

static LIST_HEAD(pathtrust_selctx_list);
static DEFINE_MUTEX(pathtrust_selctx_list_mutex);

/*
 * Clear privileged context list on a list reload
 */
static void pathtrust_selctx_clear(void)
{
	struct pathtrust_selctx *cur, *next;

	mutex_lock(&pathtrust_selctx_list_mutex);

	/* clean linked list */
	list_for_each_entry_safe(cur, next, &pathtrust_selctx_list, lhead) {

		list_del(&cur->lhead);
		kfree(cur);
	}
	mutex_unlock(&pathtrust_selctx_list_mutex);
}

/*
 * Add selinux context to list of privileged contexts
 * to watch
 */
static int pathtrust_add_selctx(const char *ctx)
{
	u32 sid;
	struct pathtrust_selctx *nctx;
	int err = 0;

	if (!ctx)
		return -EINVAL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	err = security_context_to_sid(ctx, strlen(ctx), &sid, GFP_KERNEL);
#else
	err = security_context_to_sid(ctx, strlen(ctx), &sid);
#endif
	if (err) {
		pr_err("sid conversion of '%s' failed\n", ctx);
		goto out;
	} else{
		pr_info("added context '%s' as sid '%u'\n", ctx, sid);
	}

	nctx = kzalloc(sizeof(*nctx), GFP_KERNEL);

	if (unlikely(!nctx)) {
		err = -ENOMEM;
		pr_err("no memory for context node\n");
		goto out;
	}

	nctx->sid = sid;

	mutex_lock(&pathtrust_selctx_list_mutex);
	INIT_LIST_HEAD(&nctx->lhead);
	list_add_tail(&nctx->lhead, &pathtrust_selctx_list);
	mutex_unlock(&pathtrust_selctx_list_mutex);

out:

	return err;
}

/*
 * Parse pathtrust context config file lines
 *
 * data: nul terminated string of line delimited
 *       context data to load
 */
int pathtrust_selctx_load(char *data, ssize_t size)
{
	char *item;
	int err = 0;

	if (size == 0)
		return -EINVAL;

	/* clear list in case of reload */
	pathtrust_selctx_clear();

	if (pathtrust_debug)
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 1, data,
			size + 1, 1);

	/* Load configured whitelisted pathnames */
	while ((item = strsep(&data, "\n")) != NULL) {

		/* skip whitespace */
		while (isspace(*item))
			item++;

		/* skip empty items and comments */
		if (!*item || *item == '#')
			continue;

		/* If we have an error, stop the loop*/
		err = pathtrust_add_selctx(item);
		if (err)
			break;
	}

	return err;
}

/*
 * Return true if sid is privileged and should
 * be blocked from executing an untrusted source
 */
bool is_forbidden_sid(u32 sid)
{
	bool found = false;
	struct pathtrust_selctx *ctx;

	mutex_lock(&pathtrust_selctx_list_mutex);

	list_for_each_entry(ctx, &pathtrust_selctx_list, lhead) {
		if (ctx->sid == sid) {
			found = true;
			break;
		}
	}

	mutex_unlock(&pathtrust_selctx_list_mutex);
	return found;
}
