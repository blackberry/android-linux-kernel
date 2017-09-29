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

#include <linux/init.h>
#include <linux/list.h>
#include <linux/lsm_audit.h>
#include <linux/types.h>

/*
 * Pathtrust enforcement flag
 */
#if defined(CONFIG_SECURITY_PATHTRUST_DEVELOP) || defined(CONFIG_SECURITY_PATHTRUST_BOOTPARAM)
extern int pathtrust_enforce;
#else
#define pathtrust_enforce 1
#endif

#ifdef CONFIG_SECURITY_PATHTRUST_DEVELOP
extern int pathtrust_debug;
#else
#define pathtrust_debug 0
#endif

/*
 * Whitelisted block device storage
 */
struct pathtrust_devnode {
	dev_t dev;
	struct list_head lhead;
};

/*
 * Whitelisted pathname storage
 */
struct pathtrust_pathnode {
	char *pathname;
	struct list_head lhead;
};

/*
 * LSM audit data used in common_audit_data
 */
struct pathtrust_audit_data {
	char type;
#define PATHTRUST_AUDIT_TYPE_EXEC		1 /* Binary Execution */
#define PATHTRUST_AUDIT_TYPE_MMAP		2 /* PROT_EXEC MMAP */
#define PATHTRUST_AUDIT_TYPE_KMOD		3 /* Kernel Module Loading (finit_module) */
#define PATHTRUST_AUDIT_TYPE_KMODA		4 /* Anonymous Kernel Module Loading (init_module) */
#define PATHTRUST_AUDIT_TYPE_MOUNT		5 /* Mount */
#define PATHTRUST_AUDIT_TYPE_WHITELIST	6 /* Device Whitelisting */
#define PATHTRUST_AUDIT_TYPE_IOCTL		7 /* ioctl call to know if file can be trusted */
#define PATHTRUST_AUDIT_TYPE_FIRM		8 /* firmware loading */

	char btype;
#define PATHTRUST_AUDIT_BTYPE_NONE		1 /* All block types */
#define PATHTRUST_AUDIT_BTYPE_ROOT		2 /* euid=0|egid=0 blocking */
#define PATHTRUST_AUDIT_BTYPE_CAPS		3 /* forbidden caps blocking */
#define PATHTRUST_AUDIT_BTYPE_SELCTX	4 /* forbidden selinux context blocking */
	bool allowed;
	dev_t dev;
	char *extra;
 };

int __init pathtrust_fs_init(void);
bool has_banned_caps(void);
bool is_root(const struct cred *cred);
void pathtrust_lsm_audit(struct common_audit_data *cad);
void ptnl_notify_enforce(int enforce);

#ifdef CONFIG_SECURITY_PATHTRUST_SELINUX
bool is_forbidden_sid(u32 sid);
int pathtrust_selctx_load(char *data, ssize_t size);
#else
bool is_forbidden_sid(u32 sid) { return false; }
#endif
