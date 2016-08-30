/*
 * Copyright (C) 2014 BlackBerry Limited
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

#ifndef __BIDE_INTERNAL_H__
#define __BIDE_INTERNAL_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/crypto.h>
#include <linux/sched.h>
#include <linux/limits.h>
#include <linux/err.h>

/* Well known PIDs in the system */
#define INIT_PID			1
#define KTHREAD_PID			2

/* UID numbers in the system, from the following files:
 - adroid_filesystem_config.h
 - Processs.java */
#define ROOT_UID			0
#define SYSTEM_UID			1000
#define PHONE_UID			1001
#define BLUETOOTH_UID			1002
#define WIFI_UID			1010
#define INSTALLER_UID			1012
#define DHCP_UID			1014
#define GPS_UID			1021
#define CLAT_UID			1029
#define SHELL_UID			2000
#define BIDE_UID			8000
#define NOBODY_UID			9999
#define APP_UID			10000
#define USER_UID			100000

int dev_init(void);
int dev_exit(void);

int tz_init(void);
int tz_exit(void);
int tz_init_kernel(void);
int tz_add_section(void *p, unsigned sz);
int tz_remove_section(void *p, unsigned sz);
int tz_gen_keypair(bide_keystore_t *ks, bide_p10_t *p10);
int tz_set_keypair(bide_keystore_t *ks);
int tz_sign_data(char *jrep, unsigned jrep_sz, char *tzrep, unsigned tzrep_sz,
		uint8_t *sig, unsigned sig_sz);

int secop_init(void);
int secop_exit(void);

int report_init(void);
int report_exit(void);

int thread_init(void);
int thread_exit(void);

int netlink_init(void);
int netlink_exit(void);

int vma_init(void);
int vma_exit(void);

struct crypto_context {
	struct hash_desc desc;
};

#define HASH_ALG_SHA256		0
#define HASH_ALG_SHA512		1
#define HASH_ALG_LAST			HASH_ALG_SHA512

int crypto_begin(unsigned alg, struct crypto_context *ctx);
int crypto_update(struct crypto_context *ctx, void *data, unsigned sz);
int crypto_update_page(struct crypto_context *ctx, struct page *page, unsigned sz, unsigned offset);
int crypto_end(struct crypto_context *ctx, uint8_t *hash, unsigned hash_sz);
int crypto_once(unsigned alg, void *data, unsigned sz, uint8_t *hash, unsigned hash_sz);
int crypto_get_digestsize(struct crypto_context *ctx, int *out);

int report_incident(int severity, const char *sensor, const char *msg,
		struct task_struct *task);
int report_dequeue(char **buf_ptr, unsigned *buf_sz);
int report_enqueue(bide_reports_t *buf);
int report_bind_current(bide_reports_t *rep);
int report_unbind_current(bide_reports_t **rep);
int report_end(bide_reports_t *rep);
int report_begin(bide_reports_t *rep);

#define VMA_FLAG_DISALLOW_NEW_SECTIONS		1

int vma_scan_processes(void);
int vma_scan_task(struct task_struct *task, int flags);

struct rb_root;
struct rb_node;

int hash_search(struct rb_root *root, const char *key, unsigned key_sz, void **out);
int hash_insert(struct rb_root *root, const char *key, unsigned key_sz, void *data);
int hash_remove(struct rb_root *root, const char *key, unsigned key_sz, void **out);
int hash_get(struct rb_node *n, void **out);

void add_pid_for_system_uid(int pid);
void add_pid_for_privileged_groups(int pid);

int ctl_init(void);
int ctl_exit(void);
int ctl_snapshot_complete(void);
int ctl_first_boot_check(void *p);
int ctl_snapshot_initialize(void *p);
int ctl_generate_keys(void *p);
int ctl_generate_report(void *p);
int ctl_kick_process_scan(void *p);
int ctl_set_property_check(void *p);
int ctl_add_system_server_pid(void);
int ctl_get_system_server_pid(void *p);
int ctl_add_process(void *p);


int caps_add_process(const char *path, unsigned path_sz, int pid);
int caps_update_process(int pid, int caps);
void caps_clean_list(int all);
int caps_get_caps_for_process(int pid);
void caps_print(void);
int caps_init(void);
int caps_exit(void);

int util_get_time_string(char *buf, unsigned buf_sz);
int util_get_task_path(struct mm_struct *mm, char *path, unsigned path_sz);
int util_get_task_cmdline(struct task_struct *task, char *cmdline, unsigned sz);
int util_base64_encode(uint8_t *src, unsigned src_sz, char *dst, unsigned dst_sz);
int util_is_task_root(struct task_struct *t);
unsigned long util_get_real_parent_pid(void);
int util_get_tgid(struct task_struct *task);

typedef enum {
	CAP_CURRENT,
	CAP_NEW
} CAP_STATE;

int xml_capset_msg(char *buf, unsigned sz, const kernel_cap_t *eff,
		const kernel_cap_t *inh, const kernel_cap_t *per,
		CAP_STATE state);
int xml_property_msg(char *buf, unsigned sz,
		const char *name,const char *value);
int xml_mount_path_msg(char *buf, unsigned sz, const char *path);
int xml_vma_msg(char *buf, unsigned sz, const char *proc, const char *section);
int xml_jbide_hash_msg(char *buf, unsigned sz, const char *jhash, unsigned hash_sz);
int xml_process_dump_msg(char *buf, unsigned sz, struct task_struct *task);
int xml_process_cmdline_msg(char *buf, unsigned sz, struct task_struct *task);
int xml_begin_report(char *buf, unsigned sz);
int xml_end_report(char *buf, unsigned sz);
int xml_begin_incident_report(char *buf, unsigned sz, int severity, const char *sensor);
int xml_end_incident_report(char *buf, unsigned sz);

unsigned tlv_size(unsigned value_sz);
int tlv_write_value(void *buf, unsigned sz, uint32_t tag, const uint8_t *value,
		unsigned val_sz);

#define AUTH_PERM_PRIVILEGED_GID	(1 << 0)
#define AUTH_PERM_SYSTEM_UID		(1 << 1)
#define AUTH_PERM_ZYGOTEMGR		(1 << 2)
#define AUTH_PERM_PRIVILEGED_LINEAGE	(1 << 3)
#define AUTH_PERM_PRIVELEGED_CHILDREN	(1 << 4)
#define AUTH_PERM_PRIVILEGED_ZYGOTE	(1 << 5)
#define AUTH_PERM_INSTALLD		(1 << 6)
#define AUTH_PERM_BUGREPORT		(1 << 7)
#define AUTH_PERM_ZYGOTEMGR_CHILD	(1 << 8)

int auth_remove_pid(int pid, int *perm);
int auth_add_pid(int pid, int perm);
int auth_drop_pid_permission(int pid, int perm);
int auth_check_permission(int pid, int perm);
int auth_check_parents_permission(struct task_struct *task, int perm);
int auth_check_capabilities(int uid, int caps);
int auth_check_property(const char *name, const char *value);
int auth_check_banned_caps(struct task_struct *task);
int auth_init(void);
int auth_exit(void);

#define COUNT_OF(a) ARRAY_SIZE(a)

#endif
