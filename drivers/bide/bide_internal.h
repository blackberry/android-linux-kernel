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

#ifndef __BIDE_INTERNAL_H__
#define __BIDE_INTERNAL_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/crypto.h>
#include <linux/sched.h>
#include <linux/limits.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/capability.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#ifdef BID_USER_DEBUG
#include <linux/atomic.h>
#endif

/* SELinux internals */
#include <objsec.h>


/* Well known PIDs in the system */
#define IDLE_PID			0
#define INIT_PID			1
#define KTHREAD_PID			2

/* UID/GID numbers in the system, from the following files:
 * - android_filesystem_config.h
 * - Process.java
 *
 * NOTE: These are userspace UIDs/GIDs
 */
#define ROOT_UID            0
#define ROOT_GID            0
#define SYSTEM_GID          1000
#define SYSTEM_UID          1000
#define PHONE_UID           1001
#define BLUETOOTH_UID       1002
#define WIFI_UID            1010
#define INSTALLER_UID       1012
#define DHCP_UID            1014
#define GPS_UID             1021
#define CLAT_UID            1029
#define SHELL_UID           2000
#define RESET_CAUSE_UID     2904
#define BIDE_UID            8000
#define NOBODY_UID          9999
#define APP_UID             10000
#define USER_UID            100000

#define ROOT_KUID			KUIDT_INIT(ROOT_UID)
#define ROOT_KGID			KGIDT_INIT(ROOT_GID)
#define SYSTEM_KGID			KGIDT_INIT(SYSTEM_GID)

#define CRED_UID(cred, var)		from_kuid((cred)->user_ns, (cred)->var)
#define CRED_GID(cred, var)		from_kgid((cred)->user_ns, (cred)->var)
int dev_init(void);
int dev_exit(void);

#define KBIDE			0
#define JBIDE			1

#define OS_VERSION_BUILD_SIZE       6
#define BUILD_MAX_LEN               10

/* KBIDE sensors.  Preserve Order : if you want to add more sensors,
 * add them AT THE END and update the SN_ARRAY_SIZE below.
 * SN_ARRAY_SIZE must be <= BIDE_STORAGE_NUMBER_SENSORS */
#define SN_ARRAY_SIZE	15
static const char* const sensor_names[SN_ARRAY_SIZE] = {
		SN_ROOT_PROCESS_DETECTOR,
		SN_ESCALATED_GID,
		SN_SYSTEM_UID,
		SN_ESCALATED_UID,
		SN_MPROTECT,
		SN_CAPSET,
		SN_LOW_MMAP_ADDR,
		SN_NODEV,
		SN_NOSUID,
		SN_INVALID_PROPERTY,
		SN_MISMATCHED_PROC_HASH,
		SN_SELINUX_CHANGED,
		SN_SELINUX_DISABLED,
		SN_PATHTRUST_DISABLED,
		SN_KBIDE_TZ_NONCE_MISMATCH
        };

int tz_init(void);
int tz_exit(void);
int tz_init_kernel(void);
int tz_add_section(void *p, unsigned sz);
int tz_remove_section(void *p, unsigned sz);
int tz_gen_keypair(bide_keystore_t *ks, bide_p10_t *p10);
int tz_set_keypair(bide_keystore_t *ks);
int tz_verify_keypair(bide_keystore_t *ks);
int tz_sign_data(char *jrep, unsigned jrep_sz, char *tzrep, unsigned tzrep_sz,
		uint8_t *sig, unsigned sig_sz);
int tz_set_build_version(build_version_t* build_version);
int tz_set_compromised_state(uint32_t sensor, unsigned called_from);
int tz_read_sensor_state(uint32_t *kbide_sensors, unsigned kbide_sz,
		uint32_t *jbide_sensors, unsigned jbide_sz);

int secop_init(void);
int secop_exit(void);

int report_init(void);
int report_exit(void);

int thread_init(void);
int thread_exit(void);

int bide_netlink_init(void);
int bide_netlink_exit(void);

int vma_init(void);
int vma_exit(void);

struct crypto_context {
	struct hash_desc desc;
};

#define HASH_ALG_SHA256		0
#define HASH_ALG_SHA512		1
#define HASH_ALG_LAST			HASH_ALG_SHA512

int crypto_begin(unsigned alg, struct crypto_context *ctx);
int crypto_update(struct crypto_context *ctx, const void *data, unsigned sz);
int crypto_update_page(struct crypto_context *ctx, struct page *page, unsigned sz, unsigned offset);
int crypto_end(struct crypto_context *ctx, uint8_t *hash, unsigned hash_sz);
int crypto_once(unsigned alg, const void *data, unsigned sz, uint8_t *hash, unsigned hash_sz);
int crypto_get_digestsize(struct crypto_context *ctx, int *out);

#define SIGNATURE_INCIDENT	-1
#define MAX_XML_MSG_LENGTH	100

struct report_queue_node {
	char			*report;
	unsigned		report_sz;
	struct list_head	queue;
};

int report_inject_incident(void *data);
int report_incident(int severity, const char *sensor, const char *msg,
		struct task_struct *task);
int report_dequeue(char **buf_ptr, unsigned *buf_sz);
int report_enqueue(bide_reports_t *buf);
int report_add_to_tail(struct report_queue_node *node);
int report_end(bide_reports_t *rep);
int report_begin(bide_reports_t *rep);

int audit_init(void);

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
int ctl_resign_report(void *p);
int ctl_kick_process_scan(void *p);
int ctl_set_property_check(void *p);
int ctl_add_system_server_pid(void *p);
int ctl_get_system_server_pid(void *p);
int ctl_add_process(void *p);
int ctl_zygote_exec_embryo(void *p);
int ctl_set_build_version(void *p);
int ctl_report_issue(void *p);
int ctl_set_private_key(void *p);
int ctl_check_nonce(uint8_t *hash);
void ctl_increment_nonce(void);

/*
 * Verify that the specified pid is allowed to have
 * the indicated capabilities.
 *
 * The list of allowed capabilities is retrieved from
 * selinux using the specified security domain
 *
 * If a value for denied is passed it is populated with
 * the capabilities that are requested, but are denied. If
 * no capabilities are denied it will be set to 0.
 *
 * @NOTE This function is safe to call within a critical section
 *       or rcu read-lock. It will not block.
 *
 * @param in  pid		The PID of the process to check
 * @param in  sid		The ID of the security domain that defines
 *				what capabilities are allowed
 * @param in  effective		The effective capabilities the process wants
 * @param in  inheritable	The inheritable capabilities the process wants
 * @param in  permitted		The permitted capabilities the process wants
 * @param out denied		If non-NULL will be populated with the
 *				capabilities that are denied to the process
 *
 * @return 1 If the process is allowed these capabilities
 *         0 If the process is NOT allowed these capabilities
 */
int caps_verify(int pid, u32 sid,
		const kernel_cap_t *effective,
		const kernel_cap_t *inheritable,
		const kernel_cap_t *permitted,
		kernel_cap_t *denied);

/*
 * Generates a KBIDE report indicating capabilities were requested
 * that are not allowed for a given process.
 *
 * It is intended that this API will be called after a call to
 * caps_verify returns 0.
 *
 * @NOTE This functions is NOT safe to call within an interrupt
 *       context, critical section or rcu read lock.
 *
 * @param in  pid		The PID of the process to check
 * @param in  effective		The effective capabilities the process wants
 * @param in  inheritable	The inheritable capabilities the process wants
 * @param in  permitted		The permitted capabilities the process wants
 * @param out denied		The capabilities that were denied to the process
 */
void caps_gen_report(int pid,
		const kernel_cap_t *effective,
		const kernel_cap_t *inheritable,
		const kernel_cap_t *permitted,
		const kernel_cap_t *denied);

int util_get_time_string(char *buf, unsigned buf_sz);
int util_get_task_path(struct mm_struct *mm, char *path, unsigned path_sz);
int util_get_task_cmdline(struct task_struct *task, char *cmdline, unsigned sz);
int util_base64_encode(uint8_t *src, unsigned src_sz, char *dst, unsigned dst_sz);
int util_is_task_root(struct task_struct *t);
unsigned long util_get_real_parent_pid(void);
int util_get_tgid(struct task_struct *task);
void util_save_build_fingerprint(const char* fp);
const char* util_get_build_fingerprint(void);
int util_sensor_name_to_integer(const char* sensor);
const char* util_integer_to_sensor_name(int sensor_id);
int utils_bide_storage_build_to_number(const char *build, uint32_t *number);
int utils_bide_storage_number_to_build(uint32_t number, char *build);

static inline int util_get_current_uid(void)
{
	const struct cred *cred;
	int uid;
	rcu_read_lock();
	cred = rcu_dereference(current->cred);
	uid = CRED_UID(cred, uid);
	rcu_read_unlock();

	return uid;
}

static inline u32 util_get_current_sid(void)
{
	const struct task_security_struct *sec;
	u32 sid;
	rcu_read_lock();
	sec = rcu_dereference(current->cred)->security;
	sid = sec->sid;
	rcu_read_unlock();

	return sid;
}

#define CAP2HEXSTR_BUFFER_SIZE (_KERNEL_CAPABILITY_U32S*8+3)
char* util_cap2hexstr(const kernel_cap_t *caps, char *buffer, size_t sz);

typedef enum {
	CAP_CURRENT,
	CAP_NEW
} CAP_STATE;

int xml_capset_msg(char *buf, unsigned sz, const kernel_cap_t *eff,
		const kernel_cap_t *inh, const kernel_cap_t *per,
		CAP_STATE state);
int xml_denied_caps_msg(char *buf, unsigned sz, const kernel_cap_t *denied);
int xml_property_msg(char *buf, unsigned sz,
		const char *name,const char *value);
int xml_nonce_mismatch_msg(char *buf, unsigned sz, int error_code);
int xml_mount_path_msg(char *buf, unsigned sz, const char *path);
int xml_vma_msg(char *buf, unsigned sz, const char *proc, const char *section);
int xml_cat(bide_reports_t *rep, const char *msg);
int xml_jbide_hash_msg(bide_reports_t *rep, const char *jhash, unsigned hash_sz);
int xml_process_dump_msg(bide_reports_t *rep, struct task_struct *task);
int xml_begin_report(bide_reports_t *rep);
int xml_end_report(bide_reports_t *rep);
int xml_jbide_sensor_msg(bide_reports_t *rep, char *msg);
int xml_kbide_sensor_msg(bide_reports_t *rep, char *msg);
int xml_begin_incident_report(bide_reports_t *rep, int severity, const char *sensor);
int xml_end_incident_report(bide_reports_t *rep);


#define GID_REAL 1
#define GID_EFFECTIVE 2
#define GID_SUPPLEMENTARY 4
typedef struct {
	unsigned gid_types;
	kgid_t gid;
} gid_info_t;
int xml_escalated_gid_msg(char *buf, unsigned sz, const gid_info_t *infos,
			  int num_infos);

unsigned tlv_size(unsigned value_sz);
int tlv_write_value(void *buf, unsigned sz, uint32_t tag, const uint8_t *value,
		unsigned val_sz);

#define AUTH_PERM_PRIVILEGED_GID        (1 << 0)
#define AUTH_PERM_SYSTEM_UID            (1 << 1)
#define AUTH_PERM_ZYGOTEMGR             (1 << 2)
#define AUTH_PERM_PRIVILEGED_LINEAGE    (1 << 3)
#define AUTH_PERM_PRIVILEGED_CHILDREN   (1 << 4)
#define AUTH_PERM_PRIVILEGED_ZYGOTE     (1 << 5)
#define AUTH_PERM_INSTALLD              (1 << 6)
#define AUTH_PERM_BUGREPORT             (1 << 7)
#define AUTH_PERM_ZYGOTEMGR_CHILD       (1 << 8)
#define AUTH_PERM_ADBD                  (1 << 9)
#define AUTH_PERM_DUMPSTATE             (1 << 10)
#define AUTH_PERM_PRIVILEGED            (1 << 11)

int auth_remove_pid(int pid, int *perm);
int auth_add_pid(int pid, int perm);
int auth_add_debug_pid(int pid, bool allow_children);
int auth_process_debug_pids(int debug_token_present);
int auth_drop_pid_permission(int pid, int perm);
int auth_check_permission(int pid, int perm);
int auth_has_permission(int pid, int perm);
int auth_does_not_have_permission(int pid, int perm);
int auth_check_parents_permission(struct task_struct *task, int perm);
int auth_check_property(const char *name, const char *value);
int auth_check_banned_caps(struct task_struct *task);
int auth_debugfs_open(struct inode *inode, struct file *file);
int auth_init(void);
int auth_exit(void);

int bide_perm_search(int pid);
int bide_perm_insert(int pid, int perm);
int bide_perm_drop(int pid, int perm);
int bide_perm_remove(int pid);
int perm_init(void);
int perm_exit(void);

int debugfs_init(void);
int debugfs_exit(void);

#define COUNT_OF(a) ARRAY_SIZE(a)

#ifdef BID_USER_DEBUG
#define STAT_ADD(id)	stat_increment(id)

void stat_increment(int fct_id);
void stat_display(void);
int stat_init(void);

void bide_perm_dump_list(void);
void auth_dump_list(void);
void vma_print_section_tree(void);
void ctl_print_nonce(void);
void netlink_print_tree_size(void);
void report_print_queue(void);


/*************************************************************************/
/*                    THIS SECTION CAN BE MODIFIED                       */
/*************************************************************************/

/* To count the number of calls made to a function:
 * -> Add an identifier to the enum (fct_ID).
 * -> Add a name to stat_fct_name.
 * -> Add the line "stat_increment(<fct_ID>);" in the function you want
 *    to monitor.
 *
 * Note: The identifier and the name must be in the same order.
 * Note2: Do not remove "stat_last_element" from the enum. It must stay
 *        the last value enumerated.
 */
typedef enum {
	stat_secop_task_free,
	stat_secop_bprm_set_creds,
	stat_secop_task_fix_setuid,
	stat_secop_task_fix_setgid,
	stat_secop_task_set_groups,
	stat_secop_sb_mount,
	stat_secop_file_mmap,
	stat_secop_capset,
	stat_secop_file_mprotect,
	stat_secop_module_phys_mem,
	stat_secop_kernel_module_init,
	stat_secop_kernel_module_free,
	stat_secop_task_created_notify,

	stat_auth_check_capabilities,
	stat_auth_check_banned_caps,
	stat_auth_check_parents_permission,
	stat_auth_check_property,
	stat_auth_check_permission,
	stat_auth_remove_debug_pid,
	stat_auth_remove_pid,
	stat_auth_drop_pid_permission,
	stat_auth_add_pid_lock,
	stat_auth_internal_process_debug_pids,
	stat_auth_add_debug_pid,
	stat_auth_process_debug_pids,
	/*
	 * Add your identifier here
	 */
	stat_last_element	/* This value must stay the last element */
}STAT_FCT_ID;

static const char* const stat_fct_name[] = {
					"secop_task_free",
					"secop_bprm_set_creds",
					"secop_task_fix_setuid",
					"secop_task_fix_setgid",
					"secop_task_set_groups",
					"secop_sb_mount",
					"secop_file_mmap",
					"secop_capset",
					"secop_file_mprotect",
					"secop_module_phys_mem",
					"secop_kernel_module_init",
					"secop_kernel_module_free",
					"secop_task_created_notify",

					"auth_check_capabilities",
					"auth_check_banned_caps",
					"auth_check_parents_permission",
					"auth_check_property",
					"auth_check_permission",
					"auth_remove_debug_pid",
					"auth_remove_pid",
					"auth_drop_pid_permission",
					"auth_add_pid_lock",
					"auth_internal_process_debug_pids",
					"auth_add_debug_pid",
					"auth_process_debug_pids"
					/*
					* Add your function name here
					*/
					};

/*************************************************************************/
#else
#define STAT_ADD(id)
#endif	/* BID_USER_DEBUG */
#endif	/* __BIDE_INTERNAL_H__ */
