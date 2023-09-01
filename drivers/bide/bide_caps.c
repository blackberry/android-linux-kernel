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

/* Kernel Includes */
#include <linux/errno.h>
#include <linux/capability.h>

/* SELinux */
#include <avc.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/*************************************************************************/
#define EXTRA_XML_MESSAGE_LEN		256

/*************************************************************************/

#if CAP_LAST_CAP > 63
#error Fix caps_verify() to handle capabilities > 63
#endif

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
		kernel_cap_t* denied)
{
	kernel_cap_t all_caps = {{0}};
	kernel_cap_t local_denied;
	int cap;
	int idx;
	int mask;
	struct av_decision avd;
	int allowed = 1;
	int sclass;
	int rc;
	int uid;

	if (denied == NULL)
		denied = &local_denied;
	memset(denied, 0, sizeof(*denied));

	/*
	 * A root process is allowed any capabilities.
	 *
	 * It might be nice to harden this a bit as selinux
	 * may restrict which capabilities a root process can
	 * actually use. However if that is so, the effective and
	 * permitted caps will still be all set. So from our
	 * point of view it wants more than it needs.
	 */
	uid = util_get_current_uid();
	if ((uid % USER_UID) == ROOT_UID)
		return 1;

	for (idx = 0; idx < _KERNEL_CAPABILITY_U32S; ++idx) {
		all_caps.cap[idx] |= effective->cap[idx] |
			inheritable->cap[idx] |
			permitted->cap[idx];
	}

	for (cap = 0; cap <= CAP_LAST_CAP; ++cap) {
		idx = CAP_TO_INDEX(cap);
		mask = CAP_TO_MASK(cap);
		if (all_caps.cap[idx] & mask) {
			if (idx == 0) {
				sclass = SECCLASS_CAPABILITY;
			} else if (idx == 1) {
				sclass = SECCLASS_CAPABILITY2;
			} else {
				logError("Capability %d out of range", cap);
				BUG();
				break;
			}
			rc = avc_has_perm_noaudit(sid, sid, sclass, mask,
						  AVC_STRICT, &avd);
			if (rc) {
				denied->cap[idx] |= mask;
				allowed = 0;
			}
		}
	}

	if (!allowed) {
		char buf_e[CAP2HEXSTR_BUFFER_SIZE];
		char buf_i[CAP2HEXSTR_BUFFER_SIZE];
		char buf_p[CAP2HEXSTR_BUFFER_SIZE];
		char buf_d[CAP2HEXSTR_BUFFER_SIZE];
		char name[BIDE_MAX_FILE_PATH];

		proc_task_get_name(pid, name, sizeof(name));
		logInfo("Process %d (%s) wants caps eff=%s inher=%s perm=%s but is denied %s",
			pid, name,
			util_cap2hexstr(effective, buf_e, sizeof(buf_e)),
			util_cap2hexstr(inheritable, buf_i, sizeof(buf_i)),
			util_cap2hexstr(permitted, buf_p, sizeof(buf_p)),
			util_cap2hexstr(denied, buf_d, sizeof(buf_d)));
	}

	return allowed;
}

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
		const kernel_cap_t *denied)
{
	char msg[EXTRA_XML_MESSAGE_LEN] = { 0 };
	char *p = msg;
	int rc;
	unsigned sz = sizeof(msg);

	/* Compile a capset message */
	rc = xml_capset_msg(p,
			    sz,
			    effective,
			    inheritable,
			    permitted,
			    CAP_NEW);
	p += rc;
	sz -= rc;

	if (rc >= 0)
		rc = xml_denied_caps_msg(p, sz, denied);

	if (rc >= 0)
		p = msg;
	else
		p = NULL;
		/* Send an incident report to JBIDE */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_CAPSET,
				     p,
				     current);

	if (rc)
		logDebug("Failed on report_incident(SN_CAPSET). rc=%d.", rc);
}

