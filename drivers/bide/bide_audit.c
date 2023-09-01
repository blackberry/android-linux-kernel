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
#include <linux/err.h>
#include <linux/string.h>
#include <linux/threads.h>

/* SELinux internals */
#include <avc.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

#define TLV_TYPE_BUNDLE		100
#define TLV_TYPE_AUDIT		120
#define TLV_TYPE_AUDIT_PNAME	121
#define TLV_TYPE_AUDIT_SNAPSHOT	122

/*************************************************************************/
#ifdef AVEN_109207
/*
 * This function extracts the pid from the buffer.
 *
 * @param   buffer              The audit denial message buffer.
 *
 * @return  0+                  The pid.
 *          -EINVAL             Invalid argument.
 *          -ERANGE             Math result not representable.
 *          -EBADE              Invalid exchange.
 */
static int audit_extract_pid(char *buffer)
{
	const char *const needle = "pid=";
	char *location = NULL;
	int i = 0;
	int pid_num = 0;

	if (!buffer)
		return -EINVAL;

	location = strstr(buffer, needle);
	if (!location) {
		logError("audit_extract_pid(): could not find pid.");
		return -EBADE;
	}

	location += strlen(needle);

	while (location[i] >= '0' && location[i] <= '9') {
		pid_num *= 10;
		pid_num += location[i] - '0';
		i++;
		if (pid_num > PID_MAX_LIMIT) {
			logError("audit_extract_pid(): pid too large.");
			return -ERANGE;
		}
	}

	if (pid_num == 0 && location[0] != '0') {
		logError("audit_extract_pid(): no provided pid num.");
		return -EBADE;
	}

	return pid_num;
}

/*************************************************************************/

/*
 * This function sends the audit denial to the java size in order to be
 * added to the audit denial database.
 *
 * @param   buffer              The audit denial message buffer.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ERANGE             Math result not representable.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int audit_send_to_database(char *buffer)
{
	int rc = 0;
	struct report_queue_node *node = NULL;
	int buffer_sz = 0;
	int pid = 0;
	const struct process_info *pinfo = NULL;
	int name_sz = 0;
	unsigned offset_sz = 0;

	if (!buffer)
		return -EINVAL;

	/* Allocate a node for the queue. */
	node = kzalloc(sizeof(struct report_queue_node), GFP_ATOMIC);
	if (!node) {
		logError("audit_send_to_database(): failed to "
			 "allocate report queue node.");
		return -ENOMEM;
	}

	/* Get the pid value. */
	pid = audit_extract_pid(buffer);
	if (pid < 0) {
		logError("audit_send_to_database(): invalid pid.");
		rc = -EBADE;
		goto cleanup;
	}

	/* Get the pid name. */
	pinfo = proc_get_info(pid);
	if (!pinfo) {
		logError("audit_send_to_database(): invalid pinfo.");
		rc = -EBADE;
		goto cleanup;
	}

	buffer_sz = strlen(buffer);
	name_sz = strlen(pinfo->name);
	node->report_sz = tlv_size(0) + tlv_size(buffer_sz) + tlv_size(name_sz) + tlv_size(1);

	/* Allocate enough space for the TLV bundle. */
	node->report = kzalloc(node->report_sz + 1, GFP_ATOMIC);
	if (!node->report) {
		logError("audit_send_to_database(): failed to allocate "
			 "space for report TLV.");
		proc_put_info(pinfo);
		rc = -ENOMEM;
		goto cleanup;
	}

	offset_sz = tlv_size(0);

	/* Write the audit tlv. */
	rc = tlv_write_value(node->report + offset_sz,
			     node->report_sz - offset_sz,
			     TLV_TYPE_AUDIT,
			     buffer,
			     buffer_sz);
	if (rc < 0) {
		logError("audit_send_to_database(): failed on "
			 "tlv_write_value() of report. rc=%d.", rc);
		proc_put_info(pinfo);
		goto cleanup;
	}

	offset_sz += tlv_size(buffer_sz);

	/* Write the pid name tlv. */
	rc = tlv_write_value(node->report + offset_sz,
			     node->report_sz - offset_sz,
			     TLV_TYPE_AUDIT_PNAME,
			     pinfo->name,
			     name_sz);
	proc_put_info(pinfo);
	if (rc < 0) {
		logError("audit_send_to_database(): failed on "
			 "tlv_write_value() of pinfo. rc=%d.", rc);
		goto cleanup;
	}

	offset_sz += tlv_size(name_sz);

	/* Write snapshot to tlv. */
	rc = tlv_write_value(node->report + offset_sz,
			     node->report_sz - offset_sz,
			     TLV_TYPE_AUDIT_SNAPSHOT,
			     ctl_snapshot_complete() ? "1" : "0",
			     1);
	if (rc < 0) {
		logError("audit_send_to_database(): failed on "
			 "tlv_write_value() of snapshot. rc=%d.", rc);
		goto cleanup;
	}

	/* Write the bundle tlv. */
	rc = tlv_write_value(node->report,
			     node->report_sz,
			     TLV_TYPE_BUNDLE,
			     node->report + tlv_size(0),
			     node->report_sz - tlv_size(0));
	if (rc < 0) {
		logError("audit_send_to_database(): failed on "
			 "tlv_write_value() of bundle. rc=%d.", rc);
		goto cleanup;
	}

	/* Add to the queue and send to jbide. */
	rc = report_add_to_tail(node);
	if (rc < 0) {
		logError("audit_send_to_database(): failed on "
			 "report_add_to_tail(). rc=%d.", rc);
		goto cleanup;
	}

cleanup:
	if (rc < 0) {
		kfree(node->report);
		kfree(node);
		return rc;
	}
	return 0;
}

/*************************************************************************/

/*
 * This function is called whenever an avc denial occurs.
 *
 * @param   buffer              The avc denial message buffer.
 */
static void audit_callback(char *buffer)
{
	int rc = 0;

	if (!buffer) {
		logError("audit_callback(): received NULL callback.");
		return;
	}

	/* Verify that the buffer is an audit avc. */
	if (strncmp(buffer, "audit", 5) != 0) {
		logError("audit_callback(): avc audit format has changed.");
		return;
	}

	/* Send audit denial up to jbide. */
	rc = audit_send_to_database(buffer);
	if (rc) {
		logError("audit_callback(): audit_send_to_database() "
			 "failed. rc=%d.", rc);
	}
}
#endif //AVEN_109207

/*************************************************************************/

/*
 * This function initializes auditing of avc denials. It sets a callback
 * for avc, so that every time there is a denial, the callback function
 * is called with the avc message.
 *
 * @return  0                   Regardless of error.
 */
int __init audit_init(void)
{
// Use AVEN-109207-Tune Bide Audit logging to re-enable this feature
// once it's in good state.
#ifdef AVEN_109207
	int rc = avc_set_current_audit_callback(audit_callback);
	if (rc) {
		logError("audit_init(): avc_set_current_audit_callback() "
			 "failed. rc=%d.", rc);
	} else {
		logInfo("audit_init(): audit initialized.");
	}
#endif //AVEN_109207
	return 0;
}
