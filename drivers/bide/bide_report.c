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
#include <linux/uaccess.h>

/* Security Services Includes */
#include <tzbb_protocol_public.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

/* Bundle type (TLV = Type Length Value) */
#define TLV_TYPE_BUNDLE_BIDE	100

/* Contents of a bundle */
#define TLV_TYPE_XML_REPORT		110
#define TLV_TYPE_TZ_REPORT		111
#define TLV_TYPE_TZ_SIGNATURE		112

/* Misc. Defines */
#define PID_BUFFER_SIZE			20

/*************************************************************************/

struct _report_globals {
	wait_queue_head_t	rwq;		/* Wait variable for queue */
	struct list_head	queue;		/* A queue of reports */
	struct rb_root		buffers;	/* Map of output buffers */
	struct mutex 		lock;		/* Thread safety. */
	unsigned		sensors;	/* Sensors in queue */
	int			queue_sz;	/* Wake condition variable */
};

static struct _report_globals ctx = {};

/*************************************************************************/

/*
 * This function creates a SHA 256 of a report and then encodes it using
 * base 64. The b64 buffer must be large enough to encapsulate the encoded
 * hash.
 * This function does not guaranty that the call to tz will be successful.
 * The caller should set the value of tzrep and tzsig before calling this
 * function and verify that the values have changed after the call. If
 * they are the same, it's very likely that the call to trustzone failed.
 *
 * @param   rep                 Pointer to a XML report.
 * @param   rep_sz              Size of the report buffer.
 * @param   tzrep               A buffer that will receive the TZ report.
 * @param   tzrep_sz            Size of buffer (in) and bytes written (out).
 * @param   tzsig               A buffer that will receive the report signature.
 * @param   tzsig_sz            Size of buffer (must be ECC256_SIG_SIZE).
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid parameters.
 */
static int report_hash_encode_sign(char *rep,
				   unsigned rep_sz,
				   char *tzrep,
				   unsigned tzrep_sz,
				   uint8_t *tzsig,
				   unsigned tzsig_sz)
{
	int rc = 0;

	if (!rep || !tzrep || !tzsig)
		return -EINVAL;

	/* Call into trustzone and sign the report */
	rc = tz_sign_data(rep, rep_sz, tzrep, tzrep_sz, tzsig, tzsig_sz);
	if (rc) {
		logError("report_hash_encode_sign():tz_sign_data() failed. rc=%d.", rc);

		/* There is a small twist to make everything work in case of a nonce
		 * mismatch here. There are 2 typical scenario tested (which in theory
		 * should never happen).
		 *
		 * 1 - Nonce mismatch between TZ and KBide: This function will return
		 *     the error -144 which will be logged in the KBide report.
		 *
		 * 2 - Trustzone not initialized properly or any other trustzone
		 *     errors: This will return 0, which will cause a subsequent call
		 *     to fail as the signature won't be valid. The error handling
		 *     is done at an higher level. */
		if (rc != -TZ_BIDE_NONCE_HASH_MISMATCH_PUBLIC)
			return 0;

		return rc;
	}

	return 0;
}

/*************************************************************************/

/*
 * This function adds a bundle to the queue containing the given XML
 * report, TZ report, and corresponding signature. This function
 * fails if the attempt to do so would exceed the buffer capacity.
 * This function manages synchronization with the reader and notifies
 * that data is available.
 *
 * @param   xml_report          Pointer to XML report.
 * @param   xml_report_sz       The size of xml_report.
 * @param   tz_report           Pointer to TZ report.
 * @param   tz_signature        Pointer to corresponding signature.
 * @param   tz_signature_sz     The size of tz_signature.
 *
 * @return  0                   No Error.
 *          -EINVAL             Given input exceeds queue capacity.
 *          -ENOMEM             Memory allocation error.
 */
static int report_enqueue_internal(const char *xml_report,
				   unsigned xml_report_sz,
				   const char *tz_report,
				   const uint8_t *tz_signature,
				   unsigned tz_signature_sz)
{
	const int report_tags[] = { TLV_TYPE_XML_REPORT, TLV_TYPE_TZ_REPORT};
	const int report_array_size = COUNT_OF(report_tags);
	unsigned report_sz[report_array_size];
	const char *report[] = { xml_report, tz_report };
	const int report_size[] = { xml_report_sz, BIDE_MAX_REPORT_SIZE};

	struct report_queue_node *node = NULL;
	unsigned offset = 0;
	int i = 0;
	int rc = 0;

	/* Allocate a node for the queue */
	node = kzalloc(sizeof(struct report_queue_node), GFP_ATOMIC);
	if (!node) {
		logError("Failed to allocate report queue node.");
		return -ENOMEM;
	}

	node->report_sz = tlv_size(0);

	/* Compute sizes */
	for (i = 0; i < report_array_size; ++i) {
		report_sz[i] = strnlen(report[i], report_size[i]);
		node->report_sz += tlv_size(report_sz[i]);
	}
	node->report_sz += tlv_size(tz_signature_sz);

	/* Allocate enough space for the TLV bundle */
	node->report = kzalloc(node->report_sz + 1, GFP_ATOMIC);
	if (!node->report) {
		logError("Failed to allocate space for report TLV.");
		rc = -ENOMEM;
		goto cleanup;
	}

	/* Offset to the first inner TLV */
	offset = tlv_size(0);

	for (i = 0; i < report_array_size; ++i) {
		/* Write the two inner TLV blobs */
		rc = tlv_write_value(node->report + offset,
				     node->report_sz - offset,
				     report_tags[i],
				     report[i],
				     report_sz[i]);
		if (rc < 0) {
			logError("Failed on tlv_write_value(%i). rc=%d.", i, rc);
			goto cleanup;
		}

		offset += rc;
	}

	/* Add the signature */
	rc = tlv_write_value(node->report + offset,
				node->report_sz - offset,
				TLV_TYPE_TZ_SIGNATURE,
				tz_signature,
				tz_signature_sz);
	if (rc < 0) {
		logError("Failed on tlv_write_value for signature. rc=%d.", rc);
		goto cleanup;
	}

	/* Write the bundle TLV */
	rc = tlv_write_value(node->report,
			     node->report_sz,
			     TLV_TYPE_BUNDLE_BIDE,
			     node->report + tlv_size(0),
			     node->report_sz - tlv_size(0));
	if (rc < 0) {
		logError("Failed on tlv_write_value(BUNDLE). rc=%d.", rc);
		goto cleanup;
	}

	rc = report_add_to_tail(node);
	if (rc < 0) {
		logError("Failed on report_add_to_tail(). rc=%d.", rc);
		goto cleanup;
	}

cleanup:
	if (rc < 0) {
		kfree(node->report);
		kfree(node);
	}

	return rc > 0 ? 0 : rc;
}

/*************************************************************************/

/*
 * This function adds a node to the tail of the list which will be queued
 * to be sent up to JBIDE.
 *
 * @param   node                The node to add to the tail.
 *
 * @return  0                   No Error.
 *          -EINVAL             Given input exceeds queue capacity.
 */
int report_add_to_tail(struct report_queue_node *node)
{
	if (!node)
		return -EINVAL;

	/* Enter critical section */
	spin_lock(&ctx.rwq.lock);
	{
		/* Add the node to the queue */
		INIT_LIST_HEAD(&node->queue);
		list_add_tail(&node->queue, &ctx.queue);

		/* Notify if reader is blocked (innocuous otherwise) */
		ctx.queue_sz++;
		wake_up_all_locked(&ctx.rwq);

		logDebug("Report has been queued. Queue size is %d.", ctx.queue_sz);
	}
	spin_unlock(&ctx.rwq.lock);
	return 0;
}

/*************************************************************************/

/*
 * This function looks up a report buffer based on the current task's pid.
 *
 * @param   out                 Pointer that will receive a report buffer.
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid parameters.
 *          -ENOENT             No entry found for PID.
 *          -ERESTARTSYS        System restart detected.
 */
static int report_lookup_buffers(bide_reports_t **out)
{
	char pid[PID_BUFFER_SIZE] = { 0 };
	int rc = 0;

	if (!out)
		return -EINVAL;

	/* Kernel context buffers are not stored in hash table */
	if (!current)
		return -ENOENT;

	rc = snprintf(pid, PID_BUFFER_SIZE, "%d", util_get_tgid(current));
	if (rc < 0) {
		logError("Failed on sprintf(). rc=%d.", rc);
		return rc;
	}

	/* Hash table is being used, lock */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Do the hash table lookup for the report buffers */
	rc = hash_search(&ctx.buffers, pid, rc, (void **) out);
	if (rc && rc != -ENOENT)
		logError("report_lookup_buffers():hash_search() failed. rc=%d.", rc);

	mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function verify if the trustzone report and the signature were
 * generated by trustzone or if they still have the values they were
 * initialized with.
 *
 * @param   buf                 A pointer to a report buffer.
 *
 * @return  0                   Valid signature
 *          1                   Invalid signature
 *          -EINVAL             Invalid parameters.
 */
static int report_verify_signature(bide_reports_t *buf)
{
	int i;

	if (buf == NULL)
		return -EINVAL;

	/* Keep 4 bytes to store the return value in case of an error */
	for (i = sizeof(int); i < BIDE_MAX_REPORT_SIZE; i++){
		if (buf->tzbide[i] != 1)
			return 0;
	}
	for (i = 0; i < ECC256_SIG_SIZE; i++){
		if (buf->sig[i] != 1)
			return 0;
	}
	return 1;
}

/*************************************************************************/

/*
 * This function begins a report. If the caller does not provide a buffer,
 * the funciton will query for a bound buffer to the current task.
 * Functions such as report_incident() will use the current task's pid to
 * retrieve the buffer from the hash table when adding new xml data.
 *
 * @param   buf                 A pointer to a report buffer (optional).
 *
 * @return  0                   No error.
 *          -ENOENT             No entry found for PID.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -ERESTARTSYS        System restart detected.
 */
int report_begin(bide_reports_t *buf)
{
	int rc = 0;

	/* If called from kernel context, a buffer needs to be provided */
	if (!current && !buf) {
		logError("Kernel context requires a pre-existing buffer.");
		return -EINVAL;
	}

	if (!buf) {
		/* Find the report buffer bound to the current task */
		rc = report_lookup_buffers(&buf);
		if (rc) {
			logError("report_begin():report_lookup_buffers() failed. rc=%d.", rc);
			return rc;
		}
	}

	if (!buf)
		return -EINVAL;

	/* Set TZ report and signature to 1 to make it easy to detect
	 * if there was a problem with the communication with TZ */
	memset(buf->tzbide, 1, BIDE_MAX_REPORT_SIZE);
	memset(buf->sig, 1, ECC256_SIG_SIZE);

	rc = xml_begin_report(buf);
	if (rc) {
		logError("report_begin():xml_begin_report() failed. rc=%d.", rc);
		return rc;
	}

	return rc < 0 ? rc : 0;
}

/*************************************************************************/

/*
 * This function will hash, encode, and sign a report with existing xml.
 * It will also check to make sure that the signature is non-problematic.
 *
 * @param   rep                 A report buffer pointer.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int report_sign_and_package(bide_reports_t *rep) {
	int invalid_report = 0;
	char msg[MAX_XML_MSG_LENGTH] = { 0 };
	int rc = 0;

	if (!rep)
		return -EINVAL;

	if (rep->kbide_size == 0) {
		logError("report_sign_and_package(): kbide size is 0, likely uninitialized.");
		return -EBADE;
	}

	/* Hash and encode the KBIDE report */
	rc = report_hash_encode_sign(rep->kbide,
				     strnlen(rep->kbide, rep->kbide_size),
				     rep->tzbide,
				     BIDE_MAX_REPORT_SIZE,
				     rep->sig,
				     ECC256_SIG_SIZE);
	if (rc < 0) {
		logError("report_sign_and_package():report_hash_encode_sign() failed. rc=%d.", rc);
		return rc;
	}

	/* Report if there is a problem with the signature */
	invalid_report = report_verify_signature(rep);
	if (invalid_report) {
		logError("report_sign_and_package(): invalid report found.");

		rc = xml_nonce_mismatch_msg(msg, sizeof(msg), rc);
		if (rc < 0) {
			logError("report_sign_and_package():xml_nonce_mismatch_msg() failed. rc=%d.", rc);
			return rc;
		}

		if (ctl_snapshot_complete()) {
			rc = report_incident(SIGNATURE_INCIDENT,
					     SN_KBIDE_TZ_NONCE_MISMATCH,
					     msg,
					     current);
			if (rc)
				logError("report_sign_and_package():report_incident() failed. rc=%d.", rc);
		} else {
			/* TODO: We could queue these reports up and log them once
			   snapshot is done. */
			logError("Pre-Snapshot Report: %s, PID:%d", msg, util_get_tgid(current));
		}
		memset(rep->tzbide, 0, BIDE_MAX_REPORT_SIZE);
		memset(rep->sig, 0, ECC256_SIG_SIZE);
	}

	return rc < 0 ? rc : 0;
}

/*************************************************************************/

/*
 * This function will finalize a report that has been started using
 * report_begin(). If the caller provides a buffer, that buffer will be
 * used instead of looking up in the hash table. Otherwise, the buffer will
 * be looked up in the hash table. The lifetime of the buffer is the
 * responsibility of the calling function.
 *
 * @param   rep                 A report buffer pointer (optional).
 *
 * @return  0                   No error.
 *          -ENOENT             No entry found for PID.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -ERESTARTSYS        System restart detected.
 */
int report_end(bide_reports_t *rep)
{
	int rc = 0;

	if (!rep) {
		/* Find the report buffer bound to the current task */
		rc = report_lookup_buffers(&rep);
		if (rc) {
			logError("report_end():report_lookup_buffers() failed. rc=%d.", rc);
			return rc;
		}
	}

	if (!rep)
		return -EINVAL;

	/* Finish report */
	rc = xml_end_report(rep);
	if (rc) {
		logError("report_end():xml_end_report() failed. rc=%d.", rc);
		return rc;
	}

	return report_sign_and_package(rep);
}

/*************************************************************************/

/*
 * Called to inject a kbide report with existing xml.
 *
 * @param   data                A pointer to a bide_inject_report_cmd_t struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int report_inject_incident(void *data) {
#ifdef BID_USER_DEBUG
	int rc = 0;
	bide_inject_report_cmd_t *rep = (bide_inject_report_cmd_t *) data;

	if (!rep)
		return -EINVAL;

	if (rep->report.kbide_size == 0) {
		logError("report_inject_incident(): kbide size is 0, "
			 "likely not initialized.");
		return -EBADE;
	}

	/* If null, it is on stack. */
	if (rep->report.kbide == NULL)
		rep->report.kbide = rep->report.kbide_stack;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&rep->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot sign report being injected.");
		return rc;
	}

	/* Set TZ report and signature to 1 to make it easy to detect
	 * if there was a problem with the communication with TZ */
	memset(rep->report.tzbide, 1, BIDE_MAX_REPORT_SIZE);
	memset(rep->report.sig, 1, ECC256_SIG_SIZE);

	logDebug("The following report is being injected: %s", (char *) rep->report.kbide);

	rc = report_sign_and_package(&rep->report);
	if (rc) {
		logError("report_inject_incident(): report_sign_and_package() failed. rc=%d", rc);
		return rc;
	}

	rc = report_enqueue(&rep->report);
	if (rc) {
		logError("report_inject_incident(): report_enqueue() failed. rc=%d", rc);
		return rc;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

	return 0;
#else
	logError("report_inject_incident(): must only call this function in user debug mode.");
	return -EINVAL;
#endif
}

/*************************************************************************/

/*
 * Creates a report and sends it to JBIDE.
 *
 * @param   severity            The severity of this report.
 * @param   sensor              The KBIDE sensor in question.
 * @param   msg                 A custom message, for logging purposes.
 * @param   task                Task info to append to the report.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int report_incident(int severity,
		    const char *sensor,
		    const char *msg,
		    struct task_struct *task)
{
	bide_reports_t *rep = NULL;
	int alloc = 0;
	int rc = 0;
	int sensor_id = 0;
	int local_rc;

	if (!sensor)
		return -EINVAL;

	/* Register a copy of the event in RPMB */
	sensor_id = util_sensor_name_to_integer(sensor);
	if (sensor_id < 0) {
		logError("Error getting the sensor id. Unable to register in RPMB");
	} else {
		rc = tz_set_compromised_state(sensor_id, KBIDE);
		if (rc)
			logError("Failed on tz_set_compromised_state(%d, KBIDE). rc=%d.",
				sensor_id, rc);
	}

	/* Look up or begin a new report */
	rc = report_lookup_buffers(&rep);
	if (rc == -ENOENT) {
		rep = kzalloc(sizeof(bide_reports_t), GFP_ATOMIC);
		if (!rep) {
			logError("Failed to allocate bide_report_t.");
			return -ENOMEM;
		}

		/* Set initial struct values. */
		rep->kbide_size           = KBIDE_START_SIZE;
		rep->kbide_offset         = 0;
		rep->is_kbide_on_heap     = 0;
		rep->can_kbide_be_on_heap = 1;
		rep->kbide                = rep->kbide_stack;

		alloc = 1;

		rc = report_begin(rep);
		if (rc) {
			logError("report_incident():report_begin() failed. rc=%d.", rc);
			goto cleanup;
		}
	} else if (rc) {
		logError("report_incident():report_lookup_buffers() failed. rc=%d.", rc);
		return rc;
	}

	if (severity == SIGNATURE_INCIDENT) {
		severity = SEVERITY_CRITICAL;
		logInfo("report_incident: SIGNATURE_INCIDENT");

		/* As we already know that we won't be able to sign, set report and
		 * signature to 0 so that we don't get stuck in an infinite loop */
		memset(rep->tzbide, 0, BIDE_MAX_REPORT_SIZE);
		memset(rep->sig, 0, ECC256_SIG_SIZE);
	}

	/* Begin the failure tag */
	local_rc = xml_begin_incident_report(rep,
				       severity,
				       sensor);
	if (local_rc) {
		logError("report_incident():xml_begin_incident_report() failed. rc=%d.",
			local_rc);
		rc = local_rc;
		goto cleanup;
	}

	/* Add details about the failing task, if available */
	if (task) {
		local_rc = xml_process_dump_msg(rep, task);
		if (local_rc) {
			logError("report_incident():xml_process_dump_msg() failed. rc=%d.",
				local_rc);
			rc = local_rc;
			goto cleanup;
		}
	}

	/* Add in details of the failure, if available */
	if (msg) {
		local_rc = xml_cat(rep, msg);
		if (local_rc) {
			logError("report_incident():xml_cat() failed. rc=%d.", local_rc);
			rc = local_rc;
			goto cleanup;
		}
	}

	/* End failure tag */
	local_rc = xml_end_incident_report(rep);
	if (local_rc) {
		logError("report_incident():xml_end_incident_report() failed. rc=%d.",
			local_rc);
		rc = local_rc;
		goto cleanup;
	}

	if (alloc) {
		rc = report_end(rep);
		if (rc) {
			logError("report_incident():report_end() failed. rc=%d.", rc);
			goto cleanup;
		}

		/* Compile a report and send it to JBIDE */
		rc = report_enqueue(rep);
		if (rc)
			logError("report_incident():enqueue_report() failed. rc=%d.", rc);
	}

cleanup:

	if (rep->is_kbide_on_heap) {
		kfree(rep->kbide);
		rep->kbide = NULL;
	}

	if (alloc) {
		kfree(rep);
		rep = NULL;
	}

	return rc;
}

/*************************************************************************/

/*
 * This function deques the next available report. If the report is not
 * available, this function will wait. The caller is responsible for
 * deallocating the buffer that it is assigned.
 *
 * @param   buf_ptr             A buffer pointer that will be assigned (out).
 * @param   buf_sz              Buffer size that will be assigned (out).
 *
 * @return  0                   Successfully assigned the report.
 *          -EINVAL             Invalid parameters.
 *          -ERESTARTSYS        Wait interrupted by system.
 */
int report_enqueue(bide_reports_t *buf)
{
	int rc = 0;

	if (!buf) {
		rc = report_lookup_buffers(&buf);
		if (rc) {
			logError("report_enqueue():report_lookup_buffers() failed. rc=%d.", rc);
			return rc;
		}
	}

	/* Compile a report and send it to JBIDE */
	rc = report_enqueue_internal(buf->kbide,
				     buf->kbide_size,
				     buf->tzbide,
				     buf->sig,
				     ECC256_SIG_SIZE);
	if (rc)
		logError("report_enqueue():report_enqueue_internal() failed. rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function deques the next available report. If the report is not
 * available, this function will wait. The caller is responsible for
 * deallocating the buffer that it is assigned.
 *
 * @param   buf_ptr             A buffer pointer that will be assigned (out).
 * @param   buf_sz              Buffer size that will be assigned (out).
 *
 * @return  0                   Successfully assigned the report.
 *          -EINVAL             Invalid parameters.
 *          -ERESTARTSYS        Wait interrupted by system.
 */
int report_dequeue(char **buf_ptr,
		   unsigned *buf_sz)
{
	struct report_queue_node *node = NULL;
	int rc = 0;

	if (!buf_ptr || !buf_sz)
		return -EINVAL;

	spin_lock(&ctx.rwq.lock);

	/* Note short circuit here */
	rc = wait_event_interruptible_locked(ctx.rwq, ctx.queue_sz > 0);
	if (rc)
		goto end;

	/* Retrieve the node and assign buffers (to be free'd by client) */
	node = list_entry(ctx.queue.next, struct report_queue_node, queue);
	if (node) {
		*buf_ptr = node->report;
		*buf_sz = node->report_sz;

		/* Deallocate the node */
		list_del(&node->queue);
		kfree(node);
	}

	ctx.queue_sz--;

end:
	spin_unlock(&ctx.rwq.lock);

	return rc;
}

/*************************************************************************/

/*
 * Prints the length of the report queue and the size of each report in
 * the queue.
 */
#ifdef BID_USER_DEBUG
void report_print_queue(void)
{
	int count = 0;
	struct list_head *pos;
	struct report_queue_node *report;


	logError("---- Report queue ----");

	spin_lock(&ctx.rwq.lock);
	list_for_each(pos, &ctx.queue) {
		report = list_entry(pos, struct report_queue_node, queue);
		count ++;

		logError("Size of report %d : %d", count, report->report_sz);
	}
	spin_unlock(&ctx.rwq.lock);

	logError("Report queue length = %d", count);
	logError("---- End Report queue ----");
}
#endif

/*************************************************************************/

/*
 * Entry point for reporting initialization.
 *
 * @return  0                   No Error.
 */
int __init report_init(void)
{
	init_waitqueue_head(&ctx.rwq);
	mutex_init(&ctx.lock);

	/* Initialize report queue */
	INIT_LIST_HEAD(&ctx.queue);

	return 0;
}

/*************************************************************************/

/*
 * Context clean up.
 *
 * @return  0                   No Error.
 */
int __exit report_exit(void)
{
	mutex_destroy(&ctx.lock);

	return 0;
}
