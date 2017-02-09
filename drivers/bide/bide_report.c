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
#define TLV_TYPE_BUNDLE			100

/* Contents of a bundle */
#define TLV_TYPE_XML_REPORT		110
#define TLV_TYPE_TZ_REPORT		111
#define TLV_TYPE_TZ_SIGNATURE		112

/* Misc. Defines */
#define PID_BUFFER_SIZE			20

/*************************************************************************/

struct report_queue_node {
	char			*report;
	unsigned 		report_sz;
	struct list_head 	queue;
};

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

static const int REPORT_TAGS[] = { TLV_TYPE_XML_REPORT,
				   TLV_TYPE_TZ_REPORT};

static const int REPORT_SIZE[] = { BIDE_MAX_XML_SIZE,
				   BIDE_MAX_REPORT_SIZE};

static const int REPORT_ARRAY_SIZE = COUNT_OF(REPORT_TAGS);

/*************************************************************************/

/*
 * This function creates a SHA 256 of a report and then encodes it using
 * base 64. The b64 buffer must be large enough to encapsulate the encoded
 * hash.
 *
 * @param   rep              Pointer to a XML report.
 * @param   rep_sz           Size of the report buffer.
 * @param   tzrep            A buffer that will receive the TZ report.
 * @param   tzrep_sz         Size of buffer (in) and bytes written (out).
 * @param   tzsig            A buffer that will receive the report signature.
 * @param   tzsig_sz         Size of buffer (must be BIDE_TZ_SIGNATURE_SIZE).
 *
 * @return  0                No Error.
 *          -EINVAL          Given input exceeds queue capacity.
 *          -EMSGSIZE        Output buffer is too small.
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
		logError("Failed on tz_sign_data(). rc=%d.", -rc);
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
 * @param   xml_report       Pointer to XML report.
 * @param   tz_report        Pointer to TZ report.
 * @param   tz_signature     Pointer to corresponding signature.
 * @param   tz_signature_sz  The size of tz_signature.
 *
 * @return  0                No Error.
 *          -EINVAL          Given input exceeds queue capacity.
 *          -ENOMEM          Memory allocation error.
 */
static int report_enqueue_internal(const char *xml_report,
				   const char *tz_report,
				   const uint8_t *tz_signature,
				   unsigned tz_signature_sz)
{
	struct report_queue_node *node = NULL;
	unsigned report_sz[REPORT_ARRAY_SIZE];
	const char *report[] = { xml_report,
				 tz_report };
	unsigned offset = 0;
	int i = 0;
	int rc = 0;

	/* Allocate a node for the queue */
	node = kzalloc(sizeof(struct report_queue_node), GFP_KERNEL);
	if (!node) {
		logError("Failed to allocate report queue node.");
		return -ENOMEM;
	}

	node->report_sz = tlv_size(0);

	/* Compute sizes */
	for (i = 0; i < REPORT_ARRAY_SIZE; ++i) {
		report_sz[i] = strnlen(report[i], REPORT_SIZE[i]);
		node->report_sz += tlv_size(report_sz[i]);
	}
	node->report_sz += tlv_size(tz_signature_sz);

	/* Allocate enough space for the TLV bundle */
	node->report = kzalloc(node->report_sz + 1, GFP_KERNEL);
	if (!node->report) {
		logError("Failed to allocate space for report TLV.");
		rc = -ENOMEM;
		goto cleanup;
	}

	/* Offset to the first inner TLV */
	offset = tlv_size(0);

	for (i = 0; i < REPORT_ARRAY_SIZE; ++i) {
		/* Write the two inner TLV blobs */
		rc = tlv_write_value(node->report + offset,
				     node->report_sz - offset,
				     REPORT_TAGS[i],
				     report[i],
				     report_sz[i]);
		if (rc < 0) {
			logError("Failed on tlv_write_value(%i). rc=%d.", i, -rc);
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
		logError("Failed on tlv_write_value for signature. rc=%d.", -rc);
		goto cleanup;
	}

	/* Write the bundle TLV */
	rc = tlv_write_value(node->report,
			     node->report_sz,
			     TLV_TYPE_BUNDLE,
			     node->report + tlv_size(0),
			     node->report_sz - tlv_size(0));
	if (rc < 0) {
		logError("Failed on tlv_write_value(BUNDLE). rc=%d.", -rc);
		goto cleanup;
	}

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

cleanup:
	if (rc < 0) {
		kfree(node->report);
		kfree(node);
	}

	return rc > 0 ? 0 : rc;
}

/*************************************************************************/

/*
 * This function inserts a report buffer into the hash table. This cannot
 * be called from within the kernel context since it uses the task's pid
 * as the key in the hash table.
 *
 * @param   buf              Report buffer to be added.
 *
 * @return  0                No Error.
 *          -EINVAL          Bad parameters or called from kernel context.
 *          -EEXIST          PID already has a buffer in the hash table.
 *          -ENOMEM          Memory allocation error.
 *          -ERESTARTSYS     System restart detected.
 */
static int report_add_buffers(bide_reports_t *buf)
{
	char pid[PID_BUFFER_SIZE] = { 0 };
	int rc = 0;

	if (!current || !buf)
		return -EINVAL;

	rc = snprintf(pid, PID_BUFFER_SIZE, "%d", current->pid);
	if (rc < 0) {
		logError("Failed on sprintf(). rc=%d.", -rc);
		return rc;
	}

	/* Hash table is being used, lock */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Add it to the hash table, using the pid as key */
	rc = hash_insert(&ctx.buffers, pid, rc, buf);
	if (rc)
		logError("Failed on hash_insert(). rc=%d.", -rc);

	mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function looks up a report buffer based on the current task's pid.
 *
 * @param   out              Pointer that will receive a report buffer.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
 *          -ENOENT          No entry found for PID.
 *          -ERESTARTSYS     System restart detected.
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

	rc = snprintf(pid, PID_BUFFER_SIZE, "%d", current->pid);
	if (rc < 0) {
		logError("Failed on sprintf(). rc=%d.", -rc);
		return rc;
	}

	/* Hash table is being used, lock */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Do the hash table lookup for the report buffers */
	rc = hash_search(&ctx.buffers, pid, rc, (void **) out);
	if (rc && rc != -ENOENT)
		logError("Failed on hash_search(). rc=%d.", -rc);

	mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function removes a report buffer from the hash table and returns it
 * to the caller. It is the caller's responsibility to deallocate the
 * buffer.
 *
 * @param   out              Pointer that will receive a report buffer.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
 *          -ENOENT          No entry found for PID.
 *          -ERESTARTSYS     System restart detected.
 */
static int report_remove_buffers(bide_reports_t **out)
{
	char pid[PID_BUFFER_SIZE] = { 0 };
	int rc = 0;

	if (!out)
		return -EINVAL;

	/* Kernel context buffers are not stored in hash table */
	if (!current)
		return -ENOENT;

	rc = snprintf(pid, PID_BUFFER_SIZE, "%d", current->pid);
	if (rc < 0) {
		logError("Failed on sprintf(). rc=%d.", -rc);
		return rc;
	}

	/* Hash table is being used, lock */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Do the hash table lookup for the report buffers */
	rc = hash_remove(&ctx.buffers, pid, rc, (void **) out);
	if (rc && rc != -ENOENT)
		logError("Failed on hash_remove(). rc=%d.", -rc);

	mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function begins a report. If the caller does not provide a buffer,
 * the funciton will query for a bound buffer to the current task.
 * Functions such as report_incident() will use the current task's pid to
 * retrieve the buffer from the hash table when adding new xml data.
 *
 * @param   buf              A pointer to a report buffer (optional).
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
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
			logError("Failed on report_lookup_buffers(). rc=%d.", -rc);
			return rc;
		}
	}

	if (!buf)
		return -EINVAL;

	rc = xml_begin_report(buf->kbide, BIDE_MAX_XML_SIZE);
	if (rc < 0) {
		logError("Failed on xml_begin_report(). rc=%d.", -rc);
		return rc;
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
 * @param   rep              A report buffer pointer (optional).
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
 *          -ENOMEM          Memory allocation error.
 */
int report_end(bide_reports_t *rep)
{
	int rc = 0;

	if (!rep) {
		/* Find the report buffer bound to the current task */
		rc = report_lookup_buffers(&rep);
		if (rc) {
			logError("Failed on report_lookup_buffers(). rc=%d.", -rc);
			return rc;
		}
	}

	if (!rep)
		return -EINVAL;

	/* Finish report */
	rc = xml_end_report(rep->kbide, BIDE_MAX_XML_SIZE);
	if (rc < 0) {
		logError("Failed on xml_end_report(). rc=%d.", -rc);
		return rc;
	}

	/* Hash and encode the KBIDE report */
	rc = report_hash_encode_sign(rep->kbide,
				     strnlen(rep->kbide, BIDE_MAX_XML_SIZE),
				     rep->tzbide,
				     BIDE_MAX_REPORT_SIZE,
				     rep->sig,
				     ECC256_SIG_SIZE);
	if (rc)
		logError("Failed on report_hash_and_encode(). rc=%d.", -rc);

	return rc < 0 ? rc : 0;
}

/*************************************************************************/

/*
 * This function will bind a bide_report_t buffer to the current task. When
 * functions such as report_begin() or report_incident() are called, they
 * will query the hash table for a bound report buffer. This allows for
 * multiple failures to be written to a single buffer. The lifetime of the
 * buffer is the responsibility of the calling function.
 *
 * @param   rep              A report buffer pointer to be bound.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
 */
int report_bind_current(bide_reports_t *rep)
{
	bide_reports_t *buf = NULL;
	int rc = 0;

	if (!rep)
		return -EINVAL;

	/* Check that we're not clobbering other buffers */
	rc = report_lookup_buffers(&buf);
	if (rc != -ENOENT) {
		logError("Failed on report_lookup_buffers(). rc=%d.", -rc);
		return rc;
	}

	/* Add the new buffer to the hash table */
	rc = report_add_buffers(rep);
	if (rc)
		logError("Failed on report_add_buffers(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function will unbind the buffer from the current task. The lifetime
 * of the buffer is the responsibility of the calling function.
 *
 * @param   rep              A pointer to receive a buffer (optional).
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameters.
 *          -ENOMEM          Memory allocation error.
 */
int report_unbind_current(bide_reports_t **rep)
{
	bide_reports_t *buf = NULL;
	int rc = 0;

	/* Retrieve and remove the buffers */
	rc = report_remove_buffers(&buf);
	if (rc) {
		logError("Failed on report_lookup_buffers(). rc=%d.4", -rc);
		return rc;
	}

	if (rep)
		*rep = buf;

	return 0;
}

/*************************************************************************/

/*
 * Creates a report and sends it to JBIDE.
 *
 * @param   severity         The severity of this report.
 * @param   sensor           The sensor in question.
 * @param   msg              A custom message, for logging purposes.
 * @param   task             Task info to append to the report.
 *
 * @return  0                Successfully sent report.
 *          -EINVAL          Invalid parameters.
 *          -ENOMEM          Memory allocation error.
 *          -EMSGSIZE        Report buffer too small.
 */
int report_incident(int severity,
		    const char *sensor,
		    const char *msg,
		    struct task_struct *task)
{
	bide_reports_t *rep = NULL;
	int alloc = 0;
	int rc = 0;

	if (!sensor)
		return -EINVAL;

	/* Look up or begin a new report */
	rc = report_lookup_buffers(&rep);
	if (rc == -ENOENT) {
		rep = kzalloc(sizeof(bide_reports_t), GFP_KERNEL);
		if (!rep) {
			logError("Failed to allocate bide_report_t.");
			return -ENOMEM;
		}

		alloc = 1;

		rc = report_begin(rep);
		if (rc) {
			logError("Failed on report_begin(). rc=%d.", -rc);
			goto cleanup;
		}
	} else if (rc) {
		logError("Failed on report_lookup_buffers(). rc=%d.", -rc);
		return rc;
	}

	/* Begin the failure tag */
	rc = xml_begin_incident_report(rep->kbide,
				       BIDE_MAX_XML_SIZE,
				       severity,
				       sensor);
	if (rc < 0) {
		logError("Failed on xml_begin_incident_report(). rc=%d.", -rc);
		goto cleanup;
	}

	/* Add details about the failing task, if available */
	if (task) {
		rc = xml_process_dump_msg(rep->kbide,
					  BIDE_MAX_XML_SIZE,
					  task);
		if (rc < 0) {
			logError("Failed on xml_process_dump_msg(). rc=%d.", -rc);
			goto cleanup;
		}
	}

	/* Add in details of the failure, if available */
	if (msg) {
		rc = strlcat(rep->kbide, msg, BIDE_MAX_XML_SIZE);
		if (rc >= BIDE_MAX_XML_SIZE) {
			logError("Failed to concatenate. Message too long.");

			rc = -EMSGSIZE;
			goto cleanup;
		}
	}

	/* End failure tag */
	rc = xml_end_incident_report(rep->kbide, BIDE_MAX_XML_SIZE);
	if (rc < 0) {
		logError("Failed on xml_end_incident_report(). rc=%d.", -rc);
		goto cleanup;
	}

	if (alloc) {
		rc = report_end(rep);
		if (rc) {
			logError("Failed on report_end(). rc=%d.", -rc);
			goto cleanup;
		}

		/* Compile a report and send it to JBIDE */
		rc = report_enqueue(rep);
		if (rc)
			logError("Failed on enqueue_report(). rc=%d.", -rc);
	}

cleanup:

	if (alloc)
		kfree(rep);

	return rc;
}

/*************************************************************************/

/*
 * This function deques the next available report. If the report is not
 * available, this function will wait. The caller is responsible for
 * deallocating the buffer that it is assigned.
 *
 * @param   buf_ptr          A buffer pointer that will be assigned (out).
 * @param   buf_sz           Buffer size that will be assigned (out).
 *
 * @return  0                Successfully assigned the report.
 *          -EINVAL          Invalid parameters.
 *          -ERESTARTSYS     Wait interrupted by system.
 */
int report_enqueue(bide_reports_t *buf)
{
	int rc = 0;

	if (!buf) {
		rc = report_lookup_buffers(&buf);
		if (rc) {
			logError("Failed on report_lookup_buffers(). rc=%d.", -rc);
			return rc;
		}
	}

	/* Compile a report and send it to JBIDE */
	rc = report_enqueue_internal(buf->kbide, buf->tzbide, buf->sig, BIDE_TZ_SIGNATURE_SIZE);
	if (rc)
		logError("Failed on enqueue_report(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function deques the next available report. If the report is not
 * available, this function will wait. The caller is responsible for
 * deallocating the buffer that it is assigned.
 *
 * @param   buf_ptr          A buffer pointer that will be assigned (out).
 * @param   buf_sz           Buffer size that will be assigned (out).
 *
 * @return  0                Successfully assigned the report.
 *          -EINVAL          Invalid parameters.
 *          -ERESTARTSYS     Wait interrupted by system.
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
	if (rc) {
		logError("report_dequeue(): call to wait_event_interruptible_locked failed with  %d", rc);
		goto end;
	}

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
 * Entry point for reporting initialization.
 *
 * @return  0                No Error.
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
 * @return  0                No Error.
 */
int __exit report_exit(void)
{
	mutex_destroy(&ctx.lock);

	return 0;
}
