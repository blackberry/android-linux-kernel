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
#include <linux/err.h>
#include <linux/types.h>
#include <linux/random.h>
#include <linux/version.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct _ctl_globals {
	struct mutex 		lock;	/* Thread safety. */
	bide_nonce_t 		nonce;	/* Nonce between this and JBIDE. */
	atomic_t		taken;	/* Whether the snapshot is taken. */
	int			system_server_pid; /* For Zygote servers */
};

static struct _ctl_globals ctx = { {}, {}, ATOMIC_INIT(0), -1 };

/*************************************************************************/

/*
 * This function checks that the nonce is incremented correctly by the
 * caller.
 *
 * @param   n                Pointer to a JBIDE nonce.
 *
 * @return  0                No Error.
 *          -ESTALE          Nonce is not correct.
 *          -EINVAL          Invalid parameter passed.
 */
static int ctl_check_nonce(bide_nonce_t *n)
{
	int rc = 0;

	if (!n) {
		rc = -EINVAL;
		goto end;
	}

	/* If system is shutting down, invalidate access to BIDE */
	if (mutex_lock_interruptible(&ctx.lock)) {
		rc = -ERESTARTSYS;
		goto end;
	}

	if (memcmp(n, &ctx.nonce, sizeof(ctx.nonce)) != 0)
		rc = -ESTALE;

	mutex_unlock(&ctx.lock);

end:
	if (rc)
		logError("ctl_check_nonce() found a nonce issue (%d).", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function increments the counter portion of the nonce.
 */
static void ctl_increment_nonce(void)
{
	/* If system is shutting down, don't bother */
	if (mutex_lock_interruptible(&ctx.lock))
		return;

	ctx.nonce.counter++;

	mutex_unlock(&ctx.lock);
}

/*************************************************************************/

/*
 * This function checks that the nonce is incremented correctly by the
 * caller.
 *
 * @param   n                Pointer to a JBIDE nonce.
 *
 * @return  0                No Error.
 *          -ERESTARTSYS     Cannot lock mutex, system is rebooting.
 *          -EINVAL          Invalid parameter passed.
 */
static int ctl_initialize_nonce(bide_nonce_t *copy)
{
	if (!copy) {
		logError("Invalid parameters. Cannot create nonce.");
		return -EINVAL;
	}

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Generate a random nonce from /dev/random */
	get_random_bytes((void *) &ctx.nonce.nonce, BIDE_NONCE_SIZE);
	ctx.nonce.counter = 10;

	memcpy(copy, &ctx.nonce, sizeof(bide_nonce_t));

	mutex_unlock(&ctx.lock);

	return 0;
}

/*************************************************************************/

/*
 * This function registers the system_server pid.
 *
 * @return  0                No Error.
 */
int ctl_add_system_server_pid()
{
	logInfo("Whitelisting system_server_pid with PID:%d.",
		current->pid);

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	if (ctx.system_server_pid != -1)
		logError("Redefining system_server_pid %d==>%d)",
			ctx.system_server_pid, current->pid);

	ctx.system_server_pid = current->pid;

	mutex_unlock(&ctx.lock);

	return 0;
}

/*************************************************************************/

/*
 * This function returns the system_server pid.
 *
 * @return  the system_server pid (or -1 if not init'd)
 */
int ctl_get_system_server_pid(void *p)
{
	int *pid = p;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	logInfo("Returning system_server pid: %d.", ctx.system_server_pid);

	if (pid != NULL)
		*pid = ctx.system_server_pid;

	mutex_unlock(&ctx.lock);

	return 0;
}

/*
 * This function adds a process to the capabilities allowance list.
 *
 * @return  the result of the addition
 */
int ctl_add_process(void *p)
{
	bide_prog_name_cmd_t *cmd = p;
	return caps_add_process(cmd->prog_name, cmd->prog_name_sz, current->pid);
}

/*************************************************************************/

/*
 * This function gets called when a change to the system properties is
 * detected.
 *
 * @param   p                A pointer to a bide_set_property_cmd_t struct.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_set_property_check(void *p)
{
	bide_set_property_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	rc = auth_check_property(cmd->name, cmd->value);
	if (rc)
		logError("Failed on auth_check_property(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function checks if the snapshot has been taken, and if it has it
 * returns the nonce alongside of the result.
 *
 * @param   p                A pointer to a bide_is_first_boot_check_cmd_t struct.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_first_boot_check(void *p)
{
	bide_is_first_boot_check_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd) {
		logInfo("ctl_first_boot_check() - Illegal argument.");
		return -EINVAL;
	}

	if (!ctl_snapshot_complete()) {
		cmd->firstboot = 1;
		return 0;
	}

	logDebug("Detected boot check. Checking JBIDE signature.");

	cmd->firstboot = 0;

	/* Scan JBIDE process memory */
	rc = vma_scan_task(current, VMA_FLAG_DISALLOW_NEW_SECTIONS);
	if (rc) {
		logError("Failed on vma_scan_task(). rc=%d.", -rc);
		return -EPERM;
	} else {
		/* Give the new JBIDE process the nonce */
		memcpy(&cmd->nonce, &ctx.nonce, sizeof(bide_nonce_t));
	}

	return 0;
}

/*************************************************************************/

/*
 * This function kicks off a process scan and returns the resulting report
 * to JBIDE.
 *
 * @param   p                A pointer to a bide_kick_process_scan_cmd_t.
 *
 * @return  0                No Error.
 *          -EFAULT          Trustzone communication failure.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_kick_process_scan(void *p)
{
	bide_kick_process_scan_cmd_t *cmd = p;
	int last_rc = 0;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce(&cmd->nonce);
	if (rc) {
		logError("Nonce is not valid. Cannot run process scan.");
		return rc;
	}

	rc = report_bind_current(&cmd->report);
	if (rc) {
		logError("Failed on report_bind_current(). rc=%d.", -rc);
		return rc;
	}

	/* Create the begining report XML tags */
	rc = report_begin(&cmd->report);
	if (rc) {
		logError("Failed on report_begin(). rc=%d.", -rc);
		goto end;
        }

	/* Run the scan and accumulate reports, if any */
	rc = vma_scan_processes();
	if (rc && rc != -EFAULT) {
		logError("Failed on vma_scan_processes(). rc=%d.", -rc);
		goto end;
	}

	/* Complete the XML report */
	rc = report_end(&cmd->report);
	if (rc) {
		logError("Failed on report_end(). rc=%d.", -rc);
		goto end;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

end:

	last_rc = rc;

	rc = report_unbind_current(NULL);
	if (rc)
		logError("Failed on report_unbind_current(). rc=%d.", -rc);

	return last_rc ? last_rc : rc;
}

/*************************************************************************/

/*
 * This function generates new keys. The new keys replace the old ones for
 * subsequent trustzone calls.
 *
 * @param   p                A pointer to a bide_request_keys_cmd_t struct.
 *
 * @return  0                No Error.
 *          -EFAULT          Trustzone communication failure.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_generate_keys(void *p)
{
	bide_request_keys_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce(&cmd->nonce);
	if (rc) {
		logError("Nonce is not valid. Cannot regenerate keys.");
		return rc;
	}

	/* Private key was requested, generate one */
	rc = tz_gen_keypair(&cmd->ks, &cmd->p10);
	if (rc) {
		logError("Failed on tz_gen_keypair(). rc=%d.", -rc);
		return rc;
	}

	/* Increase the nonce for later requests */
	ctl_increment_nonce();

	return 0;
}

/*************************************************************************/

/*
 * This function creates a report and appends it to the report passed in
 * from JBIDE. During this report generation process, the system is checked
 * for intrusion.
 *
 * @param   p                A bide_create_report_cmd_t struct pointer.
 *
 * @return  0                No Error.
 *          -EFAULT          Trustzone communication failure.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_generate_report(void *p)
{
	bide_create_report_cmd_t *cmd = p;
	int last_rc = 0;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce(&cmd->nonce);
	if (rc) {
		logError("Nonce is not valid. Cannot generate report.");
		return rc;
	}

	rc = report_bind_current(&cmd->report);
	if (rc) {
		logError("Failed on report_bind_current(). rc=%d.", -rc);
		return rc;
	}

	/* Create the begining report XML tags */
	rc = report_begin(&cmd->report);
	if (rc) {
		logError("Failed on report_begin(). rc=%d.", -rc);
		goto end;
        }

	/* Add the jbide hash into the report */
	rc = xml_jbide_hash_msg(cmd->report.kbide,
				BIDE_MAX_XML_SIZE,
				cmd->jhash,
				BASE64_SHA256_SIZE);
	if (rc < 0) {
		logError("Failed on xml_jbide_hash_msg(). rc=%d.", -rc);
		goto end;
	}

	if (cmd->flags & BIDE_FLAGS_TRIGR_FULL_SCAN) {
		/* Run a scan on the processes and accumulate incident reports */
		rc = vma_scan_processes();
		if (rc && rc != -EFAULT) {
			logError("Failed on vma_scan_processes(). rc=%d.", -rc);
			goto end;
		}
	}

	/* Complete the XML report */
	rc = report_end(&cmd->report);
	if (rc) {
		logError("Failed on report_end(). rc=%d.", -rc);
		goto end;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

end:

	last_rc = rc;

	rc = report_unbind_current(NULL);
	if (rc)
		logError("Failed on report_unbind_current(). rc=%d.", -rc);

	return last_rc ? last_rc : rc;
}

/*************************************************************************/

/*
 * This function processes the snapshot command. If a key pair is requested,
 * one will be generated in trustzone. Otherwise, the key pair is saved for
 * later use.
 *
 * @param   p                A pointer to a bide_snapshot_cmd_t struct.
 *
 * @return  0                No Error.
 *          -EFAULT          Trustzone communication failure.
 *          -EINVAL          Invalid parameter passed.
 */
int ctl_snapshot_initialize(void *p)
{
	bide_snapshot_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Initialize TZ */
	rc = tz_init_kernel();
	if (rc) {
		logError("Failed on tz_init_kernel(). rc=%d.", -rc);
		return rc;
	}

	if (cmd->flags & BIDE_FLAGS_REQ_PRIVATE_KEY) {
		logInfo("New keypair are being generated.");

		/* Private key was requested, generate one */
		rc = tz_gen_keypair(&cmd->ks, &cmd->p10);
		if (rc) {
			logError("Failed on tz_gen_keypair(). rc=%d.", -rc);
			return rc;
		}
	} else {
		/* Private key was given, save it */
		rc = tz_set_keypair(&cmd->ks);
		if (rc) {
			logError("Failed on tz_set_keypair(). rc=%d.", -rc);
			return rc;
		}
	}

	/* Initialize the nonce and make a copy */
	rc = ctl_initialize_nonce(&cmd->nonce);
	if (rc) {
		logError("Failed on ctl_initialize_nonce(). rc=%d.", -rc);
		return rc;
	}

	/* Grab the initial snapshot of JBIDE memory */
	rc = vma_scan_task(current, 0);
	if (rc && rc != -EFAULT)
		logError("Failed on vma_scan_task(). rc=%d.", -rc);

	/* Mark snapshot as taken */
	atomic_inc(&ctx.taken);

	/* Clean the Capability Process list */
	caps_clean_list(0);

	/* Kick off the scan */
	rc = vma_scan_processes();
	if (rc && rc != -EFAULT)
		logError("Failed on vma_scan_processes(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function returns the state of the snapshot.
 *
 * @return  0                Snapshot has not been taken yet.
 *          1                Snapshot is complete.
 */
int ctl_snapshot_complete()
{
	/* This operation is called before device init */
	return atomic_read(&ctx.taken);
}

/*************************************************************************/

/*
 * Entry point for initializing context.
 *
 * @return  0                No Error (Always).
 */
int __init ctl_init(void)
{
	mutex_init(&ctx.lock);

	return 0;
}

/*************************************************************************/

/*
 * Deinitialization of context data.
 *
 * @return  0                No Error (Always).
 */
int __exit ctl_exit(void)
{
	mutex_destroy(&ctx.lock);

	return 0;
}
