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
#include <linux/types.h>
#include <linux/random.h>
#include <linux/version.h>
#include <crypto/algapi.h>

/* Security Services Includes */
#include "tzbb_protocol_public.h"

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/*************************************************************************/

struct _ctl_globals {
	struct mutex        lock;  /* Thread safety. */
	bide_nonce_t        nonce; /* Nonce between this and JBIDE. */
	atomic_t            taken; /* Whether the snapshot is taken. */
	int                 system_server_pid; /* For Zygote servers */
};

static struct _ctl_globals ctx = { {}, {}, ATOMIC_INIT(0), -1 };

/*************************************************************************/

#define TRIG_SENSORS_MESSAGE_LEN    256
#define SENSOR_MAX_LEN              20

/* Considering that "##:AAF123" will take a max of 9 character + comma,
 * that let us 3 characters for "..." and a safety of 1 char */
#define SENSOR_LEN_SAFE             14

/*************************************************************************/

/*
 * Generate a hash from the nonce and counter.
 *
 * @param   hash                Pointer to a buffer that will receive the hash.
 * @param   sz                  Size of the buffer.
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid input value.
 */
 static int ctl_gen_nonce_hash(void *hash,
			      unsigned sz)
{
	/* Create a hash of the nonce */
	int rc = crypto_once(HASH_ALG_SHA512,
			     &ctx.nonce,
			     sizeof(ctx.nonce),
			     hash,
			     sz);

	if (rc) {
		logError("Failed on crypto_once(). rc=%d.", rc);
		return rc;
	}

	return 0;
}

/*************************************************************************/

/*
 * This function checks that the nonce is incremented correctly by the
 * caller.
 *
 * @param   hash                Pointer to nonce hash.
 *
 * @return  0                   No Error.
 *          -ESTALE             Nonce is not correct.
 *          -EINVAL             Invalid parameter passed.
 *          -ERESTARTSYS        The call was interrupted by a signal
 */
int ctl_check_nonce(uint8_t *hash)
{
	int rc = 0;
	uint8_t nonce_hash[SHA512_SIZE];

	if (!hash) {
		rc = -EINVAL;
		goto end;
	}

	rc = ctl_gen_nonce_hash(&nonce_hash, sizeof(nonce_hash));
	if (rc) {
		logError("Failed on tz_gen_nonce_hash(). rc=%d.", rc);
		return rc;
	}

	/* If system is shutting down, invalidate access to BIDE */
	if (mutex_lock_interruptible(&ctx.lock)) {
		rc = -ERESTARTSYS;
		goto end;
	}

	if (crypto_memneq(hash, &nonce_hash, sizeof(nonce_hash)) != 0)
		rc = -ESTALE;

	mutex_unlock(&ctx.lock);

end:
	if (rc)
		logError("ctl_check_nonce_hash() found a nonce issue (%d).", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function increments the counter portion of the nonce.
 */
void ctl_increment_nonce(void)
{
	/* If system is shutting down, don't bother */
	if (mutex_lock_interruptible(&ctx.lock))
		return;

	ctx.nonce.counter++;

	mutex_unlock(&ctx.lock);
}

/*************************************************************************/

/*
 * This function gets random data to be used as the nonce between
 * KBIDE and JBIDE, stores it in global (to this file) context, ctx,
 * also returns a copy by argument reference
 *
 * @param   copy                Pointer to struct in which to return copy of nonce
 *
 * @return  0                   No Error.
 *          -ERESTARTSYS        Cannot lock mutex, system is rebooting.
 *          -EINVAL             Invalid parameter passed.
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
 * This function reads the RPMB database appends every sensor
 * that were triggered in an XML format. When a sensor is triggered, it
 * will be appended to the buffer in the following format:
 * |sensor id|:|Build version|, where the build version is the most recent
 * build that triggered the sensor. i.e. "3:AAG123"
 *
 * Special Codes:  AAA000 is default entry, AAA001 means invalid number
 * found in RPMB:  ZZZ999 means invalid number had been found in OS at the
 * time in the past when BIDE stored the failure in RPMB.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid parameter passed.
 *          -ENODEV             TZ has not been initialized properly.
 *          -ERESTARTSYS        TZ System interrupted mutex lock.
 *          -EBADE              TZ TA did not return a valid status code.
 *          -EMSGSIZE           XML Message is too large for buffer.
 *          -EBADE              Invalid exchange.
 */
static int ctl_RPMB_check(bide_reports_t *rep)
{
	int i;
	int rc = 0;
	int failed_rc = 0;
	uint32_t kbide[BIDE_STORAGE_NUMBER_SENSORS] = {0};
	uint32_t jbide[BIDE_STORAGE_NUMBER_SENSORS] = {0};

	/* Temporary buffers */
	char build_num_str[BUILD_MAX_LEN] = {0};
	char current_sensor[SENSOR_MAX_LEN] = {0};

	char kbide_msg[TRIG_SENSORS_MESSAGE_LEN] = {0};
	char jbide_msg[TRIG_SENSORS_MESSAGE_LEN] = {0};
	int kbide_msg_remaining = TRIG_SENSORS_MESSAGE_LEN - 1;
	int jbide_msg_remaining = TRIG_SENSORS_MESSAGE_LEN - 1;

	if (!rep)
		return -EINVAL;

	rc = tz_read_sensor_state(kbide, BIDE_STORAGE_NUMBER_SENSORS,
		jbide, BIDE_STORAGE_NUMBER_SENSORS);
	if (rc) {
		logError("Failed on tz_read_sensor_state. rc=%d.", rc);
		return rc;
	}

	for (i = 0; (i < BIDE_STORAGE_NUMBER_SENSORS) && (0 == failed_rc); i++) {
		if (kbide[i] != 0) {
			rc = utils_bide_storage_number_to_build(kbide[i], build_num_str);
			/* If invalid number found in RPMB, use AAA001, finish xml, then error */
			if (rc) {
				logError("ctl_RPMB_check: invalid build # in RPMB for KBIDE %d. Using AAA001", i);
				strlcpy(build_num_str, "AAA001", sizeof(build_num_str));
				failed_rc = rc;
			}
			snprintf(current_sensor, sizeof(current_sensor), "%d:%s", i, build_num_str);
			logDebug("Ksensor: %s triggered", current_sensor);

			/* Check if there is enough place for more than 1 sensor */
			if (kbide_msg_remaining > SENSOR_LEN_SAFE) {
				/* Add a comma before each sensor except the 1st one */
				if (kbide_msg_remaining != TRIG_SENSORS_MESSAGE_LEN-1) {
					strncat(kbide_msg, ",", kbide_msg_remaining);
					kbide_msg_remaining--;
				}
				strncat(kbide_msg, current_sensor, kbide_msg_remaining);
				kbide_msg_remaining -= strlen(current_sensor);
			} else if (kbide_msg_remaining != 0) {
				/* When not enough room, try to print "..." */
				strncat(kbide_msg, ",...", kbide_msg_remaining);
				kbide_msg_remaining = 0;
			}

			memset(current_sensor, 0, SENSOR_MAX_LEN);
		}
		if (jbide[i] != 0) {
			rc = utils_bide_storage_number_to_build(jbide[i], build_num_str);
			/* If invalid number found in RPMB, use AAA001, finish xml, then error */
			if (rc) {
				logError("ctl_RPMB_check: invalid build # in RPMB for JBIDE %d. Using AAA001", i);
				strlcpy(build_num_str, "AAA001", sizeof(build_num_str));
				failed_rc = rc;
			}
			snprintf(current_sensor, sizeof(current_sensor), "%d:%s", i, build_num_str);
			logDebug("Jsensor: %s triggered", current_sensor);

			/* Check if there is enough place for more than 1 sensor */
			if (jbide_msg_remaining > SENSOR_LEN_SAFE) {
				/* Add a comma before each sensor except the 1st one */
				if (jbide_msg_remaining != TRIG_SENSORS_MESSAGE_LEN-1) {
					strncat(jbide_msg, ",", jbide_msg_remaining);
					jbide_msg_remaining--;
				}
				strncat(jbide_msg, current_sensor, jbide_msg_remaining);
				jbide_msg_remaining -= strlen(current_sensor);
			} else if (jbide_msg_remaining != 0) {
				/* When not enough room, try to print "..." */
				strncat(jbide_msg, ",...", jbide_msg_remaining);
				jbide_msg_remaining = 0;
			}

			memset(current_sensor, 0, SENSOR_MAX_LEN);
		}
	}

	/* Write reports.  Still attempt this even if failed_rc, for more debug */
	rc = xml_jbide_sensor_msg(rep, jbide_msg);
	if (rc) {
		logError("Failed on xml_jbide_sensor_msg(). rc=%d.", rc);
		return rc;
	}

	rc = xml_kbide_sensor_msg(rep, kbide_msg);
	if (rc) {
		logError("Failed on xml_kbide_sensor_msg(). rc=%d.", rc);
		return rc;
	}

	return failed_rc;
}

/*************************************************************************/

/*
 * This function registers the system_server pid.
 *
 * @return  0                   No Error.
 *          -ERESTARTSYS        The call was interrupted by a signal
 *          -EINVAL             Invalid argument.
 */
int ctl_add_system_server_pid(void *p)
{
	int pid = 0;

	if (p == NULL)
		return -EINVAL;

	pid = *(int*)p;

	logInfo("Whitelisting system_server_pid with PID:%d.",
		pid);

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	if (ctx.system_server_pid != -1)
		logError("Redefining system_server_pid %d==>%d)",
			ctx.system_server_pid, pid);

	ctx.system_server_pid = pid;

	mutex_unlock(&ctx.lock);

	return 0;
}

/*************************************************************************/

/*
 * This function returns the system_server PID.
 *
 * If the system server PID was not yet set then -1 is returned to the
 * caller through the parameter passed in, else a 0+ integer representing
 * the system server PID.
 *
 * @param   p                   Pointer to the system server pid remembered
 *
 * @return  0                   No error
 *          -ERESTARTSYS        The call was interrupted by a signal
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

/*************************************************************************/

/*
 * This function gets called when a change to the system properties is
 * detected.
 *
 * @param   p                   A pointer to a bide_set_property_cmd_t struct.
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
int ctl_set_property_check(void *p)
{
	bide_set_property_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	rc = auth_check_property(cmd->name, cmd->value);
	if (rc)
		logError("Failed on auth_check_property(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function checks if the snapshot has been taken, and if it has it
 * returns the nonce alongside of the result.
 *
 * @param   p                   A pointer to a bide_is_first_boot_check_cmd_t
 *                              struct.
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
int ctl_first_boot_check(void *p)
{
	bide_is_first_boot_check_cmd_t *cmd = p;
	struct task_struct *jbide_task;
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
	jbide_task = pid_task(find_vpid(cmd->pid), PIDTYPE_PID);
	rc = vma_scan_task(jbide_task, VMA_FLAG_DISALLOW_NEW_SECTIONS);
	if (rc) {
		logError("JBIDE Failed on vma_scan_task(). rc=%d.", rc);
		return -EPERM;
	}
	/* Scan Bide HAL process memory */
	rc = vma_scan_task(current, VMA_FLAG_DISALLOW_NEW_SECTIONS);
	if (rc) {
		logError("Bide HAL Failed on vma_scan_task(). rc=%d.", rc);
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
 * @param   p                   A pointer to a bide_kick_process_scan_cmd_t.
 *
 * @return  0                   No error.
 *          -ENOENT             No entry found for PID.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -ERESTARTSYS        System restart detected.
 */
int ctl_kick_process_scan(void *p)
{
	bide_kick_process_scan_cmd_t *cmd = p;
	int last_rc = 0;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot run process scan.");
		return rc;
	}

	/* Set initial struct values. */
	cmd->report.kbide_size           = KBIDE_START_SIZE;
	cmd->report.kbide_offset         = 0;
	cmd->report.is_kbide_on_heap     = 0;
	cmd->report.can_kbide_be_on_heap = 0;
	cmd->report.kbide                = cmd->report.kbide_stack;

	/* Create the begining report XML tags */
	rc = report_begin(&cmd->report);
	if (rc) {
		logError("Failed on report_begin(). rc=%d.", rc);
		goto end;
	}

	/* Run the scan and accumulate reports, if any */
	rc = vma_scan_processes();
	if (rc && rc != -EFAULT) {
		logError("Failed on vma_scan_processes(). rc=%d.", rc);
		goto end;
	}

	/* Complete the XML report */
	rc = report_end(&cmd->report);
	if (rc) {
		logError("Failed on report_end(). rc=%d.", rc);
		goto end;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

end:

	last_rc = rc;

	return last_rc ? last_rc : rc;
}

/*************************************************************************/

/*
 * This function generates new keys. The new keys replace the old ones for
 * subsequent trustzone calls.
 *
 * @param   p                   A pointer to a bide_request_keys_cmd_t struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int ctl_generate_keys(void *p)
{
	bide_request_keys_cmd_t *cmd = p;
	int rc = 0;
	char msg[MAX_XML_MSG_LENGTH] = { 0 };

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot regenerate keys.");
		return rc;
	}

	/* Private key was requested, generate one */
	rc = tz_gen_keypair(&cmd->ks, &cmd->p10);
	if (rc) {
		logError("Failed on tz_gen_keypair(). rc=%d.", rc);

		/* Generate a report only if we find a nonce problem */
		if (rc == -TZ_BIDE_NONCE_HASH_MISMATCH_PUBLIC) {
			rc = xml_nonce_mismatch_msg(msg, sizeof(msg), rc);
			if (rc < 0) {
				logError("Failed on xml_nonce_mismatch_msg(). rc=%d.", rc);
				return -EFAULT;
			}

			/* If the call to trustzone fail, generate a report */
			rc = report_incident(SEVERITY_CRITICAL,
							SN_KBIDE_TZ_NONCE_MISMATCH,
							msg,
							current);
			if (rc)
				logError("Failed on report_incident() from ctl_generate_keys. rc=%d.", rc);
		}

		return -EFAULT;
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
 * @param   p                   A bide_create_report_cmd_t struct pointer.
 *
 * @return  0                   No error.
 *          -ENOENT             No entry found for PID.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -ERESTARTSYS        System restart detected.
 */
int ctl_generate_report(void *p)
{
	bide_create_report_cmd_t *cmd = p;
	int last_rc = 0;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot generate report.");
		return rc;
	}

	/* Set initial struct values. */
	cmd->report.kbide_size           = KBIDE_START_SIZE;
	cmd->report.kbide_offset         = 0;
	cmd->report.is_kbide_on_heap     = 0;
	cmd->report.can_kbide_be_on_heap = 0;
	cmd->report.kbide                = cmd->report.kbide_stack;

	/* Create the begining report XML tags */
	rc = report_begin(&cmd->report);
	if (rc) {
		logError("Failed on report_begin(). rc=%d.", rc);
		goto end;
	}

	/* Add the jbide hash into the report */
	rc = xml_jbide_hash_msg(&cmd->report,
				cmd->jhash,
				BASE64_SHA256_SIZE);
	if (rc) {
		logError("Failed on xml_jbide_hash_msg(). rc=%d.", rc);
		goto end;
	}

	if (cmd->flags & BIDE_FLAGS_TRIGR_FULL_SCAN) {
		/* Run a scan on the processes and accumulate incident reports */
		rc = vma_scan_processes();
		if (rc && rc != -EFAULT) {
			logError("Failed on vma_scan_processes(). rc=%d.", rc);
			goto end;
		}
	}

	rc = ctl_RPMB_check(&cmd->report);
	if (rc) {
		logError("Failed on ctl_RPMB_check(). rc=%d.", rc);
		goto end;
	}

	/* Complete the XML report */
	rc = report_end(&cmd->report);
	if (rc) {
		logError("Failed on report_end(). rc=%d.", rc);
		goto end;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

end:

	last_rc = rc;

	return last_rc ? last_rc : rc;
}

/*************************************************************************/

/*
 * This function re-signs a report and appends it to the report passed in
 * from JBIDE.
 *
 * @param   p                   A bide_resign_report_cmd_t struct pointer.
 *
 * @return  0                   No Error.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid parameter passed.
 *          -EBADE              Invalid exchange.
 */
int ctl_resign_report(void *p) {
	bide_resign_report_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	if (cmd->report.kbide_size == 0) {
		logError("ctl_resign_report(): kbide size is 0, "
			 "likely not initialized.");
		return -EBADE;
	}

	/* If null, it is on stack. */
	if (cmd->report.kbide == NULL)
		cmd->report.kbide = cmd->report.kbide_stack;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot re-sign report");
		return rc;
	}

	/* Hash and encode the KBIDE report */
	rc = tz_sign_data(cmd->report.kbide,
			strnlen(cmd->report.kbide, cmd->report.kbide_size),
			cmd->report.tzbide,
			BIDE_MAX_REPORT_SIZE,
			cmd->report.sig,
			ECC256_SIG_SIZE);

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

	return rc;
}

/*************************************************************************/

/*
 * This function sets the private key.
 *
 * @param   p                   A pointer to a bide_keystore_t struct.
 *
 * @return  0                   No Error.
 *          -ERESTARTSYS        System interrupted mutex lock.
 *          -EINVAL             Invalid parameter passed.
 *          -ESTALE             Nonce is not correct.
 */
int ctl_set_private_key(void *p)
{
	bide_set_private_key_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot set private key.");
		return rc;
	}

	rc = tz_set_keypair(&cmd->ks);
	if (rc) {
		logError("Failed on tz_set_keypair(). rc=%d", rc);
		return rc;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

	logInfo("Private key was successfully set");

	return rc;
}

/*************************************************************************/

/*
 * This function processes the snapshot command. If a key pair is requested,
 * one will be generated in trustzone. Otherwise, the key pair is saved for
 * later use.
 *
 * @param   p                   A pointer to a bide_snapshot_cmd_t struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int ctl_snapshot_initialize(void *p)
{
	bide_snapshot_cmd_t *cmd = p;
	int rc = 0;
	int local_rc = 0;
	char msg[MAX_XML_MSG_LENGTH] = { 0 };
	struct task_struct *jbide_task;

	if (!cmd)
		return -EINVAL;

	/* Initialize TZ - note that when netlink_qseecom_data() runs first, this
	   call does nothing */
	rc = tz_init_kernel();
	if (rc) {
		logError("Failed on ctl_snapshot_initialize: tz_init_kernel(). rc=%d.", rc);
		return rc;
	}

	/* Save the build fingerprint to add to reports in the future*/
	util_save_build_fingerprint(cmd->fp);

	if (cmd->flags & BIDE_FLAGS_REQ_KEY_PAIR) {
		logInfo("New keypair are being generated.");

		/* Private key was requested, generate one */
		rc = tz_gen_keypair(&cmd->ks, &cmd->p10);
		if (rc) {
			logError("Failed on tz_gen_keypair(). rc=%d.", rc);

			/* Generate a report only if we find a nonce problem */
			if (rc == -TZ_BIDE_NONCE_HASH_MISMATCH_PUBLIC) {
				rc = xml_nonce_mismatch_msg(msg, sizeof(msg), rc);
				if (rc < 0) {
					logError("Failed on xml_nonce_mismatch_msg(). rc=%d.", rc);
					return -EFAULT;
				}

				/* Generate a report if the call to trust zone fails */
				rc = report_incident(SEVERITY_CRITICAL,
								SN_KBIDE_TZ_NONCE_MISMATCH,
								msg,
								current);
				if (rc)
					logError("Failed on report_incident() from ctl_snapshot_initialize. rc=%d.", rc);
			}

			return -EFAULT;
		}
	} else {

		/* Private key was given, save it */
		rc = tz_set_keypair(&cmd->ks);
		if (rc) {
			logError("Failed on tz_set_keypair(). rc=%d.", rc);
			return rc;
		}

		if (cmd->flags & BIDE_FLAGS_VERIFY_PUBLIC_KEY) {

			local_rc = tz_verify_keypair(&cmd->ks);
			if (local_rc == -1) {
				logInfo("Private and Public Keys don't match.");
				cmd->verify_results = -EFAULT;
			} else if (local_rc == 0) {
				logInfo("Private and Public Keys match.");
				cmd->verify_results = 0;
			} else {
				logInfo("Unable to determine if Private and Public Keys"
					" match(%d).", local_rc);
				cmd->verify_results = -EFAULT;
			}
		}
	}

	/* Initialize the nonce and make a copy */
	rc = ctl_initialize_nonce(&cmd->nonce);
	if (rc) {
		logError("Failed on ctl_initialize_nonce(). rc=%d.", rc);
		return rc;
	}

	/* Grab the initial snapshot of JBIDE memory */
	jbide_task = pid_task(find_vpid(cmd->pid), PIDTYPE_PID);
	rc = vma_scan_task(jbide_task, 0);
	if (rc && rc != -EFAULT)
		logError("JBide Failed on vma_scan_task(). rc=%d.", rc);

	/* Grab the initial snapshot of HAL memory */
	rc = vma_scan_task(current, 0);
	if (rc && rc != -EFAULT)
		logError("HAL Failed on vma_scan_task(). rc=%d.", rc);

	/* Mark snapshot as taken */
	atomic_inc(&ctx.taken);

	/* Process the debug PIDs registered while Token Service was not available */
	rc = auth_process_debug_pids(cmd->debug_token);
	if (rc)
		logError("Failed on auth_process_debug_pids(%d). rc=%d.", cmd->debug_token, rc);

	/* Kick off the scan */
	rc = vma_scan_processes();
	if (rc && rc != -EFAULT)
		logError("Failed on vma_scan_processes(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function returns the state of the snapshot.
 *
 * @return  0                   Snapshot has not been taken yet.
 *          1                   Snapshot is complete.
 */
int ctl_snapshot_complete()
{
	/* This operation is called before device init */
	return atomic_read(&ctx.taken);
}

/*************************************************************************/

/*
 * This function processes the BIDE_IOCTL_SET_BUILD_VERSION command.
 *
 * @param   p                   A pointer to a bide_set_build_cmd_t struct.
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid parameter passed.
 *          -EPERM              Operation not permitted (a second time)
 *          -ERESTARTSYS        System interrupted mutex lock.
 */
int ctl_set_build_version(void *p)
{
	bide_set_build_cmd_t *cmd = p;
	int rc = 0;
	static int already_set = 0;

	if (!cmd)
		return -EINVAL;

	if (already_set) {
		logInfo("ctl_set_build_version called a second time");
		return -EPERM;
	}

	rc = tz_set_build_version(&cmd->version);
	if (rc)
		logError("Failed on tz_set_build_version. rc=%d.", rc);
	else
		already_set = 1;

	return rc;
}

/*************************************************************************/

/*
 * This function records that a zygote has forked and begun
 * to transform into a full process.
 *
 * @param   p                   A pointer to a bide_zygote_exec_embryo_cmd_t.
 *
 * @return  0                   No Error.
 */
int ctl_zygote_exec_embryo(void *p)
{
	bide_zygote_exec_embryo_cmd_t *cmd = p;
	proc_task_exec(cmd->pid, cmd->name);
	return 0;
}

/*************************************************************************/

/*
 * This function processes the reportIssue command.
 *
 * @param   p                   A pointer to a bide_report_issue_cmd_t struct.
 *
 * @return  0                   No Error.
 *          -EFAULT             Trustzone communication failure.
 *          -EINVAL             Invalid parameter passed.
 */
int ctl_report_issue(void *p)
{
	bide_report_issue_cmd_t *cmd = p;
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	/* Validate that the nonce is properly incremented */
	rc = ctl_check_nonce((uint8_t *)&cmd->nonce_hash);
	if (rc) {
		logError("Nonce is not valid. Cannot ctl_report_issue.");
		return rc;
	}

	rc = tz_set_compromised_state(cmd->sensor, JBIDE);
	if (rc) {
		logError("Failed on tz_set_compromised_state(%d, JBIDE). rc=%d.", cmd->sensor, rc);
		return rc;
	}

	/* Increase the nonce on a successful call */
	ctl_increment_nonce();

	return rc;
}

/*************************************************************************/

/*
 * Prints the value of the nonce
 */
#ifdef BID_USER_DEBUG
void ctl_print_nonce(void)
{
	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return;

	logError("Nonce counter: %llu", ctx.nonce.counter);

	mutex_unlock(&ctx.lock);
}
#endif

/*************************************************************************/

/*
 * Entry point for initializing context.
 *
 * @return  0                   No Error (Always).
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
 * @return  0                   No Error (Always).
 */
int __exit ctl_exit(void)
{
	mutex_destroy(&ctx.lock);

	return 0;
}
