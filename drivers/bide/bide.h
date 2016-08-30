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

#ifndef __BIDE_H__
#define __BIDE_H__

/* BIDE device identification */
#define BIDE_DEVICE_NAME		"bide"
#define BIDE_DEVICE_PATH		"/dev/" BIDE_DEVICE_NAME

/* IOCTL Command identifiers */
#define BIDE_IOCTL_TAKE_SNAPSHOT			100
#define BIDE_IOCTL_SCAN_PROCESS				101
#define BIDE_IOCTL_WHITELIST_SYSTEM_PROCESS		102
#define BIDE_IOCTL_CREATE_BIDE_REPORT			103
#define BIDE_IOCTL_CREATE_NEW_KEYS			104
#define BIDE_IOCTL_SET_PROPERTY				105
#define BIDE_IOCTL_KICK_PROC_SCAN			106
#define BIDE_IOCTL_WHITELIST_PRIVILEGED_PROCESS		107
#define BIDE_IOCTL_WHITELIST_ZYGOTEMGR			108
#define BIDE_IOCTL_WHITELIST_SYSTEM_SERVER      	109
#define BIDE_IOCTL_WHITELIST_PRIVILEGED_LINEAGE		110
#define BIDE_IOCTL_WHITELIST_PRIVILEGED_CHILD		111
#define BIDE_IOCTL_WHITELIST_PRIVILEGED_ZYGOTE		112
#define BIDE_IOCTL_GET_SYSTEM_SERVER_PID        	113
#define BIDE_IOCTL_FIRST_BOOT_CHECK			114
#define BIDE_IOCTL_REGISTER_INSTALLD			115
#define BIDE_IOCTL_REGISTER_BUGREPORT_FOR_CAPS		116
#define BIDE_IOCTL_FOR_CAPS				117
#define BIDE_IOCTL_WHITELIST_ZYGOTEMGR_CHILD		118

/* Command struct constants */
#define BIDE_MAX_FILE_PATH		256
#define BIDE_MAX_XML_SIZE		1024
#define BIDE_TZ_SIGNATURE_SIZE		64
#define BIDE_NONCE_SIZE 		64
#define BIDE_KEYSTORE_SIZE		116
#define BSIS_KEYSTORE_SIZE		164
#define MAX_PKCS10_SIZE                 1024
#define BASE64_SHA256_SIZE		48
#define HASH_SHA256_SIZE		32
#define DEVICE_PIN_SIZE			16
#define PROPERTY_LENGTH			32

typedef struct {
	uint8_t 	nonce[BIDE_NONCE_SIZE];
	uint64_t 	counter;
} bide_nonce_t;

typedef struct {
	uint8_t 	data[BIDE_KEYSTORE_SIZE];
} bide_keystore_t;

typedef struct {
	uint8_t 	data[MAX_PKCS10_SIZE];
	uint32_t	sz;
} bide_p10_t;

typedef struct {
	uint8_t		kbide[BIDE_MAX_XML_SIZE];
	uint8_t		tzbide[BIDE_MAX_XML_SIZE];
	uint8_t		sig[BIDE_TZ_SIGNATURE_SIZE];
} bide_reports_t;

typedef uint8_t 	bide_hash_t[BASE64_SHA256_SIZE];
typedef uint8_t 	device_pin_t[DEVICE_PIN_SIZE];
typedef char 		property_name_t[PROPERTY_LENGTH];
typedef char 		property_value_t[PROPERTY_LENGTH];
typedef char		bide_exe_file_name_t[BIDE_MAX_FILE_PATH];

/* Bide command flags */
#define BIDE_FLAGS_REQ_PRIVATE_KEY	1
#define BIDE_FLAGS_TRIGR_FULL_SCAN	2

typedef struct {
	uint32_t		firstboot;	/* out */
	bide_nonce_t		nonce;		/* out (only if the request comes from JBide) */
} bide_is_first_boot_check_cmd_t;

typedef struct {
	uint32_t		flags;		/* in */
	device_pin_t		pin;		/* in */
	bide_keystore_t		ks;		/* in/out */
	bide_nonce_t 		nonce;		/* out */
	bide_p10_t		p10;		/* out (only if generating keys) */
} bide_snapshot_cmd_t;

typedef struct {
	bide_nonce_t		nonce;		/* in */
	device_pin_t		pin;		/* in */
	bide_keystore_t		ks;		/* out */
	bide_p10_t		p10;		/* out */
} bide_request_keys_cmd_t;

typedef struct {
	bide_nonce_t		nonce;		/* in */
	uint32_t		flags;		/* in */
	bide_hash_t		jhash;		/* in */
	bide_reports_t		report;		/* out */
} bide_create_report_cmd_t;

typedef struct {
	property_name_t		name;		/* in */
	property_value_t	value;		/* in */
	uint32_t		disallow;	/* out (reserved for future use) */
} bide_set_property_cmd_t;

typedef struct {
	bide_nonce_t		nonce;		/* in */
	bide_reports_t		report;		/* out */
} bide_kick_process_scan_cmd_t;

typedef struct {
	bide_exe_file_name_t	prog_name;		/* in */
	uint32_t				prog_name_sz;	/* in */
} bide_prog_name_cmd_t;

/* Sensor names */
#define SN_ROOT_PROCESS_DETECTOR   "RootProcessDetector"
#define SN_ESCALATED_GID           "EscalatedGID"
#define SN_SYSTEM_UID              "SystemUID"
#define SN_ESCALATED_UID           "EscalatedUID"
#define SN_MPROTECT                "MProtect"
#define SN_CAPSET                  "Capset"
#define SN_LOW_MMAP_ADDR           "LowMmapAddr"
#define SN_NODEV                   "Nodev"
#define SN_NOSUID                  "Nosuid"
#define SN_INVALID_PROPERTY        "InvalidProperty"
#define SN_MISMATCHED_PROC_HASH    "MismatchedProcessHash"
#define SN_SELINUX_CHANGED         "SELinuxFilesChanged"
#define SN_SELINUX_DISABLED        "SELinuxDisabled"
#define SN_PATHTRUST_DISABLED      "PathtrustDisabled"

/* Severity names */
#define SEVERITY_FATAL    		10
#define SEVERITY_CRITICAL 		5
#define SEVERITY_WARNING  		2

#endif
