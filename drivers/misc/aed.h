#ifndef __aed_h
#define __aed_h

#include <generated/autoconf.h>
#include <linux/types.h>
#include <linux/bug.h>
//#include <linux/aee.h>
#include <linux/kallsyms.h>
#include <linux/ptrace.h>

#define LOGD(fmt, msg...)	pr_notice(fmt, ##msg)
#define LOGV(fmt, msg...)
#define LOGI	LOGD
#define LOGE(fmt, msg...)	pr_err(fmt, ##msg)
#define LOGW	LOGE

#define IPANIC_MODULE_TAG "KERNEL-PANIC"

#define AE_INVALID              0xAEEFF000
#define AE_NOT_AVAILABLE        0xAEE00000
#define AE_DEFAULT              0xAEE00001

typedef enum {
	AEE_MODE_MTK_ENG = 1,
	AEE_MODE_MTK_USER,
	AEE_MODE_CUSTOMER_ENG,
	AEE_MODE_CUSTOMER_USER
} AEE_MODE;


#define AEEIOCTL_DAL_SHOW       _IOW('p', 0x01, struct aee_dal_show)	/* Show string on DAL layer  */
#define AEEIOCTL_DAL_CLEAN      _IO('p', 0x02)	/* Clear DAL layer */
#define AEEIOCTL_SETCOLOR       _IOW('p', 0x03, struct aee_dal_setcolor)	/* RGB color 0x00RRGGBB */
									    /*#define AEEIOCTL_GET_PROCESS_BT _IOW('p', 0x04, struct aee_process_bt) *//*change new ID for KK */
#define AEEIOCTL_GET_PROCESS_BT _IOW('p', 0x04, struct aee_ioctl)
#define AEEIOCTL_GET_SMP_INFO   _IOR('p', 0x05, int)
#define AEEIOCTL_SET_AEE_MODE   _IOR('p', 0x06, int)
#define AEEIOCTL_GET_THREAD_REG _IOW('p', 0x07, struct aee_thread_reg)
#define AEEIOCTL_CHECK_SUID_DUMPABLE _IOR('p', 0x08, int)
/* AED debug support */
#define AEEIOCTL_WDT_KICK_POWERKEY _IOR('p', 0x09, int)

#define AEEIOCTL_RT_MON_Kick _IOR('p', 0x0A, int)
#define AEEIOCTL_SET_FORECE_RED_SCREEN _IOR('p', 0x0B, int)
#define AEEIOCTL_SET_SF_STATE _IOR('p', 0x0C, long long)
#define AEEIOCTL_GET_SF_STATE _IOW('p', 0x0D, long long)
#define AEEIOCTL_USER_IOCTL_TO_KERNEL_WANING _IOR('p', 0x0E, int)

#endif
