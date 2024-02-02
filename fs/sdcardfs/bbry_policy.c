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

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#ifdef CONFIG_BBSECURE_ADBAFW
#include <linux/list.h>
#endif /* CONFIG_BBSECURE_ADBAFW */

#include "bbry_policy.h"

static int managed_profile;

#ifdef CONFIG_BBSECURE_SDAFW
static int mediacard_disabled[MAX_USERS_POLICY];
static int usbotg_disabled[MAX_USERS_POLICY];
#endif /* CONFIG_BBSECURE_SDAFW */

#ifdef CONFIG_BBSECURE_ADBAFW
struct DisableADB {
	int AID;
	struct list_head list;
};

static struct DisableADB disable_adb;
static char *disable_adb_buf;

static ssize_t disable_adb_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	if (NULL != disable_adb_buf) {
		return sprintf(buf, "%s\n",disable_adb_buf);
	} else {
		char *temp = "";
		return sprintf(buf, "%s\n", temp);
	}
}

static ssize_t disable_adb_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, toInt;
	struct DisableADB *entry = NULL;
	struct DisableADB *tempEntry = NULL;

	char *local_buf = NULL;
	char *end = NULL;
	char *restricted_user = NULL;
	size_t len;

	if(NULL != disable_adb_buf) {
		kfree(disable_adb_buf);
		disable_adb_buf = NULL;
	}

	// clean up any existing list
	list_for_each_entry_safe(entry, tempEntry, &disable_adb.list, list) {
		list_del(&entry->list);
		kfree(entry);
	}

	if(NULL == buf) {
		return count;
	}

	len = strlen(buf);

	if(len > 0) {
		local_buf = kzalloc(sizeof(char) * (len + 1), GFP_KERNEL);
		disable_adb_buf = kzalloc(sizeof(char) * (len + 1), GFP_KERNEL);
		strncpy(local_buf, buf, len);
		strncpy(disable_adb_buf, buf, len);
		end = local_buf;
	} else {
		return count;
	}

	// populate list from buffer
	restricted_user = strsep(&end, ",");

	while (restricted_user != NULL && 0 < strlen(restricted_user)) {
		entry = kmalloc(sizeof(*entry), GFP_KERNEL);
		toInt = kstrtoint(restricted_user, 10, &entry->AID);
		if (toInt != 0) {
			ret = toInt;
			kfree(entry);
		} else {
			INIT_LIST_HEAD(&entry->list);
			list_add(&(entry->list), &(disable_adb.list));
		}
		restricted_user = strsep(&end, ",");
	}

	if(NULL != local_buf) {
		kfree(local_buf);
	}

	if(0 != ret) {
		return ret;
	}
	return count;
}

static struct kobj_attribute disable_adb_attribute =
	__ATTR(disable_adb, 0644, disable_adb_show,
		disable_adb_store);

int get_adb_disabled(userid_t userid)
{
	struct DisableADB *entry = NULL;
	printk("bbry_policy get_adb_disabled userid: %d\n", userid);
	list_for_each_entry(entry, &disable_adb.list, list) {
		printk("bbry_policy get_adb_disabled entry->AID: %d\n", entry->AID);
		if(entry->AID == userid) {
			printk("bbry_policy get_adb_disabled entry->AID == userid\n");
			return 1;
		}
	}
	return 0;
}

#endif /* CONFIG_BBSECURE_ADBAFW */

static ssize_t managed_profile_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(managed_profile)+1, "%d\n",
		managed_profile);
}

#ifdef CONFIG_BBSECURE_SDAFW
static ssize_t mediacard_disabled_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(mediacard_disabled)+2, "%d,%d\n",
		mediacard_disabled[0], mediacard_disabled[1]);
}

static ssize_t usbotg_disabled_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(usbotg_disabled)+2, "%d,%d\n",
		usbotg_disabled[0], usbotg_disabled[1]);
}
#endif /* CONFIG_BBSECURE_SDAFW */

static ssize_t managed_profile_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	managed_profile = -1;

	ret = kstrtoint(buf, 10, &managed_profile);
	if (ret < 0)
		return ret;

	return count;
}

#ifdef CONFIG_BBSECURE_SDAFW
static ssize_t mediacard_disabled_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	memset(mediacard_disabled, -1, sizeof(mediacard_disabled));

	ret = sscanf(buf, "%d,%d", &mediacard_disabled[0],
		&mediacard_disabled[1]);

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t usbotg_disabled_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	memset(usbotg_disabled, -1, sizeof(usbotg_disabled));

	ret = sscanf(buf, "%d,%d", &usbotg_disabled[0],
		&usbotg_disabled[1]);

	if (ret < 0)
		return ret;

	return count;
}
#endif /* CONFIG_BBSECURE_SDAFW */

static struct kobj_attribute managed_profile_attribute =
	__ATTR(managed_profile, 0644, managed_profile_show,
		managed_profile_store);

#ifdef CONFIG_BBSECURE_SDAFW
static struct kobj_attribute mediacard_disabled_attribute =
	__ATTR(mediacard_disabled, 0644, mediacard_disabled_show,
		mediacard_disabled_store);

static struct kobj_attribute usbotg_disabled_attribute =
	__ATTR(usbotg_disabled, 0644, usbotg_disabled_show,
		usbotg_disabled_store);
#endif /* CONFIG_BBSECURE_SDAFW */

static struct attribute *attrs[] = {
	&managed_profile_attribute.attr,
#ifdef CONFIG_BBSECURE_SDAFW
	&mediacard_disabled_attribute.attr,
	&usbotg_disabled_attribute.attr,
#endif /* CONFIG_BBSECURE_SDAFW */
#ifdef CONFIG_BBSECURE_ADBAFW
	&disable_adb_attribute.attr,
#endif /* CONFIG_BBSECURE_ADBAFW */
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *policy_kobj;
static struct kobject *sdcardfs_kobj;

int bbry_policy_init(void)
{
	int retval;

	managed_profile = -1;
#ifdef CONFIG_BBSECURE_SDAFW
	memset(mediacard_disabled, -1, sizeof(mediacard_disabled));
	memset(usbotg_disabled, -1, sizeof(usbotg_disabled));
#endif /* CONFIG_BBSECURE_SDAFW */
#ifdef CONFIG_BBSECURE_ADBAFW
	INIT_LIST_HEAD(&disable_adb.list);
	disable_adb_buf = NULL;
#endif /* CONFIG_BBSECURE_ADBAFW */
	/*
	 * Create a kobject at /sys/fs/sdcardfs/policy
	 */
	sdcardfs_kobj = kobject_create_and_add("sdcardfs", fs_kobj);
	if (!sdcardfs_kobj)
		return -ENOMEM;

	policy_kobj = kobject_create_and_add("policy", sdcardfs_kobj);
	if (!policy_kobj) {
		kobject_put(sdcardfs_kobj);
		return -ENOMEM;
	}

	retval = sysfs_create_group(policy_kobj, &attr_group);
	if (retval) {
		kobject_put(sdcardfs_kobj);
		kobject_put(policy_kobj);
	}
	return retval;
}

void bbry_policy_exit(void)
{
	kobject_put(policy_kobj);
}

int get_managed_profile(userid_t *userid)
{
	if (userid && (-1 < managed_profile)) {
		*userid = managed_profile;
		return 0;
	}

	return -ENOENT;
}

#ifdef CONFIG_BBSECURE_SDAFW
int get_mediacard_disabled(userid_t *userid, size_t len)
{
	userid_t *puser;
	int i;

	if (!userid || (len <= 0))
		return -EINVAL;

	puser = &userid[0];

	for (i = 0; i < MAX_USERS_POLICY; i++) {
		if (-1 >= mediacard_disabled[i])
			continue;

		*puser++ = mediacard_disabled[i];

		if (--len == 0) {
			break;
		}
	}

	if (len == 0 && MAX_USERS_POLICY > (puser - userid) / sizeof(userid_t))
		return -ENOBUFS;
	else if (puser == userid)
		return -ENOENT;
	else
		return 0;
}

int get_usbotg_disabled(userid_t *userid, size_t len)
{
	userid_t *puser;
	int i;

	if (!userid || (len <= 0))
		return -EINVAL;

	puser = &userid[0];

	for (i = 0; i < MAX_USERS_POLICY; i++) {
		if (-1 >= usbotg_disabled[i])
			continue;

		*puser++ = usbotg_disabled[i];

		if (--len == 0) {
			break;
		}
	}

	if (len == 0 && MAX_USERS_POLICY > (puser - userid) / sizeof(userid_t))
		return -ENOBUFS;
	else if (puser == userid)
		return -ENOENT;
	else
		return 0;
}
#endif /* CONFIG_BBSECURE_SDAFW */

MODULE_AUTHOR("Ben Vander Schaaf <bvanderschaaf@blackberry.com>");
