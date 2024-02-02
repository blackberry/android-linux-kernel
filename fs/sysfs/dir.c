/*
 * fs/sysfs/dir.c - sysfs core and dir operation implementation
 *
 * Copyright (c) 2001-3 Patrick Mochel
 * Copyright (c) 2007 SUSE Linux Products GmbH
 * Copyright (c) 2007 Tejun Heo <teheo@suse.de>
 *
 * This file is released under the GPLv2.
 *
 * Please see Documentation/filesystems/sysfs.txt for more information.
 */

#undef DEBUG

#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include "sysfs.h"

DEFINE_SPINLOCK(sysfs_symlink_target_lock);

void sysfs_warn_dup(struct kernfs_node *parent, const char *name)
{
	char *buf, *path = NULL;

	buf = kzalloc(PATH_MAX, GFP_KERNEL);
	if (buf)
		path = kernfs_path(parent, buf, PATH_MAX);

	WARN(1, KERN_WARNING "sysfs: cannot create duplicate filename '%s/%s'\n",
	     path, name);

	kfree(buf);
}

#ifdef CONFIG_GRKERNSEC_SYSFS_RESTRICT
extern int grsec_enable_sysfs_restrict;
#endif

/**
 * sysfs_create_dir_ns - create a directory for an object with a namespace tag
 * @kobj: object we're creating directory for
 * @ns: the namespace tag to use
 */
int sysfs_create_dir_ns(struct kobject *kobj, const void *ns)
{
	struct kernfs_node *parent, *kn;
	const char *name;
	umode_t mode = S_IRWXU | S_IRUGO | S_IXUGO;
#ifdef CONFIG_GRKERNSEC_SYSFS_RESTRICT
	const char *parent_name;
#endif

	BUG_ON(!kobj);

	name = kobject_name(kobj);

	if (kobj->parent)
		parent = kobj->parent->sd;
	else
		parent = sysfs_root_kn;

	if (!parent)
		return -ENOENT;

#ifdef CONFIG_GRKERNSEC_SYSFS_RESTRICT
	parent_name = parent->name;
	mode = S_IRWXU;

	if ((!strcmp(parent_name, "") && (!strcmp(name, "devices") || !strcmp(name, "fs"))) ||
	    (!strcmp(parent_name, "devices") && !strcmp(name, "system")) ||
	    (!strcmp(parent_name, "fs") && (!strcmp(name, "selinux") || !strcmp(name, "fuse") || !strcmp(name, "ecryptfs"))) ||
	    (!strcmp(parent_name, "system") && !strcmp(name, "cpu")))
		mode = S_IRWXU | S_IRUGO | S_IXUGO;
	if (!grsec_enable_sysfs_restrict)
		mode = S_IRWXU | S_IRUGO | S_IXUGO;
#endif

	kn = kernfs_create_dir_ns(parent, name,
				  mode, kobj, ns);
	if (IS_ERR(kn)) {
		if (PTR_ERR(kn) == -EEXIST)
			sysfs_warn_dup(parent, name);
		return PTR_ERR(kn);
	}

	kobj->sd = kn;
	return 0;
}

/**
 *	sysfs_remove_dir - remove an object's directory.
 *	@kobj:	object.
 *
 *	The only thing special about this is that we remove any files in
 *	the directory before we remove the directory, and we've inlined
 *	what used to be sysfs_rmdir() below, instead of calling separately.
 */
void sysfs_remove_dir(struct kobject *kobj)
{
	struct kernfs_node *kn = kobj->sd;

	/*
	 * In general, kboject owner is responsible for ensuring removal
	 * doesn't race with other operations and sysfs doesn't provide any
	 * protection; however, when @kobj is used as a symlink target, the
	 * symlinking entity usually doesn't own @kobj and thus has no
	 * control over removal.  @kobj->sd may be removed anytime
	 * and symlink code may end up dereferencing an already freed node.
	 *
	 * sysfs_symlink_target_lock synchronizes @kobj->sd
	 * disassociation against symlink operations so that symlink code
	 * can safely dereference @kobj->sd.
	 */
	spin_lock(&sysfs_symlink_target_lock);
	kobj->sd = NULL;
	spin_unlock(&sysfs_symlink_target_lock);

	if (kn) {
		WARN_ON_ONCE(kernfs_type(kn) != KERNFS_DIR);
		kernfs_remove(kn);
	}
}

int sysfs_rename_dir_ns(struct kobject *kobj, const char *new_name,
			const void *new_ns)
{
	struct kernfs_node *parent;
	int ret;

	parent = kernfs_get_parent(kobj->sd);
	ret = kernfs_rename_ns(kobj->sd, parent, new_name, new_ns);
	kernfs_put(parent);
	return ret;
}

int sysfs_move_dir_ns(struct kobject *kobj, struct kobject *new_parent_kobj,
		      const void *new_ns)
{
	struct kernfs_node *kn = kobj->sd;
	struct kernfs_node *new_parent;

	new_parent = new_parent_kobj && new_parent_kobj->sd ?
		new_parent_kobj->sd : sysfs_root_kn;

	return kernfs_rename_ns(kn, new_parent, kn->name, new_ns);
}

/**
 * sysfs_create_mount_point - create an always empty directory
 * @parent_kobj:  kobject that will contain this always empty directory
 * @name: The name of the always empty directory to add
 */
int sysfs_create_mount_point(struct kobject *parent_kobj, const char *name)
{
	struct kernfs_node *kn, *parent = parent_kobj->sd;

	kn = kernfs_create_empty_dir(parent, name);
	if (IS_ERR(kn)) {
		if (PTR_ERR(kn) == -EEXIST)
			sysfs_warn_dup(parent, name);
		return PTR_ERR(kn);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sysfs_create_mount_point);

/**
 *	sysfs_remove_mount_point - remove an always empty directory.
 *	@parent_kobj: kobject that will contain this always empty directory
 *	@name: The name of the always empty directory to remove
 *
 */
void sysfs_remove_mount_point(struct kobject *parent_kobj, const char *name)
{
	struct kernfs_node *parent = parent_kobj->sd;

	kernfs_remove_by_name_ns(parent, name, NULL);
}
EXPORT_SYMBOL_GPL(sysfs_remove_mount_point);
