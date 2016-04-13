/*
 * Copyright (C) 2015 BlackBerry Limited
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pathtrust_netlink.h>
#include <linux/version.h>
#include <net/net_namespace.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#include <net/netlink.h>
#else
#include <linux/netlink.h>
#endif

#include "pathtrust_private.h"

static struct sock *ptnl;

/*
 * Retrieve netlink message type size
 */
static int ptnl_msglen(int msgtype)
{
	int ret = 0;

	switch (msgtype) {
	case PTNL_MSG_ENFORCE_TOGGLE:
		ret = sizeof(struct ptnl_msg_enforce_toggle);
		break;

	default:
		BUG();
	}
	return ret;
}

/*
 * Set netlink message data
 */
static void ptnl_add_payload(struct nlmsghdr *nlh, int len,
		int msgtype, void *data)
{
	switch (msgtype) {
	case PTNL_MSG_ENFORCE_TOGGLE: {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
		struct ptnl_msg_enforce_toggle *msg = nlmsg_data(nlh);
#else
		struct ptnl_msg_enforce_toggle *msg = NLMSG_DATA(nlh);
#endif

		memset(msg, 0, len);
		msg->enforce = *((int *)data);
		break;
	}

	default:
		BUG();
	}
}


static void ptnl_notify(int msgtype, void *data)
{
	int len;
	sk_buff_data_t tmp;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;

	len = ptnl_msglen(msgtype);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	skb = nlmsg_new(len, GFP_USER);
#else
	skb = alloc_skb(NLMSG_SPACE(len), GFP_USER);
#endif
	if (!skb)
		goto oom;

	tmp = skb->tail;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	nlh = nlmsg_put(skb, 0, 0, msgtype, len, 0);
#else
	nlh = NLMSG_PUT(skb, 0, 0, msgtype, len);
#endif
	if (!nlh)
		goto nlmsg_failure;

	ptnl_add_payload(nlh, len, msgtype, data);
	nlh->nlmsg_len = skb->tail - tmp;
	NETLINK_CB(skb).dst_group = PTNLGRP_ALL;
	netlink_broadcast(ptnl, skb, 0, PTNLGRP_ALL, GFP_USER);
out:
	return;

nlmsg_failure:
	kfree_skb(skb);
oom:
	pr_err("oom in %s\n", __func__);
	goto out;
}

/*
 * Notify clients of enforce status change
 */
void ptnl_notify_enforce(int enforce) {
	ptnl_notify(PTNL_MSG_ENFORCE_TOGGLE, &enforce);
}

static int __init pathtrust_nl_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct netlink_kernel_cfg cfg = {
			.groups	= PTNLGRP_MAX,
			.flags	= NL_CFG_F_NONROOT_RECV,
		};
	ptnl = netlink_kernel_create(&init_net, NETLINK_PATHTRUST, &cfg);
#else
	ptnl = netlink_kernel_create(&init_net, NETLINK_PATHTRUST,
			PTNLGRP_MAX, NULL, NULL, THIS_MODULE);
#endif
	if (ptnl == NULL)
		panic("Pathtrust: Cannot create netlink socket.");

	pr_info("netlink socket created");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	netlink_set_nonroot(NETLINK_PATHTRUST, NL_NONROOT_RECV);
#endif
	return 0;
}

postcore_initcall(pathtrust_nl_init);
