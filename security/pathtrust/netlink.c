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
#include <net/netlink.h>

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
		struct ptnl_msg_enforce_toggle *msg = nlmsg_data(nlh);

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

	skb = nlmsg_new(len, GFP_USER);
	if (!skb)
		goto oom;

	tmp = skb->tail;
	nlh = nlmsg_put(skb, 0, 0, msgtype, len, 0);
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
	struct netlink_kernel_cfg cfg = {
			.groups	= PTNLGRP_MAX,
			.flags	= NL_CFG_F_NONROOT_RECV,
		};

	ptnl = netlink_kernel_create(&init_net, NETLINK_PATHTRUST, &cfg);

	if (ptnl == NULL)
		panic("Pathtrust: Cannot create netlink socket.");

	pr_info("netlink socket created");

	return 0;
}

postcore_initcall(pathtrust_nl_init);
