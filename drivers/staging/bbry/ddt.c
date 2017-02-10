/*
 * Copyright (C) 2016 BlackBerry Limited
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
#include <linux/kmod.h>
#include <linux/string.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <misc/ddt.h>

/* Netlink socket group */
#define DDTSEND_GROUP 2

struct logworthy_event_details_tp {
	uint32_t d1;      /* Details Field 1 */
	uint32_t d2;      /* Details Field 2 */
	uint32_t d3;      /* Details Field 3 */
	uint32_t d4;      /* Details Field 4 */
	char creator_id[64]; /* Creator id Field */
};

struct event_info {
	int    etype;
	struct logworthy_event_details_tp edetails;
	char   command[256];
};

static struct sock *ddt_socket;
extern int bbry_get_debug_extra(void);

int ddt_send(int etype, const struct logworthy_event_details_t *d,
		const char *cmds)
{
	struct event_info *ddt_send_event;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	u32 len;

	if (!bbry_get_debug_extra()) {
		pr_info("%s:Not sending, no one is listening\n", __func__);
		return 0; /* Not an error */
	}

	if (ddt_socket == NULL) {
		pr_err("%s:ddt_socket is not initialized\n", __func__);
		return -EINVAL;
	}

	if (!d || !d->creator_id || !cmds) {
		pr_err("%s:invalid parameters\n", __func__);
		return -EINVAL;
	}

	len = NLMSG_ALIGN(sizeof(*ddt_send_event));
	skb = nlmsg_new(len, GFP_ATOMIC);
	if (!skb) {
		pr_err("%s:failed to allocate socket buffer\n", __func__);
		return -ENOBUFS;
	}

	nlh = nlmsg_put(skb, 0, 1, NLMSG_DONE, len, 0);
	if (!nlh) {
		pr_err("%s:netlink header not  created\n", __func__);
		kfree_skb(skb);
		return  -ENOBUFS;
	}
	ddt_send_event = nlmsg_data(nlh);

	ddt_send_event->etype       = etype;
	ddt_send_event->edetails.d1 = d->d1;
	ddt_send_event->edetails.d2 = d->d2;
	ddt_send_event->edetails.d3 = d->d3;
	ddt_send_event->edetails.d4 = d->d4;
	strlcpy(ddt_send_event->edetails.creator_id, d->creator_id,
		sizeof(ddt_send_event->edetails.creator_id));
	strlcpy(ddt_send_event->command, cmds, sizeof(ddt_send_event->command));

	nlmsg_multicast(ddt_socket, skb, 0, DDTSEND_GROUP, GFP_ATOMIC);
	return 0;
}
EXPORT_SYMBOL(ddt_send);

static int __init ddt_send_init(void)
{
	ddt_socket = netlink_kernel_create(&init_net,
			NETLINK_DDT, NULL);

	if (!ddt_socket) {
		pr_err("%s: Cannot create netlink socket\n", __func__);
		return -ENOBUFS;
	}
	return 0;
}
module_init(ddt_send_init)

static void __exit ddt_send_exit(void)
{
	netlink_kernel_release(ddt_socket);
}
module_exit(ddt_send_exit);
