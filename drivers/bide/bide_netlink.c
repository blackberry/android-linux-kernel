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
#include <linux/module.h>
#include <linux/selinux_netlink.h>
#include <linux/pathtrust_netlink.h>
#include <linux/qseecom_netlink.h>
#include <linux/netlink.h>
#include <linux/net.h>
#include <net/sock.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct netlink_entry {
	struct socket *sock;
	int (*cb)(int, void*);
};

struct _netlink_globals {
	struct rb_root socks;		/* Lookup table of sockets */
};

static struct _netlink_globals ctx = {};

/*************************************************************************/

/*
 * SELinux changes come with a sequence number.  The sequence number 1
 * denotes the first time the policy is loaded - at boot time.  We don't
 * want a BIDE report logged for this case.
 */
#define SEQUENCE_NUMBER_ONE	1
#define LOOKUP_STR_LEN		(2 * sizeof(void *) + 1)

/*************************************************************************/

/*
 * Netlink data handling function for SELinux state changes.
 *
 * @param   type            Message type (identifier).
 * @param   data            Pointer to the message structure.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid Parameters.
 */
static int netlink_selinux_data(int type,
				void *data)
{
	struct selnl_msg_policyload *msg = data;
	int rc = 0;

	if (!msg)
		return -EINVAL;

	switch (type)
	{
	case SELNL_MSG_SETENFORCE:
		/* SEPolicy was turend off */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_SELINUX_DISABLED,
				     NULL,
				     NULL);

		break;

	case SELNL_MSG_POLICYLOAD:
		/*
		 * We ignore the first instance as this is where the policy is
		 * initially set up.
		 */
		logDebug("SELinux state changed. Sequence=%d", msg->seqno);

		if (msg->seqno != SEQUENCE_NUMBER_ONE) {
			rc = report_incident(SEVERITY_CRITICAL,
					     SN_SELINUX_CHANGED,
					     NULL,
					     NULL);
		}

		break;
	}

	if (rc)
		logError("Failed on report_incident(SN_SELINUX). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * Netlink data handling function for QSeeCom state changes
 *
 * @param   type            Message type (identifier).
 * @param   data            Pointer to the message structure.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid Parameters.
 */
static int netlink_qseecom_data(int type,
				void *data)
{
	if (type == QSCNL_MSG_INIT_DONE) {
		/* QC is ready, initialize our TZ applet */
		int rc = tz_init_kernel();

		if (rc)
			logError("Failed on tz_init_kernel(). rc=%d.", -rc);
	} else
		logWarn("Unknown message type from qseecom.");

	return 0;
}

/*************************************************************************/

/*
 * Netlink data handling function for SELinux state changes.
 *
 * @param   type            Message type (identifier).
 * @param   data            Pointer to the message structure.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid Parameters.
 */
#ifdef CONFIG_SECURITY_PATHTRUST
static int netlink_pathtrust_data(int type,
				  void *data)
{
	struct ptnl_msg_enforce_toggle *msg = data;
	int rc = 0;

	if (!msg)
		return -EINVAL;

	if (!msg->enforce) {
		logInfo("Pathtrust has been disabled.");

		rc = report_incident(SEVERITY_CRITICAL,
				     SN_PATHTRUST_DISABLED,
				     NULL,
				     NULL);
		if (rc)
			logError("Failed on report_incident(SN_PATHTRUST_DISABLED). rc=%d.", -rc);
	}

	return rc;
}
#endif

/*************************************************************************/

/*
 * This function is a callback to handle received data from a netlink
 * broadcast. When data is received, a data handling function is looked up
 * based on socket.
 *
 * @param   sk              The socket the data came from.
 * @param   bytes           The amount of bytes available.
 */
static void netlink_data_ready(struct sock *sk, int bytes)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	struct netlink_entry *entry = NULL;
	char key[LOOKUP_STR_LEN] = { 0 };
	int rc = 0;

	rc = snprintf(key, LOOKUP_STR_LEN, "%p", sk);
	if (rc < 0) {
		logError("Failed on snprintf(). rc=%d.", -rc);
		return;
	}

	/* Look up the data handling callback */
	rc = hash_search(&ctx.socks, key, rc, (void **) &entry);
	if (rc) {
		logError("Failed on hash_search(). rc=%d.", -rc);
		return;
	}

	if (unlikely(!entry) || unlikely(!entry->cb)) {
		logError("Entry or callback pointer is NULL.");
		return;
	}

	/* Receive the data packet (blocking) */
	skb = skb_recv_datagram(sk, 0, 0, &rc);
	if (!skb || rc) {
		logError("Failed on skb_recv_datagram(). rc=%d.", -rc);
		return;
	}

	nlh = (struct nlmsghdr *) skb->data;
	if (nlh && NLMSG_OK(nlh, bytes)) {
		/* Call into the data handling function */
		rc = entry->cb(nlh->nlmsg_type, NLMSG_DATA(nlh));
		if (rc)
			logError("Failed on netlink data handling. rc=%d.", -rc);
	}

	kfree_skb(skb);
}

/*************************************************************************/

/*
 * Entry point for netlink operations. Creates and sets up a netlink socket
 * for receiving broadcast data.
 *
 * @param   id              Netlink service identifier.
 * @param   grp             Group number for service data filtering.
 * @param   cb              Function for processing received data.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid Parameters.
 */
static int netlink_create(int id,
			  int grp,
			  int (*cb)(int, void*))
{
	int rc = 0;
	char key[LOOKUP_STR_LEN] = { 0 };
	struct socket *sock = NULL;
	struct netlink_entry *entry = NULL;
	struct sockaddr_nl addr = { .nl_family = AF_NETLINK,
				    .nl_pid = 0,
				    .nl_groups = grp };

	if (!cb)
		return -EINVAL;

	/* Create a netlink socket for specified identifier */
	rc = sock_create_kern(AF_NETLINK, SOCK_RAW, id, &sock);
	if (rc) {
		logError("Failed on sock_create_kern(id=%d). rc=%d", id, -rc);
		goto err;
	}

	sock->sk->sk_data_ready = netlink_data_ready;
	sock->sk->sk_allocation = GFP_KERNEL;

	/* Bind sock to address */
	rc = kernel_bind(sock, (struct sockaddr *) &addr, sizeof(addr));
	if (rc) {
		logError("Failed on kernel_bind(). rc=%d", -rc);
		goto err;
	}

	entry = kzalloc(sizeof(struct netlink_entry), GFP_KERNEL);
	if (!entry) {
		logError("Failed to allocated netlink_entry.");
		goto err;
	}

	entry->sock = sock;
	entry->cb = cb;

	rc = snprintf(key, LOOKUP_STR_LEN, "%p", sock->sk);
	if (rc < 0) {
		logError("Failed on snprintf(). rc=%d.", -rc);
		goto err;
	}

	/* Add to hash table for easy lookup of callback */
	rc = hash_insert(&ctx.socks, key, rc, entry);
	if (rc) {
		logError("Failed on hash_insert(). rc=%d.", -rc);
		goto err;
	}

	return 0;

err:
	kfree(entry);

	if (sock)
		sock_release(sock);

	return rc;
}

/*************************************************************************/

/*
 * Entry point for netlink operations. Creates and sets up a netlink sockets
 * for SELinux, Pathtrust, and QSeeCom data.
 *
 * @return  0               No Error.
 */
int __init netlink_init(void)
{
	struct nl_data {
		int nlid;
		int grp;
		int (*cb)(int, void*);
	};

	struct nl_data nld[] = { { .nlid = NETLINK_SELINUX,
				   .grp  = SELNLGRP_AVC,
				   .cb   = netlink_selinux_data },
#ifdef CONFIG_SECURITY_PATHTRUST
				 { .nlid = NETLINK_PATHTRUST,
				   .grp  = PTNLGRP_ALL,
				   .cb   = netlink_pathtrust_data },
#endif
				 { .nlid = NETLINK_QSEECOM,
				   .grp  = QSCNLGRP_ALL,
				   .cb   = netlink_qseecom_data } };
	int i = 0;

	for (i = 0; i < COUNT_OF(nld); ++i) {
		int rc = netlink_create(nld[i].nlid,
					nld[i].grp,
					nld[i].cb);
		if (rc)
			logError("Failed on netlink_create(). rc=%d.", -rc);
	}

	logInfo("Netlink Initialized.");

	return 0;
}

/*************************************************************************/

/*
 * Socket clean up.
 *
 * @return  0               No Error.
 */
int __exit netlink_exit(void)
{
	for (;;) {
		struct netlink_entry *entry = NULL;
		char key[LOOKUP_STR_LEN] = { 0 };
		int rc = 0;

		/* Retrieve root node until empty */
		rc = hash_get(ctx.socks.rb_node, (void **) &entry);
		if (rc || !entry)
			break;

		rc = snprintf(key, LOOKUP_STR_LEN, "%p", entry->sock->sk);
		if (rc < 0) {
			logError("Failed on snprintf(). rc=%d.", -rc);
			break;
		}

		/* Then remove it from the tree */
		rc = hash_remove(&ctx.socks, key, rc, (void **) &entry);
		if (rc) {
			logError("Failed on hash_remove(). rc=%d.", -rc);
			break;
		}

		/* Release the socket */
		kernel_sock_shutdown(entry->sock, SHUT_RDWR);
		sock_release(entry->sock);

		kfree(entry);
	}

	return 0;
}
