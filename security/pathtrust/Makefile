#
# Makefile for building the Pathtrust module as part of the kernel tree.
#

obj-$(CONFIG_SECURITY_PATHTRUST) := pathtrust.o

pathtrust-y := hooks.o pathtrustfs.o ioctl.o netlink.o
pathtrust-$(CONFIG_SECURITY_PATHTRUST_SELINUX) += selinux.o

ccflags-y := -Isecurity/pathtrust -Isecurity/pathtrust/include

# Access selinux internals
ccflags-y += -Isecurity/selinux/include

# Generate duped selinux headers
$(addprefix $(obj)/,$(pathtrust-y)): $(obj)/flask.h

quiet_cmd_flask = GEN     $(obj)/flask.h $(obj)/av_permissions.h
      cmd_flask = scripts/selinux/genheaders/genheaders $(obj)/flask.h $(obj)/av_permissions.h

targets += flask.h av_permissions.h
$(obj)/flask.h: security/selinux/include/classmap.h FORCE
	$(call if_changed,flask)