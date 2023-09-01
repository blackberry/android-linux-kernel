ccflags-$(CONFIG_BBRY_BIDE)     += -Idrivers/misc/

ifeq ($(TARGET_BUILD_VARIANT),userdebug)
ccflags-$(CONFIG_BBRY_BIDE) += -DBID_USER_DEBUG
endif

obj-$(CONFIG_BBRY_BIDE)         += bide_module.o   \
                                   bide_vma.o      \
                                   bide_secop.o    \
                                   bide_dev.o      \
                                   bide_thread.o   \
                                   bide_tz.o       \
                                   bide_crypto.o   \
                                   bide_xml.o      \
                                   bide_hash.o     \
                                   bide_netlink.o  \
                                   bide_report.o   \
                                   bide_tlv.o      \
                                   bide_ctl.o      \
                                   bide_util.o     \
                                   bide_auth.o     \
                                   bide_caps.o     \
                                   bide_perm.o     \
                                   bide_proc.o     \
                                   bide_debugfs.o  \
                                   bide_audit.o

ifeq ($(TARGET_BUILD_VARIANT),userdebug)
obj-$(CONFIG_BBRY_BIDE)         += bide_stat.o
endif

# Access selinux internals
ccflags-y += -Isecurity/selinux/include

# Add dependency on duped selinux headers
# Any file that wants to include SELinux headers
# needs to explicitly added to this list
SELINUX_DEPENDENT               := bide_secop.c
SELINUX_DEPENDENT               += bide_audit.c

# Generate duped selinux headers
quiet_cmd_flask = GEN     $(obj)/flask.h $(obj)/av_permissions.h
      cmd_flask = scripts/selinux/genheaders/genheaders $(obj)/flask.h $(obj)/av_permissions.h

targets += flask.h av_permissions.h
$(obj)/flask.h: security/selinux/include/classmap.h FORCE
	$(call if_changed,flask)

$(obj)/$(SELINUX_DEPENDENT:.c=.o) : $(obj)/flask.h
