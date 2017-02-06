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
                                   bide_perm.o

ifeq ($(TARGET_BUILD_VARIANT),userdebug)
obj-$(CONFIG_BBRY_BIDE)         += bide_stat.o
endif

