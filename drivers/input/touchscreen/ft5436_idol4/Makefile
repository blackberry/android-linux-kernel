obj-$(CONFIG_TOUCHSCREEN_FT5436_IDOL4) += ft5x06_ts.o
obj-$(CONFIG_TOUCHSCREEN_FT5436_IDOL4) += ini.o
obj-$(CONFIG_TOUCHSCREEN_FT5436_IDOL4) += test_lib.o

# for focaltech tp firmware power on upgrade
	#EXTRA_CFLAGS += -DFOCALTECH_PWRON_UPGRADE
	#EXTRA_CFLAGS += -DFOCALTECH_TP_GESTURE
	#EXTRA_CFLAGS += -DFOCALTECH_FAE_MOD
	#EXTRA_CFLAGS += -DFOCALTECH_FW_COMPAT
# xingchen.wang add for forced upgrading of tp firmware
ifeq ($(TARGET_BUILD_MMITEST), true)
KBUILD_CFLAGS += -DMINI_MODE_TO_CTP
endif
# xingchen.wang add for forced upgrading of tp firmware
