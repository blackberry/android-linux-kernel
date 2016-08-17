ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
obj-$(CONFIG_MSM_CCI) += msm_cci.o

ifeq ($(CONFIG_BBRY),y)
ifdef CONFIG_DEBUG_FS
obj-$(CONFIG_MSM_CCI) += msm_cci_debugfs.o
endif
endif
