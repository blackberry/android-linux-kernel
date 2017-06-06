ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSM_EEPROM) += msm_eeprom.o
ifeq ($(CONFIG_BBRY),y)
obj-$(CONFIG_MSM_EEPROM) += bbry_eeprom_map.o
endif
