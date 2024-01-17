ohio-objs := anx_ohio_driver.o   anx_ohio_private_interface.o  anx_ohio_public_interface.o
obj-y += ohio.o

ifneq ($(filter bbry_qc8953_sfi bbry_qc8953_sficn bbry_qc8953krypton_sfi bbry_qc8953krypton_sficn,$(TARGET_PRODUCT)),)
ccflags-y := -DUSB_8953=1
endif
