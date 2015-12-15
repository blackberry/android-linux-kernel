#
# Makefile for building the Pathtrust module as part of the kernel tree.
#

obj-$(CONFIG_SECURITY_PATHTRUST) := pathtrust.o

pathtrust-y := hooks.o pathtrustfs.o ioctl.o netlink.o

ccflags-y := -Isecurity/pathtrust -Isecurity/pathtrust/include
