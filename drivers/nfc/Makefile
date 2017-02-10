#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o
# [FEATURE]-Add-BEGIN by TCTSH.Cedar, 526254, 2015/09/23, add nq210 for Idol455
obj-$(CONFIG_NFC_NQ)		+= nq210.o # nq-nci.o
# [FEATURE]-Add-END   by TCTSH.Cedar, 526254, 2015/09/23, add nq210 for Idol455

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
