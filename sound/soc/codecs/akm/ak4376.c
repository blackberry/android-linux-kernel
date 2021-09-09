/* Copyright (C) 2016 Tcl Corporation Limited */
/*
 * ak4376.c  --  audio driver for AK4376
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/06/25	    1.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm/io.h>

#include "ak4376.h"

/*
 * PLL means BLCk
 * NO PLL means mclk
 * */
#define PLL_BICK_MODE

//#define AK4376_DEBUG			//used at debug mode

#define AUDIO_NAME "AK4376"
#ifdef AK4376_DEBUG
#define codec_dbg(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format, ## arg)
#define codec_err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format, ## arg)
#define codec_info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format, ## arg)
#define codec_warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format, ## arg)
#define codec_trace() \
	printk(KERN_INFO AUDIO_NAME " %s(%d)\n", __func__, __LINE__)
#else
#define codec_dbg(format, arg...) do {} while (0)
#define codec_info(format, arg...) do {} while (0)
#define codec_warn(format, arg...) do {} while (0)
#define codec_trace(format, arg...) do {} while (0)
#define codec_err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format, ## arg)
#endif

extern int i2c_check_status_create(char *name,int value); // MODIFIED by hongwei.tian
static int ak4376_hw_params_set(struct snd_soc_codec *codec, int nfs1);

/* AK4376 Codec Private Data */
struct ak4376_priv {
	struct snd_soc_codec codec;
	u8 reg_cache[AK4376_MAX_REGISTERS];
	int fs1;
	int fs2;
	int rclk;			//Master Clock
	int nSeldain;		//0:Bypass, 1:SRC	(Dependent on a register bit)
	int nBickFreq;		//0:32fs, 1:48fs, 2:64fs
	int nSrcOutFsSel;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
	int nPllMode;		//0:PLL OFF, 1: PLL ON
	int nPllMCKI;		//0:PLL not use, 1: PLL use
	int nSmt;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
	int dfsrc8fs;		//DFTHR bit and SRCO8FS bit
};
struct ak4376_sys_data_s{
    u32 dac_ctl_pin;
    u32 dac_ctl_pin_flags;
    u32 ldo_ctl_pin;
    u32 ldo_ctl_pin_flags;
    struct regulator *vdd;
    struct i2c_client *client;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
};

static struct ak4376_sys_data_s *ak4376_sys_data;
static struct snd_soc_codec *ak4376_codec;
static struct ak4376_priv *ak4376_data;

/* ak4376 register cache & default register settings */
static const u8 ak4376_reg[AK4376_MAX_REGISTERS] = {
	0x00,	/*	0x00	AK4376_00_POWER_MANAGEMENT1			*/
	0x00,	/*	0x01	AK4376_01_POWER_MANAGEMENT2			*/
	0x00,	/*	0x02	AK4376_02_POWER_MANAGEMENT3			*/
	0x00,	/*	0x03	AK4376_03_POWER_MANAGEMENT4			*/
	0x00,	/*	0x04	AK4376_04_OUTPUT_MODE_SETTING		*/
	0x00,	/*	0x05	AK4376_05_CLOCK_MODE_SELECT			*/
	0x00,	/*	0x06	AK4376_06_DIGITAL_FILTER_SELECT		*/
	0x00,	/*	0x07	AK4376_07_DAC_MONO_MIXING			*/
	0x00,	/*	0x08	AK4376_08_JITTER_CLEANER_SETTING1	*/
	0x00,	/*	0x09	AK4376_09_JITTER_CLEANER_SETTING2	*/
	0x00,	/*	0x0A	AK4376_0A_JITTER_CLEANER_SETTING3	*/
	0x11,	/*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME			*/
	0x11,	/*	0x0C	AK4376_0C_RCH_OUTPUT_VOLUME			*/
	0x0B,	/*	0x0D	AK4376_0D_HP_VOLUME_CONTROL			*/
	0x01,	/*	0x0E	AK4376_0E_PLL_CLK_SOURCE_SELECT		*/
	0x00,	/*	0x0F	AK4376_0F_PLL_REF_CLK_DIVIDER1		*/
	0x00,	/*	0x10	AK4376_10_PLL_REF_CLK_DIVIDER2		*/
	0x00,	/*	0x11	AK4376_11_PLL_FB_CLK_DIVIDER1		*/
	0x00,	/*	0x12	AK4376_12_PLL_FB_CLK_DIVIDER2		*/
	0x00,	/*	0x13	AK4376_13_SRC_CLK_SOURCE			*/
	0x00,	/*	0x14	AK4376_14_DAC_CLK_DIVIDER			*/
	0x00,	/*	0x15	AK4376_15_AUDIO_IF_FORMAT			*/
	0x00,	/*	0x16	AK4376_16_DUMMY						*/
	0x00,	/*	0x17	AK4376_17_DUMMY						*/
	0x00,	/*	0x18	AK4376_18_DUMMY						*/
	0x00,	/*	0x19	AK4376_19_DUMMY						*/
	0x00,	/*	0x1A	AK4376_1A_DUMMY						*/
	0x00,	/*	0x1B	AK4376_1B_DUMMY						*/
	0x00,	/*	0x1C	AK4376_1C_DUMMY						*/
	0x00,	/*	0x1D	AK4376_1D_DUMMY						*/
	0x00,	/*	0x1E	AK4376_1E_DUMMY						*/
	0x00,	/*	0x1F	AK4376_1F_DUMMY						*/
	0x00,	/*	0x20	AK4376_20_DUMMY						*/
	0x00,	/*	0x21	AK4376_21_DUMMY						*/
	0x00,	/*	0x22	AK4376_22_DUMMY						*/
	0x00,	/*	0x23	AK4376_23_DUMMY						*/
	0x00,	/*	0x24	AK4376_24_MODE_CONTROL				*/
	0x00,	/*	0x25	AK4376_25_DUMMY						*/
	0x20,	/*	0x26	AK4376_26_DUMMY						*/
	0x00,	/*	0x27	AK4376_27_DUMMY						*/
	0x00,	/*	0x28	AK4376_28_DUMMY						*/
	0x00,	/*	0x29	AK4376_29_DUMMY						*/
	0x00,	/*	0x2a	AK4376_2a_DUMMY						*/

};

static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4376_access_masks[] = {
	{ 0xFF, 0xFF },	//0x00
	{ 0xFF, 0xFF },	//0x01
	{ 0xFF, 0xFF },	//0x02
	{ 0xFF, 0xFF },	//0x03
	{ 0xFF, 0xFF },	//0x04
	{ 0xFF, 0xFF },	//0x05
	{ 0xFF, 0xFF },	//0x06
	{ 0xFF, 0xFF },	//0x07
	{ 0xFF, 0xFF },	//0x08
	{ 0xFF, 0xFF },	//0x09
	{ 0xFF, 0xFF },	//0x0A
	{ 0xFF, 0xFF },	//0x0B
	{ 0xFF, 0xFF },	//0x0C
	{ 0xFF, 0xFF },	//0x0D
	{ 0xFF, 0xFF },	//0x0E
	{ 0xFF, 0xFF },	//0x0F
	{ 0xFF, 0xFF },	//0x10
	{ 0xFF, 0xFF },	//0x11
	{ 0xFF, 0xFF },	//0x12
	{ 0xFF, 0xFF },	//0x13
	{ 0xFF, 0xFF },	//0x14
	{ 0xFF, 0xFF },	//0x15
	{ 0x00, 0x00 },	//0x16	//DUMMY
	{ 0x00, 0x00 },	//0x17	//DUMMY
	{ 0x00, 0x00 },	//0x18	//DUMMY
	{ 0x00, 0x00 },	//0x19	//DUMMY
	{ 0x00, 0x00 },	//0x1A	//DUMMY
	{ 0x00, 0x00 },	//0x1B	//DUMMY
	{ 0x00, 0x00 },	//0x1C	//DUMMY
	{ 0x00, 0x00 },	//0x1D	//DUMMY
	{ 0x00, 0x00 },	//0x1E	//DUMMY
	{ 0x00, 0x00 },	//0x1F	//DUMMY
	{ 0x00, 0x00 },	//0x20	//DUMMY
	{ 0x00, 0x00 },	//0x21	//DUMMY
	{ 0x00, 0x00 },	//0x22	//DUMMY
	{ 0x00, 0x00 },	//0x23	//DUMMY
	{ 0xFF, 0xFF },	//0x24
	/* MODIFIED-BEGIN by hongwei.tian, 2018-01-04,BUG-5854780*/
	{ 0x00, 0x00 },	//0x25
	{ 0xFF, 0xFF },	//0x26
	{ 0x00, 0x00 },	//0x27
	{ 0x00, 0x00 },	//0x28
	{ 0x00, 0x00 },	//0x29
	{ 0xFF, 0xFF },	//0x2a
	/* MODIFIED-END by hongwei.tian,BUG-5854780*/
};

/* Output Digital volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB) */
static DECLARE_TLV_DB_SCALE(ovl_tlv, -1250, 50, 0);
static DECLARE_TLV_DB_SCALE(ovr_tlv, -1250, 50, 0);

/* HP-Amp Analog volume control:
 * from -42 to 6 dB in 2 dB steps (mute instead of -42 dB) */
static DECLARE_TLV_DB_SCALE(hpg_tlv, -4200, 20, 0);

static const char *ak4376_ovolcn_select_texts[] = {"Dependent", "Independent"};
static const char *ak4376_mdacl_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_mdacr_select_texts[] = {"x1", "x1/2"};
static const char *ak4376_invl_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_invr_select_texts[] = {"Normal", "Inverting"};
static const char *ak4376_cpmod_select_texts[] =
{"Automatic Switching", "+-VDD Operation", "+-1/2VDD Operation"};
static const char *ak4376_hphl_select_texts[] = {"9ohm", "200kohm"};
static const char *ak4376_hphr_select_texts[] = {"9ohm", "200kohm"};
static const char *ak4376_dacfil_select_texts[]  =
{"Sharp Roll-Off", "Slow Roll-Off", "Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};
static const char *ak4376_srcfil_select_texts[] =
{"Sharp Roll-Off", "Slow Roll-Off", "Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};

static const struct soc_enum ak4376_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4376_0B_LCH_OUTPUT_VOLUME, 7,
			ARRAY_SIZE(ak4376_ovolcn_select_texts), ak4376_ovolcn_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 2,
			ARRAY_SIZE(ak4376_mdacl_select_texts), ak4376_mdacl_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 6,
			ARRAY_SIZE(ak4376_mdacr_select_texts), ak4376_mdacr_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 3,
			ARRAY_SIZE(ak4376_invl_select_texts), ak4376_invl_select_texts),
	SOC_ENUM_SINGLE(AK4376_07_DAC_MONO_MIXING, 7,
			ARRAY_SIZE(ak4376_invr_select_texts), ak4376_invr_select_texts),
	SOC_ENUM_SINGLE(AK4376_03_POWER_MANAGEMENT4, 2,
			ARRAY_SIZE(ak4376_cpmod_select_texts), ak4376_cpmod_select_texts),
	SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 0,
			ARRAY_SIZE(ak4376_hphl_select_texts), ak4376_hphl_select_texts),
	SOC_ENUM_SINGLE(AK4376_04_OUTPUT_MODE_SETTING, 1,
			ARRAY_SIZE(ak4376_hphr_select_texts), ak4376_hphr_select_texts),
	SOC_ENUM_SINGLE(AK4376_06_DIGITAL_FILTER_SELECT, 6,
			ARRAY_SIZE(ak4376_dacfil_select_texts), ak4376_dacfil_select_texts),
	SOC_ENUM_SINGLE(AK4376_09_JITTER_CLEANER_SETTING2, 4,
			ARRAY_SIZE(ak4376_srcfil_select_texts), ak4376_srcfil_select_texts),
};

static const char *bickfreq_on_select[] = {"fs_32", "fs_48", "fs_64"}; // MODIFIED by hongwei.tian, 2018-01-25,BUG-5929027

static const char *srcoutfs_on_select[] =
#ifdef SRC_OUT_FS_48K
{"48kHz", "96kHz", "192kHz"};
#else
{"44.1kHz", "88.2kHz", "176.4kHz"};
#endif

static const char *pllmode_on_select[] = {"OFF", "ON"};
static const char *smtcycle_on_select[] = {"1024", "2048", "4096", "8192"};
static const char *dfsrc8fs_on_select[] = {"Digital Filter", "Bypass", "8fs mode"};
static const char *is_inCall[] = {"OFF", "ON"};

static const struct soc_enum ak4376_bitset_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bickfreq_on_select), bickfreq_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(srcoutfs_on_select), srcoutfs_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmode_on_select), pllmode_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smtcycle_on_select), smtcycle_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dfsrc8fs_on_select), dfsrc8fs_on_select),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(is_inCall), is_inCall),
};

/*rong.fu@jrdcom.com add 2016/06/24. Task ID:2392922, add for MINISW Robust Test, to check i2c ok or not, add begin*/
static int i2c_ok = 0;
module_param_named(i2c_ok, i2c_ok, int, 0644);
/*rong.fu@jrdcom.com add 2016/06/24. Task ID:2392922, add for MINISW Robust Test, to check i2c ok or not, add end*/
u8 hw_params_status = 0; // MODIFIED by hongwei.tian, 2018-04-17,BUG-6204707

static unsigned int ak4376_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
	if (ret < 0) {
		codec_err("[%s](%d) error\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int ak4376_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	codec_dbg("[%s]: (addr,data)=(%x, %x)\n",__FUNCTION__, reg, value);

	if(i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		codec_err("[%s](%d) error\n",__FUNCTION__,__LINE__);
		return EIO;
	}

	return 0;
}

static int ak4376_writeMask(struct snd_soc_codec *codec, u16 reg, u16 mask, u16 value)
{
	u32 old;
	u32 new;
	int ret = 0;
	old = ak4376_i2c_read(codec, reg);
	new = (old & ~(mask)) | value;
	ret = ak4376_i2c_write(codec, reg, new);

	codec_dbg("[ak4376_writeMask] %s(%d): (addr,data)=(%x, %x)\n",__FUNCTION__,__LINE__, reg, new);

	return 0;
}

static int get_bickfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);

	//struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	//struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	codec_trace();
	ucontrol->value.enumerated.item[0] = ak4376->nBickFreq;
	codec_dbg("[%s]nBickFreq=%d\n",__func__,  ucontrol->value.enumerated.item[0]);

	return 0;
}

static int ak4376_set_bickfs(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	codec_trace();
	if ( ak4376->nBickFreq == 0 ) { 	//32fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
	}
	else if( ak4376->nBickFreq == 1 ) {	//48fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x00);	//DL1-0=00(24bit, >=48fs)
	}
	else {								//64fs
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x02, 0x02);	//DL1-0=1x(32bit, >=64fs)
	}

	return 0;
}

static int set_bickfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);

	ak4376->nBickFreq = ucontrol->value.enumerated.item[0];
	codec_dbg("[%s]nBickFreq=%d\n", __func__, ak4376->nBickFreq);
	ak4376_set_bickfs(component->codec);

	return 0;
}

static int get_srcfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = ak4376->nSrcOutFsSel;
	codec_dbg("[%s]nSrcOutFsSel=%d\n", __func__, ucontrol->value.enumerated.item[0]);

	return 0;
}

static int set_srcfs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);

	ak4376->nSrcOutFsSel = ucontrol->value.enumerated.item[0];
	codec_dbg("[%s]nSrcOutFsSel=%d\n", __func__, ak4376->nSrcOutFsSel);

	return 0;
}

//static int get_smtcycle(
//struct snd_kcontrol       *kcontrol,
//struct snd_ctl_elem_value  *ucontrol)
//{
//    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
//	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
//
//    ucontrol->value.enumerated.item[0] = ak4376->nSmt;
//
//    return 0;
//}
//
//static int set_smtcycle(
//struct snd_kcontrol       *kcontrol,
//struct snd_ctl_elem_value  *ucontrol)
//{
//    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
//	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
//
//	ak4376->nSmt = ucontrol->value.enumerated.item[0];
//
//	//0:1024, 1:2048, 2:4096, 3:8192
//	ak4376_writeMask(codec, AK4376_09_JITTER_CLEANER_SETTING2, 0x0C, ((ak4376->nSmt) << 2));
//
//    return 0;
//}

static int get_dfsrc8fs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol); //rong.fu modify
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);//rong.fu modify

	//struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	//	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.enumerated.item[0] = ak4376->dfsrc8fs;
	codec_trace();

	return 0;
}


static int ak4376_set_dfsrc8fs(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
	codec_trace();

	if (ak4376 == NULL) {
		codec_dbg("[%s] error #1\n", __func__);
		return 0;
	}
	codec_trace();
	codec_dbg("[%s]ak4376->dfsrc8fs=%d\n", __func__, ak4376->dfsrc8fs);

	switch (ak4376->dfsrc8fs) {
		case 0:		//DAC Filter
			snd_soc_update_bits(codec, AK4376_06_DIGITAL_FILTER_SELECT, 0x08, 0x00); 	//DFTHR=0
			snd_soc_update_bits(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00); //SRCO8FS=0
			break;
		case 1:		//Bypass
			snd_soc_update_bits(codec, AK4376_06_DIGITAL_FILTER_SELECT, 0x08, 0x08); 	//DFTHR=1
			snd_soc_update_bits(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00); //SRCO8FS=0
			break;
		case 2:		//8fs mode
			snd_soc_update_bits(codec, AK4376_06_DIGITAL_FILTER_SELECT, 0x08, 0x08); 	//DFTHR=1
			snd_soc_update_bits(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0x20, 0x20); //SRCO8FS=1
			break;
	}

	return 0;
}


static int set_dfsrc8fs(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);

	//struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	//struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	ak4376->dfsrc8fs = ucontrol->value.enumerated.item[0];
	codec_dbg("[%s]ak4376->dfsrc8fs=%d\n", __func__, ak4376->dfsrc8fs);
	ak4376_set_dfsrc8fs(component->codec);

	return 0;
}

#ifdef AK4376_DEBUG

static const char *test_reg_select[] =
{
	"read AK4376 Reg 00:24",
};

static const struct soc_enum ak4376_enum[] =
{
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol); //rong.fu modify
	struct ak4376_priv *ak4376 = snd_soc_component_get_drvdata(component);//rong.fu modify

	//struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol); //codec is not NULL, already proved,snd_kcontrol_chip(kcontrol) is codec->conponent, not codec
	//struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);

	if (ak4376)
		codec_dbg("[%s]%d\n", __func__, ak4376->dfsrc8fs);
	else
		codec_dbg("[%s]%d ERROR\n", __func__, __LINE__);

	codec_trace();
	/* Get the current output routing */
	ucontrol->value.enumerated.item[0] = nTestRegNo;

	return 0;
}

static int set_test_reg(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;
	u32 currMode = ucontrol->value.enumerated.item[0];
	unsigned int i, value;
	unsigned int regs, rege;

	nTestRegNo = currMode;
	regs = 0x00;
	rege = 0x15;

	codec_trace();
	for ( i = regs ; i <= rege ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***AK4376 Addr,Reg=(%x, %x)\n", i, value);
	}
	value = snd_soc_read(codec, 0x24);
	printk("***AK4376 Addr,Reg=(%x, %x)\n", 0x24, value);

	return 0;
}
#endif

static int inCall = 0;
static int get_incall_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = inCall;
	codec_dbg("%s inCall=%d\n", __func__, inCall);

	return 0;
}

static int set_incall_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value  *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;

	inCall = ucontrol->value.enumerated.item[0];
	codec_dbg("%s inCall=%d\n", __func__, inCall);
	snd_soc_write(codec, AK4376_0D_HP_VOLUME_CONTROL, 0x73);
	if (inCall) {
		snd_soc_write(codec, AK4376_03_POWER_MANAGEMENT4, 0x73);
		snd_soc_write(codec, AK4376_04_OUTPUT_MODE_SETTING, 0x0c);
		snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, 0x24);
		snd_soc_write(codec, AK4376_07_DAC_MONO_MIXING, 0x11);//mono for speech
		snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, 0xef);
		snd_soc_write(codec, AK4376_14_DAC_CLK_DIVIDER, 0x0e);
		snd_soc_write(codec, AK4376_15_AUDIO_IF_FORMAT, 0x21);
	} else {
		snd_soc_write(codec, AK4376_07_DAC_MONO_MIXING, 0x21);//stereo for music
		//snd_soc_write(codec, AK4376_07_DAC_MONO_MIXING, 0x21);
		//snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, 0x27);
		//snd_soc_write(codec, AK4376_15_AUDIO_IF_FORMAT, 0x22);
	}
	return 0;

}

static int g_codec_hp_state = 0;

enum hp_ctl_enum {
	HP_OFF = 0,
	HP_ON,
};

int akm4376_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	printk("%s state %d\n", __func__, g_codec_hp_state);
	ucontrol->value.integer.value[0] = g_codec_hp_state;
	return 0;
}

/*
 * keep it simple and quick
 * check each mdelay and udelay later
 * follow spec P43
 * */
int akm4376_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;

	int state = ucontrol->value.enumerated.item[0];

	codec_dbg("\n>>> %s  set  %d <<<\n", __func__, state);

	if (state == HP_ON)
	{
	       /* MODIFIED-BEGIN by hongwei.tian, 2018-04-17,BUG-6204707*/
	       codec_dbg("hw_params_status = %d \n",hw_params_status);
		if(hw_params_status)
		{		// 0x00 == 0x00 PLL start for blck pmosc stop
			snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
			mdelay(7);                                                          //spec need 6.5
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
			mdelay(1);															//wait 1ms

			//pwr up dac
			snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
			mdelay(5);															//spec need 4.5ms

			//open hp amp
			snd_soc_update_bits(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);

			//spec need 25.9ms@44K1
		}
		/* MODIFIED-END by hongwei.tian,BUG-6204707*/
	}
	else
	{
		//close hp amp
		snd_soc_update_bits(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x00);

		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

		//pwr down dac
		snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

		// 0x00 == 0x00 PLL start for blck pmosc stop
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x00);
	}

	g_codec_hp_state = state;
	hw_params_status = 0; // MODIFIED by hongwei.tian, 2018-04-17,BUG-6204707

	return 0;
}

/* MODIFIED-BEGIN by hongwei.tian, 2018-04-28,BUG-6267565*/
int akm4376_get_param(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	printk("%s state %d\n", __func__, hw_params_status);
	ucontrol->value.integer.value[0] = hw_params_status;
	return 0;
}

/*
 * keep it simple and quick
 * check each mdelay and udelay later
 * follow spec P43
 * */
int akm4376_set_param(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = component->codec;

	int state = ucontrol->value.enumerated.item[0];

	codec_dbg("\n>>> %s  set  %d <<<\n", __func__, state);
	if(hw_params_status)
		return 0;
	if (state == 1)
	{
		ak4376_hw_params_set(codec, ak4376_data->fs1);
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10, 0x00);

		 /* MODIFIED-BEGIN by hongwei.tian, 2018-04-17,BUG-6204707*/
		 codec_dbg("g_codec_hp_state = %d \n",g_codec_hp_state);
		 if(g_codec_hp_state)
		 {
			int ret;
			snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
			mdelay(7);                                                          //spec need 6.5
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
			mdelay(1);															//wait 1ms

			//pwr up dac
			snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
			mdelay(5);															//spec need 4.5ms

			ret = snd_soc_read(codec, AK4376_03_POWER_MANAGEMENT4);
			codec_dbg("%s AK4376_03_POWER_MANAGEMENT4=0x%x \n", __func__, ret);
			//open hp amp
			snd_soc_update_bits(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);

			//spec need 25.9ms@44K1
	 	}
		hw_params_status = 1;
	}
	return 0;
}
/* MODIFIED-END by hongwei.tian,BUG-6267565*/




static const char *hp_ctl_txt[] = {"off", "on"};

static const struct soc_enum hp_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, hp_ctl_txt),
};


static const struct snd_kcontrol_new ak4376_snd_controls[] = {
	SOC_SINGLE_TLV("AK4376 Digital Output VolumeL",
			AK4376_0B_LCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovl_tlv),
	SOC_SINGLE_TLV("AK4376 Digital Output VolumeR",
			AK4376_0C_RCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovr_tlv),
	SOC_SINGLE_TLV("AK4376 HP-Amp Analog Volume",
			AK4376_0D_HP_VOLUME_CONTROL, 0, 0x1F, 0, hpg_tlv),

	SOC_ENUM("AK4376 Digital Volume Control", ak4376_dac_enum[0]),
	SOC_ENUM("AK4376 DACL Signal Level", ak4376_dac_enum[1]),
	SOC_ENUM("AK4376 DACR Signal Level", ak4376_dac_enum[2]),
	SOC_ENUM("AK4376 DACL Signal Invert", ak4376_dac_enum[3]),
	SOC_ENUM("AK4376 DACR Signal Invert", ak4376_dac_enum[4]),
	SOC_ENUM("AK4376 Charge Pump Mode", ak4376_dac_enum[5]),
	SOC_ENUM("AK4376 HPL Power-down Resistor", ak4376_dac_enum[6]),
	SOC_ENUM("AK4376 HPR Power-down Resistor", ak4376_dac_enum[7]),
	SOC_ENUM("AK4376 DAC Digital Filter Mode", ak4376_dac_enum[8]),
	SOC_ENUM("AK4376 SRC Digital Filter Mode", ak4376_dac_enum[9]),

	SOC_ENUM_EXT("AK4376 Data Output mode", ak4376_bitset_enum[4], get_dfsrc8fs, set_dfsrc8fs),
	SOC_ENUM_EXT("AK4376 BICK Frequency Select", ak4376_bitset_enum[0], get_bickfs, set_bickfs),
	SOC_ENUM_EXT("AK4376 SRC Output FS", ak4376_bitset_enum[1], get_srcfs, set_srcfs),
	//	SOC_ENUM_EXT("AK4376 Soft Mute Cycle Select", ak4376_bitset_enum[3], get_smtcycle, set_smtcycle),
	SOC_ENUM_EXT("AK4376 Incall Mode", ak4376_bitset_enum[5], get_incall_mode, set_incall_mode),
	SOC_ENUM_EXT("AK4376 PDN Control", hp_control_enum[0], akm4376_get_hp, akm4376_set_hp),

	SOC_SINGLE("AK4376 SRC Semi-Auto Mode", AK4376_09_JITTER_CLEANER_SETTING2, 1, 1, 0),
	SOC_SINGLE("AK4376 SRC Dither", AK4376_0A_JITTER_CLEANER_SETTING3, 4, 1, 0),
	SOC_SINGLE("AK4376 Soft Mute Control", AK4376_09_JITTER_CLEANER_SETTING2, 0, 1, 0),

#ifdef AK4376_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4376_enum[0], get_test_reg, set_test_reg),
#endif
	SOC_ENUM_EXT("AK4376 Param", hp_control_enum[0], akm4376_get_param, akm4376_set_param), // MODIFIED by hongwei.tian, 2018-04-28,BUG-6267565

};



/*
   static int ak4376_set_PLL_MCKI(struct snd_soc_codec *codec, int pll)
   {
   int PLDbit, PLMbit, MDIVbit, DIVbit;
   int nTemp;

   if (pll) {	//PLL use
   PLDbit = 8 - 1;
   PLMbit = 40 - 1;
   MDIVbit = 1 - 1;
   DIVbit = 1;

//PLD15-0
ak4376_i2c_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
ak4376_i2c_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
//PLM15-0
ak4376_i2c_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
ak4376_i2c_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
//DIVbit
nTemp = ak4376_i2c_read(codec, AK4376_13_SRC_CLK_SOURCE);
nTemp &= ~0x10;
nTemp |= ( DIVbit << 4 );
ak4376_i2c_write(codec, AK4376_13_SRC_CLK_SOURCE, nTemp);
//MDIV7-0
ak4376_i2c_write(codec, AK4376_14_DAC_CLK_DIVIDER, MDIVbit);
//PLL=ON
ak4376_writeMask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);	//PMPLL=1
ak4376_writeMask(codec, AK4376_13_SRC_CLK_SOURCE, 0x01, 0x01);		//SRCCKS=1
ak4376_data->nPllMCKI=1;
}
else {		//PLL not use
ak4376_writeMask(codec, AK4376_13_SRC_CLK_SOURCE, 0x01, 0x00);		//SRCCKS=0
ak4376_writeMask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x00);	//PMPLL=0
ak4376_data->nPllMCKI=0;
}

return 0;
}
 */

/* DAC control */
//pop noise, spec page47, power up sequence, CP1->LDO1P, LDO1N->CP2, ->PMHPR
/* MODIFIED-BEGIN by hongwei.tian, 2018-04-17,BUG-6204707*/
#if 0
static int ak4376_dac_event2(struct snd_soc_codec *codec, int event)
{
	codec_trace();
	codec_dbg("%s event=%d\n", __func__, event);
	/* MODIFIED-END by hongwei.tian,BUG-6204707*/

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
			mdelay(6);															//wait 6ms
			udelay(500);														//wait 0.5ms
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
			mdelay(1);															//wait 1ms
			break;
		case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
			mdelay(4);															//wait 4ms
			udelay(500);														//wait 0.5ms
			break;
		case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0
			break;
		case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
			snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0
			//		if (ak4376_data->nPllMode==1) ak4376_set_PLL_MCKI(codec, 0);
			break;
	}
	return 0;
}
/* MODIFIED-BEGIN by hongwei.tian, 2018-04-17,BUG-6204707*/
#endif
static int ak4376_dac_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
//	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	codec_trace();
//	ak4376_dac_event2(codec, event);
/* MODIFIED-END by hongwei.tian,BUG-6204707*/

	return 0;
}

/* DAC MUX */
static const char *ak4376_seldain_select_texts[] = {"SDTI", "SRC"};

static const struct soc_enum ak4376_seldain_mux_enum =
SOC_ENUM_SINGLE(AK4376_0A_JITTER_CLEANER_SETTING3, 1,
		ARRAY_SIZE(ak4376_seldain_select_texts), ak4376_seldain_select_texts);

static const struct snd_kcontrol_new ak4376_seldain_mux_control =
SOC_DAPM_ENUM("SRC Select", ak4376_seldain_mux_enum);

/* HPL Mixer */
static const struct snd_kcontrol_new ak4376_hpl_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACL", AK4376_07_DAC_MONO_MIXING, 0, 1, 0),
	SOC_DAPM_SINGLE("RDACL", AK4376_07_DAC_MONO_MIXING, 1, 1, 0),
};

/* HPR Mixer */
static const struct snd_kcontrol_new ak4376_hpr_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACR", AK4376_07_DAC_MONO_MIXING, 4, 1, 0),
	SOC_DAPM_SINGLE("RDACR", AK4376_07_DAC_MONO_MIXING, 5, 1, 0),
};


/* ak4376 dapm widgets */
static const struct snd_soc_dapm_widget ak4376_dapm_widgets[] = {
	// DAC
	SND_SOC_DAPM_DAC_E("AK4376 DAC", NULL, AK4376_02_POWER_MANAGEMENT3, 0, 0, // MODIFIED by hongwei.tian, 2018-01-15,BUG-5875012
			ak4376_dac_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

#ifdef PLL_BICK_MODE
	SND_SOC_DAPM_SUPPLY("AK4376 PLL", AK4376_00_POWER_MANAGEMENT1, 0, 0, NULL, 0),
#endif
	SND_SOC_DAPM_SUPPLY("AK4376 OSC", AK4376_00_POWER_MANAGEMENT1, 4, 0, NULL, 0),

	SND_SOC_DAPM_MUX("AK4376 DAC MUX", SND_SOC_NOPM, 0, 0, &ak4376_seldain_mux_control),

	SND_SOC_DAPM_AIF_IN("AK4376 SRC", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AK4376 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

	// Analog Output
	SND_SOC_DAPM_OUTPUT("AK4376 HPL"),
	SND_SOC_DAPM_OUTPUT("AK4376 HPR"),

	SND_SOC_DAPM_MIXER("AK4376 HPR Mixer", AK4376_03_POWER_MANAGEMENT4, 1, 0,
			&ak4376_hpr_mixer_controls[0], ARRAY_SIZE(ak4376_hpr_mixer_controls)),

	SND_SOC_DAPM_MIXER("AK4376 HPL Mixer", AK4376_03_POWER_MANAGEMENT4, 0, 0,
			&ak4376_hpl_mixer_controls[0], ARRAY_SIZE(ak4376_hpl_mixer_controls)),

};

static const struct snd_soc_dapm_route ak4376_intercon[] =
{
#if 1
#ifdef PLL_BICK_MODE
	{"AK4376 DAC", NULL, "AK4376 PLL"},
#endif

	{"AK4376 SRC", NULL, "AK4376 OSC"},
	{"AK4376 DAC MUX", "SRC", "AK4376 SRC"},
	{"AK4376 DAC MUX", "SDTI", "AK4376 SDTI"},
	{"AK4376 DAC", NULL, "AK4376 DAC MUX"},

	{"AK4376 HPL Mixer", "LDACL", "AK4376 DAC"},
	{"AK4376 HPL Mixer", "RDACL", "AK4376 DAC"},
	{"AK4376 HPR Mixer", "LDACR", "AK4376 DAC"},
	{"AK4376 HPR Mixer", "RDACR", "AK4376 DAC"},

	{"AK4376 HPL", NULL, "AK4376 HPL Mixer"},
	{"AK4376 HPR", NULL, "AK4376 HPR Mixer"},
#endif
};

void ak4376_ldo_power_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4376_sys_data->ldo_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4376_sys_data->ldo_ctl_pin, 0);
    }

  //  codec_dbg("%s onoff=%d, %d", __func__, onoff,  mt_get_gpio_out(GPIO115_HIFI_LDO));
}

void ak4376_dac_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4376_sys_data->dac_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4376_sys_data->dac_ctl_pin, 0);
    }
//	codec_dbg("%s onoff=%d, %d", __func__, onoff,  mt_get_gpio_out(GPIO89_HIFI_PDN));
}

static int ak4376_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8 mode;
	u8 mode2;
	int mcki_rate;

	codec_dbg("[%s] fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

	if ((fs != 0)&&(rclk != 0)) {
		if (rclk > 28800000) return -EINVAL;
		mcki_rate = rclk/fs;

		mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
		mode &= ~AK4376_CM;

		if (ak4376_data->nSeldain == 0) {	//SRC Bypass Mode
			switch (mcki_rate) {
				case 128:
					mode |= AK4376_CM_3;
					break;
				case 256:
					mode |= AK4376_CM_0;
					mode2 = snd_soc_read(codec, AK4376_24_MODE_CONTROL);
					if ( fs <= 12000 ) {
						mode2 &= 0x40;	//DSMLP=1
						snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
					}
					else {
						mode2 &= ~0x40;	//DSMLP=0
						snd_soc_write(codec, AK4376_24_MODE_CONTROL, mode2);
					}
					break;
				case 512:
					mode |= AK4376_CM_1;
					break;
				case 1024:
					mode |= AK4376_CM_2;
					break;
				default:
					return -EINVAL;
			}
		}
		else {								//SRC Mode
			switch (mcki_rate) {
				case 256:
					mode |= AK4376_CM_0;
					break;
				case 512:
					mode |= AK4376_CM_1;
					break;
				case 1024:
					mode |= AK4376_CM_2;
					break;
					//			case 128:
					//				mode |= AK4376_CM_0;
					//				if (fs <= 96000) return -EINVAL;
					//				ak4376_set_PLL_MCKI(codec, 1);
					//				break;
				default:
					return -EINVAL;
			}
		}
		snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);

	}

	return 0;
}

static int ak4376_set_src_mcki(struct snd_soc_codec *codec, int fs)
{
	u8 nrate;
	int oclk_rate;
	int src_out_fs;

	nrate = snd_soc_read(codec, AK4376_08_JITTER_CLEANER_SETTING1);
	nrate &= ~0x7F;	//CM21-0 bits, FS24-0 bits

	codec_trace();
#ifdef SRC_OUT_FS_48K
	src_out_fs = 48000 * ( 1 << (ak4376_data->nSrcOutFsSel));
#else
	src_out_fs = 44100 * ( 1 << (ak4376_data->nSrcOutFsSel));
#endif
	switch (src_out_fs) {
		case 44100:
			nrate |= AK4376_FS_44_1KHZ;
			break;
		case 48000:
			nrate |= AK4376_FS_48KHZ;
			break;
		case 88200:
			nrate |= AK4376_FS_88_2KHZ;
			break;
		case 96000:
			nrate |= AK4376_FS_96KHZ;
			break;
		case 176400:
			nrate |= AK4376_FS_176_4KHZ;
			break;
		case 192000:
			nrate |= AK4376_FS_192KHZ;
			break;
		default:
			return -EINVAL;
	}

	oclk_rate = XTAL_OSC_FS/src_out_fs;
	switch (oclk_rate) {
		case 128:
			nrate |= AK4376_CM_3;
			break;
		case 256:
			nrate |= AK4376_CM_0;
			break;
		case 512:
			nrate |= AK4376_CM_1;
			break;
		case 1024:
			nrate |= AK4376_CM_2;
			break;
		default:
			return -EINVAL;
	}

	ak4376_data->fs2 = src_out_fs;
	snd_soc_write(codec, AK4376_08_JITTER_CLEANER_SETTING1, nrate);

	return 0;
}

static int ak4376_set_pllblock(struct snd_soc_codec *codec, int fs)
{
	u8 mode;
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit, DIVbit;
	int nTemp;

	mode = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	mode &= ~AK4376_CM;

	codec_dbg("%s fs=%d\n", __func__, fs);
	if (ak4376_data->nSeldain == 0) {	//SRC bypass
		if ( fs <= 24000 ) {
			mode |= AK4376_CM_1;
			nMClk = 512 * fs;
		}
		else if ( fs <= 96000 ) {
			mode |= AK4376_CM_0;
			nMClk = 256 * fs;
		}
		else {
			mode |= AK4376_CM_3;
			nMClk = 128 * fs;
		}
	}
	else {								//SRC
		if ( fs <= 24000 ) {
			mode |= AK4376_CM_1;
			nMClk = 512 * fs;
		}
		else  {
			mode |= AK4376_CM_0;
			nMClk = 256 * fs;
		}
	}
	snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);

	if ( (fs % 8000) == 0 ) {
		nPLLClk = 122880000;
	}
	else if ( (fs == 11025 ) && ( ak4376_data->nBickFreq == 1 )) {
		nPLLClk = 101606400;
	}
	else {
		nPLLClk = 112896000;
	}

	if ( ak4376_data->nBickFreq == 0 ) {		//32fs
		if ( fs <= 96000 ) PLDbit = 1;
		else PLDbit = 2;
		nRefClk = 32 * fs / PLDbit;
	}
	else if ( ak4376_data->nBickFreq == 1 ) {	//48fs
		if ( fs <= 16000 ) PLDbit = 1;
		else PLDbit = 3;
		nRefClk = 48 * fs / PLDbit;
	}
	else {  									// 64fs
		if ( fs <= 48000 ) PLDbit = 1;
		else if ( fs <= 96000 ) PLDbit = 2;
		else PLDbit = 4;
		nRefClk = 64 * fs / PLDbit;
	}

	PLMbit = nPLLClk / nRefClk;
	codec_dbg("PLDbit=%d, PLMbit=%d, nPLLClk=%d, nRefClk=%d\n", PLDbit, PLMbit, nPLLClk, nRefClk);

	if ( ( ak4376_data->nSeldain == 0 ) || ( fs <= 96000 ) ) {
		MDIVbit = nPLLClk / nMClk;
		DIVbit = 0;
	}
	else {
		MDIVbit = 5;
		DIVbit = 1;
	}

	PLDbit--;
	PLMbit--;
	MDIVbit--;

	//PLD15-0
	snd_soc_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	//PLM15-0
	snd_soc_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
	//DIVbit
	nTemp = snd_soc_read(codec, AK4376_13_SRC_CLK_SOURCE);
	nTemp &= ~0x10;
	nTemp |= ( DIVbit << 4 );
	snd_soc_write(codec, AK4376_13_SRC_CLK_SOURCE, (nTemp|0x01));		//DIV=0or1,SRCCKS=1(SRC Clock Select=PLL) set
	snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x01, 0x01);	//PLS=1(BICK)

	//MDIV7-0
	snd_soc_write(codec, AK4376_14_DAC_CLK_DIVIDER, MDIVbit);

	return 0;
}

static int ak4376_set_timer(struct snd_soc_codec *codec)
{
	int ret, curdata;
	int count, tm, nfs;
	int lvdtm, vddtm, hptm;

	lvdtm = 0;
	vddtm = 0;
	hptm = 0;

	codec_trace();
	if ( ak4376_data->nSeldain == 1 ) nfs = ak4376_data->fs2;
	else 	nfs = ak4376_data->fs1;

	//LVDTM2-0 bits set
	ret = snd_soc_read(codec, AK4376_03_POWER_MANAGEMENT4);
	curdata = (ret & 0x70) >> 4;	//Current data Save
	ret &= ~0x70;
	do {
		count = 1000 * (64 << lvdtm);
		tm = count / nfs;
		if ( tm > LVDTM_HOLD_TIME ) break;
		lvdtm++;
	} while ( lvdtm < 7 );			//LVDTM2-0 = 0~7
	if ( curdata != lvdtm) {
		snd_soc_write(codec, AK4376_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));
	}

	//VDDTM3-0 bits set
	ret = snd_soc_read(codec, AK4376_04_OUTPUT_MODE_SETTING);
	curdata = (ret & 0x3C) >> 2;	//Current data Save
	ret &= ~0x3C;
	do {
		count = 1000 * (1024 << vddtm);
		tm = count / nfs;
		if ( tm > VDDTM_HOLD_TIME ) break;
		vddtm++;
	} while ( vddtm < 8 );			//VDDTM3-0 = 0~8
	if ( curdata != vddtm) {
		snd_soc_write(codec, AK4376_04_OUTPUT_MODE_SETTING, (ret | (vddtm<<2)));
	}

	//HPTM2-0 bits set
	ret = snd_soc_read(codec, AK4376_0D_HP_VOLUME_CONTROL);
	curdata = (ret & 0xE0) >> 5;	//Current data Save
	ret &= ~0xE0;
	do {
		count = 1000 * (128 << hptm);
		tm = count / nfs;
		if ( tm > HPTM_HOLD_TIME ) break;
		hptm++;
	} while ( hptm < 4 );			//HPTM2-0 = 0~4
	if ( curdata != hptm) {
		snd_soc_write(codec, AK4376_0D_HP_VOLUME_CONTROL, (ret | (hptm<<5)));
	}

	return 0;
}

static int ak4376_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
	u8 	fs;
	u8  src;

	codec_trace();

	src = snd_soc_read(codec, AK4376_0A_JITTER_CLEANER_SETTING3);
	src = (src & 0x02) >> 1;

	fs = snd_soc_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	fs &= ~AK4376_FS;

	//	ak4376_data->fs1 = params_rate(params);

	switch (nfs1) {
		case 8000:
			fs |= AK4376_FS_8KHZ;
			break;
		case 11025:
			fs |= AK4376_FS_11_025KHZ;
			break;
		case 16000:
			fs |= AK4376_FS_16KHZ;
			break;
		case 22050:
			fs |= AK4376_FS_22_05KHZ;
			break;
		case 32000:
			fs |= AK4376_FS_32KHZ;
			break;
		case 44100:
			fs |= AK4376_FS_44_1KHZ;
			break;
		case 48000:
			fs |= AK4376_FS_48KHZ;
			break;
		case 88200:
			fs |= AK4376_FS_88_2KHZ;
			break;
		case 96000:
			fs |= AK4376_FS_96KHZ;
			break;
		case 176400:
			fs |= AK4376_FS_176_4KHZ;
			break;
		case 192000:
			fs |= AK4376_FS_192KHZ;
			break;
		default:
			return -EINVAL;
	}
	snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, fs);

	if ( ak4376_data->nPllMode == 0 ) {	//Not PLL mode
		ak4376_set_mcki(codec, nfs1, ak4376_data->rclk);
	}
	else {								//PLL mode
		ak4376_set_pllblock(codec, nfs1);
	}

	if ( src == 1 ) {				//SRC mode
		ak4376_data->nSeldain = 1;
		snd_soc_update_bits(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0xC2, 0xC2);	//XCKSEL=XCKCPSEL=SELDAIN=1
		ak4376_set_src_mcki(codec, nfs1);
	}
	else {							//SRC Bypass mode
		ak4376_data->nSeldain = 0;
		snd_soc_update_bits(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0xC2, 0x00);	//XCKSEL=XCKCPSEL=SELDAIN=0
	}

	ak4376_set_timer(codec);

	return 0;
}

static int ak4376_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
#ifdef AK4376_DEBUG
	int rate = params_rate(params); // MODIFIED by hongwei.tian, 2016-11-18,BUG-3467595
	int format = params_format(params);
#endif

	/* MODIFIED-BEGIN by hongwei.tian, 2018-04-28,BUG-6267565*/
	if(hw_params_status)
		return 0;
		/* MODIFIED-END by hongwei.tian,BUG-6267565*/

	ak4376_data->fs1 = params_rate(params);

	//add by rong.fu, set right BickFreq, task:2508967, 2016/07/14, add start
	codec_dbg("%s sample rate=%d, rate=%d, format=%d\n", __func__, ak4376_data->fs1, rate, format);

	ak4376_hw_params_set(codec, ak4376_data->fs1);
	snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10, 0x00);

	 /* MODIFIED-BEGIN by hongwei.tian, 2018-04-17,BUG-6204707*/
	 codec_dbg("g_codec_hp_state = %d \n",g_codec_hp_state);
	 if(g_codec_hp_state)
	 {
		int ret;
		snd_soc_update_bits(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
		mdelay(7);                                                          //spec need 6.5
		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
		mdelay(1);															//wait 1ms

		//pwr up dac
		snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

		snd_soc_update_bits(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
		mdelay(5);															//spec need 4.5ms

		ret = snd_soc_read(codec, AK4376_03_POWER_MANAGEMENT4);
		codec_dbg("%s AK4376_03_POWER_MANAGEMENT4=0x%x \n", __func__, ret);
		//open hp amp
		snd_soc_update_bits(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);

		//spec need 25.9ms@44K1
	 }
	hw_params_status = 1;
/*[Defect]-Modified-END by TCT-NB.Tianhongwei, 1527674 , 2016/03/31*/
/* MODIFIED-END by hongwei.tian,BUG-6204707*/
	return 0;
}

static int ak4376_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	/* MODIFIED-BEGIN by hongwei.tian, 2016-11-18,BUG-3467595*/
	//codec_dbg("%s rate=%d\n", __func__, substream->runtime->rate);
	codec_trace();
	/* MODIFIED-END by hongwei.tian,BUG-3467595*/
	return 0;
}

static int ak4376_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;

	ak4376_data->rclk = freq;
	codec_dbg("%s freq=%d\n", __func__, freq);

	if ( ak4376_data->nPllMode == 0 ) {	//Not PLL mode
		ak4376_set_mcki(codec, ak4376_data->fs1, freq);
	}

	return 0;
}

static int ak4376_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 format;

	codec_dbg("%s fmt=%d\n", __func__, fmt);

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4376_15_AUDIO_IF_FORMAT);
	format &= ~AK4376_DIF;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			format |= AK4376_DIF_I2S_MODE;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			format |= AK4376_DIF_MSB_MODE;
			break;
		default:
			return -EINVAL;
	}

	/* set format */
	snd_soc_write(codec, AK4376_15_AUDIO_IF_FORMAT, format);

	return 0;
}


static int ak4376_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	codec_trace();

	return 0;
}

static int ak4376_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	codec_dbg("%s level=%d\n", __func__, level);

	switch (level) {
		case SND_SOC_BIAS_ON:
		case SND_SOC_BIAS_PREPARE:
		case SND_SOC_BIAS_STANDBY:
			break;
		case SND_SOC_BIAS_OFF:
			break;
	}
	dapm->bias_level = level;

	return 0;
}
#if 0
static int ak4376_set_dai_mute2(struct snd_soc_codec *codec, int mute)
{
	int ret = 0;
	int nfs, ndt, ndt2;

	if ( ak4376_data->nSeldain == 1 ) nfs = ak4376_data->fs2;
	else	nfs = ak4376_data->fs1;

	codec_dbg("[%s] mute[%s]\n",__FUNCTION__, mute ? "ON":"OFF");

	if (mute) {	//SMUTE: 1 , MUTE
		if (ak4376_data->nSeldain) {
			ret = snd_soc_update_bits(codec, AK4376_09_JITTER_CLEANER_SETTING2, 0x01, 0x01);
			ndt = (1024000 << ak4376_data->nSmt) / nfs;
			mdelay(ndt);
			ret = snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x80, 0x00);
		}
	}
	else {		// SMUTE: 0 ,NORMAL operation
		ak4376_data->nSmt = (ak4376_data->nSrcOutFsSel + SMUTE_TIME_MODE);
		ret = snd_soc_update_bits(codec, AK4376_09_JITTER_CLEANER_SETTING2, 0x0C, (ak4376_data->nSmt << 2));
		ndt = (26 * nfs) / 44100;		//for After HP-Amp Power up
		if (ak4376_data->nSeldain) {
			ret = snd_soc_update_bits(codec, AK4376_02_POWER_MANAGEMENT3, 0x80, 0x80);
			ndt2 = (1024000 << ak4376_data->nSmt) / nfs;
			ndt -= ndt2;
			if (ndt < 4) ndt=4;
			mdelay(ndt);
			ret = snd_soc_update_bits(codec, AK4376_09_JITTER_CLEANER_SETTING2, 0x01, 0x00);
			mdelay(ndt2);
		}
		else {
			mdelay(ndt);
		}
	}
	return ret;
}

static int ak4376_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	codec_dbg("%s mute=%d\n", __func__, mute);
	ak4376_set_dai_mute2(codec, mute);

	return 0;
}
#endif
#define AK4376_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
		SNDRV_PCM_RATE_192000)

#define AK4376_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE

static struct snd_soc_dai_ops ak4376_dai_ops = {
	.prepare = ak4376_prepare,
	.hw_params	= ak4376_hw_params,
	.set_sysclk	= ak4376_set_dai_sysclk,
	.set_fmt	= ak4376_set_dai_fmt,
	.trigger = ak4376_trigger,
	//.digital_mute = ak4376_set_dai_mute,
};

struct snd_soc_dai_driver ak4376_dai[] = {
	{
		.name = "ak4376-AIF1",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AK4376_RATES,
			.formats = AK4376_FORMATS,
		},
		.ops = &ak4376_dai_ops,
	},
};
#if 0
static int ak4376_init_reg(struct snd_soc_codec *codec)
{
	udelay(800);
	codec_trace();

	ak4376_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

#ifdef PLL_BICK_MODE
	snd_soc_update_bits(codec, AK4376_13_SRC_CLK_SOURCE, 0x01, 0x01);			//SRCCKS=1(SRC Clock Select=PLL)
	snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x01, 0x01);	//PLS=1(BICK)
#endif

	return 0;
}
#endif

/*
 * platform data
 * */
static int ak4376_parse_dt(struct device *dev, struct ak4376_sys_data_s *sys_data)
{
    struct device_node *np = dev->of_node;
    sys_data->dac_ctl_pin = of_get_named_gpio(np, "akm,dac-gpio", 0);
    codec_dbg("adc pdm gpio(%d) \n",sys_data->dac_ctl_pin);
    if (sys_data->dac_ctl_pin < 0) {
        return sys_data->dac_ctl_pin;
    }

    sys_data->ldo_ctl_pin = of_get_named_gpio(np, "akm,ldo-gpio", 0);
    codec_dbg("ldo gpio(%d) \n",sys_data->ldo_ctl_pin);
    if (sys_data->ldo_ctl_pin < 0) {
        return sys_data->ldo_ctl_pin;
    }

    return 0;
}

/*
 * default ouput 0
 * follow akm startup timeline
 * */
static int ak4376_config_pins(struct ak4376_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->dac_ctl_pin, "akm_dac_ctl") < 0) {
        codec_dbg("gpio err  %d\n", sys_data->dac_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->dac_ctl_pin, 0);


    if (gpio_request(sys_data->ldo_ctl_pin, "akm_ldo_ctl") < 0) {
        codec_dbg("gpio err %d\n", sys_data->ldo_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->ldo_ctl_pin, 0);

    return  0;
}


static int ak4376_regulator_init(struct ak4376_sys_data_s *sys_data)
{
    int rc = -1;

    /*
     * codec V_L6_1P8 src VREG_L6
     * I2C pull up  V_L6_1P8 src VREG_L6
     * */
    sys_data->vdd = regulator_get(&sys_data->client->dev, "vdd");
    if (IS_ERR(sys_data->vdd)) {
        codec_dbg("%s %d get vdd error\n", __func__, __LINE__);
        goto err_get_vdd;
    }

    if (regulator_count_voltages(sys_data->vdd) > 0)
    {
        rc = regulator_set_voltage(sys_data->vdd, 1800000, 1800000);
        if (rc) {
            codec_dbg("%s %d set vdd error\n", __func__, __LINE__);
            goto err_set_vdd;
        }
    }

    /*
     * VLDO_1V8  <== LDO src (VPH_PWR == VBATT)
     * */
    rc = regulator_enable(sys_data->vdd);
    if (rc) {
        dev_err(&sys_data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }


    return 0;

err_set_vdd:
    regulator_put(sys_data->vdd);

err_get_vdd:
    if (regulator_count_voltages(sys_data->vdd) > 0) {
        regulator_set_voltage(sys_data->vdd, 0, 1800000);
    }

    return rc;
}


#include "ak4376_debug.c"
static int ak4376_probe(struct snd_soc_codec *codec)
{
	struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);//from (1)(2) => codec->component->dev->driver_data = ak4376_data
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec); // MODIFIED by hongwei.tian, 2018-01-15,BUG-5875012
	int ret = 0;
	int chipId = 0;

	codec_trace();
	if (!ak4376) {
		codec_err("[%s][%d] error\n", __FUNCTION__,__LINE__);

	}
	snd_soc_codec_set_drvdata(codec, ak4376_data);
	codec->control_data = ak4376_codec->control_data;
	//codec->write = ak4376_i2c_write;
	//codec->read = ak4376_i2c_read;

	codec_trace();
	chipId = snd_soc_read(codec, 0x15);
	if (chipId & 0x40) // MODIFIED by hongwei.tian, 2016-11-03,BUG-3275213
		i2c_ok = 1;
	codec_dbg("chipId=%d, i2c_ok=%d\n", chipId, i2c_ok);

	//ak4376_init_reg(codec);
	ak4376_pre_init(codec);

	ak4376->fs1 = 48000;
	ak4376->fs2 = 48000;
	ak4376->rclk = 0;
	ak4376->nSeldain = 0;		//0:Bypass, 1:SRC	(Dependent on a register bit)
	ak4376->nBickFreq = 0;		//0:32fs, 1:48fs, 2:64fs
	ak4376->nSrcOutFsSel = 0;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
#ifdef PLL_BICK_MODE
	ak4376->nPllMode = 1;		//1: PLL ON
#else
	ak4376->nPllMode = 0;		//0:PLL OFF
#endif
	ak4376->nSmt = 0;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
	ak4376->dfsrc8fs = 0;		//0:DAC Filter, 1:Bypass, 2:8fs mode


	/* MODIFIED-BEGIN by hongwei.tian, 2018-01-15,BUG-5875012*/
	snd_soc_dapm_ignore_suspend(dapm, "AK4376 DAC");
	snd_soc_dapm_ignore_suspend(dapm, "AK4376 HPL");
	snd_soc_dapm_ignore_suspend(dapm, "AK4376 HPR");
	snd_soc_dapm_ignore_suspend(dapm, "AK4376 PLL");
	snd_soc_dapm_ignore_suspend(dapm,"AK4376 HPL Mixer");
	snd_soc_dapm_ignore_suspend(dapm,"AK4376 HPR Mixer");
	snd_soc_dapm_ignore_suspend(dapm,"Playback");
	snd_soc_dapm_sync(dapm);
	/* MODIFIED-END by hongwei.tian,BUG-5875012*/

	/* MODIFIED-BEGIN by hongwei.tian, 2016-11-03,BUG-3275213*/
	if(i2c_ok)
		i2c_check_status_create("audio_hifi",1);
	else
		i2c_check_status_create("audio_hifi",0);
		/* MODIFIED-END by hongwei.tian,BUG-3275213*/
	return ret;
}

static int ak4376_remove(struct snd_soc_codec *codec)
{
	codec_trace();
	ak4376_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ak4376_suspend(struct snd_soc_codec *codec)
{
	codec_trace();
	//ak4376_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if (g_codec_hp_state == HP_OFF) {
		ak4376_ldo_power_on(0);
		ak4376_dac_on(0);
	}

	return 0;
}

static int ak4376_resume(struct snd_soc_codec *codec)
{
	codec_trace();
	if (g_codec_hp_state == HP_OFF) {
		ak4376_pre_init(&ak4376_data->codec);
	}

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4376 = {
	.probe = ak4376_probe,
	.remove = ak4376_remove,
	.suspend =	ak4376_suspend,
	.resume =	ak4376_resume,

	.write = ak4376_i2c_write,
	.read = ak4376_i2c_read,
	.controls = ak4376_snd_controls,
	.num_controls = ARRAY_SIZE(ak4376_snd_controls),

	.set_bias_level = ak4376_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ak4376_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ak4376_reg,
	.dapm_widgets = ak4376_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4376_dapm_widgets),
	.dapm_routes = ak4376_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4376_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4376);

static int ak4376_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct ak4376_priv *ak4376;
	struct snd_soc_codec *codec;
	int ret = 0;
	u8 device_id = 0;

	codec_trace();

	ak4376 = kzalloc(sizeof(struct ak4376_priv), GFP_KERNEL);
	if (ak4376 == NULL)  {
		codec_err("no memory\n");
		return -ENOMEM;
	}

	ak4376_sys_data = kzalloc(sizeof(struct ak4376_sys_data_s), GFP_KERNEL);
    	if (ak4376_sys_data == NULL) {
        	kfree(ak4376);
        	return -ENOMEM;
   	 }
	codec = &ak4376->codec;
	ak4376_codec = codec;
	i2c_set_clientdata(i2c, ak4376);//i2c->dev->driver_data = ak4376  --- (1)
	codec->control_data = i2c;
	ak4376_data = ak4376;

	ak4376_sys_data->client = i2c;

	codec->dev = &i2c->dev;
	dev_set_name(&i2c->dev, "%s", "ak4376");
	//snd_soc_codec_set_drvdata(codec, ak4376);
	codec_trace();

	ret = ak4376_parse_dt(&(i2c->dev), ak4376_sys_data);
	if (ret) {
		codec_err("ak4376_parse_dt error %d\n", ret);
	}
	codec_trace();
	ret = ak4376_regulator_init(ak4376_sys_data);
	if (ret < 0) {
		codec_err("get regulator error %d\n", ret);
	}

	ak4376_config_pins(ak4376_sys_data);
	ak4376_dac_on(0);
	mdelay(1);

	ak4376_ldo_power_on(1);
	mdelay(3);
	// PDN pin to high
	ak4376_dac_on(1);
	mdelay(3);

	device_id = i2c_smbus_read_byte_data(ak4376_sys_data->client, (u8)(AK4376_15_AUDIO_IF_FORMAT & 0xFF));
	/*
	BIT7~BIT5
	000:AK4375
	001:AK4375A
	010:AK4376
	*/
	device_id = (device_id & 0xE0)>>5;
	printk("%s:device_id = %#x\n", __func__, device_id);


	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4376, &ak4376_dai[0], ARRAY_SIZE(ak4376_dai));//codec->component->dev = i2c->dev ---(2)
	if (ret < 0){
		kfree(ak4376);
		codec_err("[%s](%d) error\n",__FUNCTION__,__LINE__);
	}

	device_create_file(&(i2c->dev), &dev_attr_ak4376_control);

	printk("furong ak4376_i2c_probe-\n");

	return ret;
}

static int ak4376_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ak4376_i2c_id[] = {
	{ "ak4376", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

/* i2c driver */
static struct of_device_id ak4376_match_tbl[] = {
	{ .compatible = "akm,ak4376", },
	{ },
};

static struct i2c_driver ak4376_i2c_driver = {
	.driver = {
		.name = "ak4376",
		.owner = THIS_MODULE,
		.of_match_table = ak4376_match_tbl,
	},
	.probe = ak4376_i2c_probe,
	.remove = ak4376_i2c_remove,
	.id_table = ak4376_i2c_id,
};

static int __init ak4376_modinit(void)
{
	codec_trace();

	return i2c_add_driver(&ak4376_i2c_driver);
}

module_init(ak4376_modinit);

static void __exit ak4376_exit(void)
{
	i2c_del_driver(&ak4376_i2c_driver);
}
module_exit(ak4376_exit);

MODULE_DESCRIPTION("ASoC ak4376 codec driver");
MODULE_LICENSE("GPL");
