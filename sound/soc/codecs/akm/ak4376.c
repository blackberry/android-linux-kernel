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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include "ak4376.h"
#include <linux/clk.h>
/*
 * PLL means BLCk
 * NO PLL means mclk
 * */
#ifndef CONFIG_SND_44P1K_SUPPORT
#define PLL_BICK_MODE
#endif
#define AK4376_DEBUG			//used at debug mode
#define AK4376_CONTIF_DEBUG		//used at debug mode
#define AUDIO_NAME "#BC akm4376"

#ifdef AK4376_DEBUG
#define b_codec_dbg(format, arg...) \
    pr_debug(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_err(format, arg...) \
    pr_err(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_info(format, arg...) \
    pr_debug(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_warn(format, arg...) \
    pr_debug(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_trace() \
    pr_debug("  %s(%d)\n", __func__, __LINE__)
#else
#define b_codec_dbg(format, arg...) do {} while (0)
#define b_codec_err(format, arg...) do {} while (0)
#define b_codec_info(format, arg...) do {} while (0)
#define b_codec_warn(format, arg...) do {} while (0)
#define b_codec_trace()  do {} while (0)
#endif
u8 hw_params_status = 0;/*[Defect]-Modified-BEGIN by TCT-NB.Tianhongwei, 1527674 , 2016/03/31, workaround for pop noise*/

/*TCT-NB Tianhongwei add for NPI info*/
static int i2c_ok = 0;/*[TASK]-Add by TCTSH.Cedar, 937744, 2015/12/17, add for RunInTest to compatible with ak4375*/

/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
//extern int  ak4375_en;/*[Task]-Modified by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
/*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/

module_param_named(i2c_ok, i2c_ok, int, 0644);
/*TCT-NB Tianhongwei end*/

int g_dump_trace = 0;
#ifdef CONFIG_SND_44P1K_SUPPORT
static struct clk *ref_clk;
#define AK4376_CLK_FREQ			(9600000)
#endif
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
    int nPllMCKI;		//0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
    int nSmt;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
    int dfsrc8fs;		//DFTHR bit and SRCO8FS bit
    int xtalfreq;		//0:12.288MHz, 1:11.2896MHz
    int lpmode;		//0:High Performance, 1:Low power mode
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
static unsigned int ak4376_read(struct snd_soc_codec *codec,  u_int reg);


/* ak4376 register cache & default register settings */
static const u8 ak4376_reg[AK4376_MAX_REGISTERS] = {
	0x00,	/*	0x00	AK4376_00_POWER_MANAGEMENT1		*/
	0x00,	/*	0x01	AK4376_01_POWER_MANAGEMENT2		*/
	0x00,	/*	0x02	AK4376_02_POWER_MANAGEMENT3		*/
	0x00,	/*	0x03	AK4376_03_POWER_MANAGEMENT4		*/
	0x00,	/*	0x04	AK4376_04_OUTPUT_MODE_SETTING	*/
	0x00,	/*	0x05	AK4376_05_CLOCK_MODE_SELECT		*/
	0x00,	/*	0x06	AK4376_06_DIGITAL_FILTER_SELECT	*/
	0x00,	/*	0x07	AK4376_07_DAC_MONO_MIXING		*/
	0x00,	/*	0x08	AK4376_08_RESERVED				*/
	0x00,	/*	0x09	AK4376_09_RESERVED				*/
	0x00,	/*	0x0A	AK4376_0A_RESERVED				*/
	0x15,	/*	0x0B	AK4376_0B_LCH_OUTPUT_VOLUME		*/
	0x15,	/*	0x0C	AK4376_0C_RCH_OUTPUT_VOLUME		*/
	0x0A,	/*	0x0D	AK4376_0D_HP_VOLUME_CONTROL		*/
	0x00,	/*	0x0E	AK4376_0E_PLL_CLK_SOURCE_SELECT	*/
	0x00,	/*	0x0F	AK4376_0F_PLL_REF_CLK_DIVIDER1	*/
	0x00,	/*	0x10	AK4376_10_PLL_REF_CLK_DIVIDER2	*/
	0x00,	/*	0x11	AK4376_11_PLL_FB_CLK_DIVIDER1	*/
	0x00,	/*	0x12	AK4376_12_PLL_FB_CLK_DIVIDER2	*/
	0x00,	/*	0x13	AK4376_13_DAC_CLK_SOURCE		*/
	0x00,	/*	0x14	AK4376_14_DAC_CLK_DIVIDER		*/
	0x40,	/*	0x15	AK4376_15_AUDIO_IF_FORMAT		*/
	0x00,	/*	0x16	AK4376_16_DUMMY					*/
	0x00,	/*	0x17	AK4376_17_DUMMY					*/
	0x00,	/*	0x18	AK4376_18_DUMMY					*/
	0x00,	/*	0x19	AK4376_19_DUMMY					*/
	0x00,	/*	0x1A	AK4376_1A_DUMMY					*/
	0x00,	/*	0x1B	AK4376_1B_DUMMY					*/
	0x00,	/*	0x1C	AK4376_1C_DUMMY					*/
	0x00,	/*	0x1D	AK4376_1D_DUMMY					*/
	0x00,	/*	0x1E	AK4376_1E_DUMMY					*/
	0x00,	/*	0x1F	AK4376_1F_DUMMY					*/
	0x00,	/*	0x20	AK4376_20_DUMMY					*/
	0x00,	/*	0x21	AK4376_21_DUMMY					*/
	0x00,	/*	0x22	AK4376_22_DUMMY					*/
	0x00,	/*	0x23	AK4376_23_DUMMY					*/
	0x00,	/*	0x24	AK4376_24_MODE_CONTROL			*/
	0x00,
	0x18,	/*	0x26	AK4376_24_MODE_CONTROL			*/
	0x00,
	0x00,
	0x00,
	0x05,	/*	0x2A	AK4376_24_MODE_CONTROL			*/
};


static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4376_access_masks[] = {
    { 0xFF, 0x11 },	//0x00
    { 0xFF, 0x33 },	//0x01
    { 0xFF, 0x11 },	//0x02
    { 0xFF, 0x7F },	//0x03
    { 0xFF, 0x3F },	//0x04
    { 0xFF, 0xFF },	//0x05
    { 0xFF, 0xCB },	//0x06
    { 0xFF, 0xFF },	//0x07
    { 0xFF, 0xFF },	//0x08
    { 0xFF, 0xFF },	//0x09
    { 0xFF, 0xFF },	//0x0A
    { 0xFF, 0x9F },	//0x0B
    { 0xFF, 0x1F },	//0x0C
    { 0xFF, 0x0F },	//0x0D
    { 0xFF, 0x21 },	//0x0E
    { 0xFF, 0xFF },	//0x0F
    { 0xFF, 0xFF },	//0x10
    { 0xFF, 0xFF },	//0x11
    { 0xFF, 0xFF },	//0x12
    { 0xFF, 0x01 },	//0x13
    { 0xFF, 0xFF },	//0x14
    { 0xFF, 0x1F },	//0x15
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
    { 0xFF, 0x50 },	//0x24
    { 0x00, 0x00 }, //0x25
    { 0xFF, 0x18 }, //0x26
    { 0x00, 0x00 }, //0x27
    { 0x00, 0x00 }, //0x28
    { 0x00, 0x00 }, //0x29
    { 0xFF, 0x05 }, //0x2A
};



static void dump_trace(void)
{
    if (g_dump_trace) {
        dump_stack();
    }
}

static int nTestRegNo = 0;
/*
 * Read ak4376 register cache
 */
static inline u32 ak4376_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
//    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4376_reg));
    return 0;
}
/*AKM sungang start*/
static const char *test_reg_select[]   =
{
    "read AK4376 Reg 00:24",
};
static const struct soc_enum ak4376_enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int get_test_reg(
struct snd_kcontrol       *kcontrol,

struct snd_ctl_elem_value  *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;
}

static int set_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
	int    i, value;
	int	   regs, rege;

	nTestRegNo = currMode;

	regs = 0x00;
	rege = 0x2a;

	for ( i = regs ; i <= rege ; i++ ){
		value = ak4376_read(codec, i);
		b_codec_dbg("***AK4376 Addr,Reg=(%x, %x)\n", i, value);
	}

	return 0;
}



#ifdef AK4376_DUMMY
static unsigned int ak4376_fake_read(struct snd_soc_codec *codec,  u_int reg)
{
    b_codec_dbg("%s (%d)\n",__FUNCTION__, __LINE__);
    return 0;
}

static int ak4376_fake_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
{
    b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);
    return 0;
}
#endif


/*
 * for normal, trust i2c operation, using cache
 * for bad, not using cache, write and read back is needed
 * */
#ifdef I2C_RW_CACHE_MODE
#else
static unsigned int ak4376_read(struct snd_soc_codec *codec,  u_int reg)
{
    int ret;

#ifdef AK4376_DUMMY
    return ak4376_fake_read(codec, reg);
#endif

    if (reg == AK4376_16_DUMMY) { // Dummy Register.
        ret = ak4376_read_reg_cache(codec, reg);
        return ret;
    }

    ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
    if (ret < 0) {
        b_codec_dbg("%s (%d)\n",__FUNCTION__, __LINE__);
    }

    return ret;
}

static int ak4376_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
{
    b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

#ifdef AK4376_DUMMY
    return ak4376_fake_write(codec, reg, value);
#endif

    if (reg == AK4376_16_DUMMY) {
        return 0;
    }

    if (i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
        b_codec_dbg("%s(%d) error\n",__FUNCTION__,__LINE__);
        return EIO;
    }

    return 0;
}

static int ak4376_write_mask(struct snd_soc_codec *codec, u32 reg, u32 mask, u32 value)
{
    u32 old;
    u32 new;
    int ret = 0;

    old = ak4376_read(codec, reg);
    new = (old & ~(mask)) | value;
    ret = ak4376_write(codec, reg, new);

    return ret;
}
#endif


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
}

/* AKM  sungang start*/
static int ak4376_set_pllblock(struct snd_soc_codec *codec, int fs)
{
	u8 mode;
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit;
	int PLLMCKI;
	mode = ak4376_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	mode &= ~AK4376_CM;

		if ( fs <= 24000 ) {
			mode |= AK4376_CM_1;
			nMClk = 512 * fs;
		}
		else if ( fs <= 96000 ) {
			mode |= AK4376_CM_0;
			nMClk = 256 * fs;
		}
		else if ( fs <= 192000 ) {
			mode |= AK4376_CM_3;
			nMClk = 128 * fs;
		}
		else {		//fs > 192kHz
			mode |= AK4376_CM_1;
			nMClk = 64 * fs;
		}

	snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);

	if ( (fs % 8000) == 0 ) {
		nPLLClk = 122880000;
	}
	else if ( (fs == 11025 ) && ( ak4376_data->nBickFreq == 1 ) && ( ak4376_data->nPllMode == 1 )) {
		nPLLClk = 101606400;
	}
	else {
		nPLLClk = 112896000;
	}

	if ( ak4376_data->nPllMode == 1 ) {		//BICK_PLL (Slave)
		if ( ak4376_data->nBickFreq == 0 ) {		//32fs
			if ( fs <= 96000 ) PLDbit = 1;
			else if ( fs <= 192000 ) PLDbit = 2;
			else PLDbit = 4;
			nRefClk = 32 * fs / PLDbit;
		}
		else if ( ak4376_data->nBickFreq == 1 ) {	//48fs
			if ( fs <= 16000 ) PLDbit = 1;
			else if ( fs <= 192000 ) PLDbit = 3;
			else PLDbit = 6;
			nRefClk = 48 * fs / PLDbit;
		}
		else {  									// 64fs
			if ( fs <= 48000 ) PLDbit = 1;
			else if ( fs <= 96000 ) PLDbit = 2;
			else if ( fs <= 192000 ) PLDbit = 4;
			else PLDbit = 8;
			nRefClk = 64 * fs / PLDbit;
		}
	}
		else {		//MCKI_PLL (Master)
			if ( ak4376_data->nPllMCKI == 0 ) { //9.6MHz
					PLLMCKI = 9600000;
					if ( (fs % 4000) == 0) nRefClk = 1920000;
					else nRefClk = 384000;
				}
				else if ( ak4376_data->nPllMCKI == 1 ) { //11.2896MHz
					PLLMCKI = 11289600;
					if ( (fs % 4000) == 0) return -EINVAL;
					else nRefClk = 2822400;
				}
				else if ( ak4376_data->nPllMCKI == 2 ) { //12.288MHz
					PLLMCKI = 12288000;
					if ( (fs % 4000) == 0) nRefClk = 3072000;
					else nRefClk = 768000;
				}
				else {								//19.2MHz
					PLLMCKI = 19200000;
					if ( (fs % 4000) == 0) nRefClk = 1920000;
					else nRefClk = 384000;
				}
				PLDbit = PLLMCKI / nRefClk;
			}
	PLMbit = nPLLClk / nRefClk;
	MDIVbit = nPLLClk / nMClk;

	PLDbit--;
	PLMbit--;
	MDIVbit--;

	//PLD15-0
	snd_soc_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	//PLM15-0
	snd_soc_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));

	if (ak4376_data->nPllMode == 1 ) {	//BICK_PLL (Slave)
		snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x01);	//PLS=1(BICK)
	}
	else {										//MCKI PLL (Slave/Master)
		snd_soc_update_bits(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x00);	//PLS=0(MCKI)
	}

	//MDIV7-0
	snd_soc_write(codec, AK4376_14_DAC_CLK_DIVIDER, MDIVbit);

	return 0;
}

static int ak4376_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8 mode;
	u8 mode2;
	int mcki_rate;

	//akdbgprt("\t[AK4376] %s fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

	if ((fs != 0)&&(rclk != 0)) {
		if (rclk > 28800000) return -EINVAL;

		if (ak4376_data->nPllMode == 0) {	//PLL_OFF
			mcki_rate = rclk/fs;
		}
		else {		//XTAL_MODE
			if ( ak4376_data->xtalfreq == 0 ) {		//12.288MHz
				mcki_rate = 12288000/fs;
			}
			else {	//11.2896MHz
				mcki_rate = 11289600/fs;
			}
		}

		mode = ak4376_read(codec, AK4376_05_CLOCK_MODE_SELECT);
		mode &= ~AK4376_CM;

		if (ak4376_data->lpmode == 0) {				//High Performance Mode
			switch (mcki_rate) {
			case 32:
				mode |= AK4376_CM_0;
				break;
			case 64:
				mode |= AK4376_CM_1;
				break;
			case 128:
				mode |= AK4376_CM_3;
				break;
			case 256:
				mode |= AK4376_CM_0;
				mode2 = ak4376_read(codec, AK4376_24_MODE_CONTROL);
				if ( fs <= 12000 ) {
					mode2 |= 0x40;	//DSMLP=1
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
		else {					//Low Power Mode (LPMODE == DSMLP == 1)
			switch (mcki_rate) {
			case 32:
				mode |= AK4376_CM_0;
				break;
			case 64:
				mode |= AK4376_CM_1;
				break;
			case 128:
				mode |= AK4376_CM_3;
				break;
			case 256:
				mode |= AK4376_CM_0;
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
		snd_soc_write(codec, AK4376_05_CLOCK_MODE_SELECT, mode);
	}

	return 0;
}

int ak4376_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
	u8 fs;

	fs = ak4376_read(codec, AK4376_05_CLOCK_MODE_SELECT);
	fs &= ~AK4376_FS;

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
	case 352800:
		fs |= AK4376_FS_352_8KHZ;
		break;
	case 384000:
		fs |= AK4376_FS_384KHZ;
		break;
	default:
		return -EINVAL;
	}
	ak4376_write(codec, AK4376_05_CLOCK_MODE_SELECT, fs);

	if ( ak4376_data->nPllMode == 0 ) {		//PLL Off
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x00);	//DACCKS=0
		ak4376_set_mcki(codec, nfs1, ak4376_data->rclk);
	}
	else if ( ak4376_data->nPllMode == 3 ) {	//XTAL MODE
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x02);	//DACCKS=2
		ak4376_set_mcki(codec, nfs1, ak4376_data->rclk);
	}
	else {											//PLL mode
		snd_soc_update_bits(codec, AK4376_13_DAC_CLK_SOURCE, 0x03, 0x01);	//DACCKS=1
		ak4376_set_pllblock(codec, nfs1);
	}

//	ak4376_set_timer(codec);

	return 0;
}
/* AKM  sungang end*/


// FAE config 0805
void ak4376_bclk_or_mclk_mode(struct snd_soc_codec *codec)
{
	b_codec_dbg("\n\n>>> BLCK MODE INIT <<<\n\n");

    /*
     *  AKM FAE suggest move power related regs into on/off
     *  but for 03 set 1/2 vdd time no need
     * */

    // LR amp power management
    // 1/2 VDD settting capless right?  QQQQQQQQQQ
    // not open LR amp
	ak4376_write(codec, AK4376_03_POWER_MANAGEMENT4, 0x50);

    // VDD hold setting QQQ
	ak4376_write(codec, AK4376_04_OUTPUT_MODE_SETTING, 0x14);
       /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	/*I dont know why,sleep more than 30min ,0x4 and 0x7 cannot write */
       mdelay(2);
       /*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/

    //256fs, 48KHZ
    /* AKM  sungang start*/
 //   ak4376_write(codec, AK4376_05_CLOCK_MODE_SELECT, 0x0a);
   /* AKM  sungang end*/

    //Sharp Roll-Off Filter
	ak4376_write(codec, AK4376_06_DIGITAL_FILTER_SELECT, 0x00);

    //LR ch select
	ak4376_write(codec, AK4376_07_DAC_MONO_MIXING, 0x21);
	/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	/*I dont know why,sleep more than 30min ,0x4 and 0x7 cannot write */
       mdelay(2);
       /*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/

    // AK4376_0B_LCH_OUTPUT_VOLUME = 0x19
    // AK4376_0C_RCH_OUTPUT_VOLUME = 0x19
	ak4376_write(codec, AK4376_0B_LCH_OUTPUT_VOLUME, 0x11);
	ak4376_write(codec, AK4376_0C_RCH_OUTPUT_VOLUME, 0x11);

    // 0x0d = 0x75 0db  zero cross time
	ak4376_write(codec, AK4376_0D_HP_VOLUME_CONTROL, 0x0b);

    // 0x0e == 0x00 SCLK
	//ak4376_write(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x01);

    // PLL
	ak4376_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, 0x00);
	ak4376_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, 0x00);
	ak4376_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, 0x00);
	ak4376_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, 0x4f);


    //BRUCE FIXME ak4376 diff with ak4375 
	ak4376_write(codec, AK4376_13_DAC_CLK_SOURCE, 0x01);

	ak4376_write(codec, AK4376_14_DAC_CLK_DIVIDER, 0x09);

    // I2S 16bit
 //   ak4376_write(codec, AK4376_15_AUDIO_IF_FORMAT, 0xe1);

    // no desample
	ak4376_write(codec, AK4376_24_MODE_CONTROL, 0x00);
	ak4376_write(codec, AK4376_26_MODE_CONTROL, 0x20);
	ak4376_write(codec, AK4376_2A_MODE_CONTROL, 0x07);
}



static int g_codec_hp_state = 0;
enum hp_ctl_enum {
    HP_OFF = 0,
    HP_ON,
};

int akm4376_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	b_codec_dbg("#B get %s  hp state %d\n", __func__, g_codec_hp_state);
	ucontrol->value.integer.value[0] = g_codec_hp_state;
	dump_trace();
	return 0;
}

/*
 * keep it simple and quick
 * check each mdelay and udelay later
 * follow spec P43
 * */
int akm4376_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4376_codec;
    int state = ucontrol->value.enumerated.item[0];

    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    dump_trace();

    if (state == HP_ON)
    {
/*[Defect]-Modified-BEGIN by TCT-NB.Tianhongwei, 1527674 , 2016/03/31, workaround for pop noise*/
       b_codec_dbg("hw_params_status = %d \n",hw_params_status);
	if(hw_params_status)
	{
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);
	 /*MODIFIED-BEGIN by hongwei.tian, 2016-03-31,BUG-1527674*/
	}
/*[Defect]-Modified-END by TCT-NB.Tianhongwei, 1527674 , 2016/03/31*/
 /*MODIFIED-END by hongwei.tian,BUG-1527674*/
    }
    else
    {
        //close hp amp
        ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x00);
    }

    g_codec_hp_state = state;
    hw_params_status = 0;/*[Defect]-Modified-BEGIN by TCT-NB.Tianhongwei, 1527674 , 2016/03/31, workaround for pop noise*/

    return 0;
}

/*
 *  for ftm loopback test
 * */
static int g_codec_ftm_hp_state = 0;

enum ftm_hp_ctl_enum {
    FTM_HP_OFF = 0,
    FTM_HP_LCH,
    FTM_HP_RCH,
};

int akm4376_ftm_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  hp state %d\n", __func__, g_codec_ftm_hp_state);
    ucontrol->value.integer.value[0] = g_codec_ftm_hp_state;
    dump_trace();
    return 0;
}

int akm4376_ftm_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4376_codec;
    int state = ucontrol->value.enumerated.item[0];

    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    dump_trace();

    if (state == FTM_HP_OFF)
    {
        //close hp amp
        ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x00);
    }
    else
    {
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        if (state == FTM_HP_LCH)
        {
            ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x01);
        }
        else
        {
            ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x02);
        }

        //spec need 25.9ms@44K1
    }

    g_codec_ftm_hp_state = state;

    return 0;
}
#ifdef CONFIG_SND_44P1K_SUPPORT
void ak4376_ftm_Mclk_enable(unsigned char enable)
{
 	int rc;
	if(enable)
	{
   		rc = clk_prepare_enable(ref_clk);
		if(rc){
			printk(KERN_ERR "%s: prepare ref_clk failed ret:%d\n", __func__, rc);
		}		
	}
	else
	{
		clk_disable_unprepare(ref_clk);
	}
		
}
EXPORT_SYMBOL_GPL(ak4376_ftm_Mclk_enable);
#endif
/*
 * Bruce add for one control on/off
 * headphone on/off
 * */
static const char *hp_ctl_txt[] = {"off", "on"};

static const struct soc_enum hp_control_enum[] = {
    SOC_ENUM_SINGLE_EXT(2, hp_ctl_txt),
};


static const char *ftm_hp_ctl_txt[] = {"off", "lch", "rch"};
static const struct soc_enum ftm_hp_control_enum[] = {
    SOC_ENUM_SINGLE_EXT(3, ftm_hp_ctl_txt),
};

static const struct snd_kcontrol_new ak4376_snd_controls[] = {
    SOC_ENUM_EXT("AKM HP", hp_control_enum[0], akm4376_get_hp, akm4376_set_hp),
    SOC_ENUM_EXT("AKM FTM HP", ftm_hp_control_enum[0], akm4376_ftm_get_hp, akm4376_ftm_set_hp),
    SOC_ENUM_EXT("Reg Read", ak4376_enum[0], get_test_reg, set_test_reg), //AKM SUNGANG
};

/* ak4376 dapm widgets */
static const struct snd_soc_dapm_widget ak4376_dapm_widgets[] = {
};

static const struct snd_soc_dapm_route ak4376_intercon[] = {
};

#ifdef CONFIG_SND_44P1K_SUPPORT
static int ak4376_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
	 /* AKM  sungang start*/
	 struct snd_soc_codec *codec = dai->codec;
    switch(params_format(params))
    {
		case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x0B, 0x09);	//DL1-0=01(16bit, >=32fs)
		ak4376_data->nBickFreq = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x0B, 0x00);	//DL1-0=1x(32bit, >=64fs)
		ak4376_data->nBickFreq = 1;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x0B, 0x02);	//DL1-0=1x(32bit, >=64fs)
		ak4376_data->nBickFreq = 2;
		break;
    }

	ak4376_data->fs1 = params_rate(params);
	 /*MODIFIED-BEGIN by hongwei.tian, 2016-03-31,BUG-1527674*/
	printk("%s:fs1 = %d \n", __func__,ak4376_data->fs1);
	ak4376_hw_params_set(codec, ak4376_data->fs1);

	if(ak4376_data->nPllMode == 2){
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x10);	//MS bit = 0
	}
	 /* AKM  sungang end*/
/*[Defect]-Modified-BEGIN by TCT-NB.Tianhongwei, 1527674 , 2016/03/31, workaround for pop noise*/
	 b_codec_dbg("g_codec_hp_state = %d \n",g_codec_hp_state);
	 if(g_codec_hp_state)
	 {
	        // 0x00 == 0x00 PLL start for blck pmosc stop
	        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
	        mdelay(7);                                                          //spec need 6.5
	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
	        mdelay(1);															//wait 1ms

	        //pwr up dac
	        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
	        mdelay(5);															//spec need 4.5ms

	        //open hp amp
	        ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);
	 }
	hw_params_status = 1;
/*[Defect]-Modified-END by TCT-NB.Tianhongwei, 1527674 , 2016/03/31*/
 /*MODIFIED-END by hongwei.tian,BUG-1527674*/
    dump_trace();
    return 0;
}
#else
static int ak4376_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
	 /* AKM  sungang start*/
	 struct snd_soc_codec *codec = dai->codec;
    switch(params_format(params))
    {
		case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
		ak4376_data->nBickFreq = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x03, 0x02);	//DL1-0=1x(32bit, >=64fs)
		ak4376_data->nBickFreq = 2;
		break;
    }

	ak4376_data->fs1 = params_rate(params);

	ak4376_hw_params_set(codec, ak4376_data->fs1);

	if(ak4376_data->nPllMode == 0){
		snd_soc_update_bits(codec, AK4376_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
	}
	 /* AKM  sungang end*/
/*[Defect]-Modified-BEGIN by TCT-NB.Tianhongwei, 1527674 , 2016/03/31, workaround for pop noise*/
	 b_codec_dbg("g_codec_hp_state = %d \n",g_codec_hp_state);
	 if(g_codec_hp_state)
	 {
	        // 0x00 == 0x00 PLL start for blck pmosc stop
	        ak4376_write_mask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
	        mdelay(7);                                                          //spec need 6.5
	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
	        mdelay(1);															//wait 1ms

	        //pwr up dac
	        ak4376_write_mask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

	        ak4376_write_mask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
	        mdelay(5);															//spec need 4.5ms

	        //open hp amp
	        ak4376_write_mask(codec, AK4376_03_POWER_MANAGEMENT4, 0x53, 0x53);

	 }
	hw_params_status = 1;
/*[Defect]-Modified-END by TCT-NB.Tianhongwei, 1527674 , 2016/03/31*/
    dump_trace();
    return 0;
}

#endif
static int ak4376_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
        unsigned int freq, int dir)
{
    b_codec_dbg("\t[AK4376] %s(%d)\n",__FUNCTION__,__LINE__);
    dump_trace();
    return 0;
}

static int ak4376_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    dump_trace();
    return 0;
}

static int ak4376_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    return true;
}

static int ak4376_readable(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    if (reg >= ARRAY_SIZE(ak4376_access_masks))
        return 0;
    return ak4376_access_masks[reg].readable != 0;
}

/*
 * platform data
 * */
static int ak4376_parse_dt(struct device *dev, struct ak4376_sys_data_s *sys_data)
{
    struct device_node *np = dev->of_node;
    sys_data->dac_ctl_pin = of_get_named_gpio_flags(np, "akm,dac-gpio", 0, &(sys_data->dac_ctl_pin_flags));
    if (sys_data->dac_ctl_pin < 0) {
        return sys_data->dac_ctl_pin;
    }

    sys_data->ldo_ctl_pin = of_get_named_gpio_flags(np, "akm,ldo-gpio", 0, &(sys_data->ldo_ctl_pin_flags));
    if (sys_data->ldo_ctl_pin < 0) {
        return sys_data->ldo_ctl_pin;
    }

    return 0;
}


static int ak4376_pinctrl_init(struct ak4376_sys_data_s *sys_data)
{
    int ret;

    sys_data->pinctrl = devm_pinctrl_get(&(sys_data->client->dev));
    if (IS_ERR_OR_NULL(sys_data->pinctrl)) {
        ret = PTR_ERR(sys_data->pinctrl);
        b_codec_trace();
        goto err_pinctrl_get;
    }

    sys_data->pinctrl_state_active = pinctrl_lookup_state(sys_data->pinctrl, "ak4376_dac_active");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_active)) {
        ret = PTR_ERR(sys_data->pinctrl_state_active);
        b_codec_trace();
        goto err_pinctrl_lookup;
    }

    sys_data->pinctrl_state_suspend = pinctrl_lookup_state(sys_data->pinctrl, "ak4376_dac_suspend");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_suspend)) {
        ret = PTR_ERR(sys_data->pinctrl_state_suspend);
        b_codec_trace();
        goto err_pinctrl_lookup;
    }

    ret = pinctrl_select_state(sys_data->pinctrl, sys_data->pinctrl_state_active);
    if (ret) {
        b_codec_trace();
        return ret;
    }
    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(sys_data->pinctrl);

err_pinctrl_get:
    sys_data->pinctrl = NULL;
    return ret;
}


/*
 * default ouput 0
 * follow akm startup timeline
 * */
static int ak4376_config_pins(struct ak4376_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->dac_ctl_pin, "akm_dac_ctl") < 0) {
        b_codec_dbg("gpio err  %d\n", sys_data->dac_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->dac_ctl_pin, 0);


    if (gpio_request(sys_data->ldo_ctl_pin, "akm_ldo_ctl") < 0) {
        b_codec_dbg("gpio err %d\n", sys_data->ldo_ctl_pin);
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
        b_codec_dbg("%s %d get vdd error\n", __func__, __LINE__);
        goto err_get_vdd;
    }

    if (regulator_count_voltages(sys_data->vdd) > 0)
    {
        rc = regulator_set_voltage(sys_data->vdd, 1800000, 1800000);
        if (rc) {
            b_codec_dbg("%s %d set vdd error\n", __func__, __LINE__);
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




/*
 * Bruce FIXME QAD
 * */
#include "ak4376_debug.c"

static int ak4376_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
    b_codec_dbg("\t[AK4376] %s(%d) cmd=0x%x\n", __FUNCTION__, __LINE__, cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            b_codec_dbg("start resume pause\n");
            break;

        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            b_codec_dbg("stop suspend push\n");
            break;
    }

    dump_trace();
    return 0;
}

static int ak4376_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
    dump_trace();
    return 0;
}

#define AK4376_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
        SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
        SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
        SNDRV_PCM_RATE_192000)

#define AK4376_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4376_dai_ops = {
    .hw_params	= ak4376_hw_params,
    .set_sysclk	= ak4376_set_dai_sysclk,
    .set_fmt	= ak4376_set_dai_fmt,
    .trigger = ak4376_trigger,
    .digital_mute = ak4376_set_dai_mute,
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


/*
 * check pre-init with spec and FAE
 * dtsi keep releated pin output 0 no bias
 * */
static int ak4376_pre_init(struct snd_soc_codec *codec)
{
    ak4376_dac_on(0);
    /*[BUG]-Add-START   by TCTSH.YK, 967577, 2015/11/23, from FAE must pull down pwd frist */
    ak4376_ldo_power_on(0);//pull down PDN
    /*[BUG]-Add-END   by TCTSH.YK, 967577, 2015/11/23, from FAE must pull down pwd frist */
    mdelay(1);

    // power on AVDD LVDD
    ak4376_ldo_power_on(1);

    mdelay(10);

    // PDN pin to high
    ak4376_dac_on(1);

   ak4376_bclk_or_mclk_mode(codec);

    return 0;
}

/*[FEATURE]-Add-BEGIN by kun.guan, 526254, 2015/10/28, add a debug function for headset*/
void ak4376_reinit(void)
{
    struct snd_soc_codec *codec = ak4376_codec;
    ak4376_pre_init(codec);
}
/*[FEATURE]-Add-END   by kun.guan, 526254, 2015/10/28, add a debug function for headset*/

static int ak4376_probe(struct snd_soc_codec *codec)
{
    struct ak4376_priv *ak4376 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;

    b_codec_dbg("%s(%d)\n", __FUNCTION__, __LINE__);

    ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
        return ret;
    }

    /*
     * just for trace asoc GDS
     * */
#ifdef AK4376_DUMMY
    codec->write = ak4376_fake_write;
    codec->read = ak4376_fake_read;
#else
    codec->write = ak4376_write;
    codec->read = ak4376_read;

#endif

    codec->control_data = ak4376_sys_data->client;

    ak4376->fs1 = 48000;
    ak4376->fs2 = 48000;
    ak4376->rclk = 0;
    ak4376->nSeldain = 0;		//0:Bypass, 1:SRC	(Dependent on a register bit)
    ak4376->nBickFreq = 0;		//0:32fs, 1:48fs, 2:64fs
    ak4376->nSrcOutFsSel = 0;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
#ifdef PLL_BICK_MODE
    ak4376->nPllMode = 1;		//1: PLL ON
#else
    ak4376->nPllMode = 2;		//0:PLL OFF; 1:PLL ON SLAVE 2:PLL ON MASTER
    ak4376->nPllMCKI = 0;          //0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
#endif
    ak4376->nSmt = 0;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
    ak4376->dfsrc8fs = 0;		//0:DAC Filter, 1:Bypass, 2:8fs mode


    b_codec_dbg("\n\n\n DUMP AK4376 CONFIG \n\n\n");
    b_codec_dbg("ak4376->fs1 = %d\n", ak4376->fs1);
    b_codec_dbg("ak4376->fs2 = %d\n", ak4376->fs2);
    b_codec_dbg("ak4376->rclk = %d\n", ak4376->rclk);
    b_codec_dbg("ak4376->nSeldain = %d\n", ak4376->nSeldain);
    b_codec_dbg("ak4376->nBickFreq = %d\n", ak4376->nBickFreq);
    b_codec_dbg("ak4376->nSrcOutFsSel = %d\n", ak4376->nSrcOutFsSel);
    b_codec_dbg("ak4376->nPllMode = %d\n", ak4376->nPllMode);
    b_codec_dbg("ak4376->nSmt = %d\n", ak4376->nSmt);
    b_codec_dbg("ak4376->dfsrc8fs = %d\n", ak4376->dfsrc8fs);

    ak4376_pre_init(ak4376_codec);
	printk("%s success!!!",__func__);
    return ret;
}

/*[BUG]-Add-START   by TCTSH.YK,967577, 2015/11/23, solve headset no sound after deep-sleep*/
static void ak4376_suspend_power_pwn(void)
{
/*From AKM FAE,we know that if you want to close power ,must pull down PWD-PIN first
   then control  TVDD,AVDD-LVDD  */
         int rc=0;

	/*pull down PDN first */
	 ak4376_dac_on(0);

	 msleep(2);

	/*close AVDD-LVDD power */
	 ak4376_ldo_power_on(0);

	/*close TVDD power*/
	rc = regulator_disable(ak4376_sys_data->vdd);
         if (rc) {
            dev_err(&ak4376_sys_data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
        }

}
static void ak4376_resume_power_pwn(void)
{
/*From AKM FAE,we know that if you want to resume AKM ,must  open the power include <TVDD,AVDD-LVDD>.
   then pull up PWD-PIN first */
   int rc=0;

 	 ak4376_dac_on(0);
  /*open  AVDD-LVDD power*/
    ak4376_ldo_power_on(1);

  /*open TVDD power*/
    rc = regulator_enable(ak4376_sys_data->vdd);
    if (rc) {
            dev_err(&ak4376_sys_data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
       }
    msleep(2);
   /*pull up PWD first */
     ak4376_dac_on(1);

}
/*[BUG]-Add-END  by TCTSH.YK,967577, 2015/11/23, solve headset no sound after deep-sleep*/

static int ak4376_suspend(struct snd_soc_codec *codec)
{

    b_codec_dbg("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {

	/*[BUG]-Add-START   by TCTSH.YK, 967577,2015/11/23, solve headset no sound after deep-sleep*/
	ak4376_suspend_power_pwn();
	/*[BUG]-Add-END   by TCTSH.YK,967577, 2015/11/23, solve headset no sound after deep-sleep*/
    }
    else
    {
        // keep everything on
    }

    return 0;
}

static int ak4376_resume(struct snd_soc_codec *codec)
{
    b_codec_dbg("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {
	/*[BUG]-Add-START   by TCTSH.YK,967577, 2015/11/23, solve headset no sound after deep-sleep*/
	ak4376_resume_power_pwn();
	/*load REG again*/
	ak4376_bclk_or_mclk_mode(ak4376_codec);
	/*[BUG]-Add-END   by TCTSH.YK,967577, 2015/11/23, solve headset no sound after deep-sleep*/
    }
    else
    {
        //everything is on
    }

    return 0;
}

static int ak4376_remove(struct snd_soc_codec *codec)
{
    b_codec_dbg("%s (%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4376 = {
    .probe = ak4376_probe,
    .remove = ak4376_remove,

    .suspend =	ak4376_suspend,
    .resume =	ak4376_resume,


    .reg_cache_size = ARRAY_SIZE(ak4376_reg),
    .reg_word_size = sizeof(u8),
    .reg_cache_default = ak4376_reg,
    .readable_register = ak4376_readable,
    .volatile_register = ak4376_volatile,

    .controls = ak4376_snd_controls,
    .num_controls = ARRAY_SIZE(ak4376_snd_controls),
    .dapm_widgets = ak4376_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(ak4376_dapm_widgets),
    .dapm_routes = ak4376_intercon,
    .num_dapm_routes = ARRAY_SIZE(ak4376_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4376);



static int ak4376_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int rc = 0;
    struct snd_soc_codec *codec;
	u8 device_id = 0;/*[FEATURE]-Add by TCTSH.Cedar, 988925, 2015/11/27, compatible with ak4375 and ak4376*/

	/*[Task]-Modified-BEGIN by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
	/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	/*i am sure that ak4375 have probed ,it is better to ignore ak4376 init, */
	#if 0
    if(ak4375_en){
	printk("%s---ak4375 have probe ,so ignore ak4376 probe =flag=%d...\n",__func__,ak4375_en);
	return -1 ;
    }
	#endif
    /*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	/*[Task]-Modified-END   by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/

    b_codec_dbg("ak4376_snd_controls = %d\n", (int)ARRAY_SIZE(ak4376_snd_controls));

    b_codec_trace();

    ak4376_data = kzalloc(sizeof(struct ak4376_priv), GFP_KERNEL);
    if (ak4376_data == NULL) {
        return -ENOMEM;
    }


    b_codec_trace();

    ak4376_sys_data = kzalloc(sizeof(struct ak4376_sys_data_s), GFP_KERNEL);
    if (ak4376_sys_data == NULL) {
        kfree(ak4376_data);
        return -ENOMEM;
    }

    b_codec_trace();


    /*
     * init structure first!!!
     * */
    codec = &ak4376_data->codec;
    ak4376_sys_data->client = i2c;
    codec->control_data = i2c;
    codec->dev = &i2c->dev;
    ak4376_codec = codec;

    b_codec_trace();

    /*
     * get resource from OF!
     * */
    rc = ak4376_parse_dt(&(i2c->dev), ak4376_sys_data);
    if (rc) {
        b_codec_err("ak4376_parse_dt error %d\n", rc);
    }

    b_codec_trace();


    rc = ak4376_regulator_init(ak4376_sys_data);
    if (rc < 0) {
        b_codec_err("get regulator error %d\n", rc);
    }

    b_codec_trace();

    rc = ak4376_pinctrl_init(ak4376_sys_data);
    if (rc < 0) {
        b_codec_err("get pinctl init error %d\n", rc);
    }

    b_codec_trace();


    /*
     *
     * */
    ak4376_config_pins(ak4376_sys_data);
	/*[FEATURE]-Add-BEGIN by TCTSH.Cedar, 988925, 2015/11/27, compatible with ak4375 and ak4376*/
    ak4376_dac_on(0);
    mdelay(1);
    // power on AVDD LVDD
    ak4376_ldo_power_on(1);
    mdelay(3);
    // PDN pin to high
    ak4376_dac_on(1);
    mdelay(3);

    b_codec_trace();

	device_id = i2c_smbus_read_byte_data(ak4376_sys_data->client, (u8)(AK4376_15_AUDIO_IF_FORMAT & 0xFF));
	/*
	BIT7~BIT5
	000:AK4375
	001:AK4375A
	010:AK4376
	*/
	device_id = (device_id & 0xE0)>>5;
	printk("%s:device_id = %#x\n", __func__, device_id);
#if 0
	if(device_id != 2)/*DEVICEID_AK4376 = 010*/
	{
	    /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	    gpio_free(ak4376_sys_data->dac_ctl_pin);
	    gpio_free(ak4376_sys_data->ldo_ctl_pin);
	    /*[BUG]-Add-END   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
        kfree(ak4376_data);
        kfree(ak4376_sys_data);
		return -1;
	}
	/*[FEATURE]-Add-END  by TCTSH.Cedar, 988925, 2015/11/27, compatible with ak4375 and ak4376*/
#endif

    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));
    dev_set_name(&i2c->dev, "%s", "akm-ak4376");
    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));

    /*
     * register sound
     * */
    snd_soc_codec_set_drvdata(codec, ak4376_data);

    rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ak4376, &ak4376_dai[0], ARRAY_SIZE(ak4376_dai));
    if (rc < 0){
        kfree(ak4376_data);
        kfree(ak4376_sys_data);
        b_codec_err("snd_soc_register_codec eror %d\n", rc);
    }

    b_codec_trace();
#ifdef CONFIG_SND_44P1K_SUPPORT
    ref_clk = clk_get(&i2c->dev, "ref_clk");
	if (IS_ERR(ref_clk)) {
		printk(KERN_ERR "%s:ak4376 Error getting ref_clk\n", __func__);
		ref_clk = NULL;
	}
	else{
		clk_set_rate(ref_clk, AK4376_CLK_FREQ);
		printk("%s: set_rate bb_clk2\n", __func__);
	}
#endif
    /*
     * register debug interface for cmdline mode
     * */
    rc = device_create_file(&(i2c->dev), &dev_attr_ak4376_control);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    rc = device_create_file(&(i2c->dev), &dev_attr_ak4376_dac);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    rc = device_create_file(&(i2c->dev), &dev_attr_ak4376_ldo);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    b_codec_trace();

    i2c_ok = 1;/*TCT-NB Tianhongwei add for npi info*/
	printk("%s success!!!",__func__);
    return rc;
}

static int ak4376_i2c_remove(struct i2c_client *client)
{
    b_codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

static const struct i2c_device_id ak4376_i2c_id[] = {
    { "ak4376", 0 },
    { }
};

static struct of_device_id ak4376_match_table[] = {
    { .compatible = "akm,ak4376", },
    { },
};

MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
    .driver = {
        .name = "akm-ak4376",
        .owner = THIS_MODULE,
        .of_match_table = ak4376_match_table,
    },
    .probe = ak4376_i2c_probe,
    .remove = ak4376_i2c_remove,
    .id_table = ak4376_i2c_id,
};

module_i2c_driver(ak4376_i2c_driver);

MODULE_DESCRIPTION("IDOL3 ak4376 codec driver");
MODULE_LICENSE("GPL");
