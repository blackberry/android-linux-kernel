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
#include "ak4375.h"
#include <linux/clk.h>
/*
 * PLL means BLCk
 * NO PLL means mclk
 * */
#ifndef CONFIG_SND_44P1K_SUPPORT
#define PLL_BICK_MODE
#endif

/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
//bool  ak4375_en=false;/*[Task]-Modified by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
/*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/

//#define AK4375_DEBUG			//used at debug mode
#define AUDIO_NAME "#BC akm4375"

#ifdef AK4375_DEBUG
#define b_codec_dbg(format, arg...) \
    pr_debug(KERN_INFO AUDIO_NAME ": " format, ## arg)

#define b_codec_err(format, arg...) \
    pr_debug(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_info(format, arg...) \
    pr_debug(KERN_INFO AUDIO_NAME ": " format, ## arg)

#define b_codec_warn(format, arg...) \
    pr_debug(KERN_WARNING AUDIO_NAME ": " format, ## arg)

#define b_codec_trace() \
    pr_debug("%s(%d)\n", __func__, __LINE__)
#else
#define b_codec_dbg(format, arg...) do {} while (0)
#define b_codec_info(format, arg...) do {} while (0)
#define b_codec_warn(format, arg...) do {} while (0)
#define b_codec_trace(format, arg...) do {} while (0)
#define b_codec_err(format, arg...) \
    pr_debug(KERN_ERR AUDIO_NAME ": " format, ## arg)

#endif

/*[Task]-Modified-BEGIN by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
int g_dump_trace = 0;
static int i2c_ok = 0;/*[TASK]-Add by TCTSH.Cedar, 937744, 2015/12/17, add for RunInTest*/
module_param_named(i2c_ok, i2c_ok, int, 0644);
/*[Task]-Modified-END   by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/

#ifdef CONFIG_SND_44P1K_SUPPORT
static struct clk *ref_clk;
#define AK4375_CLK_FREQ			(9600000)
#endif
/* AK4375 Codec Private Data */
struct ak4375_priv {
    struct snd_soc_codec codec;
    u8 reg_cache[AK4375_MAX_REGISTERS];
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

struct ak4375_sys_data_s{
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

static struct ak4375_sys_data_s *ak4375_sys_data;
static struct snd_soc_codec *ak4375_codec;
static struct ak4375_priv *ak4375_data;
static unsigned int ak4375_read(struct snd_soc_codec *codec,  u_int reg);

/* ak4375 register cache & default register settings */
static const u8 ak4375_reg[AK4375_MAX_REGISTERS] = {
    0x00,	/*	0x00	AK4375_00_POWER_MANAGEMENT1			*/
    0x00,	/*	0x01	AK4375_01_POWER_MANAGEMENT2			*/
    0x00,	/*	0x02	AK4375_02_POWER_MANAGEMENT3			*/
    0x00,	/*	0x03	AK4375_03_POWER_MANAGEMENT4			*/
    0x00,	/*	0x04	AK4375_04_OUTPUT_MODE_SETTING		*/
    0x00,	/*	0x05	AK4375_05_CLOCK_MODE_SELECT			*/
    0x00,	/*	0x06	AK4375_06_DIGITAL_FILTER_SELECT		*/
    0x00,	/*	0x07	AK4375_07_DAC_MONO_MIXING			*/
    0x00,	/*	0x08	AK4375_08_JITTER_CLEANER_SETTING1	*/
    0x00,	/*	0x09	AK4375_09_JITTER_CLEANER_SETTING2	*/
    0x00,	/*	0x0A	AK4375_0A_JITTER_CLEANER_SETTING3	*/
    0x15,	/*	0x0B	AK4375_0B_LCH_OUTPUT_VOLUME			*/
    0x15,	/*	0x0C	AK4375_0C_RCH_OUTPUT_VOLUME			*/
    0x0A,	/*	0x0D	AK4375_0D_HP_VOLUME_CONTROL			*/
    0x00,	/*	0x0E	AK4375_0E_PLL_CLK_SOURCE_SELECT		*/
    0x00,	/*	0x0F	AK4375_0F_PLL_REF_CLK_DIVIDER1		*/
    0x00,	/*	0x10	AK4375_10_PLL_REF_CLK_DIVIDER2		*/
    0x00,	/*	0x11	AK4375_11_PLL_FB_CLK_DIVIDER1		*/
    0x00,	/*	0x12	AK4375_12_PLL_FB_CLK_DIVIDER2		*/
    0x00,	/*	0x13	AK4375_13_SRC_CLK_SOURCE			*/
    0x00,	/*	0x14	AK4375_14_DAC_CLK_DIVIDER			*/
    0x00,	/*	0x15	AK4375_15_AUDIO_IF_FORMAT			*/
    0x00,	/*	0x16	AK4375_16_DUMMY						*/
    0x00,	/*	0x17	AK4375_17_DUMMY						*/
    0x00,	/*	0x18	AK4375_18_DUMMY						*/
    0x00,	/*	0x19	AK4375_19_DUMMY						*/
    0x00,	/*	0x1A	AK4375_1A_DUMMY						*/
    0x00,	/*	0x1B	AK4375_1B_DUMMY						*/
    0x00,	/*	0x1C	AK4375_1C_DUMMY						*/
    0x00,	/*	0x1D	AK4375_1D_DUMMY						*/
    0x00,	/*	0x1E	AK4375_1E_DUMMY						*/
    0x00,	/*	0x1F	AK4375_1F_DUMMY						*/
    0x00,	/*	0x20	AK4375_20_DUMMY						*/
    0x00,	/*	0x21	AK4375_21_DUMMY						*/
    0x00,	/*	0x22	AK4375_22_DUMMY						*/
    0x00,	/*	0x23	AK4375_23_DUMMY						*/
    0x00,	/*	0x24	AK4375_24_MODE_CONTROL				*/
};

static const struct {
    int readable;   /* Mask of readable bits */
    int writable;   /* Mask of writable bits */
} ak4375_access_masks[] = {
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
    { 0xFF, 0x50 },	//0x24
};


static void dump_trace(void)
{
    if (g_dump_trace) {
        dump_stack();
    }
}

static int nTestRegNo = 0;
/*
 * Read ak4375 register cache
 */
static inline u32 ak4375_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    //u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4375_reg));
    return 0;
}

static const char *test_reg_select[]   =
{
    "read AK4375 Reg 00:24",
};
static const struct soc_enum ak4375_enum[] =
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
		value = ak4375_read(codec, i);
		b_codec_dbg("***AK4375 Addr,Reg=(%x, %x)\n", i, value);
	}

	return 0;
}



#ifdef AK4375_DUMMY
static unsigned int ak4375_fake_read(struct snd_soc_codec *codec,  u_int reg)
{
    b_codec_dbg("%s (%d)\n",__FUNCTION__, __LINE__);
    return 0;
}

static int ak4375_fake_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
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
static unsigned int ak4375_read(struct snd_soc_codec *codec,  u_int reg)
{
    int ret;

#ifdef AK4375_DUMMY
    return ak4375_fake_read(codec, reg);
#endif

    if (reg == AK4375_16_DUMMY) { // Dummy Register.
        ret = ak4375_read_reg_cache(codec, reg);
        return ret;
    }
    ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
    if (ret < 0) {
        b_codec_err("%s (%d)\n",__FUNCTION__, __LINE__);
    }

    return ret;
}

static int ak4375_write(struct snd_soc_codec *codec, unsigned reg, u_int value)
{
    b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

#ifdef AK4375_DUMMY
    return ak4375_fake_write(codec, reg, value);
#endif

    if (reg == AK4375_16_DUMMY) {
        return 0;
    }

    if (i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
        b_codec_err("%s(%d) error\n",__FUNCTION__,__LINE__);
        return EIO;
    }

    return 0;
}

static int ak4375_write_mask(struct snd_soc_codec *codec, u32 reg, u32 mask, u32 value)
{
    u32 old;
    u32 new;
    int ret = 0;

    old = ak4375_read(codec, reg);
    new = (old & ~(mask)) | value;
    ret = ak4375_write(codec, reg, new);

    return ret;
}
#endif


void ak4375_ldo_power_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4375_sys_data->ldo_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4375_sys_data->ldo_ctl_pin, 0);
    }
}


void ak4375_dac_on(int onoff)
{
    if (onoff)
    {
        gpio_direction_output(ak4375_sys_data->dac_ctl_pin, 1);
    }
    else
    {
        gpio_direction_output(ak4375_sys_data->dac_ctl_pin, 0);
    }
}

static int ak4375_set_pllblock(struct snd_soc_codec *codec, int fs)
{
	u8 mode;
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit;
	int PLLMCKI;
	mode = ak4375_read(codec, AK4375_05_CLOCK_MODE_SELECT);
	mode &= ~AK4375_CM;

		if ( fs <= 24000 ) {
			mode |= AK4375_CM_1;
			nMClk = 512 * fs;
		}
		else if ( fs <= 96000 ) {
			mode |= AK4375_CM_0;
			nMClk = 256 * fs;
		}
		else if ( fs <= 192000 ) {
			mode |= AK4375_CM_3;
			nMClk = 128 * fs;
		}
		else {		//fs > 192kHz
			mode |= AK4375_CM_1;
			nMClk = 64 * fs;
		}

	snd_soc_write(codec, AK4375_05_CLOCK_MODE_SELECT, mode);

	if ( (fs % 8000) == 0 ) {
		nPLLClk = 122880000;
	}
	else if ( (fs == 11025 ) && ( ak4375_data->nBickFreq == 1 ) && ( ak4375_data->nPllMode == 1 )) {
		nPLLClk = 101606400;
	}
	else {
		nPLLClk = 112896000;
	}

	if ( ak4375_data->nPllMode == 1 ) {		//BICK_PLL (Slave)
		if ( ak4375_data->nBickFreq == 0 ) {		//32fs
			if ( fs <= 96000 ) PLDbit = 1;
			else if ( fs <= 192000 ) PLDbit = 2;
			else PLDbit = 4;
			nRefClk = 32 * fs / PLDbit;
		}
		else if ( ak4375_data->nBickFreq == 1 ) {	//48fs
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
			if ( ak4375_data->nPllMCKI == 0 ) { //9.6MHz
					PLLMCKI = 9600000;
					if ( (fs % 4000) == 0) nRefClk = 1920000;
					else nRefClk = 384000;
				}
				else if ( ak4375_data->nPllMCKI == 1 ) { //11.2896MHz
					PLLMCKI = 11289600;
					if ( (fs % 4000) == 0) return -EINVAL;
					else nRefClk = 2822400;
				}
				else if ( ak4375_data->nPllMCKI == 2 ) { //12.288MHz
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
	snd_soc_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	//PLM15-0
	snd_soc_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));

	if (ak4375_data->nPllMode == 1 ) {	//BICK_PLL (Slave)
		snd_soc_update_bits(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x01);	//PLS=1(BICK)
	}
	else {										//MCKI PLL (Slave/Master)
		snd_soc_update_bits(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x03, 0x00);	//PLS=0(MCKI)
	}

	//MDIV7-0
	snd_soc_write(codec, AK4375_14_DAC_CLK_DIVIDER, MDIVbit);

	return 0;
}

static int ak4375_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8 mode;
	u8 mode2;
	int mcki_rate;

	//akdbgprt("\t[AK4375] %s fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

	if ((fs != 0)&&(rclk != 0)) {
		if (rclk > 28800000) return -EINVAL;

		if (ak4375_data->nPllMode == 0) {	//PLL_OFF
			mcki_rate = rclk/fs;
		}
		else {		//XTAL_MODE
			if ( ak4375_data->xtalfreq == 0 ) {		//12.288MHz
				mcki_rate = 12288000/fs;
			}
			else {	//11.2896MHz
				mcki_rate = 11289600/fs;
			}
		}

		mode = ak4375_read(codec, AK4375_05_CLOCK_MODE_SELECT);
		mode &= ~AK4375_CM;

		if (ak4375_data->lpmode == 0) {				//High Performance Mode
			switch (mcki_rate) {
			case 32:
				mode |= AK4375_CM_0;
				break;
			case 64:
				mode |= AK4375_CM_1;
				break;
			case 128:
				mode |= AK4375_CM_3;
				break;
			case 256:
				mode |= AK4375_CM_0;
				mode2 = ak4375_read(codec, AK4375_24_MODE_CONTROL);
				if ( fs <= 12000 ) {
					mode2 |= 0x40;	//DSMLP=1
					snd_soc_write(codec, AK4375_24_MODE_CONTROL, mode2);
				}
				else {
					mode2 &= ~0x40;	//DSMLP=0
					snd_soc_write(codec, AK4375_24_MODE_CONTROL, mode2);
				}
				break;
			case 512:
				mode |= AK4375_CM_1;
				break;
			case 1024:
				mode |= AK4375_CM_2;
				break;
			default:
				return -EINVAL;
			}
		}
		else {					//Low Power Mode (LPMODE == DSMLP == 1)
			switch (mcki_rate) {
			case 32:
				mode |= AK4375_CM_0;
				break;
			case 64:
				mode |= AK4375_CM_1;
				break;
			case 128:
				mode |= AK4375_CM_3;
				break;
			case 256:
				mode |= AK4375_CM_0;
				break;
			case 512:
				mode |= AK4375_CM_1;
				break;
			case 1024:
				mode |= AK4375_CM_2;
				break;
			default:
				return -EINVAL;
			}
		}
		snd_soc_write(codec, AK4375_05_CLOCK_MODE_SELECT, mode);
	}

	return 0;
}

int ak4375_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
	u8 fs;

	fs = ak4375_read(codec, AK4375_05_CLOCK_MODE_SELECT);
	fs &= ~AK4375_FS;

	switch (nfs1) {
	case 8000:
		fs |= AK4375_FS_8KHZ;
		break;
	case 11025:
		fs |= AK4375_FS_11_025KHZ;
		break;
	case 16000:
		fs |= AK4375_FS_16KHZ;
		break;
	case 22050:
		fs |= AK4375_FS_22_05KHZ;
		break;
	case 32000:
		fs |= AK4375_FS_32KHZ;
		break;
	case 44100:
		fs |= AK4375_FS_44_1KHZ;
		break;
	case 48000:
		fs |= AK4375_FS_48KHZ;
		break;
	case 88200:
		fs |= AK4375_FS_88_2KHZ;
		break;
	case 96000:
		fs |= AK4375_FS_96KHZ;
		break;
	case 176400:
		fs |= AK4375_FS_176_4KHZ;
		break;
	case 192000:
		fs |= AK4375_FS_192KHZ;
		break;
	case 352800:
		fs |= AK4375_FS_352_8KHZ;
		break;
	case 384000:
		fs |= AK4375_FS_384KHZ;
		break;
	default:
		return -EINVAL;
	}
	ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, fs);

	if ( ak4375_data->nPllMode == 0 ) {		//PLL Off
		snd_soc_update_bits(codec, AK4375_13_SRC_CLK_SOURCE, 0x03, 0x00);	//DACCKS=0
		ak4375_set_mcki(codec, nfs1, ak4375_data->rclk);
	}
	else if ( ak4375_data->nPllMode == 3 ) {	//XTAL MODE
		snd_soc_update_bits(codec, AK4375_13_SRC_CLK_SOURCE, 0x03, 0x02);	//DACCKS=2
		ak4375_set_mcki(codec, nfs1, ak4375_data->rclk);
	}
	else {											//PLL mode
		snd_soc_update_bits(codec, AK4375_13_SRC_CLK_SOURCE, 0x03, 0x01);	//DACCKS=1
		ak4375_set_pllblock(codec, nfs1);
	}

//	ak4375_set_timer(codec);

	return 0;
}

void ak4375_bclk_or_mclk_mode(struct snd_soc_codec *codec)
{
    b_codec_dbg("\n\n>>> BLCK MODE INIT <<<\n\n");

    /*
     *  AKM FAE suggest move power related regs iak4375_bclk_modento on/off
     *  but for 03 set 1/2 vdd time no need
     * */

    // LR amp power management
    // 1/2 VDD settting capless right?  QQQQQQQQQQ
    // not open LR amp
    /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
    /*from akm spec  reg-0x4 should be set 0x53  */
    ak4375_write(codec, AK4375_03_POWER_MANAGEMENT4, 0x50);
     /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/


    // VDD hold setting QQQ
    ak4375_write(codec, AK4375_04_OUTPUT_MODE_SETTING, 0x14);

       mdelay(2);

    //256fs, 48KHZ
    //ak4375_write(codec, AK4375_05_CLOCK_MODE_SELECT, 0x0a);

    //Sharp Roll-Off Filter
    ak4375_write(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x00);

    //LR ch select
	ak4375_write(codec, AK4375_07_DAC_MONO_MIXING, 0x21);
       mdelay(2);
    /*[BUG]-Add-START   by TCTSH.YK, 1623417/1570399 , 2016/02/19, The volume is small in Headset mode*/
    ak4375_write(codec, AK4375_0B_LCH_OUTPUT_VOLUME, 0x19);
    ak4375_write(codec, AK4375_0C_RCH_OUTPUT_VOLUME, 0x19);

    ak4375_write(codec, AK4375_0D_HP_VOLUME_CONTROL, 0x73);
  /*[BUG]-Add-START   by TCTSH.YK, 1623417/1570399 , 2015/12/19, The volume is small in Headset mode*/

    // 0x0e == 0x00 SCLK
    //ak4375_write(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x01);

    // PLL
    ak4375_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, 0x00);
    ak4375_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, 0x00);
    ak4375_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, 0x4f);
    ak4375_write(codec, AK4375_13_SRC_CLK_SOURCE, 0x01);
    ak4375_write(codec, AK4375_14_DAC_CLK_DIVIDER, 0x09);

    // I2S 16bit
    //ak4375_write(codec, AK4375_15_AUDIO_IF_FORMAT, 0xe1);

    // no desample
    ak4375_write(codec, AK4375_24_MODE_CONTROL, 0x00);
    ak4375_write(codec, AK4375_26_MODE_CONTROL, 0x18);
    ak4375_write(codec, AK4375_2A_MODE_CONTROL, 0x05);
}

static int g_codec_hp_state = 0;
enum hp_ctl_enum {
    HP_OFF = 0,
    HP_ON,
};

int akm4375_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
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
int akm4375_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4375_codec;
    int state = ucontrol->value.enumerated.item[0];

    b_codec_dbg("\n>>> %s  set  %d <<<\n", __func__, state);

    dump_trace();

    if (state == HP_ON)
    {
/*[Defect]-Modified-BEGIN by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/
#if 0
		//to avoid pop noise, we called the ak power and HP Pa settings after set the codec parameters

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

	  /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
        /*from akm spec  reg-0x4 should be set 0x53  */
        //open hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x53, 0x53);
        /*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/

        //spec need 25.9ms@44K1
#endif
/*[Defect]-Modified-END   by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/
    }
    else
    {
        //close hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00);

    }

    g_codec_hp_state = state;

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

int akm4375_ftm_get_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  hp state %d\n", __func__, g_codec_ftm_hp_state);
    ucontrol->value.integer.value[0] = g_codec_ftm_hp_state;
    dump_trace();
    return 0;
}

int akm4375_ftm_set_hp(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = ak4375_codec;
    int state = ucontrol->value.enumerated.item[0];

    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    dump_trace();

    if (state == FTM_HP_OFF)
    {
        //close hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x00);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0

        //pwr down dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x00);   //PMDA=0

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0

        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00);
    }
    else
    {
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        if (state == FTM_HP_LCH)
        {
            ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x01);
        }
        else
        {
            ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x03, 0x02);
        }

        //spec need 25.9ms@44K1
    }

    g_codec_ftm_hp_state = state;

    return 0;
}
#ifdef CONFIG_SND_44P1K_SUPPORT
void ak4375_ftm_Mclk_enable(unsigned char enable)
{
 	int rc;
	if(enable)
	{
   		rc = clk_prepare_enable(ref_clk);
		if(rc){
			b_codec_err("%s: prepare ref_clk failed ret:%d\n", __func__, rc);
		}		
	}
	else
	{
		clk_disable_unprepare(ref_clk);
	}
		
}
EXPORT_SYMBOL_GPL(ak4375_ftm_Mclk_enable);
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

static const struct snd_kcontrol_new ak4375_snd_controls[] = {
    SOC_ENUM_EXT("AKM HP", hp_control_enum[0], akm4375_get_hp, akm4375_set_hp),
    SOC_ENUM_EXT("AKM FTM HP", ftm_hp_control_enum[0], akm4375_ftm_get_hp, akm4375_ftm_set_hp),
    SOC_ENUM_EXT("Reg Read", ak4375_enum[0], get_test_reg, set_test_reg),
};

/* ak4375 dapm widgets */
static const struct snd_soc_dapm_widget ak4375_dapm_widgets[] = {
};

static const struct snd_soc_dapm_route ak4375_intercon[] = {
};

#ifdef CONFIG_SND_44P1K_SUPPORT
static int ak4375_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
	 struct snd_soc_codec *codec = dai->codec;

    switch(params_format(params))
    {
		case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x0B, 0x09);	//DL1-0=01(16bit, >=32fs)
		ak4375_data->nBickFreq = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x0B, 0x00);	//DL1-0=1x(32bit, >=64fs)
		ak4375_data->nBickFreq = 1;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x0B, 0x02);	//DL1-0=1x(32bit, >=64fs)
		ak4375_data->nBickFreq = 2;
		break;
    }

	ak4375_data->fs1 = params_rate(params);
	ak4375_hw_params_set(codec, ak4375_data->fs1);

	if(ak4375_data->nPllMode == 2){
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x10,0x10);	//MS bit = 0
	}

/*[Defect]-Modified-BEGIN by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/
#if 1
	//get state HP_ON from akm4375_set_hp(), becase here is excuted after akm4375_set_hp()
	//to avoid pop noise, we called the ak power and HP Pa settings after set the codec parameters
	if (g_codec_hp_state == HP_ON)
	{
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x53, 0x53);
	}
#endif
/*[Defect]-Modified-END   by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/


    dump_trace();
    return 0;
}
#else
static int ak4375_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params,
        struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
    switch(params_format(params))
    {
		case SNDRV_PCM_FORMAT_S16_LE:
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
		ak4375_data->nBickFreq = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x02);	//DL1-0=1x(32bit, >=64fs)
		ak4375_data->nBickFreq = 2;
		break;
    }

	ak4375_data->fs1 = params_rate(params);
	ak4375_hw_params_set(codec, ak4375_data->fs1);

	if(ak4375_data->nPllMode == 0){
		snd_soc_update_bits(codec, AK4375_15_AUDIO_IF_FORMAT, 0x10,0x00);	//MS bit = 0
	}

/*[Defect]-Modified-BEGIN by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/
#if 1
	//get state HP_ON from akm4375_set_hp(), becase here is excuted after akm4375_set_hp()
	//to avoid pop noise, we called the ak power and HP Pa settings after set the codec parameters
	if (g_codec_hp_state == HP_ON)
	{
        // 0x00 == 0x00 PLL start for blck pmosc stop
        ak4375_write_mask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
        mdelay(7);                                                          //spec need 6.5
        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
        mdelay(1);															//wait 1ms

        //pwr up dac
        ak4375_write_mask(codec, AK4375_02_POWER_MANAGEMENT3, 0x01,0x01);   //PMDA=1

        ak4375_write_mask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
        mdelay(5);															//spec need 4.5ms

        //open hp amp
        ak4375_write_mask(codec, AK4375_03_POWER_MANAGEMENT4, 0x53, 0x53);
	}
#endif
/*[Defect]-Modified-END   by TCTSH.Cedar, 1746009, 2016/03/19, workaround for pop noise*/

    dump_trace();
    return 0;
}
#endif

static int ak4375_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
        unsigned int freq, int dir)
{
    b_codec_dbg("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);
    dump_trace();
    return 0;
}

static int ak4375_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    dump_trace();
    return 0;
}

static int ak4375_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    return true;
}

static int ak4375_readable(struct snd_soc_codec *codec, unsigned int reg)
{
    dump_trace();
    if (reg >= ARRAY_SIZE(ak4375_access_masks))
        return 0;
    return ak4375_access_masks[reg].readable != 0;
}

/*
 * platform data
 * */
static int ak4375_parse_dt(struct device *dev, struct ak4375_sys_data_s *sys_data)
{
    struct device_node *np = dev->of_node;
    sys_data->dac_ctl_pin = of_get_named_gpio_flags(np, "akm,dac-gpio", 0, &(sys_data->dac_ctl_pin_flags));
    if (sys_data->dac_ctl_pin < 0) {
        b_codec_err("sys_data->dac_ctl_pin\n");
        return sys_data->dac_ctl_pin;
    }

    sys_data->ldo_ctl_pin = of_get_named_gpio_flags(np, "akm,ldo-gpio", 0, &(sys_data->ldo_ctl_pin_flags));
    if (sys_data->ldo_ctl_pin < 0) {
        b_codec_err("sys_data->ldo_ctl_pin\n");
        return sys_data->ldo_ctl_pin;
    }

    return 0;
}


static int ak4375_pinctrl_init(struct ak4375_sys_data_s *sys_data)
{
    int ret;

    sys_data->pinctrl = devm_pinctrl_get(&(sys_data->client->dev));
    if (IS_ERR_OR_NULL(sys_data->pinctrl)) {
        ret = PTR_ERR(sys_data->pinctrl);
        b_codec_err("sys_data->pinctrl\n");
        goto err_pinctrl_get;
    }

    sys_data->pinctrl_state_active = pinctrl_lookup_state(sys_data->pinctrl, "ak4375_dac_active");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_active)) {
        ret = PTR_ERR(sys_data->pinctrl_state_active);
        b_codec_err("sys_data->pinctrl_state_active\n");
        goto err_pinctrl_lookup;
    }

    sys_data->pinctrl_state_suspend = pinctrl_lookup_state(sys_data->pinctrl, "ak4375_dac_suspend");
    if (IS_ERR_OR_NULL(sys_data->pinctrl_state_suspend)) {
        ret = PTR_ERR(sys_data->pinctrl_state_suspend);
        b_codec_err("sys_data->pinctrl_state_suspend\n");
        goto err_pinctrl_lookup;
    }

    ret = pinctrl_select_state(sys_data->pinctrl, sys_data->pinctrl_state_active);
    if (ret) {
        b_codec_err("sys_data->pinctrl_state_active\n");
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
static int ak4375_config_pins(struct ak4375_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->dac_ctl_pin, "akm_dac_ctl") < 0) {
        b_codec_err("gpio err sys_data->dac_ctl_pin\n");
        return -1;
    }
    gpio_direction_output(sys_data->dac_ctl_pin, 0);


    if (gpio_request(sys_data->ldo_ctl_pin, "akm_ldo_ctl") < 0) {
        b_codec_err("gpio err sys_data->ldo_ctl_pin\n");
        return -1;
    }
    gpio_direction_output(sys_data->ldo_ctl_pin, 0);

    return  0;
}


static int ak4375_regulator_init(struct ak4375_sys_data_s *sys_data)
{
    int rc = -1;

    /*
     * codec V_L6_1P8 src VREG_L6
     * I2C pull up  V_L6_1P8 src VREG_L6
     * */
    sys_data->vdd = regulator_get(&sys_data->client->dev, "vdd");
    if (IS_ERR(sys_data->vdd)) {
        b_codec_err("%s %d get vdd error\n", __func__, __LINE__);
        goto err_get_vdd;
    }

    if (regulator_count_voltages(sys_data->vdd) > 0)
    {
        rc = regulator_set_voltage(sys_data->vdd, 1800000, 1800000);
        if (rc) {
            b_codec_err("%s %d set vdd error\n", __func__, __LINE__);
            goto err_set_vdd;
        }
    }

    /*
     * VLDO_1V8  <== LDO src (VPH_PWR == VBATT)
     * */
    rc = regulator_enable(sys_data->vdd);
    if (rc) {
        b_codec_err("Regulator vdd enable failed rc=%d\n", rc);
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
#include "ak4375_debug.c"

static int ak4375_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
    b_codec_dbg("\t[AK4375] %s(%d) cmd=0x%x\n", __FUNCTION__, __LINE__, cmd);

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

static int ak4375_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
    dump_trace();
    return 0;
}

#define AK4375_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
        SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
        SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
        SNDRV_PCM_RATE_192000)

#define AK4375_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4375_dai_ops = {
    .hw_params	= ak4375_hw_params,
    .set_sysclk	= ak4375_set_dai_sysclk,
    .set_fmt	= ak4375_set_dai_fmt,
    .trigger = ak4375_trigger,
    .digital_mute = ak4375_set_dai_mute,
};

struct snd_soc_dai_driver ak4375_dai[] = {
    {
        .name = "ak4375-AIF1",
        .playback = {
            .stream_name = "Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = AK4375_RATES,
            .formats = AK4375_FORMATS,
        },
        .ops = &ak4375_dai_ops,
    },
};

static int ak4375_pre_init(struct snd_soc_codec *codec)
{
    ak4375_dac_on(0);
   /*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
    ak4375_ldo_power_on(0);
   /*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
    mdelay(1);

    // power on AVDD LVDD
    ak4375_ldo_power_on(1);

    mdelay(3);

    // PDN pin to high
    ak4375_dac_on(1);

    // if system loop in suspend/resume
    // the first reg may write error
    // check it with FAE
    mdelay(3);

   ak4375_bclk_or_mclk_mode(codec);

    return 0;
}

void ak4375_reinit(void)
{
    struct snd_soc_codec *codec = ak4375_codec;
    ak4375_pre_init(codec);
}

static int ak4375_probe(struct snd_soc_codec *codec)
{
    struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);
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
#ifdef AK4375_DUMMY
    codec->write = ak4375_fake_write;
    codec->read = ak4375_fake_read;
#else
    codec->write = ak4375_write;
    codec->read = ak4375_read;
#endif

    codec->control_data = ak4375_sys_data->client;

    ak4375->fs1 = 48000;
    ak4375->fs2 = 48000;
    ak4375->rclk = 0;
    ak4375->nSeldain = 0;		//0:Bypass, 1:SRC	(Dependent on a register bit)
    ak4375->nBickFreq = 0;		//0:32fs, 1:48fs, 2:64fs
    ak4375->nSrcOutFsSel = 0;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
#ifdef PLL_BICK_MODE
    ak4375->nPllMode = 1;		//1: PLL ON
#else
    ak4375->nPllMode = 2;		//0:PLL OFF; 1:PLL ON SLAVE 2:PLL ON MASTER
    ak4375->nPllMCKI = 0;          //0:9.6MHz, 1:11.2896MHz, 2:12.288MHz, 3:19.2MHz
#endif
    ak4375->nSmt = 0;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
    ak4375->dfsrc8fs = 0;		//0:DAC Filter, 1:Bypass, 2:8fs mode


    b_codec_dbg("\n\n\n DUMP AK4375 CONFIG \n\n\n");
    b_codec_dbg("ak4375->fs1 = %d\n", ak4375->fs1);
    b_codec_dbg("ak4375->fs2 = %d\n", ak4375->fs2);
    b_codec_dbg("ak4375->rclk = %d\n", ak4375->rclk);
    b_codec_dbg("ak4375->nSeldain = %d\n", ak4375->nSeldain);
    b_codec_dbg("ak4375->nBickFreq = %d\n", ak4375->nBickFreq);
    b_codec_dbg("ak4375->nSrcOutFsSel = %d\n", ak4375->nSrcOutFsSel);
    b_codec_dbg("ak4375->nPllMode = %d\n", ak4375->nPllMode);
    b_codec_dbg("ak4375->nSmt = %d\n", ak4375->nSmt);
    b_codec_dbg("ak4375->dfsrc8fs = %d\n", ak4375->dfsrc8fs);

    ak4375_pre_init(ak4375_codec);
	printk("%s success!!!",__func__);
    return ret;
}

/*[BUG]-Add-START   by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
static void ak4375_suspend_power_pwn(void)
{
/*From AKM FAE,we know that if you want to close power ,must pull down PWD-PIN first
   then control  TVDD,AVDD-LVDD  */
         int rc=0;

	/*pull down PDN first */
	 ak4375_dac_on(0);

	mdelay(1);

	/*close AVDD-LVDD power */
	 ak4375_ldo_power_on(0);

	/*close TVDD power*/
	rc = regulator_disable(ak4375_sys_data->vdd);
         if (rc) {
            dev_err(&ak4375_sys_data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
        }

}
static void ak4375_resume_power_pwn(void)
{
/*From AKM FAE,we know that if you want to resume AKM ,must  open the power include <TVDD,AVDD-LVDD>.
   then pull up PWD-PIN first */
   int rc=0;

  /*open  AVDD-LVDD power*/
    ak4375_ldo_power_on(1);

  /*open TVDD power*/
    rc = regulator_enable(ak4375_sys_data->vdd);
    if (rc) {
            dev_err(&ak4375_sys_data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
       }
    mdelay(1);
   /*pull up PWD */
     ak4375_dac_on(1);
   /*
     From akm-spec page 47 ,we know when pull up PWD, 10ms(min) wait time is necessary
     this issue offer appear,and it have diffenrent reasons, eg: IIC ERROR ,reg cannot write .
     maybe this is reason...[from 3ms to 15ms]
   */
   mdelay(15);

}
/*[BUG]-Add-END  by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
static int ak4375_suspend(struct snd_soc_codec *codec)
{

    b_codec_dbg("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {
    /*[BUG]-Add-START   by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
	ak4375_suspend_power_pwn();
    /*[BUG]-Add-END  by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
    }
    else
    {
        // keep everything on
    }

    return 0;
}

static int ak4375_resume(struct snd_soc_codec *codec)
{

    b_codec_dbg("%s %d g_codec_hp_state = %s\n", __FUNCTION__, __LINE__, g_codec_hp_state?"on":"off");

    if (g_codec_hp_state == HP_OFF)
    {
      /*[BUG]-Add-START   by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
	ak4375_resume_power_pwn();
	ak4375_bclk_or_mclk_mode(ak4375_codec);
     /*[BUG]-Add-END  by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
    }
    else
    {
        //everything is on
    }

    return 0;
}

static int ak4375_remove(struct snd_soc_codec *codec)
{
    b_codec_err("%s (%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

struct snd_soc_codec_driver soc_codec_dev_ak4375 = {
    .probe = ak4375_probe,
    .remove = ak4375_remove,

    .suspend =	ak4375_suspend,
    .resume =	ak4375_resume,


    .reg_cache_size = ARRAY_SIZE(ak4375_reg),
    .reg_word_size = sizeof(u8),
    .reg_cache_default = ak4375_reg,
    .readable_register = ak4375_readable,
    .volatile_register = ak4375_volatile,

    .controls = ak4375_snd_controls,
    .num_controls = ARRAY_SIZE(ak4375_snd_controls),
    .dapm_widgets = ak4375_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(ak4375_dapm_widgets),
    .dapm_routes = ak4375_intercon,
    .num_dapm_routes = ARRAY_SIZE(ak4375_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4375);



static int ak4375_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int rc = 0;
    struct snd_soc_codec *codec;
	u8 device_id = 0;

    b_codec_dbg("ak4375_snd_controls = %d\n", (int)ARRAY_SIZE(ak4375_snd_controls));

    b_codec_trace();

    ak4375_data = kzalloc(sizeof(struct ak4375_priv), GFP_KERNEL);
    if (ak4375_data == NULL) {
         b_codec_err("no memory\n");
        return -ENOMEM;
    }


    b_codec_trace();

    ak4375_sys_data = kzalloc(sizeof(struct ak4375_sys_data_s), GFP_KERNEL);
    if (ak4375_sys_data == NULL) {
        kfree(ak4375_data);
        b_codec_err("no memory\n");
        return -ENOMEM;
    }

    b_codec_trace();


    /*
     * init structure first!!!
     * */
    codec = &ak4375_data->codec;
	//i2c->addr = 0x10;/*!!!for compatible, we set fake address in dtsi, so here should set the correct value*/
    ak4375_sys_data->client = i2c;
    codec->control_data = i2c;
    codec->dev = &i2c->dev;
    ak4375_codec = codec;

    b_codec_trace();

    /*
     * get resource from OF!
     * */
    rc = ak4375_parse_dt(&(i2c->dev), ak4375_sys_data);
    if (rc) {
        b_codec_err("ak4375_parse_dt error %d\n", rc);
    }

    b_codec_trace();


    rc = ak4375_regulator_init(ak4375_sys_data);
    if (rc < 0) {
        b_codec_err("get regulator error %d\n", rc);
    }

    b_codec_trace();

    rc = ak4375_pinctrl_init(ak4375_sys_data);
    if (rc < 0) {
        b_codec_err("get pinctl init error %d\n", rc);
    }

    b_codec_trace();


    /*
     *
     init ak4375
     * */
    ak4375_config_pins(ak4375_sys_data);
    ak4375_dac_on(0);
    mdelay(1);
    // power on AVDD LVDD
    ak4375_ldo_power_on(1);
	mdelay(1);
    // PDN pin to high
    ak4375_dac_on(1);
    /*[BUG]-Add-START   by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
   /*
     From akm-spec page 47 ,we know when pull up PWD, 10ms(min) wait time is necessary
     this issue offer appear,and it have diffenrent reasons, eg: IIC ERROR ,reg cannot write .
     maybe this is reason...[from 3ms to 15ms]
   */
    mdelay(10);
   /*[BUG]-Add-END  by TCTSH.YK,1158920, 2015/12/17, solve headset no sound after deep-sleep*/
    b_codec_trace();

	device_id = i2c_smbus_read_byte_data(ak4375_sys_data->client, (u8)(AK4375_15_AUDIO_IF_FORMAT & 0xFF));
	/*
	BIT7~BIT5
	000:AK4375
	001:AK4375A
	010:AK4376
	*/
	device_id = (device_id & 0xE0)>>5;
	printk("%s:device_id = %#x\n", __func__, device_id);

	/*[Task]-Modified-BEGIN by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
#if 0
	if(device_id != 1)/*DEVICEID_AK4375A = 001*/
	{
	/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	 gpio_free(ak4375_sys_data->dac_ctl_pin);
	 gpio_free(ak4375_sys_data->ldo_ctl_pin);
	/*[BUG]-Add-END   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
        kfree(ak4375_data);
        kfree(ak4375_sys_data);
	  return -1;
	}
#endif
	/*[Task]-Modified-END   by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/

    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));
    dev_set_name(&i2c->dev, "%s", "akm-ak4375");
    b_codec_dbg("i2c_device_name = %s\n", dev_name(&i2c->dev));

    /*
     * register sound
     * */
    snd_soc_codec_set_drvdata(codec, ak4375_data);

    rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_ak4375, &ak4375_dai[0], ARRAY_SIZE(ak4375_dai));
    if (rc < 0){
        kfree(ak4375_data);
        kfree(ak4375_sys_data);
        b_codec_err("snd_soc_register_codec eror %d\n", rc);
    }

    b_codec_trace();
#ifdef CONFIG_SND_44P1K_SUPPORT
    ref_clk = clk_get(&i2c->dev, "ref_clk");
	if (IS_ERR(ref_clk)) {
		b_codec_err("%s:ak4375 Error getting ref_clk\n", __func__);
		ref_clk = NULL;
	}
	else{
		clk_set_rate(ref_clk, AK4375_CLK_FREQ);
		b_codec_dbg("%s: set_rate bb_clk2\n", __func__);
	}
#endif
    /*
     * register debug interface for cmdline mode
     * */
    rc = device_create_file(&(i2c->dev), &dev_attr_ak4375_control);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    rc = device_create_file(&(i2c->dev), &dev_attr_ak4375_dac);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    rc = device_create_file(&(i2c->dev), &dev_attr_ak4375_ldo);
    if (rc) {
        b_codec_err("device_create_file eror %d\n", rc);
    }

    b_codec_trace();
/*[BUG]-Add-START   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	//ak4375_en=true;/*[Task]-Modified by TCTSH.Cedar, 1784755, 2016/03/11, idol4 only use ak4375*/
/*[BUG]-Add-End   by TCTSH.YK, 1097898 , 2015/12/10, The audio can not heard from headset*/
	i2c_ok = 1;/*[TASK]-Add by TCTSH.Cedar, 937744, 2015/12/17, add for RunInTest*/
	printk("%s success!!!",__func__);
    return rc;
}

static int ak4375_i2c_remove(struct i2c_client *client)
{
    b_codec_err("%s(%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

static const struct i2c_device_id ak4375_i2c_id[] = {
    { "ak4375", 0 },
    { }
};

static struct of_device_id ak4375_match_table[] = {
    { .compatible = "akm,ak4375", },
    { },
};

MODULE_DEVICE_TABLE(i2c, ak4375_i2c_id);

static struct i2c_driver ak4375_i2c_driver = {
    .driver = {
        .name = "akm-ak4375",
        .owner = THIS_MODULE,
        .of_match_table = ak4375_match_table,
    },
    .probe = ak4375_i2c_probe,
    .remove = ak4375_i2c_remove,
    .id_table = ak4375_i2c_id,
};

module_i2c_driver(ak4375_i2c_driver);

MODULE_DESCRIPTION("IDOL3 ak4375 codec driver");
MODULE_LICENSE("GPL");
