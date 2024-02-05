/* Copyright (C) 2016 Tcl Corporation Limited */
void ak4376_bclk_mode(struct snd_soc_codec *codec)
{
	printk("\n\n>>> BLCK MODE INIT <<<\n\n");

	/*
	 *  AKM FAE suggest move power related regs into on/off
	 *  but for 03 set 1/2 vdd time no need
	 * */
#if 0
	// 0x00 == 0x00 PLL start for blck pmosc stop
	ak4376_i2c_write(codec, AK4376_00_POWER_MANAGEMENT1, 0x01);
#endif

	// LR amp power management
	// 1/2 VDD settting capless right?  QQQQQQQQQQ
	// not open LR amp
	ak4376_i2c_write(codec, AK4376_03_POWER_MANAGEMENT4, 0x50); // MODIFIED by hongwei.tian, 2018-04-28,BUG-6267565


	// VDD hold setting QQQ
	ak4376_i2c_write(codec, AK4376_04_OUTPUT_MODE_SETTING, 0x14);


	//256fs(12Mhz, rong), 48KHZ
	// ak4376_i2c_write(codec, AK4376_05_CLOCK_MODE_SELECT, 0x0a);
	ak4376_i2c_write(codec, AK4376_05_CLOCK_MODE_SELECT, 0x09); //44.1khz

	//Sharp Roll-Off Filter
	ak4376_i2c_write(codec, AK4376_06_DIGITAL_FILTER_SELECT, 0x00);

	//LR ch select
	ak4376_i2c_write(codec, AK4376_07_DAC_MONO_MIXING, 0x21);

	//FIXME
	ak4376_i2c_write(codec, AK4376_08_JITTER_CLEANER_SETTING1, 0x00);


	// 0x00 src alll set default
	ak4376_i2c_write(codec, AK4376_09_JITTER_CLEANER_SETTING2, 0x00);


	// DAC Input Data Select = SDATA not src
	// DAC Operation Clock  = SRCMCLK
	// Charge Pump Operation Clock = XCKCPSEL
	ak4376_i2c_write(codec, AK4376_0A_JITTER_CLEANER_SETTING3, 0x00);


	// AK4376_0B_LCH_OUTPUT_VOLUME = 0x19
	// AK4376_0C_RCH_OUTPUT_VOLUME = 0x19
	/* MODIFIED-BEGIN by hongwei.tian, 2017-11-02,BUG-5556350*/
	ak4376_i2c_write(codec, AK4376_0B_LCH_OUTPUT_VOLUME, 0x11);
	ak4376_i2c_write(codec, AK4376_0C_RCH_OUTPUT_VOLUME, 0x11);

	// 0x0d = 0x75 0db  zero cross time
	ak4376_i2c_write(codec, AK4376_0D_HP_VOLUME_CONTROL, 0x0B);
	/* MODIFIED-END by hongwei.tian,BUG-5556350*/

	// 0x0e == 0x00 SCLK
	ak4376_i2c_write(codec, AK4376_0E_PLL_CLK_SOURCE_SELECT, 0x01);

	// PLL
	ak4376_i2c_write(codec, AK4376_0F_PLL_REF_CLK_DIVIDER1, 0x00);
	ak4376_i2c_write(codec, AK4376_10_PLL_REF_CLK_DIVIDER2, 0x00);
	ak4376_i2c_write(codec, AK4376_11_PLL_FB_CLK_DIVIDER1, 0x00);
	ak4376_i2c_write(codec, AK4376_12_PLL_FB_CLK_DIVIDER2, 0x4f);
	ak4376_i2c_write(codec, AK4376_13_SRC_CLK_SOURCE, 0x01);
	ak4376_i2c_write(codec, AK4376_14_DAC_CLK_DIVIDER, 0x09);

	// I2S 16bit
	ak4376_i2c_write(codec, AK4376_15_AUDIO_IF_FORMAT, 0x01);

	// no desample
	ak4376_i2c_write(codec, AK4376_24_MODE_CONTROL, 0x00);

//	ak4376_i2c_write(codec, AK4376_26_DUMMY, 0x20);//need by akm tuning // MODIFIED by hongwei.tian, 2018-01-04,BUG-5854780
}



/*
 * check pre-init with spec and FAE
 * dtsi keep releated pin output 0 no bias
 * */
static int ak4376_pre_init(struct snd_soc_codec *codec)
{
	/**
	 * PDN pin to down,
	 * power on AVDD LVDD,
	 * PDN pin to high,
	 * follow spec page9
	 */

	ak4376_dac_on(0);
	mdelay(1);

	// power on AVDD LVDD
	ak4376_ldo_power_on(1);

	mdelay(3);

	// PDN pin to high
	ak4376_dac_on(1);

	// if system loop in suspend/resume
	// the first reg may write error
	// check it with FAE
	mdelay(3);

	ak4376_bclk_mode(codec);
	//akm4376_set_hp(codec);
	return 0;
}

void akm4376_set_hp_on(struct snd_soc_codec *codec)
{
	ak4376_writeMask(codec, AK4376_00_POWER_MANAGEMENT1, 0x01, 0x01);

	ak4376_writeMask(codec, AK4376_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
	mdelay(7);															//spec need 6.5
	ak4376_writeMask(codec, AK4376_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
	mdelay(1);															//wait 1ms

	//pwr up dac
	ak4376_writeMask(codec, AK4376_02_POWER_MANAGEMENT3, 0x01,0x01);	//PMDA=1

	ak4376_writeMask(codec, AK4376_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
	mdelay(5);															//spec need 4.5ms

	//open hp amp
	ak4376_writeMask(codec, AK4376_03_POWER_MANAGEMENT4, 0x03, 0x03);


}

unsigned int ak4376_i2c_raw_read(struct i2c_client *client, unsigned int reg)
{

	int ret;
	ret = i2c_smbus_read_byte_data(client, (u8)(reg & 0xFF));
	if (ret < 0) {
		codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int ak4376_i2c_raw_write(struct i2c_client *client, unsigned int reg, unsigned int value)
{
	codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

	if(i2c_smbus_write_byte_data(client, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		codec_dbg("%s(%d) error\n",__FUNCTION__,__LINE__);
		return EIO;
	}

	return 0;
}

void ak4376_cmdline_fast_test(void)
{
	codec_trace();

	ak4376_pre_init(&ak4376_data->codec);
	akm4376_set_hp_on(&ak4376_data->codec);

}

int hex2dec(u8 ch)
{
	if(ch >= '0' && ch <= '9')
		return ch - '0';
	else if(ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;
	else if(ch >= 'A' && ch <= 'F')
		return ch - 'A' + 10;

	return -1;
}

static ssize_t ak4376_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 i;

	for (i = 0; i < AK4376_MAX_REGISTERS; i++) {
		sprintf(buf, "%s[0x%02x] = 0x%02x\r\n", buf, i, ak4376_i2c_raw_read(ak4376_codec->control_data, i));
	}
	codec_dbg("i2c_ok=%d\n", i2c_ok);

	return sprintf(buf, "%sregister read done.\r\n", buf);
}

static ssize_t ak4376_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	u8 reg, value = 0;

	if (size != 4) {
		printk("echo -n 030f > ak4376_control\n");
		return -1;
	}

	ret = hex2dec(buf[0]);
	if (ret == -1) {
		printk("store error.\n");
		return -1;
	}
	reg = ret << 4;

	ret = hex2dec(buf[1]);
	if (ret == -1) {
		printk("store error.\n");
		return -1;
	}
	reg |= (ret & 0xf);

	ret = hex2dec(buf[2]);
	if (ret == -1) {
		printk("store error.\n");
		return -1;
	}
	value = ret << 4;

	ret = hex2dec(buf[3]);
	if (ret == -1) {
		printk("store error.\n");
		return -1;
	}
	value |= (ret & 0xf);

	printk("%s reg=0x%x, value=0x%x\n", __func__, reg, value);
	/*
	 *  Bruce hack cmd in here to ops gpio ldo or etc
	 *  address from 0x25 is free using
	 * */
	if (reg == 0xff) {
		ak4376_cmdline_fast_test();
	}

	if (reg == 0xfe) {
		ak4376_ldo_power_on(value);
		codec_dbg("ldo power %s\n", value?"on":"off");
	}

	if (reg == 0xfd) {
		ak4376_dac_on(value);
		codec_dbg("DAC %s\n", value?"on":"off");
	}

	if (reg == 0xfc) {
		akm4376_set_hp_on(&ak4376_data->codec);
		codec_dbg("akm4376_set_hp_on\n");
	}

	/*
	 * real reg
	 * */
	if (reg >=0 && reg < AK4376_MAX_REGISTERS) {
		ak4376_i2c_raw_write(ak4376_codec->control_data, reg, value);
		printk("Set  : reg = 0x%02x, value = 0x%02x\n", reg, value);
		printk("Read : reg = 0x%02x, value = 0x%02x\n", reg, ak4376_i2c_raw_read(ak4376_codec->control_data, reg));
	}


	return size;
}


static DEVICE_ATTR(ak4376_control, 0664, ak4376_control_show, ak4376_control_store);
