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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm/io.h>


#define TFA9890_DEBUG
#define AUDIO_NAME "#BC tfa9890"

static int g_tfa9890_cnt = 0;
#define TFA9890_I2C_ADDR_TOP 0x38
#define TFA9890_I2C_ADDR_BTM 0x40

#ifdef TFA9890_DEBUG
#define b_codec_dbg(format, arg...) \
    pr_debug(AUDIO_NAME ": " format, ## arg)

#define b_codec_err(format, arg...) \
    pr_err(AUDIO_NAME ": " format, ## arg)

#define b_codec_info(format, arg...) \
    pr_debug(AUDIO_NAME ": " format, ## arg)

#define b_codec_warn(format, arg...) \
    pr_debug(AUDIO_NAME ": " format, ## arg)

#define b_codec_trace() \
    pr_debug(AUDIO_NAME" %s(%d)\n", __func__, __LINE__)
#else
#define b_codec_dbg(format, arg...) do {} while (0)
#define b_codec_err(format, arg...) do {} while (0)
#define b_codec_info(format, arg...) do {} while (0)
#define b_codec_warn(format, arg...) do {} while (0)
#define b_codec_trace() do {} while (0)
#endif
/*TCT-NB Tianhongwei add for NPI info*/
static int i2c_ok = 0;
module_param_named(i2c_ok, i2c_ok, int, 0644);
/*TCT-NB Tianhongwei end*/
struct tfa9890_sys_data_s{
    u32 reset_pin;
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
    u32 reset1_pin;
#endif
    u32 switch1_ctl_pin;
    u32 switch2_ctl_pin;
    int spk_box_id_pin1;
    int spk_box_id_pin2;
    u32 reset_pin_flags;
    u32 switch1_pin_flags;
    u32 switch2_pin_flags;
    u32 spk_box_id_flags;
    struct regulator *vdd;
    struct i2c_client *client1;
    struct i2c_client *client2;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct pinctrl_state *spk_box_id_active;
    struct pinctrl_state *spk_box_id_suspend;
    int reset_pin_state;
    int switch1_pin_state;
    int switch2_pin_state;
    int spk_box_id;
};

struct tfa9890_sys_data_s  *g_tfa9890_sys = NULL;

unsigned long idol347_board_id = 0;
static int __init idol347_board_setup(char *str)
{
	unsigned long board_id;
	if (!strict_strtoul(str, 0, &board_id))
		idol347_board_id = board_id;
	return 1;
}
__setup("androidboot.BOARD_ID=", idol347_board_setup);

static int hex2dec(u8 ch)
{
    if(ch >= '0' && ch <= '9')
        return ch - '0';
    else if(ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if(ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;

    return -1;
}

static int raw_i2c_read(struct i2c_client *client, char *buf, int count)
{
    if (count != i2c_master_recv(client, buf, count)){
        b_codec_dbg("i2c_master_recv error\n");
        return -1;
    }

    return 0;
}

static int raw_i2c_write(struct i2c_client *client, char *buf, int count)
{
    if(count != i2c_master_send(client, buf, count)){
        b_codec_dbg("i2c_master_send error\n");
        return -1;
    }

    return 0;
}

//
// 1. write addr
// 2. first msb
// 3, sec   lsb
//
static int tfa9890_read(struct i2c_client *client, int reg)
{
    int rc;
    __be16 tmp_data = 0;
    u16 reg_data  = 0;

    rc = raw_i2c_write(client, (char *)(&reg), 1);
    if (rc) {
        b_codec_dbg("tfa9890_read w 0x%02x\n", reg);
        return -1;
    }

    rc = raw_i2c_read(client, (char *)(&tmp_data), 2);
    if (rc) {
        b_codec_dbg("tfa9890_read r 0x%02x\n", reg);
        return -1;
    }
    reg_data = be16_to_cpu(tmp_data);

    b_codec_dbg("tmp_data=0x%04x, reg_data=0x%04x\n", tmp_data, reg_data);

    return reg_data;
}

//
// 1. write addr
// 2. write msb
// 3, write lsb
//
static int tfa9890_write_word(struct i2c_client *client, int reg, int data)
{
    int rc;
    u8 buf[3];
    __be16 tmp_data = data;
    u16 reg_data = be16_to_cpu(tmp_data);

    b_codec_dbg("%s chip addr=0x%02x reg=0x%02x human_data=0x%04x, reg_data=0x%04x\n", __func__, reg, client->addr, tmp_data, reg_data);

    buf[0] = reg&0xff;
    buf[1] = reg_data&0xff; //msb first
    buf[2] = (reg_data >> 8)&0xff; //lsb last

    rc = raw_i2c_write(client, (char *)buf, 3);
    if (rc < 0) {
        b_codec_dbg("%s chip addr=0x%02x err = %d\n", __func__, client->addr, rc);
    }

    return 0;
}

static int tfa9890_trans_word(struct i2c_client *client, int reg, int data)
{
    //
    // hack addr take care climax
    //
    if (client == g_tfa9890_sys->client1)
    {
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
        client->addr = 0x36;
#else
        client->addr = 0x35;
#endif
    }
    else
    {
        if (client == g_tfa9890_sys->client2)
        {
            client->addr = 0x34;
        }
    }

    tfa9890_write_word(client, reg, data);

    if (client == g_tfa9890_sys->client1)
    {
        client->addr = 0x38;
    }
    else
    {
        if (client == g_tfa9890_sys->client2)
        {
            client->addr = 0x40;
        }
    }

    return 0;
}

void tfa9890_reset(struct tfa9890_sys_data_s  *tfa9890_sys, int onoff)
{
    if (onoff)
    {
        gpio_direction_output(tfa9890_sys->reset_pin, 1);
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
        gpio_direction_output(tfa9890_sys->reset1_pin, 1);
#endif
    }
    else
    {
        gpio_direction_output(tfa9890_sys->reset_pin, 0);
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
        gpio_direction_output(tfa9890_sys->reset1_pin, 0);
#endif
    }

    tfa9890_sys->reset_pin_state = onoff;

    b_codec_dbg("tfa9890 reset = %d\n", tfa9890_sys->reset_pin_state);

    return;
}

/*
 * switch1 --> 0x35 --> BTM
 * switch2 --> 0x34 --> TOP
 * high --> rev
 * low --> spk
 * */
enum {
    SPK_REV_TOP_SWITCH = 1,
    SPK_REV_BTM_SWITCH,
};

void tfa9890_switch(struct tfa9890_sys_data_s  *tfa9890_sys, int lr, int onoff)
{
    u32 pin;
    int ret = 0;

    b_codec_trace();
    ret = pinctrl_select_state(tfa9890_sys->pinctrl, tfa9890_sys->pinctrl_state_active);
    if (ret) {
        b_codec_trace();
    }

    if (lr == SPK_REV_TOP_SWITCH)
    {
        pin = tfa9890_sys->switch1_ctl_pin;
        tfa9890_sys->switch1_pin_state = onoff;
        b_codec_dbg("set SPK_REV_BTM_SWITCH = %d onoff = %d\n", tfa9890_sys->switch1_pin_state, onoff);
    }
    else
    {
        pin = tfa9890_sys->switch2_ctl_pin;
        tfa9890_sys->switch2_pin_state = onoff;
        b_codec_dbg("set SPK_REV_TOP_SWITCH = %d onoff = %d\n", tfa9890_sys->switch2_pin_state, onoff);
    }

    if (onoff)
    {
        gpio_direction_output(pin, 1);
    }
    else
    {
        gpio_direction_output(pin, 0);
    }

    return;
}


/*
 * platform data
 * */
static int tfa9890_pinctrl_init(struct tfa9890_sys_data_s *sys_data)
{
	int ret;

	sys_data->pinctrl = devm_pinctrl_get(&(sys_data->client1->dev));
	if (IS_ERR_OR_NULL(sys_data->pinctrl)) {
		ret = PTR_ERR(sys_data->pinctrl);
        b_codec_trace();
		goto err_pinctrl_get;
	}

	sys_data->pinctrl_state_active = pinctrl_lookup_state(sys_data->pinctrl, "nxp9890_spk_rcv_switch_active");
	if (IS_ERR_OR_NULL(sys_data->pinctrl_state_active)) {
		ret = PTR_ERR(sys_data->pinctrl_state_active);
		b_codec_trace();
		goto err_pinctrl_lookup;
	}

		sys_data->pinctrl_state_suspend = pinctrl_lookup_state(sys_data->pinctrl, "nxp9890_spk_rcv_switch_suspend");
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
#if defined(CONFIG_TCT_8X76_IDOL4S)/*TCT-NB Tianhongwei add for speakerbox type*/
	sys_data->spk_box_id_active = pinctrl_lookup_state(sys_data->pinctrl, "nxp9890_spk_box_id_active");
	if (IS_ERR_OR_NULL(sys_data->spk_box_id_active)) {
		ret = PTR_ERR(sys_data->spk_box_id_active);
		b_codec_trace();
		goto err_pinctrl_lookup;
	}

	ret = pinctrl_select_state(sys_data->pinctrl, sys_data->spk_box_id_active);
	if (ret) {
        b_codec_trace();
		return ret;
	}
#endif

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(sys_data->pinctrl);

err_pinctrl_get:
	sys_data->pinctrl = NULL;
	return ret;
}


static int tfa9890_parse_dt(struct tfa9890_sys_data_s *sys_data)
{
    struct device_node *np = sys_data->client1->dev.of_node;

    b_codec_dbg("idol347_board_id=0x%02lx\n", idol347_board_id);

    sys_data->reset_pin = of_get_named_gpio_flags(np, "tfa,rst-gpio", 0, &(sys_data->reset_pin_flags));
    if (sys_data->reset_pin < 0) {
        b_codec_dbg("reset err  %d\n", sys_data->reset_pin);
        return sys_data->reset_pin;
    }
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
    sys_data->reset1_pin = of_get_named_gpio_flags(np, "tfa,rst-1-gpio", 0, &(sys_data->reset_pin_flags));
    if (sys_data->reset1_pin < 0) {
        b_codec_dbg("reset 1 err  %d\n", sys_data->reset1_pin);
        return sys_data->reset1_pin;
    }
#endif
        sys_data->switch1_ctl_pin = of_get_named_gpio_flags(np, "tfa,spk-rcv-switch1", 0, &(sys_data->switch1_pin_flags));
        if (sys_data->switch1_ctl_pin < 0) {
            b_codec_dbg("sw1 err  %d\n", sys_data->switch1_ctl_pin);
            return sys_data->switch1_ctl_pin;
        }
        sys_data->switch2_ctl_pin = of_get_named_gpio_flags(np, "tfa,spk-rcv-switch2", 0, &(sys_data->switch2_pin_flags));
        if (sys_data->switch2_ctl_pin < 0) {
            b_codec_dbg("sw2 err  %d\n", sys_data->switch2_ctl_pin);
            return sys_data->switch2_ctl_pin;
        }
	sys_data->spk_box_id_pin1 = of_get_named_gpio_flags(np, "tfa,spk-box-id1", 0, NULL);
	if(sys_data->spk_box_id_pin1<0)
		sys_data->spk_box_id_pin1 = -1;

	sys_data->spk_box_id_pin2 = of_get_named_gpio_flags(np, "tfa,spk-box-id2", 0, NULL);
	if(sys_data->spk_box_id_pin2<0)
		sys_data->spk_box_id_pin2 = -1;

    return 0;
}


static int tfa9890_config_pins(struct tfa9890_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->reset_pin, "tfa9890_reset") < 0) {
        b_codec_dbg("tfa9890_reset gpio err  %d\n", sys_data->reset_pin);
        return -1;
    }
    gpio_direction_output(sys_data->reset_pin, 0);
#if defined(CONFIG_TCT_8X76_IDOL4S) || defined(CONFIG_TCT_8X76_IDOL4S_VDF) /*TCT-NB Tianhongwei add for idol4s*/
    if (gpio_request(sys_data->reset1_pin, "tfa9890_reset1") < 0) {
        b_codec_dbg("tfa9890_reset1 gpio err  %d\n", sys_data->reset1_pin);
        return -1;
    }
    gpio_direction_output(sys_data->reset1_pin, 0);
#endif
#ifdef CONFIG_TCT_8X76_IDOL4S
	if(gpio_is_valid(sys_data->spk_box_id_pin1))
	{
		if (gpio_request(sys_data->spk_box_id_pin1, "tfa9890_spk_box_id1") < 0) {
			b_codec_err("gpio err  %d\n", sys_data->spk_box_id_pin1);
			return -1;
		}
		gpio_direction_input(sys_data->spk_box_id_pin1);
	}
	if(gpio_is_valid(sys_data->spk_box_id_pin2))
	{
		if (gpio_request(sys_data->spk_box_id_pin2, "tfa9890_spk_box_id2") < 0) {
			b_codec_err("gpio err  %d\n", sys_data->spk_box_id_pin2);
			return -1;
		}
		gpio_direction_input(sys_data->spk_box_id_pin2);
	}

#endif
    if (gpio_request(sys_data->switch1_ctl_pin, "tfa9890_switch1") < 0) {
        b_codec_dbg("tfa9890_switch1 gpio err %d\n", sys_data->switch1_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->switch1_ctl_pin, 0);


    if (gpio_request(sys_data->switch2_ctl_pin, "tfa9890_switch2") < 0) {
        b_codec_dbg("tfa9890_switch2 gpio err %d\n", sys_data->switch2_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->switch2_ctl_pin, 0);

    return  0;
}


static int tfa9890_get_speaker_id(struct tfa9890_sys_data_s *sys_data)
{
	int ret = 0;

#ifdef CONFIG_TCT_8X76_IDOL4S /*TCT-NB Tianhongwei add for idol4s*/
	sys_data->spk_box_id = 0;
	if(sys_data->spk_box_id_pin1>0)
	{
		ret= gpio_get_value(sys_data->spk_box_id_pin1);
		if(!ret)
			sys_data->spk_box_id |= 1<<0;
	}
	if(sys_data->spk_box_id_pin2>0)
	{
		ret = gpio_get_value(sys_data->spk_box_id_pin2);
		if(!ret)
			sys_data->spk_box_id |= 1<<1;
	}
	b_codec_dbg("\n\n### SPK TYPE = %d ###\n", sys_data->spk_box_id);

#endif

    return  ret;
}


static int tfa9890_regulator_init(struct tfa9890_sys_data_s *sys_data)
{
    int rc = -1;

    /*
     * codec V_L6_1P8 src VREG_L6
     * I2C pull up  V_L6_1P8 src VREG_L6
     * */
    sys_data->vdd = regulator_get(&sys_data->client1->dev, "vdd");
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

    rc = regulator_enable(sys_data->vdd);
    if (rc) {
        dev_err(&sys_data->client1->dev, "Regulator vdd enable failed rc=%d\n", rc);
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

#include "tfa9890_sysfs.c"

static int g_nxp_spk_state = 0;
static int g_nxp_rcv_state = 0;

int tfa98xx_codec_get_speaker_amp_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  spk  state %d\n", __func__, g_nxp_spk_state);
    ucontrol->value.integer.value[0] = g_nxp_spk_state;
    return 0;
}

int tfa98xx_codec_put_speaker_amp_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 0);

// TODO:
#if 0
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);
#endif		
    }
    else
    {
        //tfa9890_reset(g_tfa9890_sys, 1);
    }

    g_nxp_spk_state = state;

    return 0;
}

int tfa98xx_codec_get_rcv_amp_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_rcv_state);
    ucontrol->value.integer.value[0] = g_nxp_rcv_state;
    return 0;
}

int tfa98xx_codec_put_rcv_amp_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 1);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 1);

// TODO:
#if 0
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);
#endif		
    }
    else
    {
        //speaker mode is default
        tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 0);
    }

    g_nxp_rcv_state = state;

    return 0;
}

/*
 * add for FTM, make it simple and resonable later
 * */
static int g_nxp_ftm_spk_top_state = 0;
static int g_nxp_ftm_spk_btm_state = 0;

int tfa98xx_codec_get_ftm_spk_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_ftm_spk_top_state);
    ucontrol->value.integer.value[0] = g_nxp_ftm_spk_top_state;
    return 0;
}

int tfa98xx_codec_put_ftm_spk_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        quat_mi2s_sclk_enable(1);  //Kun.Guan & Xing.Wang, add for FTM test, enable sclk firstly, 2015-09-01

        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 0);
//        tfa9890_trans_word(g_tfa9890_sys->client1, 0x14, 0x0000);
//        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x880b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);

        //force btm power down
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
    }
    else
    {
        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);

        quat_mi2s_sclk_enable(0);  //Kun.Guan & Xing.Wang, add for FTM test, disable sclk when reset, 2015-09-01
    }

    g_nxp_ftm_spk_top_state = state;

    return 0;
}


int tfa98xx_codec_get_ftm_spk_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_ftm_spk_btm_state);
    ucontrol->value.integer.value[0] = g_nxp_ftm_spk_btm_state;
    return 0;
}

int tfa98xx_codec_put_ftm_spk_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        quat_mi2s_sclk_enable(1);  //Kun.Guan & Xing.Wang, add for FTM test, enable sclk firstly, 2015-09-01

        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 0);
//        tfa9890_trans_word(g_tfa9890_sys->client2, 0x14, 0x0000);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x880b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        //force top power down
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
    }
    else
    {
        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);

        quat_mi2s_sclk_enable(0);    //Kun.Guan & Xing.Wang, add for FTM test, disable sclk when reset, 2015-09-01
    }

    g_nxp_ftm_spk_btm_state = state;

    return 0;
}


/*
 * new
 * */
static int g_nxp_ftm_rcv_top_state = 0;
static int g_nxp_ftm_rcv_btm_state = 0;

int tfa98xx_codec_get_ftm_rcv_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_ftm_rcv_top_state);
    ucontrol->value.integer.value[0] = g_nxp_ftm_rcv_top_state;
    return 0;
}

int tfa98xx_codec_put_ftm_rcv_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        quat_mi2s_sclk_enable(1);  //Kun.Guan & Xing.Wang, add for FTM test, enable sclk firstly, 2015-09-01

        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 1);
//        tfa9890_trans_word(g_tfa9890_sys->client1, 0x14, 0x0000);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x880b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);

        //force btm power down
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
    }
    else
    {
       tfa9890_switch(g_tfa9890_sys, SPK_REV_TOP_SWITCH, 0);/*TCT-NB Tianhongwei for PR 1238417 */
       tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);

        quat_mi2s_sclk_enable(0);    //Kun.Guan & Xing.Wang, add for FTM test, disable sclk when reset, 2015-09-01
    }

    g_nxp_ftm_rcv_top_state = state;

    return 0;
}

int tfa98xx_codec_get_ftm_rcv_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_ftm_rcv_btm_state);
    ucontrol->value.integer.value[0] = g_nxp_ftm_rcv_btm_state;
    return 0;
}

int tfa98xx_codec_put_ftm_rcv_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        quat_mi2s_sclk_enable(1);  //Kun.Guan & Xing.Wang, add for FTM test, enable sclk firstly, 2015-09-01

        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 1);
//        tfa9890_trans_word(g_tfa9890_sys->client2, 0x14, 0x0000);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x04, 0x880b);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, 0x0608);

        tfa9890_trans_word(g_tfa9890_sys->client1, 0x04, 0x800b);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x0608);

        //force top power down
        tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, 0x8209);
    }
    else
    {
        tfa9890_switch(g_tfa9890_sys, SPK_REV_BTM_SWITCH, 0);/*TCT-NB Tianhongwei for PR 1238417 */
        tfa9890_reset(g_tfa9890_sys, 1);
        mdelay(10);
        tfa9890_reset(g_tfa9890_sys, 0);

        quat_mi2s_sclk_enable(0);    //Kun.Guan & Xing.Wang, add for FTM test, disable sclk when reset, 2015-09-01
    }

    g_nxp_ftm_rcv_btm_state = state;

    return 0;
}


static const char *const speaker_amp_text[] = {"Off", "On"};
static const struct soc_enum speaker_amp_enum = SOC_ENUM_SINGLE_EXT(2, speaker_amp_text);

static const char *const rcv_amp_text[] = {"Off", "On"};
static const struct soc_enum rcv_amp_enum = SOC_ENUM_SINGLE_EXT(2, rcv_amp_text);

static const char *const amp_switch_text[] = {"Off", "On"};
static const struct soc_enum amp_switch_enum = SOC_ENUM_SINGLE_EXT(2, amp_switch_text);


static const struct snd_kcontrol_new tfa98xx_snd_controls[] = {
	SOC_ENUM_EXT("TFA98XX_SPK_AMP",
		speaker_amp_enum,
		tfa98xx_codec_get_speaker_amp_control,
		tfa98xx_codec_put_speaker_amp_control),

	SOC_ENUM_EXT("TFA98XX_RCV_AMP",
		rcv_amp_enum,
		tfa98xx_codec_get_rcv_amp_control,
		tfa98xx_codec_put_rcv_amp_control),

    SOC_ENUM_EXT("TFA98XX_FTM_SPK_TOP",
		amp_switch_enum,
		tfa98xx_codec_get_ftm_spk_top_control,
		tfa98xx_codec_put_ftm_spk_top_control),

    SOC_ENUM_EXT("TFA98XX_FTM_SPK_BTM",
        amp_switch_enum,
		tfa98xx_codec_get_ftm_spk_btm_control,
		tfa98xx_codec_put_ftm_spk_btm_control),

    SOC_ENUM_EXT("TFA98XX_FTM_RCV_TOP",
		amp_switch_enum,
		tfa98xx_codec_get_ftm_rcv_top_control,
		tfa98xx_codec_put_ftm_rcv_top_control),

    SOC_ENUM_EXT("TFA98XX_FTM_RCV_BTM",
        amp_switch_enum,
		tfa98xx_codec_get_ftm_rcv_btm_control,
		tfa98xx_codec_put_ftm_rcv_btm_control),
};

static int tfa98xx_codec_probe(struct snd_soc_codec *codec)
{
    b_codec_trace();
    return 0;
}

static int tfa98xx_codec_remove(struct snd_soc_codec *codec)
{
    b_codec_trace();
    return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_tfa98xx = {
	.probe	= tfa98xx_codec_probe,
	.remove	= tfa98xx_codec_remove,
	.controls = tfa98xx_snd_controls,
	.num_controls = ARRAY_SIZE(tfa98xx_snd_controls),
};

static struct snd_soc_dai_driver tfa98xx_dais[] = {
	{
		.name = "tfa98xx-rx",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

#include "tfa9890_debug.c"
static int tfa9890_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int rc = 0;

    b_codec_dbg("i2c addr = 0x%02x\n", i2c->addr);
    b_codec_dbg("tfa98xx_snd_controls = %d\n", (int)ARRAY_SIZE(tfa98xx_snd_controls));

    if (i2c->addr != TFA9890_I2C_ADDR_TOP && i2c->addr != TFA9890_I2C_ADDR_BTM) {
        b_codec_dbg("unknow tfa9890 = 0x%02x\n", i2c->addr);
        return -1;
    }

    if (g_tfa9890_sys == NULL)
    {
        g_tfa9890_sys = kzalloc(sizeof(struct tfa9890_sys_data_s), GFP_KERNEL);
        if (g_tfa9890_sys == NULL) {
            return -ENOMEM;
        }
    }
    b_codec_trace();


    //
    // all resource bonding with 0x34
    //
    if (i2c->addr == TFA9890_I2C_ADDR_TOP)
    {
        g_tfa9890_sys->client1 = i2c;
        b_codec_trace();
    }
    else
    {
        //
        // if 0x036, get all smartPA  reg codec
        //
        g_tfa9890_sys->client2 = i2c;
        b_codec_trace();
        goto reg_codec;
    }


    /*
     * get resource from OF!
     * */
    rc = tfa9890_parse_dt(g_tfa9890_sys);
    if (rc) {
        b_codec_err("tfa9890_parse_dt error %d\n", rc);
    }

    b_codec_trace();


    rc = tfa9890_regulator_init(g_tfa9890_sys);
    if (rc < 0) {
        b_codec_err("get regulator error %d\n", rc);
    }

    tfa9890_pinctrl_init(g_tfa9890_sys);

    b_codec_trace();

    tfa9890_config_pins(g_tfa9890_sys);

    b_codec_trace();

    tfa9890_get_speaker_id(g_tfa9890_sys);
    b_codec_trace();
    tfa9890_debug_init(i2c);
    b_codec_trace();
    tfa9890_sysfs_init();

    g_tfa9890_cnt++;

    return rc;

reg_codec:
	i2c_ok = 1;/*TCT-NB Tianhongwei add for npi info*/
    dev_set_name(&i2c->dev, "%s", "tfa9890-codec");
	rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_tfa98xx,
		 tfa98xx_dais, ARRAY_SIZE(tfa98xx_dais));
	return rc;
}

static int tfa9890_i2c_remove(struct i2c_client *client)
{
    b_codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
    return 0;
}

static const struct i2c_device_id tfa9890_i2c_id[] = {
    { "tfa9890", 0 },
    { }
};

static struct of_device_id tfa9890_match_table[] = {
    { .compatible = "nxp,tfa9890", },
    { },
};

MODULE_DEVICE_TABLE(i2c, tfa9890_i2c_id);

static struct i2c_driver tfa9890_i2c_driver = {
    .driver = {
        .name = "tfa-tfa9890",
        .owner = THIS_MODULE,
        .of_match_table = tfa9890_match_table,
    },
    .probe = tfa9890_i2c_probe,
    .remove = tfa9890_i2c_remove,
    .id_table = tfa9890_i2c_id,
};

module_i2c_driver(tfa9890_i2c_driver);

MODULE_DESCRIPTION("NXP tfa9890 codec driver");
MODULE_LICENSE("GPL");

