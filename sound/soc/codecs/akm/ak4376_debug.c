/*[FEATURE]-Add-BEGIN by kun.guan, 526254, 2015/10/28, add a debug function for headset*/
extern void ak4376_reinit(void);
/*[FEATURE]-Add-END   by kun.guan, 526254, 2015/10/28, add a debug function for headset*/

static unsigned int ak4376_i2c_raw_read(struct i2c_client *client, unsigned int reg)
{

	int ret;
	ret = i2c_smbus_read_byte_data(client, (u8)(reg & 0xFF));
	if (ret < 0) {
		b_codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int ak4376_i2c_raw_write(struct i2c_client *client, unsigned int reg, unsigned int value)
{
	b_codec_dbg("%s: (addr,data)=(%x, %x)\n", __FUNCTION__, reg, value);

	if(i2c_smbus_write_byte_data(client, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		b_codec_dbg("%s(%d) error\n",__FUNCTION__,__LINE__);
		return EIO;
	}

    return 0;
}

#if 0
static int ak4376_i2c_raw_write_mask(struct i2c_client *client, u32 reg, u32 mask, u32 value)
{
    int ret = 0;
    u32 old;
    u32 new;

    old = ak4376_i2c_raw_read(client, reg);
    if (old < 0) {
        b_codec_dbg("mask read %s(%d) error\n", __func__, __LINE__);
    }

    new = (old & ~(mask)) | value;

    ret = ak4376_i2c_raw_write(client, reg, new);
    if (ret < 0) {
        b_codec_dbg("mask write %s(%d) error\n", __func__, __LINE__);
    }

    return ret;
}
#endif

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

static ssize_t ak4376_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 i;

    for (i = 0; i < AK4376_MAX_REGISTERS; i++) {
        sprintf(buf, "%s[0x%02x] = 0x%02x\r\n", buf, i, ak4376_i2c_raw_read(ak4376_sys_data->client, i));
    }

    b_codec_dbg("i2c_device_name = %s\n", dev_name(dev));

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


    /*
     *  Bruce hack cmd in here to ops gpio ldo or etc
     *  address from 0x25 is free using
     * */
    if (reg == 0xff) {
        //ak4376_cmdline_fast_test_idol347();
    }

    if (reg == 0xfe) {
        ak4376_ldo_power_on(value);
        b_codec_dbg("ldo power %s\n", value?"on":"off");
    }

    if (reg == 0xfd) {
        ak4376_dac_on(value);
        b_codec_dbg("DAC %s\n", value?"on":"off");
    }

    /*[FEATURE]-Add-BEGIN by kun.guan, 526254, 2015/10/28, add a debug function for headset*/
    if (reg == 0xfc) {
        ak4376_reinit();
    }
    /*[FEATURE]-Add-END   by kun.guan, 526254, 2015/10/28, add a debug function for headset*/

    if (reg == 0xfa) {
        g_dump_trace = value;
    }

    /*
     * real reg
     * */
    if (reg >=0 && reg < AK4376_MAX_REGISTERS) {
        ak4376_i2c_raw_write(ak4376_sys_data->client, reg, value);
        printk("Set  : reg = 0x%02x, value = 0x%02x\n", reg, value);
        printk("Read : reg = 0x%02x, value = 0x%02x\n", reg, ak4376_i2c_raw_read(ak4376_sys_data->client, reg));
    }

    return size;
}


static DEVICE_ATTR(ak4376_control, 0666, ak4376_control_show, ak4376_control_store);


static ssize_t ak4376_dac_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int err = 0;
    unsigned long value;

    if (count != 1) {
        b_codec_dbg("%s %d cnt=%d, buf=%d\n", __func__, __LINE__, (int)count, (int)sizeof(buf));
        return -EINVAL;
    }

    err = kstrtoul(buf, 10, &value);
    if (err != 0) {
        b_codec_dbg("%s %d err=%d\n", __func__, __LINE__, err);
        return err;
    }

    switch (value)
    {
        case 0:
            ak4376_dac_on(0);
            break;

        case 1:
            ak4376_dac_on(1);
            break;

        default:
            err = -EINVAL;
            b_codec_dbg("%s %d unknow value err=%d\n", __func__, __LINE__, (int)value);
            break;
    }

    return count;
}
static DEVICE_ATTR(ak4376_dac, 0220, NULL, ak4376_dac_store);


static ssize_t ak4376_ldo_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int err = 0;
    unsigned long value;

    if (count != 1) {
        b_codec_dbg("%s %d cnt=%d, buf=%d\n", __func__, __LINE__, (int)count, (int)sizeof(buf));
        return -EINVAL;
    }

    err = kstrtoul(buf, 10, &value);
    if (err != 0) {
        b_codec_dbg("%s %d err=%d\n", __func__, __LINE__, err);
        return err;
    }

    switch (value)
    {
        case 0:
            ak4376_ldo_power_on(0);
            break;

        case 1:
            ak4376_ldo_power_on(1);
            break;

        default:
            err = -EINVAL;
            b_codec_dbg("%s %d unknow value err=%d\n", __func__, __LINE__, (int)value);
            break;
    }

    return count;
}
static DEVICE_ATTR(ak4376_ldo, 0220, NULL, ak4376_ldo_store);



