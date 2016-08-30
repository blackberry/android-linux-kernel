static ssize_t tfa9890_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int reg;


 #ifdef CONFIG_TCT_8X76_IDOL4S /*TCT-NB Tianhongwei add for idol4s*/
   g_tfa9890_sys->client1->addr = 0x36;
 #else
   g_tfa9890_sys->client1->addr = 0x35;
 #endif
    g_tfa9890_sys->client2->addr = 0x34;

    reg = 0x03;
    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client1->addr,  reg, tfa9890_read(g_tfa9890_sys->client1, reg));

    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client2->addr,  reg, tfa9890_read(g_tfa9890_sys->client2, reg));

    reg = 0x04;
    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client1->addr,  reg, tfa9890_read(g_tfa9890_sys->client1, reg));

    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client2->addr,  reg, tfa9890_read(g_tfa9890_sys->client2, reg));

    reg = 0x06;
    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client1->addr,  reg, tfa9890_read(g_tfa9890_sys->client1, reg));

    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client2->addr,  reg, tfa9890_read(g_tfa9890_sys->client2, reg));
    reg = 0x09;
    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client1->addr,  reg, tfa9890_read(g_tfa9890_sys->client1, reg));

    sprintf(buf, "%s chip addr=0x%02x [reg 0x%02x] = 0x%02x\r\n", buf,
            g_tfa9890_sys->client2->addr,  reg, tfa9890_read(g_tfa9890_sys->client2, reg));


    g_tfa9890_sys->client1->addr = 0x38;
    g_tfa9890_sys->client2->addr = 0x40;


    return sprintf(buf, "%sregister read done.\r\n", buf);
}

static ssize_t tfa9890_reg_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    u8 reg = 0;
    u16 data = 0;
    int ret = 0;

    if (size != 6) {
        printk("echo -n 030000 > tfa9890_control\n");
        return -1;
    }

    /*
     * trans reg
     * */
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


    /*
     * trans data
     * */
    ret = hex2dec(buf[2]);
    if (ret == -1) {
        printk("data 0 error\n");
        return -1;
    }
    data= ret << 12;

    ret = hex2dec(buf[3]);
    if (ret == -1) {
        printk("data 1 error\n");
        return -1;
    }
    data |= (ret << 8);

    ret = hex2dec(buf[4]);
    if (ret == -1) {
        printk("data 2 error\n");
        return -1;
    }
    data |= ret << 4;

    ret = hex2dec(buf[5]);
    if (ret == -1) {
        printk("data 3 error\n");
        return -1;
    }
    data |= (ret & 0xf);

    b_codec_dbg("%s write [reg 0x%02x] = 0x%04x\n", __func__, reg, data);


 #ifdef CONFIG_TCT_8X76_IDOL4S /*TCT-NB Tianhongwei add for idol4s*/
    g_tfa9890_sys->client1->addr = 0x36;
 #else
    g_tfa9890_sys->client1->addr = 0x35;
 #endif
    g_tfa9890_sys->client2->addr = 0x34;

    /*
     * Bruce FIXME one2all
     * */
    tfa9890_write_word(g_tfa9890_sys->client1, reg, data);
    tfa9890_write_word(g_tfa9890_sys->client2, reg, data);


    g_tfa9890_sys->client1->addr = 0x38;
    g_tfa9890_sys->client2->addr = 0x40;

    return size;
}


static DEVICE_ATTR(tfa9890_reg, 0666, tfa9890_reg_show, tfa9890_reg_store);


/*
 * force set CLASS-D AMP
 * wish code is useless
 * */
void enable_top_amp(int on)
{
    u16 old = 0;
    u16 new = 0;

    g_tfa9890_sys->client1->addr = 0x35;
    old = tfa9890_read(g_tfa9890_sys->client1, 0x09);
    new = (old & ~(1 << 3)) | (on<<3);
    g_tfa9890_sys->client1->addr = 0x38;

    printk("old=0x%x new=0x%x\n", old, new);

    tfa9890_trans_word(g_tfa9890_sys->client1, 0x09, new);
}


void enable_btm_amp(int on)
{
    u16 old = 0;
    u16 new = 0;

    g_tfa9890_sys->client2->addr = 0x34;
    old = tfa9890_read(g_tfa9890_sys->client2, 0x09);
    new = (old & ~(1 << 3)) | (on<<3);
    g_tfa9890_sys->client2->addr = 0x40;

    printk("old=0x%x new=0x%x\n", old, new);

    tfa9890_trans_word(g_tfa9890_sys->client2, 0x09, new);
}

static int stero_mono_mode = 2;

static ssize_t tfa9890_sm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", stero_mono_mode);
}

static ssize_t tfa9890_sm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	switch (value)
    {
        case 0:
            enable_top_amp(1);
            enable_btm_amp(0);
            break;

        case 1:
            enable_top_amp(0);
            enable_btm_amp(1);
            break;

        case 2:
            enable_top_amp(1);
            enable_btm_amp(1);
            break;

        default:
            b_codec_dbg("%s: unknow stero_mono_mode=%d\n", __func__, value);
            break;
	}

    stero_mono_mode = value;

	b_codec_dbg("%s: stero_mono_mode=%d\n", __func__, stero_mono_mode);

	return size;
}

static DEVICE_ATTR(stereo_mono_mode, S_IRUGO | S_IWUGO, tfa9890_sm_show, tfa9890_sm_store);


int tfa9890_debug_init(struct i2c_client *i2c)
{
    int rc = 0;

    rc = device_create_file(&(i2c->dev), &dev_attr_tfa9890_reg);
    if (rc) {
        b_codec_err("device_create_file reset eror %d\n", rc);
    }

	rc = device_create_file(&(i2c->dev), &dev_attr_stereo_mono_mode);
	if (rc) {
		b_codec_err("device_create_file reset eror %d\n", rc);
	}

    return rc;
}
