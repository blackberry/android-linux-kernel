#include "ov4688_idol4s_v4l2_tct_OTP.h"

static struct msm_camera_i2c_client *ov4688_idol4s_g_client;
static uint16_t ov4688_module_id = 0xFFFF;
static uint32_t ov4688_module_id_magic = 0xFFFFFFFF;

static struct msm_camera_i2c_reg_array ov4688_init_setting_list[] = {
    {0x0103, 0x01, 0x00},
    {0x3638, 0x00, 0x00},
    {0x0300, 0x04, 0x00},
    {0x0302, 0x64, 0x00},
    {0x0303, 0x01, 0x00},
    {0x0304, 0x03, 0x00},
    {0x030b, 0x00, 0x00},
    {0x030d, 0x1e, 0x00},
    {0x030e, 0x04, 0x00},
    {0x030f, 0x01, 0x00},
    {0x0312, 0x01, 0x00},
    {0x031e, 0x00, 0x00},
    {0x3000, 0x20, 0x00},
    {0x3002, 0x00, 0x00},
    {0x3018, 0x72, 0x00},
    {0x3019, 0x00, 0x00},
    {0x3020, 0x93, 0x00},
    {0x3021, 0x03, 0x00},
    {0x3022, 0x01, 0x00},
    {0x3031, 0x0a, 0x00},
    {0x3305, 0xf1, 0x00},
    {0x3307, 0x04, 0x00},
    {0x3309, 0x29, 0x00},
    {0x3500, 0x00, 0x00},
    {0x3501, 0x60, 0x00},
    {0x3502, 0x00, 0x00},
    {0x3503, 0x04, 0x00},
    {0x3504, 0x00, 0x00},
    {0x3505, 0x00, 0x00},
    {0x3506, 0x00, 0x00},
    {0x3507, 0x00, 0x00},
    {0x3508, 0x00, 0x00},
    {0x3509, 0x80, 0x00},
    {0x350a, 0x00, 0x00},
    {0x350b, 0x00, 0x00},
    {0x350c, 0x00, 0x00},
    {0x350d, 0x00, 0x00},
    {0x350e, 0x00, 0x00},
    {0x350f, 0x80, 0x00},
    {0x3510, 0x00, 0x00},
    {0x3511, 0x00, 0x00},
    {0x3512, 0x00, 0x00},
    {0x3513, 0x00, 0x00},
    {0x3514, 0x00, 0x00},
    {0x3515, 0x80, 0x00},
    {0x3516, 0x00, 0x00},
    {0x3517, 0x00, 0x00},
    {0x3518, 0x00, 0x00},
    {0x3519, 0x00, 0x00},
    {0x351a, 0x00, 0x00},
    {0x351b, 0x80, 0x00},
    {0x351c, 0x00, 0x00},
    {0x351d, 0x00, 0x00},
    {0x351e, 0x00, 0x00},
    {0x351f, 0x00, 0x00},
    {0x3520, 0x00, 0x00},
    {0x3521, 0x80, 0x00},
    {0x3522, 0x08, 0x00},
    {0x3524, 0x08, 0x00},
    {0x3526, 0x08, 0x00},
    {0x3528, 0x08, 0x00},
    {0x352a, 0x08, 0x00},
    {0x3602, 0x00, 0x00},
    {0x3604, 0x02, 0x00},
    {0x3605, 0x00, 0x00},
    {0x3606, 0x00, 0x00},
    {0x3607, 0x00, 0x00},
    {0x3609, 0x12, 0x00},
    {0x360a, 0x40, 0x00},
    {0x360c, 0x08, 0x00},
    {0x360f, 0xe5, 0x00},
    {0x3608, 0x8f, 0x00},
    {0x3611, 0x00, 0x00},
    {0x3613, 0xf7, 0x00},
    {0x3616, 0x58, 0x00},
    {0x3619, 0x99, 0x00},
    {0x361b, 0x60, 0x00},
    {0x361c, 0x7a, 0x00},
    {0x361e, 0x79, 0x00},
    {0x361f, 0x02, 0x00},
    {0x3631, 0x60, 0x00},
    {0x3632, 0x00, 0x00},
    {0x3633, 0x10, 0x00},
    {0x3634, 0x10, 0x00},
    {0x3635, 0x10, 0x00},
    {0x3636, 0x15, 0x00},
    {0x3646, 0x86, 0x00},
    {0x364a, 0x0b, 0x00},
    {0x3700, 0x17, 0x00},
    {0x3701, 0x22, 0x00},
    {0x3703, 0x10, 0x00},
    {0x370a, 0x37, 0x00},
    {0x3705, 0x00, 0x00},
    {0x3706, 0x63, 0x00},
    {0x3709, 0x3c, 0x00},
    {0x370b, 0x01, 0x00},
    {0x370c, 0x30, 0x00},
    {0x3710, 0x24, 0x00},
    {0x3711, 0x0c, 0x00},
    {0x3716, 0x00, 0x00},
    {0x3720, 0x28, 0x00},
    {0x3729, 0x7b, 0x00},
    {0x372a, 0x84, 0x00},
    {0x372b, 0xbd, 0x00},
    {0x372c, 0xbc, 0x00},
    {0x372e, 0x52, 0x00},
    {0x373c, 0x0e, 0x00},
    {0x373e, 0x33, 0x00},
    {0x3743, 0x10, 0x00},
    {0x3744, 0x88, 0x00},
    {0x374a, 0x43, 0x00},
    {0x374c, 0x00, 0x00},
    {0x374e, 0x23, 0x00},
    {0x3751, 0x7b, 0x00},
    {0x3752, 0x84, 0x00},
    {0x3753, 0xbd, 0x00},
    {0x3754, 0xbc, 0x00},
    {0x3756, 0x52, 0x00},
    {0x375c, 0x00, 0x00},
    {0x3760, 0x00, 0x00},
    {0x3761, 0x00, 0x00},
    {0x3762, 0x00, 0x00},
    {0x3763, 0x00, 0x00},
    {0x3764, 0x00, 0x00},
    {0x3767, 0x04, 0x00},
    {0x3768, 0x04, 0x00},
    {0x3769, 0x08, 0x00},
    {0x376a, 0x08, 0x00},
    {0x376b, 0x20, 0x00},
    {0x376c, 0x00, 0x00},
    {0x376d, 0x00, 0x00},
    {0x376e, 0x00, 0x00},
    {0x3773, 0x00, 0x00},
    {0x3774, 0x51, 0x00},
    {0x3776, 0xbd, 0x00},
    {0x3777, 0xbd, 0x00},
    {0x3781, 0x18, 0x00},
    {0x3783, 0x25, 0x00},
    {0x3800, 0x00, 0x00},
    {0x3801, 0x08, 0x00},
    {0x3802, 0x00, 0x00},
    {0x3803, 0x04, 0x00},
    {0x3804, 0x0a, 0x00},
    {0x3805, 0x97, 0x00},
    {0x3806, 0x05, 0x00},
    {0x3807, 0xfb, 0x00},
    {0x3808, 0x0a, 0x00},
    {0x3809, 0x80, 0x00},
    {0x380a, 0x05, 0x00},
    {0x380b, 0xf0, 0x00},
    {0x380c, 0x09, 0x00},
    {0x380d, 0xC4, 0x00},
    {0x380e, 0x06, 0x00},
    {0x380f, 0x40, 0x00},
    {0x3810, 0x00, 0x00},
    {0x3811, 0x08, 0x00},
    {0x3812, 0x00, 0x00},
    {0x3813, 0x04, 0x00},
    {0x3814, 0x01, 0x00},
    {0x3815, 0x01, 0x00},
    {0x3819, 0x01, 0x00},
    {0x3820, 0x06, 0x00},
    {0x3821, 0x00, 0x00},
    {0x3829, 0x00, 0x00},
    {0x382a, 0x01, 0x00},
    {0x382b, 0x01, 0x00},
    {0x382d, 0x7f, 0x00},
    {0x3830, 0x04, 0x00},
    {0x3836, 0x01, 0x00},
    {0x3841, 0x02, 0x00},
    {0x3846, 0x08, 0x00},
    {0x3847, 0x07, 0x00},
    {0x3d85, 0x36, 0x00},
    {0x3d8c, 0x71, 0x00},
    {0x3d8d, 0xcb, 0x00},
    {0x3f0a, 0x00, 0x00},
    {0x4000, 0x71, 0x00},
    {0x4001, 0x40, 0x00},
    {0x4002, 0x04, 0x00},
    {0x4003, 0x14, 0x00},
    {0x400e, 0x00, 0x00},
    {0x4011, 0x00, 0x00},
    {0x401a, 0x00, 0x00},
    {0x401b, 0x00, 0x00},
    {0x401c, 0x00, 0x00},
    {0x401d, 0x00, 0x00},
    {0x401f, 0x00, 0x00},
    {0x4020, 0x00, 0x00},
    {0x4021, 0x10, 0x00},
    {0x4022, 0x07, 0x00},
    {0x4023, 0xcf, 0x00},
    {0x4024, 0x09, 0x00},
    {0x4025, 0x60, 0x00},
    {0x4026, 0x09, 0x00},
    {0x4027, 0x6f, 0x00},
    {0x4028, 0x00, 0x00},
    {0x4029, 0x02, 0x00},
    {0x402a, 0x06, 0x00},
    {0x402b, 0x04, 0x00},
    {0x402c, 0x02, 0x00},
    {0x402d, 0x02, 0x00},
    {0x402e, 0x0e, 0x00},
    {0x402f, 0x04, 0x00},
    {0x4302, 0xff, 0x00},
    {0x4303, 0xff, 0x00},
    {0x4304, 0x00, 0x00},
    {0x4305, 0x00, 0x00},
    {0x4306, 0x00, 0x00},
    {0x4308, 0x02, 0x00},
    {0x4500, 0x6c, 0x00},
    {0x4501, 0xc4, 0x00},
    {0x4502, 0x40, 0x00},
    {0x4503, 0x02, 0x00},
    {0x450b, 0x20, 0x00},
    {0x4600, 0x00, 0x00},
    {0x4601, 0xA7, 0x00},
    {0x4800, 0x24, 0x00},
    {0x4813, 0x08, 0x00},
    {0x481f, 0x40, 0x00},
    {0x4829, 0x78, 0x00},
    {0x4837, 0x28, 0x00},
    {0x4b00, 0x2a, 0x00},
    {0x4b0d, 0x00, 0x00},
    {0x4d00, 0x04, 0x00},
    {0x4d01, 0x42, 0x00},
    {0x4d02, 0xd1, 0x00},
    {0x4d03, 0x93, 0x00},
    {0x4d04, 0xf5, 0x00},
    {0x4d05, 0xc1, 0x00},
    {0x5000, 0xf3, 0x00},
    {0x5001, 0x11, 0x00},
    {0x5004, 0x00, 0x00},
    {0x500a, 0x00, 0x00},
    {0x500b, 0x00, 0x00},
    {0x5032, 0x00, 0x00},
    {0x5040, 0x00, 0x00},
    {0x5050, 0x0c, 0x00},
    {0x5500, 0x00, 0x00},
    {0x5501, 0x10, 0x00},
    {0x5502, 0x01, 0x00},
    {0x5503, 0x0f, 0x00},
    {0x8000, 0x00, 0x00},
    {0x8001, 0x00, 0x00},
    {0x8002, 0x00, 0x00},
    {0x8003, 0x00, 0x00},
    {0x8004, 0x00, 0x00},
    {0x8005, 0x00, 0x00},
    {0x8006, 0x00, 0x00},
    {0x8007, 0x00, 0x00},
    {0x8008, 0x00, 0x00},
    {0x3638, 0x00, 0x00},
    {0x3105, 0x31, 0x00},
    {0x301a, 0xf9, 0x00},
    {0x3508, 0x07, 0x00},
    {0x484b, 0x05, 0x00},
    {0x4805, 0x03, 0x00},
    {0x3601, 0x01, 0x00},
};

static struct msm_camera_i2c_reg_setting ov4688_init_setting[] = {
  {
    .reg_setting = ov4688_init_setting_list,
    .size = ARRAY_SIZE(ov4688_init_setting_list),
    .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
  },
};

static int32_t ov4688_idol4s_write_i2c(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
    if (!ov4688_idol4s_g_client)
        pr_err_ratelimited("FFFF OTP: OV4688 client null\n");
    else{
        rc = ov4688_idol4s_g_client->i2c_func_tbl->i2c_write(ov4688_idol4s_g_client,
                addr, 
                data,
                MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0)
            pr_err_ratelimited("FFFF OTP: write error\n");
    }
    return rc;
}

static int16_t ov4688_idol4s_read_i2c(uint32_t addr)
{
    uint16_t *data;
    uint16_t temp=0;
    int32_t rc = -EFAULT;
    data=&temp;
    if (!ov4688_idol4s_g_client)
        pr_err_ratelimited("FFFF OTP: OV4688 null\n");
    else{
        rc = ov4688_idol4s_g_client->i2c_func_tbl->i2c_read(ov4688_idol4s_g_client,
                addr,
                data, 
                MSM_CAMERA_I2C_BYTE_DATA);
        if(rc < 0)
            pr_err_ratelimited("FFFF OTP: read error\n");
    }
    return temp;
}

static int32_t ov4688_idol4s_write_i2c_table(struct msm_camera_i2c_reg_setting * reg_setting)
{
    int32_t rc = -EFAULT;
    if (!ov4688_idol4s_g_client) {
        pr_err("FFFF OTP: OV4688 client null\n");
    } else {
        rc = ov4688_idol4s_g_client->i2c_func_tbl->i2c_write_table(ov4688_idol4s_g_client,
            reg_setting);
        if(rc < 0)
            pr_err("FFFF OTP: write table error\n");
    }
    return rc;
}

static int32_t ov4688_idol4s_otp_access_enable(bool enable)
{
    int tmpVal;

    tmpVal = ov4688_idol4s_read_i2c(OTP_ISP_ENABLE_ADDR);
    if (enable) {
        tmpVal &= ~(1 << OTP_ISP_ENABLE_BIT);
    } else {
        tmpVal |= (1 << OTP_ISP_ENABLE_BIT);
    }

    ov4688_idol4s_write_i2c(OTP_ISP_ENABLE_ADDR, tmpVal);

    return 0;
}

static int32_t ov4688_idol4s_otp_restore_wb_auto(bool enable)
{
    int tmpVal;
    if (1 != enable) 
        return 0; 
    tmpVal = ov4688_idol4s_read_i2c(0x3d84);
    if (tmpVal) {
        ov4688_idol4s_write_i2c(0x3d84, 0x00);
    }

    return 0;
}

static int32_t ov4688_idol4s_stream_set(int isOn)
{
    if (isOn) {
        ov4688_idol4s_write_i2c(0x0100, 0x01);
        mdelay(10);
        ov4688_idol4s_write_i2c(0x3105, 0x11);
        ov4688_idol4s_write_i2c(0x301a, 0xF1);
        ov4688_idol4s_write_i2c(0x4805, 0x00);
        ov4688_idol4s_write_i2c(0x301a, 0xF0);
        ov4688_idol4s_write_i2c(0x3208, 0x00);
        ov4688_idol4s_write_i2c(0x302a, 0x00);
        ov4688_idol4s_write_i2c(0x302a, 0x00);
        ov4688_idol4s_write_i2c(0x302a, 0x00);
        ov4688_idol4s_write_i2c(0x302a, 0x00);
        ov4688_idol4s_write_i2c(0x302a, 0x00);
        ov4688_idol4s_write_i2c(0x3601, 0x00);
        ov4688_idol4s_write_i2c(0x3638, 0x00);
        ov4688_idol4s_write_i2c(0x3208, 0x10);
        ov4688_idol4s_write_i2c(0x3208, 0xa0);
        mdelay(5);
    } else {
        ov4688_idol4s_write_i2c(0x0100, 0x00);
        mdelay(67);
        ov4688_idol4s_write_i2c(0x301a, 0xF9);
        ov4688_idol4s_write_i2c(0x4805, 0x03);
        mdelay(5);
    }

    return 0;
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int ov4688_idol4s_check_otp_wb(int index)
{
    int flag, i;
    int address_start,address_end;
    if (1 == index) {
        address_start = 0x7110;
        address_end = 0x711f;
    } else if (2 == index) {
        address_start = 0x7120;
        address_end = 0x712f;
    } else {
        address_start = 0x7130;
        address_end = 0x713f;
    }


    ov4688_idol4s_otp_access_enable(1);

    // read otp into buffer
    // program disable, manual mode
    ov4688_idol4s_write_i2c(0x3d84, 0xc0);
    //partial mode OTP write start address
    ov4688_idol4s_write_i2c(0x3d88, (address_start>>8));
    ov4688_idol4s_write_i2c(0x3d89, (address_start & 0xff));
    // partial mode OTP write end address
    ov4688_idol4s_write_i2c(0x3d8A, (address_end>>8));
    ov4688_idol4s_write_i2c(0x3d8B, (address_end & 0xff));
    ov4688_idol4s_write_i2c(0x3d81, 0x01); // read otp
    mdelay(5);

    flag = ov4688_idol4s_read_i2c(address_start);
    pr_err("%s index %d, address 0x%x flag 0x%x\n", __func__, index, address_start ,flag);

    flag = flag & 0xc0;
    // clear otp buffer
    for (i=address_start;i<=address_end;i++) {
        ov4688_idol4s_write_i2c(i, 0x00);
    }

    ov4688_idol4s_otp_access_enable(0);

    if (flag == 0x00) {
        return 0;
    } else if (flag & 0x80) {
        return 1;
    } else {
        return 2;
    }
}

#if OV4688_VCM_EN
// index: index of otp group. (1, 2, 3)
// code: 0 for start code, 1 for stop code
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int ov4688_idol4s_check_otp_VCM(int index, int code)
{
    int flag, i;
    int address;
    int address_start,address_end;
    if (1 == index) {
        address_start = 0x7140;
        address_end = 0x7143;
    } else if (2 == index) {
        address_start = 0x7144;
        address_end = 0x7147;
    } else {
        address_start = 0x7148;
        address_end = 0x714B;
    }

    ov4688_idol4s_otp_access_enable(1);

    // read otp into buffer
    // program disable, manual mode
    ov4688_idol4s_write_i2c(0x3d84, 0xc0);
    //partial mode OTP write start address
    ov4688_idol4s_write_i2c(0x3d88, (address_start>>8));
    ov4688_idol4s_write_i2c(0x3d89, (address_start & 0xff));
    // partial mode OTP write end address
    ov4688_idol4s_write_i2c(0x3d8A, (address_end>>8));
    ov4688_idol4s_write_i2c(0x3d8B, (address_end & 0xff));
    ov4688_idol4s_write_i2c(0x3d81, 0x01); // read otp
    mdelay(5);
    // read flag
    address = address_start + code*2;
    flag = ov4688_idol4s_read_i2c(address);
    flag = flag & 0xC0;
    // clear otp buffer
    for (i=address_start;i<=address_end;i++) {
        ov4688_idol4s_write_i2c(i, 0x00);
    }

    ov4688_idol4s_otp_access_enable(0);

    if (flag == 0x00) {
        return 0;
    } else if (flag & 0x80) {
        return 1;
    } else {
        return 2;
    }
}

// index: index of otp group. (1, 2, 3)
// code: 0 start code, 1 stop code
// return: 0
static int ov4688_idol4s_read_otp_VCM(int index, int code, struct otp_struct * otp_ptr)
{
    int vcm, i;
    int address;
    int address_start, address_end;

    ov4688_idol4s_otp_access_enable(1);

    // read otp into buffer
    // program disable, manual mode
    ov4688_idol4s_write_i2c(0x3d84, 0xc0);
    //check group
    if (index==1) {
        address_start = 0x7140;
        address_end = 0x7143;
    } else if (index==2) {
        address_start = 0x7144;
        address_end = 0x7147;
    } else {
        address_start = 0x7148;
        address_end = 0x714b;
    }
    //partial mode OTP write start address
    ov4688_idol4s_write_i2c(0x3d88, (address_start>>8));
    ov4688_idol4s_write_i2c(0x3d89, (address_start & 0xff));
    // partial mode OTP write end address
    ov4688_idol4s_write_i2c(0x3d8A, (address_end>>8));
    ov4688_idol4s_write_i2c(0x3d8B, (address_end & 0xff));
    ov4688_idol4s_write_i2c(0x3d81, 0x01); // load otp into buffer
    mdelay(5);
    // read flag
    address = address_start + code*2;
    vcm = ov4688_idol4s_read_i2c(address);
    vcm = (vcm & 0x03) + (ov4688_idol4s_read_i2c(address+1)<<2);
    if (code==1) {
        (* otp_ptr).VCM_end = vcm;
    } else {
        (* otp_ptr).VCM_start = vcm;
    }
    // clear otp buffer
    for (i=address_start;i<=address_end;i++) {
        ov4688_idol4s_write_i2c(i, 0x00);
    }

    ov4688_idol4s_otp_access_enable(0);

    return 0;
}
#endif

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
static int ov4688_idol4s_read_otp_wb(int index, struct otp_struct *otp_ptr)
{
    int i;
    int temp;
    int address_start,address_end;
    // read otp into buffer
    ov4688_idol4s_write_i2c(0x3d84, 0xc0); // program disable, manual mode
    //select group
    if (index==1) {
        address_start = 0x7111;
        address_end = 0x711f;
    } else if(index==2) {
        address_start = 0x7121;
        address_end = 0x712f;
    } else {
        address_start = 0x7131;
        address_end = 0x713f;
    }

    ov4688_idol4s_otp_access_enable(1);

    //partial mode OTP write start address
    ov4688_idol4s_write_i2c(0x3d88, (address_start>>8));
    ov4688_idol4s_write_i2c(0x3d89, (address_start & 0xff));
    // partial mode OTP write end address
    ov4688_idol4s_write_i2c(0x3d8A, (address_end>>8));
    ov4688_idol4s_write_i2c(0x3d8B, (address_end & 0xff));
    // load otp into buffer
    ov4688_idol4s_write_i2c(0x3d81, 0x01);
    mdelay(5);
    (*otp_ptr).module_integrator_id = ov4688_idol4s_read_i2c(address_start);
    (*otp_ptr).lens_id = ov4688_idol4s_read_i2c(address_start + 1);
    (*otp_ptr).production_year = ov4688_idol4s_read_i2c(address_start + 2);
    (*otp_ptr).production_month = ov4688_idol4s_read_i2c( address_start + 3);
    (*otp_ptr).production_day = ov4688_idol4s_read_i2c(address_start + 4);
    temp = ov4688_idol4s_read_i2c(address_start + 9);
    (*otp_ptr).rg_ratio = (ov4688_idol4s_read_i2c(address_start + 5)<<2) + ((temp>>6) & 0x03);
    (*otp_ptr).bg_ratio = (ov4688_idol4s_read_i2c(address_start + 6)<<2) + ((temp>>4) & 0x03);
    (*otp_ptr).light_rg = (ov4688_idol4s_read_i2c(address_start + 7) <<2) + ((temp>>2) & 0x03);
    (*otp_ptr).light_bg = (ov4688_idol4s_read_i2c(address_start + 8)<<2) + (temp & 0x03);
    (*otp_ptr).user_data[0] = ov4688_idol4s_read_i2c(address_start + 10);
    (*otp_ptr).user_data[1] = ov4688_idol4s_read_i2c(address_start + 11);
    (*otp_ptr).user_data[2] = ov4688_idol4s_read_i2c(address_start + 12);
    (*otp_ptr).user_data[3] = ov4688_idol4s_read_i2c(address_start + 13);
    (*otp_ptr).user_data[4] = ov4688_idol4s_read_i2c(address_start + 14);
    // clear otp buffer
    for (i=address_start;i<=address_end;i++) {
        ov4688_idol4s_write_i2c(i, 0x00);
    }

    ov4688_idol4s_otp_access_enable(0);

    ov4688_module_id = (*otp_ptr).module_integrator_id;
    if ((OV4688_IDOL4S_MODULE_ID == ov4688_module_id) || (OV4688_IDOL4S_2ND_MODULE_ID == ov4688_module_id)) {
        ov4688_module_id_magic = OV4688_MODULE_MAGIC_CODE;
    } else {
        ov4688_module_id_magic = 0;
    }

    pr_err("%s result: module_id: 0x%x lens_id: 0x%x production year/month/day, /%d/%d/%d \n", __func__,
        (*otp_ptr).module_integrator_id,
        (*otp_ptr).lens_id,
        (*otp_ptr).production_year,
        (*otp_ptr).production_month,
        (*otp_ptr).production_day);
    pr_err("%s result: rg: %d, bg: %d, light_rg: %d, light_bg: %d\n", __func__, 
        (*otp_ptr).rg_ratio,
        (*otp_ptr).bg_ratio,
        (*otp_ptr).light_rg,
        (*otp_ptr).light_bg);
    return 0;
}
#ifdef OTPDEBUG 
static int ov4688_idol4s_read_wbGain(int step, uint32_t start_addr)
{
    int iTmpLongH, iTmpLongL, iTmpMidH, iTmpMidL, iTmpLowH, iTmpLowL;
    iTmpLongH = ov4688_idol4s_read_i2c(start_addr);
    iTmpLongL = ov4688_idol4s_read_i2c(start_addr+1);
    iTmpMidH = ov4688_idol4s_read_i2c(start_addr+6);
    iTmpMidL = ov4688_idol4s_read_i2c(start_addr+7);
    iTmpLowH = ov4688_idol4s_read_i2c(start_addr+12);
    iTmpLowL = ov4688_idol4s_read_i2c(start_addr+13);
    pr_err("read wb gain info %d long H 0x%x L 0x%x !\n", step, iTmpLongH, iTmpLongL);
    pr_err("read wb gain info %d mid H 0x%x L 0x%x !\n", step, iTmpMidH, iTmpMidL);
    pr_err("read wb gain info %d low H 0x%x L 0x%x !\n", step, iTmpLowH, iTmpLowL);
    return 0;
}
#endif

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int ov4688_idol4s_update_awb_gain(int R_gain, int G_gain, int B_gain)
{
    pr_err("%s R_gain, 0x%x G_gain 0x%x, B_gain 0x%x!\n", __func__, R_gain, G_gain, B_gain);
    if (R_gain>0x400) {
        //long WB R gain
        ov4688_idol4s_write_i2c(0x500c, R_gain>>8);
        ov4688_idol4s_write_i2c(0x500d, R_gain & 0x00ff);
        //middle WB R gain
        ov4688_idol4s_write_i2c(0x5012, R_gain>>8);
        ov4688_idol4s_write_i2c(0x5013, R_gain & 0x00ff);
        //short WB R gain
        ov4688_idol4s_write_i2c(0x5018, R_gain>>8);
        ov4688_idol4s_write_i2c(0x5019, R_gain & 0x00ff);
    }

    if (G_gain>0x400) {
        //long WB G gain
        ov4688_idol4s_write_i2c(0x500e, G_gain>>8);
        ov4688_idol4s_write_i2c(0x500f, G_gain & 0x00ff);
        //middle WB G gain
        ov4688_idol4s_write_i2c(0x5014, G_gain>>8);
        ov4688_idol4s_write_i2c(0x5015, G_gain & 0x00ff);
        //short WB G gain
        ov4688_idol4s_write_i2c(0x501A, G_gain>>8);
        ov4688_idol4s_write_i2c(0x501B, G_gain & 0x00ff);
    }

    if (B_gain>0x400) {
        //long WB B gain
        ov4688_idol4s_write_i2c(0x5010, B_gain>>8);
        ov4688_idol4s_write_i2c(0x5011, B_gain & 0x00ff);
        //middle WB B gain
        ov4688_idol4s_write_i2c(0x5016, B_gain>>8);
        ov4688_idol4s_write_i2c(0x5017, B_gain & 0x00ff);
        //short WB B gain
        ov4688_idol4s_write_i2c(0x501C, B_gain>>8);
        ov4688_idol4s_write_i2c(0x501D, B_gain & 0x00ff);
    }

    return 0;
}

// call this function after OV4682/OV4688/OV4689 initialization
// return value: 0 update success
// 1, no OTP
int ov4688_idol4s_update_otp_wb(struct msm_camera_i2c_client *i2c_client)
{
    struct otp_struct current_otp;
    int i;
    int otp_index = 0xFF;
    int temp;
    int R_gain, G_gain, B_gain;
    int rg,bg;
    int nR_G_gain, nB_G_gain, nG_G_gain;
    int nBase_gain;

    ov4688_idol4s_g_client = i2c_client;
    pr_err("%s E Golden RG is %d, BG is %d\n", __func__, RG_RATIO_TYPICAL, BG_RATIO_TYPICAL);

    ov4688_idol4s_stream_set(1);

    // R/G and B/G of current camera module is read out from sensor OTP
    // check first OTP with valid data
    for(i=1; i<=3; i++) {
        temp = ov4688_idol4s_check_otp_wb(i);
        if (temp == 2) {
            otp_index = i;
            break;
        }
    }

    if (otp_index > 3) {
        // no valid wb OTP data
        pr_err("%s no valid OTP issue!! X!!\n", __func__);
        ov4688_idol4s_otp_restore_wb_auto(1);
        ov4688_idol4s_stream_set(0);
        return 1;
    } else {
        pr_err("%s valid OTP index is %d!!\n", __func__, otp_index);
    }

    ov4688_idol4s_read_otp_wb(otp_index, &current_otp);
    if (current_otp.light_rg==0) {
        // no light source information in OTP, light factor = 1
        rg = current_otp.rg_ratio;
    } else {
        rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
    }

    if(current_otp.light_bg==0) {
        // not light source information in OTP, light factor = 1
        bg = current_otp.bg_ratio;
    } else {
        bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
    }

    //calculate G gain
    nR_G_gain = (RG_RATIO_TYPICAL*1000) / rg;
    nB_G_gain = (BG_RATIO_TYPICAL*1000) / bg;
    nG_G_gain = 1000;
    if (nR_G_gain < 1000 || nB_G_gain < 1000) {
        if (nR_G_gain < nB_G_gain)
        nBase_gain = nR_G_gain;
        else
        nBase_gain = nB_G_gain;
    } else {
        nBase_gain = nG_G_gain;
    }
    R_gain = 0x400 * nR_G_gain / (nBase_gain);
    B_gain = 0x400 * nB_G_gain / (nBase_gain);
    G_gain = 0x400 * nG_G_gain / (nBase_gain);
    ov4688_idol4s_update_awb_gain(R_gain, G_gain, B_gain);
    pr_err("%s X\n", __func__);
    ov4688_idol4s_stream_set(0);

    return 0;
}

uint32_t ov4688_idol4s_get_module_id(struct msm_camera_i2c_client *i2c_client, uint16_t *module_id_ptr)
{
    struct otp_struct current_otp;
    int i;
    int temp;
    int otp_index = 0xFF;

    pr_err("%s E\n", __func__);
    if (OV4688_MODULE_MAGIC_CODE != ov4688_module_id_magic) {
        ov4688_idol4s_g_client = i2c_client;
        ov4688_idol4s_write_i2c_table(ov4688_init_setting);
    } else {
        *module_id_ptr = ov4688_module_id;
        pr_err("%s known module id 0x%x, module magic 0x%x!! X\n", __func__, ov4688_module_id, ov4688_module_id_magic);
        return ov4688_module_id_magic;
    }
    ov4688_idol4s_stream_set(1);

    for(i=1; i<=3; i++) {
        temp = ov4688_idol4s_check_otp_wb(i);
        if (temp == 2) {
            otp_index = i;
            break;
        }
    }

    if (otp_index > 3) {
        // no valid wb OTP data
        pr_err("%s no valid OTP!! X\n", __func__);
        ov4688_idol4s_otp_restore_wb_auto(1);
        ov4688_idol4s_stream_set(0);
        return 1;
    }

    ov4688_idol4s_read_otp_wb(otp_index, &current_otp);
    ov4688_idol4s_stream_set(0);

    *module_id_ptr = ov4688_module_id;
    pr_err("%s now get val module id 0x%x, module magic 0x%x!! X\n", __func__, ov4688_module_id, ov4688_module_id_magic);
    return ov4688_module_id_magic;
}
//After calibration applied, the R/G and B/G of camera module is same or very close to R/G and B/G
//of typical camera module. The lens correction output of camera module is same or very close to
//lens correction output of typical camera module. Also the BLC value will be better than common
//BLC value.nBase_gain = nG_G_gain;

