#ifndef OV4688_OTP_TCT_H
#define OV4688_OTP_TCT_H

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"

#define OV4688_VCM_EN           0
#define RG_RATIO_TYPICAL        258
#define BG_RATIO_TYPICAL        259

#define OTP_ISP_ENABLE_ADDR     0x5000
#define OTP_ISP_ENABLE_BIT      5

#define OV4688_IDOL4S_MODULE_ID     0x1
#define OV4688_IDOL4S_2ND_MODULE_ID 0x9999
#define OV4688_MODULE_MAGIC_CODE  0xA5A5ABCD

//Camera driver need to load AWB calibration data and lens correction setting stored in OTP and
//write to gain registers after initialization of register settings. After lens correction applied, the
//output of every camera module is same or very close to output of typical camera .
struct otp_struct {
    int module_integrator_id;
    int lens_id;
    int production_year;
    int production_month;
    int production_day;
    int rg_ratio;
    int bg_ratio;
    int light_rg;
    int light_bg;
    int user_data[5];
    int VCM_start;
    int VCM_end;
};

// call this function after OV4682/OV4688/OV4689 initialization
// return value: 0 update success
// 1, no OTP
int ov4688_idol4s_update_otp_wb(struct msm_camera_i2c_client *i2c_client);

uint32_t ov4688_idol4s_get_module_id(struct msm_camera_i2c_client *i2c_client, uint16_t *module_id_ptr);

#endif

