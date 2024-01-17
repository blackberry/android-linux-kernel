/* Copyright (C) 2016 Tcl Corporation Limited */
/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
xiaoming.hu@tcl.com for ido4 s5k4h8_krypton
* ------------------------------------------------------------------
*/


#include "msm_camera_i2c.h"


static struct msm_camera_i2c_client *s5k4h8_krypton_g_client;

struct s5k4h8_krypton_otp_struct {
	int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
	int module_integrator_id;
	int lens_id;
	int vcm_id;
	int driver_ic;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int lenc[240];
	int checksum;
	int VCM_start;
	int VCM_end;
	int VCM_dir;

};

static char s5k4h8_krypton_g_buf[64];
#if 1
void s5k4h8_krypton_check_otp_info(struct msm_camera_i2c_client *g_client);




static int32_t s5k4h8_krypton_write_i2c(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!s5k4h8_krypton_g_client)
        pr_err_ratelimited("FFFF OTP: s5k4h8_krypton client null\n");
    else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_write(s5k4h8_krypton_g_client,addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            pr_err_ratelimited("FFFF OTP: write error\n");
    }
    return rc;
}
#endif
static int32_t s5k4h8_krypton_write_i2c_word(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!s5k4h8_krypton_g_client)
        pr_err_ratelimited("FFFF OTP: s5k4h8_krypton client null\n");
    else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_write(s5k4h8_krypton_g_client,addr, data, MSM_CAMERA_I2C_WORD_DATA);
		if(rc < 0)
            pr_err_ratelimited("FFFF OTP: write error\n");
    }
    return rc;
}
static int32_t s5k4h8_krypton_write_i2c_byte(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!s5k4h8_krypton_g_client)
        pr_err_ratelimited("FFFF OTP: s5k4h8_krypton client null\n");
    else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_write(s5k4h8_krypton_g_client,addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            pr_err_ratelimited("FFFF OTP: write error\n");
    }
    return rc;
}
#if 0
static int16_t s5k4h8_krypton_read_i2c(uint32_t addr)
{
	uint16_t *data;
	uint16_t temp=0;
	int32_t rc = -EFAULT;
	data=&temp;
    if (!s5k4h8_krypton_g_client)
        pr_err("FFFF OTP: s5k4h8_krypton null\n");
    else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_read(s5k4h8_krypton_g_client,addr,data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            pr_err("FFFF OTP: read error\n");
    }
    return temp;
}



static int16_t s5k4h8_krypton_read_i2c_word(uint32_t addr)
{
	uint16_t *data;
	uint16_t temp=0;
	int32_t rc = -EFAULT;
	data=&temp;
	pr_err("xmhu %s",__func__);
    if (!s5k4h8_krypton_g_client)
        pr_err("FFFF OTP: s5k4h8_krypton null\n");
    else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_read(s5k4h8_krypton_g_client,addr,data, MSM_CAMERA_I2C_WORD_DATA);
		if(rc < 0)
            pr_err("FFFF OTP: read error\n");
    }
    return temp;
}
#endif

static void s5k4h8_krypton_read_i2c_seq(uint32_t addr,uint8_t *data, uint32_t num_byte)
{


	int32_t rc = -EFAULT;

	if (!s5k4h8_krypton_g_client)
		pr_err("FFFF OTP: s5k4h8_krypton null\n");
	else{
        rc = s5k4h8_krypton_g_client->i2c_func_tbl->i2c_read_seq(s5k4h8_krypton_g_client,addr,data,num_byte);
	if(rc < 0)
            pr_err("FFFF OTP: read error\n");
	}

}
#define KRYPTON_AWB_GROUP1_FLAG_ADDR 0x00
#define KRYPTON_AWB_GROUP2_FLAG_ADDR 0x20
#define KRYPTON_LSC_GROUP1_FLAG_ADDR 0x16
#define KRYPTON_LSC_GROUP2_FLAG_ADDR 0x36
static int s5k4h8_krypton_awb_addr_begin = 0;
static int s5k4h8_krypton_lsc_addr_begin = 0;


static void s5k4h8_krypton_find_valid_group(void)
{

	if((s5k4h8_krypton_g_buf[KRYPTON_AWB_GROUP1_FLAG_ADDR]) == 1){
		pr_err("OTP AWB Group_1 is valid!\n");
		s5k4h8_krypton_awb_addr_begin=0x0A;
	}else if((s5k4h8_krypton_g_buf[KRYPTON_AWB_GROUP2_FLAG_ADDR]) == 1){
		pr_err("OTP AWB Group_2 is valid!\n");
		s5k4h8_krypton_awb_addr_begin=0x2A;
	}else{
		s5k4h8_krypton_awb_addr_begin = 0;
		pr_err("OTP AWB is invalid!\n");
	}



	if((s5k4h8_krypton_g_buf[KRYPTON_LSC_GROUP1_FLAG_ADDR]) == 1){
		pr_err("OTP LSC Group_1 is valid!\n");
		s5k4h8_krypton_lsc_addr_begin = 1;

		}
	else if ((s5k4h8_krypton_g_buf[KRYPTON_LSC_GROUP2_FLAG_ADDR]) == 1){
		pr_err("OTP LSC Group_2 is valid!\n");
		s5k4h8_krypton_lsc_addr_begin = 2;

	}else{
		s5k4h8_krypton_lsc_addr_begin = 0;
		pr_err("OTP LSC is invalid!\n");

	/* MODIFIED-BEGIN by guangliang.luo, 2016-07-29,BUG-2633773*/
	}

}
#if 1
static void s5k4h8_krypton_write_awb(void)
{
	static int RG_ratio_Typical = 0x277;//0x264;//612
	static int BG_ratio_Typical = 0x20d;//0x243;//583

	int rg_h = s5k4h8_krypton_awb_addr_begin;
	int rg_l = s5k4h8_krypton_awb_addr_begin + 1;
	int bg_h = s5k4h8_krypton_awb_addr_begin + 2;
	int bg_l = s5k4h8_krypton_awb_addr_begin + 3;
	int R_ration=0,B_ration=0;
	int G_gain=0,R_gain=0,B_gain=0;
	int RG_Ratio=0,BG_Ratio=0;
	int GAIN_default = 0x0100;
	if(s5k4h8_krypton_awb_addr_begin)
	{
	  RG_Ratio = (s5k4h8_krypton_g_buf[rg_h]<<8) + s5k4h8_krypton_g_buf[rg_l];
	  BG_Ratio = (s5k4h8_krypton_g_buf[bg_h]<<8) + s5k4h8_krypton_g_buf[bg_l];
          pr_err("OTP RG_ratio_Typical = 0x%x,BG_ratio_Typical=0x%x\n",RG_ratio_Typical,BG_ratio_Typical);
	  pr_err("OTP RG_Ratio = 0x%x,BG_Ratio=0x%x\n",RG_Ratio,BG_Ratio);
	  if(RG_ratio_Typical && BG_ratio_Typical){

		R_ration = 512*RG_Ratio / RG_ratio_Typical;
		B_ration = 512*BG_Ratio /BG_ratio_Typical;
	        if(R_ration >= 512 ){
		     if(B_ration >= 512){
			G_gain = GAIN_default ;
			R_gain = (uint16_t)(GAIN_default  * R_ration/512);
			B_gain = (uint16_t)(GAIN_default  * B_ration/512);
		     }
		     else{
			B_gain = GAIN_default;
			R_gain =  (uint16_t)(GAIN_default *R_ration / B_ration );
			G_gain = (uint16_t)(GAIN_default  *512/ B_ration );
		     }
	        }

	        else{
		     if(B_ration >= 512){
			R_gain = GAIN_default;
			G_gain = (uint16_t)(GAIN_default *512/R_ration);
			B_gain =  (uint16_t)(GAIN_default *B_ration / R_ration );
		     }else{
			if( R_ration >= B_ration ){
				B_gain = GAIN_default;
				G_gain = (uint16_t)(GAIN_default *512 / B_ration );
				R_gain =  (uint16_t)(GAIN_default *R_ration  / B_ration);
			}else{
				R_gain = GAIN_default;
				G_gain = (uint16_t)(GAIN_default  *512/ R_ration );
				B_gain = (uint16_t)(GAIN_default *B_ration / R_ration );
			}
		     }

	       }

		pr_err("R_gain=0x%x, B_gain=0x%x, G_gain=0x%x\n", R_gain, B_gain,G_gain);
          }
	  s5k4h8_krypton_write_i2c_word(0x6028,0x4000);
	  s5k4h8_krypton_write_i2c_word(0x602a,0x3058);
	  s5k4h8_krypton_write_i2c_byte(0x6f12,0x01);

	  s5k4h8_krypton_write_i2c_word(0x602A,0x020E);
	  s5k4h8_krypton_write_i2c_word(0x6F12,G_gain);

	  s5k4h8_krypton_write_i2c_word(0x602A,0x0210);
	  s5k4h8_krypton_write_i2c_word(0x6F12,R_gain);

	  s5k4h8_krypton_write_i2c_word(0x602A,0x0212);
	  s5k4h8_krypton_write_i2c_word(0x6F12,B_gain);

	  s5k4h8_krypton_write_i2c_word(0x602A,0x0214);
	  s5k4h8_krypton_write_i2c_word(0x6F12,G_gain);
        }
}
#endif
static void s5k4h8_krypton_write_lsc(void)
{

	pr_err(" write lsc before stream on %s\n",__func__);
	//s5k4h8_krypton_write_i2c_word(0x0B00,0x0180); // MODIFIED by guangliang.luo, 2016-07-29,BUG-2633773
}

void s5k4h8_krypton_check_otp_info(struct msm_camera_i2c_client *g_client)
{

	//uint16_t data[64];
	//uint16_t id = 0;
	s5k4h8_krypton_g_client = g_client;
	pr_err("%s\n",__func__);
       // s5k4h8_krypton_write_lsc();


	s5k4h8_krypton_write_lsc();
	s5k4h8_krypton_write_i2c_word(0x0100, 0x0103);
        mdelay(5);

	s5k4h8_krypton_write_i2c(0x0a02, 0x0f);
	mdelay(5);
	s5k4h8_krypton_write_i2c(0x0a00, 0x01);
	mdelay(5);
	s5k4h8_krypton_read_i2c_seq(0xa04,s5k4h8_krypton_g_buf,64);
	mdelay(1);
	pr_err("parse start\n");
	s5k4h8_krypton_find_valid_group();
	s5k4h8_krypton_write_awb();

        s5k4h8_krypton_write_i2c(0x0a00, 0x00);

	mdelay(1);
	s5k4h8_krypton_write_i2c_word(0x0100, 0x000);



}




