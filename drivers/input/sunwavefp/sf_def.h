#ifndef __SF_DEF_H__
#define __SF_DEF_H__

//-----------------------------------------------------------------------------
// platform lists
//-----------------------------------------------------------------------------
/*************************************************
SF_REE_MTK          ������ƽ̨��android6���ϰ汾
SF_REE_QUALCOMM     ��ͨƽ̨
SF_REE_SPREAD       չѶƽ̨
SF_REE_HIKEY9600    ��Ϊ����960
SF_REE_MTK_L5_X     ������ƽ̨��android5�汾

SF_TEE_BEANPOD      ����TEE
SF_TEE_TRUSTKERNEL  �J�jTEE
SF_TEE_QSEE         ��ͨTEE
SF_TEE_TRUSTONIC    trustonic TEE
SF_TEE_RONGCARD     �ڿ�TEE
SF_TEE_TRUSTY       չѶTEE
*************************************************/

#define SF_REE_MTK                  1
#define SF_REE_QUALCOMM             2
#define SF_REE_SPREAD               3
#define SF_REE_HIKEY9600            4
#define SF_REE_MTK_L5_X             5

#define SF_TEE_BEANPOD              80
#define SF_TEE_TRUSTKERNEL          81
#define SF_TEE_QSEE                 82
#define SF_TEE_TRUSTONIC            83
#define SF_TEE_RONGCARD             84
#define SF_TEE_TRUSTY               85

//-----------------------------------------------------------------------------
// COMPATIBLE mode lists
#define SF_COMPATIBLE_NOF           0
#define SF_COMPATIBLE_NOF_BP_V2_7   1
#define SF_COMPATIBLE_REE           100
#define SF_COMPATIBLE_BEANPOD_V1    200
#define SF_COMPATIBLE_BEANPOD_V2    201
#define SF_COMPATIBLE_BEANPOD_V2_7  202
#define SF_COMPATIBLE_TRUSTKERNEL   300
#define SF_COMPATIBLE_QSEE          400
#define SF_COMPATIBLE_TRUSTY        500
#define SF_COMPATIBLE_RONGCARD      600
#define SF_COMPATIBLE_TRUSTONIC     700

//-----------------------------------------------------------------------------
// vdd power mode lists
#define PWR_MODE_NOF                0
#define PWR_MODE_GPIO               1
#define PWR_MODE_REGULATOR          2


#endif
