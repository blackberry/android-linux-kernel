/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
*
* File Name: Config_FT8607.h
*
* Author: Software Development Team, AE
*
* Created: 2016-03-15
*
* Abstract: Set Config for FT8607
*
************************************************************************/
#ifndef _CONFIG_FT8607_H
#define _CONFIG_FT8607_H

#include "focaltech_test_main.h"

struct stCfg_FT8607_TestItem
{
	bool FW_VERSION_TEST;
	bool FACTORY_ID_TEST;
	bool PROJECT_CODE_TEST;
	bool IC_VERSION_TEST;
	bool RAWDATA_TEST;
	bool CHANNEL_NUM_TEST;
	bool INT_PIN_TEST;
	bool RESET_PIN_TEST;
	bool NOISE_TEST;
	bool CB_TEST;
	bool SHORT_TEST;
	bool LCD_NOISE_TEST;
	bool OSC60MHZ_TEST;
	bool OSCTRM_TEST;
	bool SNR_TEST;
	bool LPWG_RAWDATA_TEST;
	bool LPWG_CB_TEST;
	bool LPWG_NOISE_TEST;
	bool DIFFER_TEST;
	bool DIFFER_UNIFORMITY_TEST;
};

struct stCfg_FT8607_BasicThreshold
{
	BYTE FW_VER_VALUE;
	BYTE Factory_ID_Number;
	char Project_Code[32];
	BYTE IC_Version;
	int RawDataTest_Min;
	int RawDataTest_Max;
	BYTE ChannelNumTest_ChannelXNum;
	BYTE ChannelNumTest_ChannelYNum;
	BYTE ChannelNumTest_KeyNum;
	BYTE ResetPinTest_RegAddr;
	BYTE IntPinTest_RegAddr;
	int NoiseTest_Coefficient;
	int NoiseTest_Frames;
	int NoiseTest_Time;
	BYTE NoiseTest_SampeMode;
	BYTE NoiseTest_NoiseMode;
	BYTE NoiseTest_ShowTip;
	BYTE IsDifferMode;
	bool bCBTest_VA_Check;
	int CbTest_Min;
	int CbTest_Max;
	bool bCBTest_VKey_Check;
	int CbTest_Min_Vkey;
	int CbTest_Max_Vkey;
	int ShortCircuit_ResMin;
	/*int ShortTest_Max;
	int ShortTest_K2Value;
	bool ShortTest_Tip;*/
	int iLCDNoiseTestFrame;
	int iLCDNoiseTestMax;

	int OSC60MHZTest_OSCMin;
	int OSC60MHZTest_OSCMax;

	int OSCTRMTest_OSCMin;
	int OSCTRMTest_OSCMax;
	int OSCTRMTest_OSCDetMin;
	int OSCTRMTest_OSCDetMax;
	int SNRTest_FrameNum;
	int SNRTest_Min;

	int DIFFERTest_FrameNum;
	int DIFFERTest_DifferMax;
	int DIFFERTest_DifferMin;

	bool DifferUniformityTest_Check_CHX;
	bool DifferUniformityTest_Check_CHY;
	bool DifferUniformityTest_Check_MinMax;
	int DifferUniformityTest_CHX_Hole;
	int DifferUniformityTest_CHY_Hole;
	int DifferUniformityTest_MinMax_Hole;

	int LPWG_RawDataTest_Min;
	int LPWG_RawDataTest_Max;

	bool bLPWG_CBTest_VA_Check;
	int LPWG_CbTest_Min;
	int LPWG_CbTest_Max;
	bool bLPWG_CBTest_VKey_Check;
	int LPWG_CbTest_Min_Vkey;
	int LPWG_CbTest_Max_Vkey;

	int LPWG_NoiseTest_Coefficient;
	int LPWG_NoiseTest_Frames;
	int LPWG_NoiseTest_Time;
	BYTE LPWG_NoiseTest_SampeMode;
	BYTE LPWG_NoiseTest_NoiseMode;
	BYTE LPWG_NoiseTest_ShowTip;
	BYTE LPWG_IsDifferMode;
};


enum enumTestItem_FT8607
{
	Code_FT8607_ENTER_FACTORY_MODE,//所有IC都必备的测试项
	Code_FT8607_DOWNLOAD,//所有IC都必备的测试项
	Code_FT8607_UPGRADE,//所有IC都必备的测试项
	Code_FT8607_FACTORY_ID_TEST,
	Code_FT8607_PROJECT_CODE_TEST,
	Code_FT8607_FW_VERSION_TEST,
	Code_FT8607_IC_VERSION_TEST,
	Code_FT8607_RAWDATA_TEST,
	Code_FT8607_CHANNEL_NUM_TEST,
	//Code_FT8607_CHANNEL_SHORT_TEST,
	Code_FT8607_INT_PIN_TEST,
	Code_FT8607_RESET_PIN_TEST,
	Code_FT8607_NOISE_TEST,
	Code_FT8607_CB_TEST,
	//Code_FT8607_DELTA_CB_TEST,
	//Code_FT8607_CHANNELS_DEVIATION_TEST,
	//Code_FT8607_TWO_SIDES_DEVIATION_TEST,
	//Code_FT8607_FPC_SHORT_TEST,
	//Code_FT8607_FPC_OPEN_TEST,
	//Code_FT8607_SREF_OPEN_TEST,
	//Code_FT8607_TE_TEST,
	//Code_FT8607_CB_DEVIATION_TEST,
	Code_FT8607_WRITE_CONFIG,//所有IC都必备的测试项
	//Code_FT8607_DIFFER_TEST,
	Code_FT8607_SHORT_CIRCUIT_TEST,
	Code_FT8607_LCD_NOISE_TEST,

	Code_FT8607_OSC60MHZ_TEST,
	Code_FT8607_OSCTRM_TEST,
	Code_FT8607_SNR_TEST,
	Code_FT8607_DIFFER_TEST,
	Code_FT8607_DIFFER_UNIFORMITY_TEST,

	Code_FT8607_LPWG_RAWDATA_TEST,
	Code_FT8607_LPWG_CB_TEST,
	Code_FT8607_LPWG_NOISE_TEST,
};

extern struct stCfg_FT8607_TestItem g_stCfg_FT8607_TestItem;
extern struct stCfg_FT8607_BasicThreshold g_stCfg_FT8607_BasicThreshold;
//extern CString g_strEnumTestItem_FT8607[];


//virtual void OnInit_FT8607_TestItem(CString strIniFile);
void OnInit_FT8607_TestItem(char*  strIniFile);
//virtual void OnInit_FT8607_BasicThreshold(CString strIniFile);
void OnInit_FT8607_BasicThreshold(char* strIniFile);

// 设置测试列表，测试过程按照列表顺序执行
void SetTestItem_FT8607(void);

#endif
