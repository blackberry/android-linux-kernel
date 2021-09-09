/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Config_FT8607.c
*
* Author: Software Development Team, AE
*
* Created: 2016-03-15
*
* Abstract: Set Config for FT8607
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>

//#include "Config_FT8607.h"
//#include "ini.h"
//#include "Global.h"

#include "focaltech_test_config_ft8607.h"
#include "focaltech_test_ini.h"
#include "focaltech_test_global.h"


/*
CString g_strEnumTestItem_FT8607[] =
{	
	" Enter Factory Mode",
	" Download",
	" Upgrade",
	" Factory ID Test",
	" Project Code Test",
	" FW Version Test",
	" IC Version Test",
	" RawData Test",
	" Channel Num Test",
	//" Channel Short Test",
	" Int Pin Test",
	" Reset pin Test",
	" Noise Test",
	" CB Test",
	//" Delta CB Test",
	//" Channels Deviation Test",
	//" Two Sides Deviation Test",
	//" FPCBA Short Test",
	//" FPCBA Open Test",
	//" SREF Open Test",
	//" TE Test",
	//" CB Deviation Test",
	" Write Config",
	//" Differ Test",
	" Short Circuit Test",
	" LCD Noise Test",
	" OSC60MHZ Test",
	" OSCTRM Test",
	" SNR Test",
	" Differ Test",
	" Differ Uniformity Test",
	" LPWG RawData Test",
	" LPWG CB Test",
	" LPWG Noise Test",
};
*/
	
struct stCfg_FT8607_TestItem g_stCfg_FT8607_TestItem;
struct stCfg_FT8607_BasicThreshold g_stCfg_FT8607_BasicThreshold;


void OnInit_FT8607_BasicThreshold(char* strIniFile)
{
	char str[512] = {0};

	FTS_TEST_DBG("");

	//////////////////////////////////////////////////////////// FW Version

	GetPrivateProfileString( "Basic_Threshold", "FW_VER_VALUE", "0",str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.FW_VER_VALUE = fts_atoi(str);

	//////////////////////////////////////////////////////////// Factory ID
	GetPrivateProfileString("Basic_Threshold","Factory_ID_Number","255",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.Factory_ID_Number = fts_atoi(str);

	//////////////////////////////////////////////////////////// Project Code Test
	GetPrivateProfileString("Basic_Threshold","Project_Code"," ",str,strIniFile);
	sprintf(g_stCfg_FT8607_BasicThreshold.Project_Code, "%s", str);

	//////////////////////////////////////////////////////////// IC Version
	GetPrivateProfileString("Basic_Threshold","IC_Version","3",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.IC_Version = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","RawDataTest_Min","5000",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.RawDataTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","RawDataTest_Max","11000",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.RawDataTest_Max = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_ChannelX","15",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ChannelNumTest_ChannelXNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_ChannelY","24",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ChannelNumTest_ChannelYNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_KeyNum","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ChannelNumTest_KeyNum = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","ResetPinTest_RegAddr","136",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ResetPinTest_RegAddr = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","IntPinTest_RegAddr","175",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.IntPinTest_RegAddr = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_Coefficient","50",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_Coefficient = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_Frames","32",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_Frames = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_Time","1",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_Time = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_SampeMode","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_SampeMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_NoiseMode","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_NoiseMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_ShowTip","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.NoiseTest_ShowTip = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_IsDiffer","1",str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.IsDifferMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold", "CBTest_VA_Check", "1", str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.bCBTest_VA_Check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CBTest_Min","3",str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.CbTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CBTest_Max","60",str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.CbTest_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CBTest_VKey_Check","1",str, strIniFile);
	g_stCfg_FT8607_BasicThreshold.bCBTest_VKey_Check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CBTest_Min_Vkey","3",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.CbTest_Min_Vkey = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CBTest_Max_Vkey","100",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.CbTest_Max_Vkey = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","ShortCircuit_ResMin","200",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ShortCircuit_ResMin = fts_atoi(str);   
	/*::GetPrivateProfileString("Basic_Threshold","ShortCircuit_CBMax","120",str.GetBuffer(MAX_PATH),MAX_PATH,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ShortTest_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ShortCircuit_K2Value","150",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ShortTest_K2Value = fts_atoi(str);
	::GetPrivateProfileString("Basic_Threshold","ShortCircuit_Tip","0",str.GetBuffer(MAX_PATH),MAX_PATH,strIniFile);
	g_stCfg_FT8607_BasicThreshold.ShortTest_Tip = fts_atoi(str);*/   

	GetPrivateProfileString("Basic_Threshold","LCD_NoiseTest_Frame","50",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestFrame = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LCD_NoiseTest_Max","30",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestMax = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","OSC60MHZTest_OSCMin","12",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSC60MHZTest_OSCMin = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","OSC60MHZTest_OSCMax","17",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSC60MHZTest_OSCMax = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","OSCTRMTest_OSCMin","15",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSCTRMTest_OSCMin = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","OSCTRMTest_OSCMax","17",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSCTRMTest_OSCMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","OSCTRMTest_OSCDetMin","15",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSCTRMTest_OSCDetMin = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","OSCTRMTest_OSCDetMax","17",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.OSCTRMTest_OSCDetMax = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SNRTest_FrameNum","32",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.SNRTest_FrameNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SNRTest_Min","10",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.SNRTest_Min = fts_atoi(str);

	//differ
	GetPrivateProfileString("Basic_Threshold","DIFFERTest_Frame_Num","32",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DIFFERTest_FrameNum = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","DIFFERTest_Differ_Max","100",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DIFFERTest_DifferMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DIFFERTest_Differ_Min","10",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DIFFERTest_DifferMin = fts_atoi(str);

	//differ_uniformity
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_Check_CHX","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_Check_CHX = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_Check_CHY","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_Check_CHY = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_Check_MinMax","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_Check_MinMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_CHX_Hole","20",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_CHX_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_CHY_Hole","20",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_CHY_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","DifferUniformityTest_MinMax_Hole","70",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.DifferUniformityTest_MinMax_Hole = fts_atoi(str);


	GetPrivateProfileString("Basic_Threshold","LPWG_RawDataTest_Min","5000",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_RawDataTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_RawDataTest_Max","11000",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_RawDataTest_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_VA_Check","1",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.bLPWG_CBTest_VA_Check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_Min","3",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_CbTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_Max","60",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_CbTest_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_VKey_Check","1",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.bLPWG_CBTest_VKey_Check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_Min_Vkey","3",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_CbTest_Min_Vkey = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_CBTest_Max_Vkey","100",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_CbTest_Max_Vkey = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_Coefficient","50",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_Coefficient = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_Frames","32",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_Frames = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_Time","1",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_Time = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_SampeMode","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_SampeMode = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_NoiseMode","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_NoiseMode = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_ShowTip","0",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_NoiseTest_ShowTip = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","LPWG_NoiseTest_IsDiffer","1",str,strIniFile);
	g_stCfg_FT8607_BasicThreshold.LPWG_IsDifferMode = fts_atoi(str);
}

void OnInit_FT8607_TestItem(char*  strIniFile)
{
	char str[512] = {0};

	FTS_TEST_DBG("");

	//////////////////////////////////////////////////////////// FW Version
	GetPrivateProfileString("TestItem","FW_VERSION_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.FW_VERSION_TEST = fts_atoi(str);

	//////////////////////////////////////////////////////////// Factory ID
	GetPrivateProfileString("TestItem","FACTORY_ID_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.FACTORY_ID_TEST = fts_atoi(str);

	//////////////////////////////////////////////////////////// Project Code Test
	GetPrivateProfileString("TestItem","PROJECT_CODE_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.PROJECT_CODE_TEST = fts_atoi(str);	

	//////////////////////////////////////////////////////////// IC Version
	GetPrivateProfileString("TestItem","IC_VERSION_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.IC_VERSION_TEST = fts_atoi(str);

	/////////////////////////////////// RawData Test
	GetPrivateProfileString("TestItem","RAWDATA_TEST","1",str,strIniFile);
	g_stCfg_FT8607_TestItem.RAWDATA_TEST = fts_atoi(str);

	/////////////////////////////////// CHANNEL_NUM_TEST
	GetPrivateProfileString("TestItem","CHANNEL_NUM_TEST","1",str,strIniFile);
	g_stCfg_FT8607_TestItem.CHANNEL_NUM_TEST = fts_atoi(str);

	/////////////////////////////////// INT_PIN_TEST
	GetPrivateProfileString("TestItem","INT_PIN_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.INT_PIN_TEST = fts_atoi(str);

	/////////////////////////////////// RESET_PIN_TEST
	GetPrivateProfileString("TestItem","RESET_PIN_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.RESET_PIN_TEST = fts_atoi(str);

	/////////////////////////////////// NOISE_TEST
	GetPrivateProfileString("TestItem","NOISE_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.NOISE_TEST = fts_atoi(str);

	/////////////////////////////////// CB_TEST
	GetPrivateProfileString("TestItem","CB_TEST","1",str,strIniFile);
	g_stCfg_FT8607_TestItem.CB_TEST = fts_atoi(str);

	/////////////////////////////////// SHORT_CIRCUIT_TEST
	GetPrivateProfileString("TestItem","SHORT_CIRCUIT_TEST","1",str,strIniFile);
	g_stCfg_FT8607_TestItem.SHORT_TEST = fts_atoi(str);

	/////////////////////////////////// LCD_NOISE_TEST
	GetPrivateProfileString("TestItem","LCD_NOISE_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.LCD_NOISE_TEST = fts_atoi(str);

	/////////////////////////////////// OSC60MHZ_TEST
	GetPrivateProfileString("TestItem","OSC60MHZ_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.OSC60MHZ_TEST = fts_atoi(str);

	/////////////////////////////////// OSCTRM_TEST
	GetPrivateProfileString("TestItem","OSCTRM_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.OSCTRM_TEST = fts_atoi(str);

	/////////////////////////////////// SNR_TEST
	GetPrivateProfileString("TestItem","SNR_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.SNR_TEST = fts_atoi(str);

	/////////////////////////////////// Differ_TEST	   
	GetPrivateProfileString("TestItem","DIFFER_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.DIFFER_TEST = fts_atoi(str);

	/////////////////////////////////// DIFFER_UNIFORMITY_TEST	   
	GetPrivateProfileString("TestItem","DIFFER_UNIFORMITY_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.DIFFER_UNIFORMITY_TEST = fts_atoi(str);

	/////////////////////////////////// LPWG_RAWDATA_TEST
	GetPrivateProfileString("TestItem","LPWG_RAWDATA_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.LPWG_RAWDATA_TEST = fts_atoi(str);

	/////////////////////////////////// LPWG_CB_TEST
	GetPrivateProfileString("TestItem","LPWG_CB_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.LPWG_CB_TEST = fts_atoi(str);

	/////////////////////////////////// LPWG_NOISE_TEST
	GetPrivateProfileString("TestItem","LPWG_NOISE_TEST","0",str,strIniFile);
	g_stCfg_FT8607_TestItem.LPWG_NOISE_TEST = fts_atoi(str);
}

void SetTestItem_FT8607(void)
{
//	int value = 0;

	FTS_TEST_DBG("");

	//////////////////////////////////////////////////FACTORY_ID_TEST
	if( g_stCfg_FT8607_TestItem.FACTORY_ID_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_FACTORY_ID_TEST);
	}

	//////////////////////////////////////////////////Project Code Test
	if( g_stCfg_FT8607_TestItem.PROJECT_CODE_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_PROJECT_CODE_TEST);
	}

	//////////////////////////////////////////////////FW Version Test
	if( g_stCfg_FT8607_TestItem.FW_VERSION_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_FW_VERSION_TEST);
	}

	//////////////////////////////////////////////////IC Version Test
	if( g_stCfg_FT8607_TestItem.IC_VERSION_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_IC_VERSION_TEST);
	}

	//////////////////////////////////////////////////Enter Factory Mode
	fts_SetTestItemCodeName(Code_FT8607_ENTER_FACTORY_MODE);

	//////////////////////////////////////////////////CHANNEL_NUM_TEST
	if( g_stCfg_FT8607_TestItem.CHANNEL_NUM_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_CHANNEL_NUM_TEST);
	}

	//////////////////////////////////////////////////Short Test
	if( g_stCfg_FT8607_TestItem.SHORT_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_SHORT_CIRCUIT_TEST);
	}

	//////////////////////////////////////////////////OSC60MHZ_TEST
	if( g_stCfg_FT8607_TestItem.OSC60MHZ_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_OSC60MHZ_TEST);
	}

	//////////////////////////////////////////////////OSCTRM_TEST
	if( g_stCfg_FT8607_TestItem.OSCTRM_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_OSCTRM_TEST);
	}

	//////////////////////////////////////////////////CB_TEST
	if( g_stCfg_FT8607_TestItem.CB_TEST == 1)
	{
		FTS_TEST_DBG(" Code_FT8607_CB_TEST.");

		fts_SetTestItemCodeName(Code_FT8607_CB_TEST);
	}

	//////////////////////////////////////////////////NOISE_TEST
	if( g_stCfg_FT8607_TestItem.NOISE_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_NOISE_TEST);
	}

	//////////////////////////////////////////////////LCD_NOISE_TEST
	if( g_stCfg_FT8607_TestItem.LCD_NOISE_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_LCD_NOISE_TEST);
	}

	//////////////////////////////////////////////////RawData Test
	if( g_stCfg_FT8607_TestItem.RAWDATA_TEST == 1)
	{
		FTS_TEST_DBG(" Code_FT8607_RAWDATA_TEST.");
		fts_SetTestItemCodeName(Code_FT8607_RAWDATA_TEST);
	}

	//////////////////////////////////////////////////SNR Test
	if( g_stCfg_FT8607_TestItem.SNR_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_SNR_TEST);
	}

	//////////////////////////////////////////////////Differ_TEST
	if( g_stCfg_FT8607_TestItem.DIFFER_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_DIFFER_TEST);
	}

	//////////////////////////////////////////////////DIFFER_UNIFORMITY_TEST
	if( g_stCfg_FT8607_TestItem.DIFFER_UNIFORMITY_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_DIFFER_UNIFORMITY_TEST);
	}

	//////////////////////////////////////////////////RESET_PIN_TEST
	if( g_stCfg_FT8607_TestItem.RESET_PIN_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_RESET_PIN_TEST);
	}
	//////////////////////////////////////////////////INT_PIN_TEST
	if( g_stCfg_FT8607_TestItem.INT_PIN_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_INT_PIN_TEST);
	}

	//////////////////////////////////////////////////LPWG_CB_TEST
	if( g_stCfg_FT8607_TestItem.LPWG_CB_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_LPWG_CB_TEST);
	}

	//////////////////////////////////////////////////LPWG_RAWDATA_TEST
	if( g_stCfg_FT8607_TestItem.LPWG_RAWDATA_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_LPWG_RAWDATA_TEST);
	}

	//////////////////////////////////////////////////LPWG_NOISE_TEST
	if( g_stCfg_FT8607_TestItem.LPWG_NOISE_TEST == 1)
	{
		fts_SetTestItemCodeName(Code_FT8607_LPWG_NOISE_TEST);
	}
}

