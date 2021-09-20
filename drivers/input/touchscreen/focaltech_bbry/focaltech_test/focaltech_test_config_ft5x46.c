/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Config_FT5X46.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: Set Config for FT5X46\FT5X46i\FT5526\FT3X17\FT5436\FT3X27\FT5526i\FT5416\FT5426\FT5435
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include "focaltech_test_config_ft5x46.h"
#include "focaltech_test_ini.h"
#include "focaltech_test_global.h"


struct stCfg_FT5X22_TestItem g_stCfg_FT5X22_TestItem;
struct stCfg_FT5X22_BasicThreshold g_stCfg_FT5X22_BasicThreshold;

void OnInit_FT5X22_TestItem(char *strIniFile)
{
	char str[512];

	//////////////////////////////////////////////////////////// FW Version
	GetPrivateProfileString("TestItem","FW_VERSION_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.FW_VERSION_TEST = fts_atoi(str);

	//////////////////////////////////////////////////////////// Factory ID
	GetPrivateProfileString("TestItem","FACTORY_ID_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.FACTORY_ID_TEST = fts_atoi(str);

	//////////////////////////////////////////////////////////// Project Code Test
	GetPrivateProfileString("TestItem","PROJECT_CODE_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.PROJECT_CODE_TEST = fts_atoi(str);	

	//////////////////////////////////////////////////////////// IC Version
	GetPrivateProfileString("TestItem","IC_VERSION_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.IC_VERSION_TEST = fts_atoi(str);

	//////////////////////////////////////////////////////////// LCM ID
	GetPrivateProfileString("TestItem","LCM_ID_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.LCM_ID_TEST = fts_atoi(str);

	/////////////////////////////////// RawData Test
	GetPrivateProfileString("TestItem","RAWDATA_TEST","1",str,strIniFile);
	g_stCfg_FT5X22_TestItem.RAWDATA_TEST = fts_atoi(str);

	/////////////////////////////////// Adc Detect Test
	GetPrivateProfileString("TestItem","ADC_DETECT_TEST","1",str,strIniFile);
	g_stCfg_FT5X22_TestItem.ADC_DETECT_TEST = fts_atoi(str);

	/////////////////////////////////// RX_SHORT_TEST
	//GetPrivateProfileString("TestItem","RX_SHORT_TEST","1",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.RX_SHORT_TEST = fts_atoi(str);

	/////////////////////////////////// TX_SHORT_TEST
	//GetPrivateProfileString("TestItem","TX_SHORT_TEST","1",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.TX_SHORT_TEST = fts_atoi(str);

	/////////////////////////////////// SCAP_CB_TEST
	GetPrivateProfileString("TestItem","SCAP_CB_TEST","1",str,strIniFile);
	g_stCfg_FT5X22_TestItem.SCAP_CB_TEST = fts_atoi(str);

	/////////////////////////////////// SCAP_RAWDATA_TEST
	GetPrivateProfileString("TestItem","SCAP_RAWDATA_TEST","1",str,strIniFile);
	g_stCfg_FT5X22_TestItem.SCAP_RAWDATA_TEST = fts_atoi(str);

	/////////////////////////////////// CHANNEL_NUM_TEST
	GetPrivateProfileString("TestItem","CHANNEL_NUM_TEST","1",str,strIniFile);
	g_stCfg_FT5X22_TestItem.CHANNEL_NUM_TEST = fts_atoi(str);

	/////////////////////////////////// INT_PIN_TEST
	GetPrivateProfileString("TestItem","INT_PIN_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.INT_PIN_TEST = fts_atoi(str);

	/////////////////////////////////// RESET_PIN_TEST
	GetPrivateProfileString("TestItem","RESET_PIN_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.RESET_PIN_TEST = fts_atoi(str);

	/////////////////////////////////// NOISE_TEST
	GetPrivateProfileString("TestItem","NOISE_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.NOISE_TEST = fts_atoi(str);

	/////////////////////////////////// SCAP_DIFFER_TEST
	//GetPrivateProfileString("TestItem","SCAP_DIFFER_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.SCAP_DIFFER_TEST = fts_atoi(str);

	/////////////////////////////////// SCAP_CLB_TEST
	//GetPrivateProfileString("TestItem","SCAP_CLB_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.SCAP_CLB_TEST = fts_atoi(str);

	/////////////////////////////////// RX_CROSSTALK_TEST
	//GetPrivateProfileString("TestItem","RX_CROSSTALK_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.RX_CROSSTALK_TEST = fts_atoi(str);

	/////////////////////////////////// RAWDATA_RX_DEVIATION_TEST
	//GetPrivateProfileString("TestItem","RAWDATA_RX_DEVIATION_TEST","0",str,strIniFile);
	//g_stCfg_FT5X36_TestItem.RAWDATA_RX_DEVIATION_TEST = fts_atoi(str);

	/////////////////////////////////// RAWDATA_UNIFORMITY_TEST
	//GetPrivateProfileString("TestItem","RAWDATA_UNIFORMITY_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.RAWDATA_UNIFORMITY_TEST = fts_atoi(str);

	/////////////////////////////////// RX_LINEARITY_TEST
	//GetPrivateProfileString("TestItem","RX_LINEARITY_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.RX_LINEARITY_TEST = fts_atoi(str);

	/////////////////////////////////// TX_LINEARITY_TEST
	//GetPrivateProfileString("TestItem","TX_LINEARITY_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.TX_LINEARITY_TEST = fts_atoi(str);

	/////////////////////////////////// DIFFERDATA_UNIFORMITY_TEST
	//GetPrivateProfileString("TestItem","DIFFERDATA_UNIFORMITY_TEST","0",str,strIniFile);
	//g_stCfg_FT5X22_TestItem.DIFFERDATA_UNIFORMITY_TEST = fts_atoi(str);

	/////////////////////////////////// WEAK_SHORT_CIRCUIT_TEST
	GetPrivateProfileString("TestItem","WEAK_SHORT_CIRCUIT_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.WEAK_SHORT_CIRCUIT_TEST = fts_atoi(str);

	/////////////////////////////////// UNIFORMITY_TEST
	GetPrivateProfileString("TestItem","UNIFORMITY_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.UNIFORMITY_TEST = fts_atoi(str);

	/////////////////////////////////// CM_TEST
	GetPrivateProfileString("TestItem","CM_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.CM_TEST = fts_atoi(str);

	/////////////////////////////////// RAWDATA_MARGIN_TEST
	GetPrivateProfileString("TestItem","RAWDATA_MARGIN_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.RAWDATA_MARGIN_TEST = fts_atoi(str);

	/////////////////////////////////// panel differ_TEST
	GetPrivateProfileString("TestItem","PANEL_DIFFER_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.PANEL_DIFFER_TEST = fts_atoi(str);

	/////////////////////////////////// panel differ uniformity_TEST
	GetPrivateProfileString("TestItem","PANEL_DIFFER_UNIFORMITY_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.PANEL_DIFFER_UNIFORMITY_TEST = fts_atoi(str);

	///////////////////////////////////TE_TEST
	GetPrivateProfileString("TestItem","TE_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.TE_TEST = fts_atoi(str);

	///////////////////////////////////SITO_RAWDATA_UNIFORMITY_TEST
	GetPrivateProfileString("TestItem","SITO_RAWDATA_UNIFORMITY_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.SITO_RAWDATA_UNIFORMITY_TEST = fts_atoi(str);

	///////////////////////////////////PATTERN_TEST
	GetPrivateProfileString("TestItem","PATTERN_TEST","0",str,strIniFile);
	g_stCfg_FT5X22_TestItem.PATTERN_TEST = fts_atoi(str);
}
void OnInit_FT5X22_BasicThreshold(char *strIniFile)
{
	char str[512];

	//////////////////////////////////////////////////////////// FW Version

	GetPrivateProfileString( "Basic_Threshold", "FW_VER_VALUE", "0",str, strIniFile);
	g_stCfg_FT5X22_BasicThreshold.FW_VER_VALUE = fts_atoi(str);

	//////////////////////////////////////////////////////////// Factory ID
	GetPrivateProfileString("Basic_Threshold","Factory_ID_Number","255",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Factory_ID_Number = fts_atoi(str);

	//////////////////////////////////////////////////////////// Project Code Test
	GetPrivateProfileString("Basic_Threshold","Project_Code"," ",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.Project_Code.Format("%s", str);
	sprintf(g_stCfg_FT5X22_BasicThreshold.Project_Code, "%s", str);

	//////////////////////////////////////////////////////////// IC Version
	GetPrivateProfileString("Basic_Threshold","IC_Version","3",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.IC_Version = fts_atoi(str);

	//////////////////////////////////////////////////////////// LCM ID
	GetPrivateProfileString("Basic_Threshold","LCM_ID","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.LCM_ID = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","RawDataTest_Low_Min","3000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_low_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","RawDataTest_Low_Max","15000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_Low_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","RawDataTest_High_Min","3000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_high_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","RawDataTest_High_Max","15000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_high_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","RawDataTest_LowFreq","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_SetLowFreq  = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","RawDataTest_HighFreq","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawDataTest_SetHighFreq = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","Adc_Detect_Max","0",str,strIniFile);
    g_stCfg_FT5X22_BasicThreshold.AdcDetect_Max = fts_atoi( str ); 

	//GetPrivateProfileString("Basic_Threshold","RxShortTest_Min","1000",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RxShortTest_Min = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","RxShortTest_Max","32800",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RxShortTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","TxShortTest_Min","300",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.TxShortTest_Min = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","TxShortTest_Max","3000",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.TxShortTest_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapCbTest_OFF_Min","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_OFF_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SCapCbTest_OFF_Max","240",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_OFF_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapCbTest_ON_Min","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_ON_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SCapCbTest_ON_Max","240",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_ON_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","ScapCBTest_SetWaterproof_OFF","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_SetWaterproof_OFF = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ScapCBTest_SetWaterproof_ON","240",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_SetWaterproof_ON = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapCBTest_LetTx_Disable","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapCbTest_LetTx_Disable = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","SCapDifferTest_Min","100",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.SCapDifferTest_Min = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","SCapDifferTest_Max","10000",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.SCapDifferTest_Max = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","SCapDifferTest_CbLevel","2",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.SCapDifferTest_CbLevel = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_OFF_Min","5000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_OFF_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_OFF_Max","8500",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_OFF_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_ON_Min","5000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_ON_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_ON_Max","8500",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_ON_Max = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_SetWaterproof_OFF","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_SetWaterproof_OFF = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_SetWaterproof_ON","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_SetWaterproof_ON = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","SCapRawDataTest_LetTx_Disable","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SCapRawDataTest_LetTx_Disable = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_Mapping","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bChannelTestMapping = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_NoMapping","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bChannelTestNoMapping = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_TxNum","13",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.ChannelNumTest_TxNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_RxNum","24",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.ChannelNumTest_RxNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_Tx_NP_Num","13",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.ChannelNumTest_TxNpNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","ChannelNumTest_Rx_NP_Num","24",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.ChannelNumTest_RxNpNum = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","ResetPinTest_RegAddr","136",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.ResetPinTest_RegAddr = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","IntPinTest_RegAddr","79",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.IntPinTest_RegAddr = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","IntPinTest_TestNum","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.IntPinTest_TestNum = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_Max","20",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_Max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_Frames","32",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_Frames = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_Time","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_Time = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","NoiseTest_SampeMode","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_SampeMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_NoiseMode","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_NoiseMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_ShowTip","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_ShowTip = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_GloveMode","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bNoiseTest_GloveMode = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_RawdataMin","5000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_RawdataMin = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","GloveNoiseTest_Coefficient","100",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.GloveNoiseTest_Coefficient = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","Set_Frequency","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Set_Frequency = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseThreshold_Choose","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bNoiseThreshold_Choose = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_Threshold","50",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_Threshold = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","NoiseTest_MinNGFrame","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.NoiseTest_MinNgFrame = fts_atoi(str);
	
	//GetPrivateProfileString("Basic_Threshold","SCapClbTest_Frame","3",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.SCapClbTest_Frame = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","SCapClbTest_Max","1000",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.SCapClbTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","RxCrosstalkTest_Min","-100",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RxCrosstalkTest_Min = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","RxCrosstalkTest_Max","300",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RxCrosstalkTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","RawDataRxDeviationTest_Max","500",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RawDataRxDeviationTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","RawDataUniformityTest_Percent","90",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RawDataUniformityTest_Percent = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","RxLinearityTest_Max","50",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.RxLinearityTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","TxLinearityTest_Max","50",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.TxLinearityTest_Max = fts_atoi(str);

	//GetPrivateProfileString("Basic_Threshold","DifferDataUniformityTest_Percent","80",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.DifferDataUniformityTest_Percent = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","WeakShortTest_CG","2000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.WeakShortTest_CG = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","WeakShortTest_CC","2000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.WeakShortTest_CC = fts_atoi(str);
	//GetPrivateProfileString("Basic_Threshold","WeakShortTest_ChannelNum","38",str,strIniFile);
	//g_stCfg_FT5X22_BasicThreshold.WeakShortTest_ChannelNum = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_Check_Tx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_CheckTx = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_Check_Rx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_CheckRx = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_Check_MinMax","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_CheckMinMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_Tx_Hole","20",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_Tx_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_Rx_Hole","20",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_Rx_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","UniformityTest_MinMax_Hole","70",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.Uniformity_MinMax_Hole = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","CMTest_Check_Min","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.CMTest_CheckMin = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CMTest_Check_Max","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.CMTest_CheckMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CMTest_Min_Hole","0.5",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.CMTest_MinHole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","CMTest_Max_Hole","5",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.CMTest_MaxHole = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","RawdataMarginTest_Min","10",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawdataMarginTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","RawdataMarginTest_Max","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.RawdataMarginTest_Max = fts_atoi(str);

	//panel differ
	GetPrivateProfileString("Basic_Threshold","PanelDifferTest_Min","150",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDifferTest_Min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDifferTest_Max","1000",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDifferTest_Max = fts_atoi(str);

	//panel differ uniformity
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_Check_Tx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_Check_Tx = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_Check_Rx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_Check_Rx = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_Check_MinMax","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_Check_MinMax = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_Tx_Hole","20",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_Tx_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_Rx_Hole","20",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_Rx_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PanelDiffer_UniformityTest_MinMax_Hole","70",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.PanelDiffer_UniformityTest_MinMax_Hole = fts_atoi(str);

	//RawdtaUniformityTest		
	GetPrivateProfileString("Basic_Threshold","SITO_RawdataUniformityTest_Check_Tx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Check_Tx = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SITO_RawdataUniformityTest_Check_Rx","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Check_Rx = fts_atoi(str);
	
	GetPrivateProfileString("Basic_Threshold","SITO_RawdataUniformityTest_Tx_Hole","10",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Tx_Hole = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","SITO_RawdataUniformityTest_Rx_Hole","10",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.SITO_RawdtaUniformityTest_Rx_Hole = fts_atoi(str); 

	//Pram Test
	GetPrivateProfileString("Basic_Threshold","PramTest_Check_00","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bPattern00 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PramTest_Check_FF","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bPatternFF = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PramTest_Check_55","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bPattern55 = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PramTest_Check_AA","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bPatternAA = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold","PramTest_Check_BIN","1",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.bPatternBin = fts_atoi(str);

	////////add frank. 20160414	{{
	GetPrivateProfileString("Basic_Threshold","WeakShortTest_CC_Rsen","57",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.WeakShortTest_CC_Rsen = fts_atoi(str);

	GetPrivateProfileString("Basic_Threshold","WeakShortTest_CapShortTest","0",str,strIniFile);
	g_stCfg_FT5X22_BasicThreshold.WeakShortTest_CapShortTest = fts_atoi(str);


	////////add frank. 20160414	}}
}

void SetTestItem_FT5X22()
{
	//int value = 0;
	g_TestItemNum = 0;


	//////////////////////////////////////////////////RESET_PIN_TEST
	if( g_stCfg_FT5X22_TestItem.RESET_PIN_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_RESET_PIN_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_RESET_PIN_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////FACTORY_ID_TEST
	if( g_stCfg_FT5X22_TestItem.FACTORY_ID_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_FACTORY_ID_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_FACTORY_ID_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////Project Code Test
	if( g_stCfg_FT5X22_TestItem.PROJECT_CODE_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_PROJECT_CODE_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_PROJECT_CODE_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////FW Version Test
	if( g_stCfg_FT5X22_TestItem.FW_VERSION_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_FW_VERSION_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_FW_VERSION_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////LCM ID Test
	if( g_stCfg_FT5X22_TestItem.LCM_ID_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_LCM_ID_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_LCM_ID_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////IC Version Test
	if( g_stCfg_FT5X22_TestItem.IC_VERSION_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_IC_VERSION_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_IC_VERSION_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////Enter Factory Mode
	g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_ENTER_FACTORY_MODE;
	//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_ENTER_FACTORY_MODE];
	g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
	g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
	g_TestItemNum++;
/*
#if ENABLE_DOUBLE_CHECK
	//////////////////////////////////////////////////judge Normalize type
	//if (g_ScreenSetParam.isNormalize != None)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_JUDEG_NORMALIZE_TYPE;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_JUDEG_NORMALIZE_TYPE];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;
	}
#endif
*/
	//////////////////////////////////////////////////TE_TEST
	if( g_stCfg_FT5X22_TestItem.TE_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_TE_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_TE_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////CHANNEL_NUM_TEST
	if( g_stCfg_FT5X22_TestItem.CHANNEL_NUM_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_CHANNEL_NUM_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_CHANNEL_NUM_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////NOISE_TEST
	if( g_stCfg_FT5X22_TestItem.NOISE_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_NOISE_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_NOISE_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////RawData Test
	if( g_stCfg_FT5X22_TestItem.RAWDATA_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_RAWDATA_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_RAWDATA_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////Rawdata Uniformity Test
	if( g_stCfg_FT5X22_TestItem.UNIFORMITY_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_UNIFORMITY_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_UNIFORMITY_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////Rawdata Uniformity Test
	if( g_stCfg_FT5X22_TestItem.SITO_RAWDATA_UNIFORMITY_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_SITO_RAWDATA_UNIFORMITY_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_SITO_RAWDATA_UNIFORMITY_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////CM Test
	if( g_stCfg_FT5X22_TestItem.CM_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_CM_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_CM_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}
	
	//////////////////////////////////////////////////Adc Detect Test
	if( g_stCfg_FT5X22_TestItem.ADC_DETECT_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_ADCDETECT_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_ADCDETECT_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////SCAP_CB_TEST
	if( g_stCfg_FT5X22_TestItem.SCAP_CB_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_SCAP_CB_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_SCAP_CB_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////SCAP_RAWDATA_TEST
	if( g_stCfg_FT5X22_TestItem.SCAP_RAWDATA_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_SCAP_RAWDATA_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_SCAP_RAWDATA_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////RAWDATA_MARGIN_TEST
	if( g_stCfg_FT5X22_TestItem.RAWDATA_MARGIN_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_RAWDATA_MARGIN_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_RAWDATA_MARGIN_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////WEAK_SHORT_CIRCUIT_TEST
	if( g_stCfg_FT5X22_TestItem.WEAK_SHORT_CIRCUIT_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_WEAK_SHORT_CIRCUIT_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_WEAK_SHORT_CIRCUIT_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}
	
	//////////////////////////////////////////////////panel differ_TEST
	if( g_stCfg_FT5X22_TestItem.PANEL_DIFFER_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_PANELDIFFER_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_PANELDIFFER_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}
	//////////////////////////////////////////////////panel differ uniformity
	if( g_stCfg_FT5X22_TestItem.PANEL_DIFFER_UNIFORMITY_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_PANELDIFFER_UNIFORMITY_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_PANELDIFFER_UNIFORMITY_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}

	//////////////////////////////////////////////////INT_PIN_TEST
	if( g_stCfg_FT5X22_TestItem.INT_PIN_TEST == 1)
	{
		g_stTestItem[0][g_TestItemNum].ItemCode = Code_FT5X22_INT_PIN_TEST;
		//g_stTestItem[0][g_TestItemNum].strItemName = g_strEnumTestItem_FT5X22[Code_FT5X22_INT_PIN_TEST];
		g_stTestItem[0][g_TestItemNum].TestNum = g_TestItemNum;
		g_stTestItem[0][g_TestItemNum].TestResult= RESULT_NULL;
		g_TestItemNum++;		
	}
}


