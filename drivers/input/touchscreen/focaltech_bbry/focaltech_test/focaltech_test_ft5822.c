/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: Test_FT5822.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT5822\FT5626\FT5726\FT5826B
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "focaltech_test_global.h"
#include "focaltech_test_detail_threshold.h"
#include "focaltech_test_ft5822.h"
#include "focaltech_test_config_ft5822.h"
//#include "Comm_FT5822.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define IC_TEST_VERSION  "Test version: V1.0.0--2015-07-30, (sync version of FT_MultipleTest: V2.7.0.3--2015-07-13)"

/*buff length*/
#define BUFF_LEN_STORE_MSG_AREA		1024*10
#define BUFF_LEN_MSG_AREA_LINE2		1024*4
#define BUFF_LEN_STORE_DATA_AREA		1024*80
#define BUFF_LEN_TMP_BUFFER 			1024*16

/////////////////////////////////////////////////Reg 
#define DEVIDE_MODE_ADDR	0x00
#define REG_LINE_NUM	0x01
#define REG_TX_NUM	0x02
#define REG_RX_NUM	0x03
#define REG_PATTERN_5422        0x53
#define REG_MAPPING_SWITCH      0x54
#define REG_TX_NOMAPPING_NUM        0x55
#define REG_RX_NOMAPPING_NUM      0x56
#define REG_NORMALIZE_TYPE      0x16
#define REG_ScCbBuf0	0x4E
#define REG_ScWorkMode	0x44
#define REG_ScCbAddrR	0x45
#define REG_RawBuf0 0x36
#define REG_WATER_CHANNEL_SELECT 0x09

#define REG_FREQUENCY           0x0A
#define REG_FIR                 0XFB


/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum WaterproofType
{
	WT_NeedProofOnTest,
	WT_NeedProofOffTest,
	WT_NeedTxOnVal,
	WT_NeedRxOnVal,
	WT_NeedTxOffVal,
	WT_NeedRxOffVal,
};
/*******************************************************************************
* Static variables
*******************************************************************************/

static int m_RawData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};
static unsigned char m_ucTempData[TX_NUM_MAX * RX_NUM_MAX*2] = {0};
static bool m_bV3TP = false;
static int RxLinearity[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int TxLinearity[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};


//---------------------About Store Test Dat
//static char g_pStoreAllData[1024*80] = {0};
static char *g_pTmpBuff = NULL;
static char *g_pStoreMsgArea = NULL;
static int g_lenStoreMsgArea = 0;
static char *g_pMsgAreaLine2 = NULL;
static int g_lenMsgAreaLine2 = 0;
static char *g_pStoreDataArea = NULL;
static int g_lenStoreDataArea = 0;
static unsigned char m_ucTestItemCode = 0;
static int m_iStartLine = 0;
static int m_iTestDataCount = 0;

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/


/*******************************************************************************
* Static function prototypes
*******************************************************************************/
//////////////////////////////////////////////Communication function
static int StartScan(void);
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer);
static unsigned char GetPanelRows(unsigned char *pPanelRows);
static unsigned char GetPanelCols(unsigned char *pPanelCols);
static unsigned char GetTxSC_CB(unsigned char index, unsigned char *pcbValue);
//////////////////////////////////////////////Common function
static unsigned char GetRawData(void);
static unsigned char GetChannelNum(void);
//////////////////////////////////////////////about Test
static int InitTest(void);
static void FinishTest(void);
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount);
static void InitStoreParamOfTestData(void);
static void MergeAllTestData(void);
//////////////////////////////////////////////Others 
static int AllocateMemory(void);
static void FreeMemory(void);
static void ShowRawData(void);
static boolean GetTestCondition(int iTestType, unsigned char ucChannelValue);

static unsigned char GetChannelNumNoMapping(void);
static unsigned char SwitchToNoMapping(void);

/************************************************************************
* Name: FT5822_StartTest
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
boolean FT5822_StartTest()
{
	bool bTestResult = true;
	bool bTempResult = 1;
	unsigned char ReCode=0;
	unsigned char ucDevice = 0;
	int iItemCount=0;
	
	//--------------1. Init part
	if(InitTest() < 0)
	{
		FTS_TEST_DBG("[focal] Failed to init test.");
		return false;
	}
	
	//--------------2. test item
	if(0 == g_TestItemNum)
		bTestResult = false;
	
	for(iItemCount = 0; iItemCount < g_TestItemNum; iItemCount++)
	{
		m_ucTestItemCode = g_stTestItem[ucDevice][iItemCount].ItemCode;

		///////////////////////////////////////////////////////FT5822_ENTER_FACTORY_MODE
		if(Code_FT5822_ENTER_FACTORY_MODE == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5822_TestItem_EnterFactoryMode();
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT5822_CHANNEL_NUM_TEST
		/*if(Code_FT5822_CHANNEL_NUM_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
		)
		{
		ReCode = FT5822_TestItem_ChannelsTest(&bTempResult);
		if(ERROR_CODE_OK != ReCode || (!bTempResult))
		{
		bTestResult = false;
		}
		}*/	

		///////////////////////////////////////////////////////FT5822_RAWDATA_TEST
		if(Code_FT5822_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5822_TestItem_RawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}


		///////////////////////////////////////////////////////FT5822_SCAP_CB_TEST
		if(Code_FT5822_SCAP_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5822_TestItem_SCapCbTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT5822_SCAP_RAWDATA_TEST
		if(Code_FT5822_SCAP_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT5822_TestItem_SCapRawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT5X22_UNIFORMITY_TEST
		if(Code_FT5822_UNIFORMITY_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode			
			)
		{			
			ReCode = FT5822_TestItem_UniformityTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}


	}

	//--------------3. End Part
	FinishTest();
	
	//--------------4. return result
	return bTestResult;
}
/************************************************************************
* Name: InitTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
static int InitTest(void)
{
	int ret = 0;
	ret = AllocateMemory();//Allocate pointer Memory
	if(ret < 0)
		return -1;
	
	InitStoreParamOfTestData();
	FTS_TEST_DBG("[focal] %s ",  IC_TEST_VERSION);	//show lib version

	return 0;	
}
/************************************************************************
* Name: FinishTest
* Brief:  Init all param before test
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void FinishTest(void)
{
	MergeAllTestData();//Merge Test Result
	FreeMemory();//Release pointer memory
}
/************************************************************************
* Name: FT5822_get_test_data
* Brief:  get data of test result
* Input: none
* Output: pTestData, the returned buff
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int FT5822_get_test_data(char *pTestData)
{
	if(NULL == pTestData)
	{
		FTS_TEST_DBG("[focal] %s pTestData == NULL ",  __func__);	
		return -1;
	}
	memcpy(pTestData, g_pStoreAllData, (g_lenStoreMsgArea+g_lenStoreDataArea));
	return (g_lenStoreMsgArea+g_lenStoreDataArea);	

//return sprintf(pTestData,"Hello man!");
}

/************************************************************************
* Name: FT5822_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5822_TestItem_EnterFactoryMode(void)
{	
	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;	//������ɹ����ظ�����5��
	int i ;
	unsigned char chPattern=0;

	SysDelay(150);
	for(i = 1; i <= iRedo; i++)
	{
		ReCode = EnterFactory();
		if(ERROR_CODE_OK != ReCode)
		{
			FTS_TEST_DBG("Failed to Enter factory mode...");
			if(i < iRedo)
			{
				SysDelay(50);
				continue;
			}
		}
		else
		{
			break;
		}

	}
	SysDelay(300);


	if(ReCode != ERROR_CODE_OK)	
	{	
		return ReCode;
	}

	//������ģʽ�ɹ��󣬾Ͷ���ͨ����
	ReCode = GetChannelNum();

	////////////����FIR��0���رգ�1����
	//theDevice.m_cHidDev[m_NumDevice]->WriteReg(0xFB, 0);

	//�ж��Ƿ�ΪV3����
	ReCode = ReadReg( REG_PATTERN_5422, &chPattern );
	if (chPattern == 1)
	{
		m_bV3TP = true;
	}
	else
	{
		m_bV3TP = false;
	}

	return ReCode;
}
/************************************************************************
* Name: FT5822_TestItem_RawDataTest
* Brief:  TestItem: RawDataTest. Check if MCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5822_TestItem_RawDataTest(bool * bTestResult)
{
	unsigned char ReCode = 0;
	bool btmpresult = true;
	int RawDataMin;
	int RawDataMax;
	unsigned char ucFre;
	unsigned char strSwitch = 0;
	unsigned char OriginValue = 0xff;
	int index = 0;
	int iRow, iCol;
	int iValue = 0;


	FTS_TEST_DBG("\n\n==============================Test Item: -------- Raw Data  Test \n");
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}


	//���ж��Ƿ�Ϊv3���壬Ȼ���ȡ0x54��ֵ�����ж����趨��mapping�����Ƿ�һ�£���һ��д������
	//rawdata test mapping��mappingǰ��0x54=1;mapping��0x54=0;
	if (m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &strSwitch );
		if (strSwitch != 0)
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
		}			
	}

	//�������й�һ֮���rawdataֵ��0X16=0Ĭ��
	ReCode = ReadReg( REG_NORMALIZE_TYPE, &OriginValue );//��ȡԭʼֵ	
	if( ReCode != ERROR_CODE_OK )goto TEST_ERR;


	if (g_ScreenSetParam.isNormalize == Auto_Normalize)
	{
		if(OriginValue != 1)//ԭʼֵ����Ҫ�ı��ֵ��ͬ����д�Ĵ���Ϊ��Ҫ��ֵ
		{
			ReCode = WriteReg( REG_NORMALIZE_TYPE, 0x01 );
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
		}
		//���ø�Ƶ��

		FTS_TEST_DBG( "\n=========Set Frequecy High\n" );
		ReCode = WriteReg( 0x0A, 0x81 );
		if( ReCode != ERROR_CODE_OK )goto TEST_ERR;

		FTS_TEST_DBG( "\n=========FIR State: ON");
		ReCode = WriteReg(0xFB, 1);//FIR OFF  0���رգ�1����
		if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
		//��ǰ�ı��˼Ĵ��� �趪��֡����
		for (index = 0; index < 3; ++index )
		{
			ReCode = GetRawData();
		}

		if( ReCode != ERROR_CODE_OK )  
		{
			FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
			goto TEST_ERR;
		}

		ShowRawData();

		////////////////////////////////To Determine RawData if in Range or not
		for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
		{
			for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
			{
				if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
				RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
				RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
				iValue = m_RawData[iRow][iCol];
				if(iValue < RawDataMin || iValue > RawDataMax)
				{
					btmpresult = false;
					FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
						iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
				}
			}
		}	

		//////////////////////////////Save Test Data
		Save_Test_Data(m_RawData, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1);
	}
	else
	{	
		if(OriginValue != 0)//ԭʼֵ����Ҫ�ı��ֵ��ͬ����д�Ĵ���Ϊ��Ҫ��ֵ
		{
			ReCode = WriteReg( REG_NORMALIZE_TYPE, 0x00 );	
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
		}

		ReCode =  ReadReg( 0x0A, &ucFre );
		if( ReCode != ERROR_CODE_OK )goto TEST_ERR;


		//���õ�Ƶ��
		if(g_stCfg_FT5822_BasicThreshold.RawDataTest_SetLowFreq)
		{
			FTS_TEST_DBG("\n=========Set Frequecy Low");
			ReCode = WriteReg( 0x0A, 0x80 );
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;

			//FIR OFF  0���رգ�1����

			FTS_TEST_DBG("\n=========FIR State: OFF\n" );
			ReCode = WriteReg(0xFB, 0);
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
			SysDelay(100);
			//��ǰ�ı��˼Ĵ��� �趪��֡����
			for (index = 0; index < 3; ++index )
			{
				ReCode = GetRawData();
			}

			if( ReCode != ERROR_CODE_OK )  
			{
				FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
				goto TEST_ERR;
			}
			ShowRawData();

			////////////////////////////////To Determine RawData if in Range or not
			for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
			{

				for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
				{
					if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
					RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
					RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
					iValue = m_RawData[iRow][iCol];
					if(iValue < RawDataMin || iValue > RawDataMax)
					{
						btmpresult = false;
						FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
							iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
					}
				}
			}

			//////////////////////////////Save Test Data
			Save_Test_Data(m_RawData, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 1);
		}


		//���ø�Ƶ��
		if ( g_stCfg_FT5822_BasicThreshold.RawDataTest_SetHighFreq )
		{

			FTS_TEST_DBG( "\n=========Set Frequecy High");
			ReCode = WriteReg( 0x0A, 0x81 );
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;

			//FIR OFF  0���رգ�1����

			FTS_TEST_DBG("\n=========FIR State: OFF\n" );
			ReCode = WriteReg(0xFB, 0);
			if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
			SysDelay(100);
			//��ǰ�ı��˼Ĵ��� �趪��֡����
			for (index = 0; index < 3; ++index )
			{
				ReCode = GetRawData();
			}

			if( ReCode != ERROR_CODE_OK )  
			{
				FTS_TEST_DBG("\nGet Rawdata failed, Error Code: 0x%x",  ReCode);
				if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
			}
			ShowRawData();

			////////////////////////////////To Determine RawData if in Range or not
			for(iRow = 0; iRow<g_ScreenSetParam.iTxNum; iRow++)
			{

				for(iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
				{
					if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
					RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Min[iRow][iCol];
					RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_High_Max[iRow][iCol];
					iValue = m_RawData[iRow][iCol];
					if(iValue < RawDataMin || iValue > RawDataMax)
					{
						btmpresult = false;
						FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
							iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
					}
				}
			}

			//////////////////////////////Save Test Data
			Save_Test_Data(m_RawData, 0, g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum, 2);			
		}

	}



	ReCode = WriteReg( REG_NORMALIZE_TYPE, OriginValue );//�ָ�ԭ���Ĵ���ֵ
	if( ReCode != ERROR_CODE_OK )goto TEST_ERR;

	//�ָ�v3�����mappingֵ
	if (m_bV3TP)
	{
		ReCode = WriteReg( REG_MAPPING_SWITCH, strSwitch );
		if( ReCode != ERROR_CODE_OK )goto TEST_ERR;
	}

	//-------------------------Result
	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//RawData Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//RawData Test is NG!");
	}
	return ReCode;

TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//RawData Test is NG!");
	return ReCode;

}
/************************************************************************
* Name: FT5822_TestItem_SCapRawDataTest
* Brief:  TestItem: SCapRawDataTest. Check if SCAP RawData is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5822_TestItem_SCapRawDataTest(bool * bTestResult)
{
	int i =0;
	int RawDataMin = 0;
	int RawDataMax = 0; 
	int Value = 0;
	boolean bFlag = true;
	unsigned char ReCode = 0;
	boolean btmpresult = true;
	int iMax=0;
	int iMin=0;
	int iAvg=0;
	int ByteNum=0;
	unsigned char wc_value = 0;//waterproof channel value
	unsigned char ucValue = 0;
	int iCount = 0;
	int ibiggerValue = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Scap RawData Test \n");
	//-------1.Preparatory work	
	//in Factory Mode
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//get waterproof channel setting, to check if Tx/Rx channel need to test
	ReCode = ReadReg( REG_WATER_CHANNEL_SELECT, &wc_value );
	if(ReCode != ERROR_CODE_OK)	goto TEST_ERR;

	//If it is V3 pattern, Get Tx/Rx Num again
	ReCode= SwitchToNoMapping();
	if(ReCode != ERROR_CODE_OK)	goto TEST_ERR;

	//-------2.Get SCap Raw Data, Step:1.Start Scanning; 2. Read Raw Data
	ReCode = StartScan();
	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to Scan SCap RawData! ");
		goto TEST_ERR;
	}
	for(i = 0; i < 3; i++)
	{
		memset(m_iTempRawData, 0, sizeof(m_iTempRawData));

		//��ˮrawdata
		ByteNum = (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)*2;
		ReCode = ReadRawData(0, 0xAC, ByteNum, m_iTempRawData);
		if(ReCode != ERROR_CODE_OK)goto TEST_ERR;
		memcpy( m_RawData[0+g_ScreenSetParam.iTxNum], m_iTempRawData, sizeof(int)*g_ScreenSetParam.iRxNum );
		memcpy( m_RawData[1+g_ScreenSetParam.iTxNum], m_iTempRawData + g_ScreenSetParam.iRxNum, sizeof(int)*g_ScreenSetParam.iTxNum );

		//�Ƿ�ˮrawdata
		ByteNum = (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)*2;
		ReCode = ReadRawData(0, 0xAB, ByteNum, m_iTempRawData);
		if(ReCode != ERROR_CODE_OK)goto TEST_ERR;
		memcpy( m_RawData[2+g_ScreenSetParam.iTxNum], m_iTempRawData, sizeof(int)*g_ScreenSetParam.iRxNum );
		memcpy( m_RawData[3+g_ScreenSetParam.iTxNum], m_iTempRawData + g_ScreenSetParam.iRxNum, sizeof(int)*g_ScreenSetParam.iTxNum );	
	}


	//-----3. Judge

	//Waterproof ON
	bFlag=GetTestCondition(WT_NeedProofOnTest, wc_value);		
	if(g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_SetWaterproof_ON && bFlag )
	{
		iCount = 0;
		RawDataMin = g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_ON_Min;
		RawDataMax = g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_ON_Max;
		iMax = -m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOnVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("Judge Rx in Waterproof-ON:");
		for( i = 0; bFlag && i < g_ScreenSetParam.iRxNum; i++ )
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Min[0][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Max[0][i];				
			Value = m_RawData[0+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;//find the Max value
			if(iMin > Value) iMin = Value;//fine the min value
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("Judge Tx in Waterproof-ON:");
		for(i = 0;bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Min[1][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_ON_Max[1][i];				
			Value = m_RawData[1+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;//find the Max value
			if(iMin > Value) iMin = Value;//fine the min value
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap RawData in Waterproof-ON, Max : %d, Min: %d, Deviation: %d, Average: %d",  iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+0, 2, ibiggerValue, 1);	
	}

	//Waterproof OFF
	bFlag=GetTestCondition(WT_NeedProofOffTest, wc_value);
	if(g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_SetWaterproof_OFF && bFlag)
	{
		iCount = 0;
		RawDataMin = g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_OFF_Min;
		RawDataMax = g_stCfg_FT5822_BasicThreshold.SCapRawDataTest_OFF_Max;
		iMax = -m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		
		bFlag=GetTestCondition(WT_NeedRxOffVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("Judge Rx in Waterproof-OFF:");
		for(i = 0; bFlag && i < g_ScreenSetParam.iRxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Min[0][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Max[0][i];
			Value = m_RawData[2+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			//FTS_TEST_DBG("zaxzax3 Value %d RawDataMin %d  RawDataMax %d  ",  Value, RawDataMin, RawDataMax);
			//strTemp += str;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		
		bFlag=GetTestCondition(WT_NeedTxOffVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("Judge Tx in Waterproof-OFF:");
		for(i = 0; bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;

			Value = m_RawData[3+g_ScreenSetParam.iTxNum][i];
			RawDataMin = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Min[1][i];
			RawDataMax = g_stCfg_MCap_DetailThreshold.SCapRawDataTest_OFF_Max[1][i];
			//FTS_TEST_DBG("zaxzax4 Value %d RawDataMin %d  RawDataMax %d  ",  Value, RawDataMin, RawDataMax);
			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > RawDataMax || Value < RawDataMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, RawDataMin, RawDataMax);
			}
			iCount++;
		}
		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap RawData in Waterproof-OFF, Max : %d, Min: %d, Deviation: %d, Average: %d",  iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+2, 2, ibiggerValue, 2);	
	}
	//-----4. post-stage work
	if(m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &ucValue );
		if (0 !=ucValue )
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			SysDelay(10); 			
			if( ReCode != ERROR_CODE_OK)	
			{
				FTS_TEST_DBG("Failed to switch mapping type!\n ");
				btmpresult = false;
			}
		}	

		//ֻ�����ݲŻ�ʹ��Mappingǰ�ģ����Ըò���������Ժ���Ҫת��Mapping��
		GetChannelNum();
	}

	//-----5. Test Result
	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//SCap RawData Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//SCap RawData Test is NG!");
	}
	return ReCode;

TEST_ERR:
	* bTestResult = false;
	FTS_TEST_DBG("\n\n//SCap RawData Test is NG!");	
	return ReCode;	
}

/************************************************************************
* Name: FT5822_TestItem_SCapCbTest
* Brief:  TestItem: SCapCbTest. Check if SCAP Cb is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT5822_TestItem_SCapCbTest(bool* bTestResult)
{
	int i,/* j, iOutNum,*/index,Value,CBMin,CBMax;
	boolean bFlag = true;
	unsigned char ReCode;
	boolean btmpresult = true;
	int iMax, iMin, iAvg;
	unsigned char wc_value = 0;
	unsigned char ucValue = 0;
	int iCount = 0;
	int ibiggerValue = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: -----  Scap CB Test \n");	
	//-------1.Preparatory work	
	//in Factory Mode
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_ERR;
	}

	//get waterproof channel setting, to check if Tx/Rx channel need to test
	ReCode = ReadReg( REG_WATER_CHANNEL_SELECT, &wc_value );
	if(ReCode != ERROR_CODE_OK)	goto TEST_ERR;

	//If it is V3 pattern, Get Tx/Rx Num again
	bFlag= SwitchToNoMapping();
	if( bFlag )
	{	
		FTS_TEST_DBG("Failed to SwitchToNoMapping! ");
		goto TEST_ERR;
	}

	//-------2.Get SCap Raw Data, Step:1.Start Scanning; 2. Read Raw Data
	ReCode = StartScan();
	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to Scan SCap RawData! ");
		goto TEST_ERR;
	}


	for(i = 0; i < 3; i++)
	{
		memset(m_RawData, 0, sizeof(m_RawData));
		memset(m_ucTempData, 0, sizeof(m_ucTempData));

		//��ˮCB
		ReCode = WriteReg( REG_ScWorkMode, 1 );//���ݹ�����ʽѡ��:  1����ˮ 0:�Ƿ�ˮ
		ReCode = StartScan();
		ReCode = WriteReg( REG_ScCbAddrR, 0 );			
		ReCode = GetTxSC_CB( g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum + 128, m_ucTempData );
		for ( index = 0; index < g_ScreenSetParam.iRxNum; ++index )
		{
			m_RawData[0 + g_ScreenSetParam.iTxNum][index]= m_ucTempData[index];
		}
		for ( index = 0; index < g_ScreenSetParam.iTxNum; ++index )
		{
			m_RawData[1 + g_ScreenSetParam.iTxNum][index] = m_ucTempData[index + g_ScreenSetParam.iRxNum];
		}

		//�Ƿ�ˮrawdata
		ReCode = WriteReg( REG_ScWorkMode, 0 );//���ݹ�����ʽѡ��:  1����ˮ 0:�Ƿ�ˮ
		ReCode = StartScan();
		ReCode = WriteReg( REG_ScCbAddrR, 0 );
		ReCode = GetTxSC_CB( g_ScreenSetParam.iRxNum + g_ScreenSetParam.iTxNum + 128, m_ucTempData );
		for ( index = 0; index < g_ScreenSetParam.iRxNum; ++index )
		{
			m_RawData[2 + g_ScreenSetParam.iTxNum][index]= m_ucTempData[index];
		}
		for ( index = 0; index < g_ScreenSetParam.iTxNum; ++index )
		{
			m_RawData[3 + g_ScreenSetParam.iTxNum][index] = m_ucTempData[index + g_ScreenSetParam.iRxNum];
		}

		if( ReCode != ERROR_CODE_OK )	
		{
			FTS_TEST_DBG("Failed to Get SCap CB!");
		}		
	}

	if(ReCode != ERROR_CODE_OK)	goto TEST_ERR;

	//-----3. Judge

	//Waterproof ON
	bFlag=GetTestCondition(WT_NeedProofOnTest, wc_value);			
	if(g_stCfg_FT5822_BasicThreshold.SCapCbTest_SetWaterproof_ON && bFlag)
	{
		FTS_TEST_DBG("SCapCbTest in WaterProof On Mode:  ");

		iMax = -m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[0+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		iCount = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Rx:  ");
		for( i = 0;bFlag && i < g_ScreenSetParam.iRxNum; i++ )
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Min[0][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Max[0][i];
			Value = m_RawData[0+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			if(iMax < Value) iMax = Value;//find the Max Value
			if(iMin > Value) iMin = Value;//find the Min Value
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, CBMin, CBMax);
			}			
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOnVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Tx:  ");
		for(i = 0;bFlag &&  i < g_ScreenSetParam.iTxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Min[1][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_ON_Max[1][i];
			Value = m_RawData[1+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap CB in Waterproof-ON, Max : %d, Min: %d, Deviation: %d, Average: %d",  iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+0, 2, ibiggerValue, 1);			
	}

	bFlag=GetTestCondition(WT_NeedProofOffTest, wc_value);
	if(g_stCfg_FT5822_BasicThreshold.SCapCbTest_SetWaterproof_OFF && bFlag)
	{			
		FTS_TEST_DBG("SCapCbTest in WaterProof OFF Mode:  ");
		iMax = -m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iMin = 2 * m_RawData[2+g_ScreenSetParam.iTxNum][0];
		iAvg = 0;
		Value = 0;
		iCount = 0;

		
		bFlag=GetTestCondition(WT_NeedRxOffVal, wc_value);
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Rx:  ");
		for(i = 0;bFlag &&  i < g_ScreenSetParam.iRxNum; i++)
		{
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[0][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Min[0][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Max[0][i];
			Value = m_RawData[2+g_ScreenSetParam.iTxNum][i];
			iAvg += Value;

			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		
		bFlag=GetTestCondition(WT_NeedTxOffVal, wc_value);	
		if(bFlag)
			FTS_TEST_DBG("SCap CB_Tx:  ");
		for(i = 0; bFlag && i < g_ScreenSetParam.iTxNum; i++)
		{
			//if( m_ScapInvalide[1][i] == 0 )      continue;
			if( g_stCfg_MCap_DetailThreshold.InvalidNode_SC[1][i] == 0 )      continue;
			CBMin = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Min[1][i];
			CBMax = g_stCfg_MCap_DetailThreshold.SCapCbTest_OFF_Max[1][i];
			Value = m_RawData[3+g_ScreenSetParam.iTxNum][i];

			iAvg += Value;
			if(iMax < Value) iMax = Value;
			if(iMin > Value) iMin = Value;
			if(Value > CBMax || Value < CBMin) 
			{
				btmpresult = false;
				FTS_TEST_DBG("Failed. Num = %d, Value = %d, range = (%d, %d):",  i+1, Value, CBMin, CBMax);
			}
			iCount++;
		}

		if(0 == iCount)
		{
			iAvg = 0;
			iMax = 0;
			iMin = 0;
		}
		else				
			iAvg = iAvg/iCount;

		FTS_TEST_DBG("SCap CB in Waterproof-OFF, Max : %d, Min: %d, Deviation: %d, Average: %d",  iMax, iMin, iMax - iMin, iAvg);
		//////////////////////////////Save Test Data
		ibiggerValue = g_ScreenSetParam.iTxNum>g_ScreenSetParam.iRxNum?g_ScreenSetParam.iTxNum:g_ScreenSetParam.iRxNum;
		Save_Test_Data(m_RawData, g_ScreenSetParam.iTxNum+2, 2, ibiggerValue, 2);	
	}
	//-----4. post-stage work
	if(m_bV3TP)
	{
		ReCode = ReadReg( REG_MAPPING_SWITCH, &ucValue );
		if (0 != ucValue )
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 0 );
			SysDelay(10); 			
			if( ReCode != ERROR_CODE_OK)	
			{
				FTS_TEST_DBG("Failed to switch mapping type!\n ");
				btmpresult = false;
			}
		}	

		//ֻ�����ݲŻ�ʹ��Mappingǰ�ģ����Ըò���������Ժ���Ҫת��Mapping��
		GetChannelNum();
	}

	//-----5. Test Result

	if( btmpresult )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n//SCap CB Test Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//SCap CB Test Test is NG!");
	}
	return ReCode;

TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//SCap CB Test Test is NG!");
	return ReCode;	
}

/************************************************************************
* Name: GetPanelRows(Same function name as FT_MultipleTest)
* Brief:  Get row of TP
* Input: none
* Output: pPanelRows
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelRows(unsigned char *pPanelRows)
{
	return ReadReg(REG_TX_NUM, pPanelRows);
}

/************************************************************************
* Name: GetPanelCols(Same function name as FT_MultipleTest)
* Brief:  get column of TP
* Input: none
* Output: pPanelCols
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetPanelCols(unsigned char *pPanelCols)
{
	return ReadReg(REG_RX_NUM, pPanelCols);
}
/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0;
	unsigned char times = 0;
	const unsigned char MaxTimes = 20;	//��ȴ�160ms
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
	if(ReCode == ERROR_CODE_OK)
	{
		RegVal |= 0x80;		//���λ��1������ɨ��
		ReCode = WriteReg(DEVIDE_MODE_ADDR, RegVal);
		if(ReCode == ERROR_CODE_OK)
		{
			while(times++ < MaxTimes)		//�ȴ�ɨ�����
			{
				SysDelay(8);	//8ms
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
				if(ReCode == ERROR_CODE_OK)
				{
					if((RegVal>>7) == 0)	break;
				}
				else
				{
					break;
				}
			}
			if(times < MaxTimes)	ReCode = ERROR_CODE_OK;
			else ReCode = ERROR_CODE_COMM_ERROR;
		}
	}
	return ReCode;

}	
/************************************************************************
* Name: ReadRawData(Same function name as FT_MultipleTest)
* Brief:  read Raw Data
* Input: Freq(No longer used, reserved), LineNum, ByteNum
* Output: pRevBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1=0;

	//unsigned short BytesNumInTestMode3=0, BytesNumInTestMode2=0,BytesNumInTestMode1=0;
	//unsigned short BytesNumInTestMode6=0, BytesNumInTestMode5=0,BytesNumInTestMode4=0;

	iReadNum=ByteNum/BYTES_PER_TIME;

	if(0 != (ByteNum%BYTES_PER_TIME)) iReadNum++;

	if(ByteNum <= BYTES_PER_TIME)
	{
		BytesNumInTestMode1 = ByteNum;		
	}
	else
	{
		BytesNumInTestMode1 = BYTES_PER_TIME;
	}

	ReCode = WriteReg(REG_LINE_NUM, LineNum);//Set row addr;


	//***********************************************************Read raw data		
	I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
	if(ReCode == ERROR_CODE_OK)
	{
		focal_msleep(10);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, m_ucTempData, BytesNumInTestMode1);
	}

	for(i=1; i<iReadNum; i++)
	{
		if(ReCode != ERROR_CODE_OK) break;

		if(i==iReadNum-1)//last packet
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+BYTES_PER_TIME*i, ByteNum-BYTES_PER_TIME*i);
		}
		else
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, m_ucTempData+BYTES_PER_TIME*i, BYTES_PER_TIME);	
		}

	}

	if(ReCode == ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (m_ucTempData[i<<1]<<8)+m_ucTempData[(i<<1)+1];
			//if(pRevBuffer[i] & 0x8000)//�з���λ
			//{
			//	pRevBuffer[i] -= 0xffff + 1;
			//}
		}
	}

	return ReCode;

}
/************************************************************************
* Name: GetTxSC_CB(Same function name as FT_MultipleTest)
* Brief:  get CB of Tx SCap
* Input: index
* Output: pcbValue
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char GetTxSC_CB(unsigned char index, unsigned char *pcbValue)
{
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char wBuffer[4];

	if(index<128)//������ȡ
	{	
		*pcbValue = 0;
		WriteReg(REG_ScCbAddrR, index);
		ReCode = ReadReg(REG_ScCbBuf0, pcbValue);
	}
	else//������ȡ������Ϊindex-128
	{
		WriteReg(REG_ScCbAddrR, 0);
		wBuffer[0] = REG_ScCbBuf0;	
		ReCode = Comm_Base_IIC_IO(wBuffer, 1, pcbValue, index-128);

	}	

	return ReCode;
}


//////////////////////////////////////////////
/************************************************************************
* Name: AllocateMemory
* Brief:  Allocate pointer Memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
static int AllocateMemory(void)
{
	//New buff
	g_pStoreMsgArea =NULL;	
	if(NULL == g_pStoreMsgArea)
		g_pStoreMsgArea = fts_malloc(BUFF_LEN_STORE_MSG_AREA);
	if(NULL == g_pStoreMsgArea)
		goto ERR;
	
	g_pMsgAreaLine2 =NULL;	
	if(NULL == g_pMsgAreaLine2)
		g_pMsgAreaLine2 = fts_malloc(BUFF_LEN_MSG_AREA_LINE2);
	if(NULL == g_pMsgAreaLine2)
		goto ERR;
	
	g_pStoreDataArea =NULL;	
	if(NULL == g_pStoreDataArea)
		g_pStoreDataArea = fts_malloc(BUFF_LEN_STORE_DATA_AREA);
	if(NULL == g_pStoreDataArea)
		goto ERR;
		
	g_pTmpBuff =NULL;	
	if(NULL == g_pTmpBuff)
		g_pTmpBuff = fts_malloc(BUFF_LEN_TMP_BUFFER);
	if(NULL == g_pTmpBuff)
		goto ERR;
	
	return 0;
	
	ERR:
	FTS_TEST_DBG("fts_malloc memory failed in function.");
	return -1;

}
/************************************************************************
* Name: FreeMemory
* Brief:  Release pointer memory
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void FreeMemory(void)
{
	//Release buff
	if(NULL != g_pStoreMsgArea)
		fts_free(g_pStoreMsgArea);

	if(NULL != g_pMsgAreaLine2)
		fts_free(g_pMsgAreaLine2);

	if(NULL != g_pStoreDataArea)
		fts_free(g_pStoreDataArea);

	/*if(NULL == g_pStoreAllData)
		fts_free(g_pStoreAllData);*/

	if(NULL != g_pTmpBuff)
		fts_free(g_pTmpBuff);
}

/************************************************************************
* Name: InitStoreParamOfTestData
* Brief:  Init store param of test data
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void InitStoreParamOfTestData(void)
{


	g_lenStoreMsgArea = 0;
	//Msg Area, Add Line1
	g_lenStoreMsgArea += sprintf(g_pStoreMsgArea,"ECC, 85, 170, IC Name, %s, IC Code, %x\n",  g_strIcName,  g_ScreenSetParam.iSelectedIC);

	//Line2
	//g_pMsgAreaLine2 = NULL;
	g_lenMsgAreaLine2 = 0;

	//Data Area
	//g_pStoreDataArea = NULL;
	g_lenStoreDataArea = 0;
	m_iStartLine = 11;//The Start Line of Data Area is 11

	m_iTestDataCount = 0;	
}
/************************************************************************
* Name: MergeAllTestData
* Brief:  Merge All Data of test result
* Input: none
* Output: none
* Return: none
***********************************************************************/
static void MergeAllTestData(void)
{
	int iLen = 0;

	//Add the head part of Line2
	iLen= sprintf(g_pTmpBuff,"TestItem, %d, ", m_iTestDataCount);
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea+=iLen;

	//Add other part of Line2, except for "\n"
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pMsgAreaLine2, g_lenMsgAreaLine2);
	g_lenStoreMsgArea+=g_lenMsgAreaLine2;	

	//Add Line3 ~ Line10
	iLen= sprintf(g_pTmpBuff,"\n\n\n\n\n\n\n\n\n");
	memcpy(g_pStoreMsgArea+g_lenStoreMsgArea, g_pTmpBuff, iLen);
	g_lenStoreMsgArea+=iLen;

	///1.Add Msg Area
	memcpy(g_pStoreAllData, g_pStoreMsgArea, g_lenStoreMsgArea);

	///2.Add Data Area
	if(0!= g_lenStoreDataArea)
	{
		memcpy(g_pStoreAllData+g_lenStoreMsgArea, g_pStoreDataArea, g_lenStoreDataArea);
	}

	FTS_TEST_DBG("[focal] %s lenStoreMsgArea=%d,  lenStoreDataArea = %d",  __func__, g_lenStoreMsgArea, g_lenStoreDataArea);
}


/************************************************************************
* Name: Save_Test_Data
* Brief:  Storage format of test data
* Input: int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount
* Output: none
* Return: none
***********************************************************************/
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount)
{
	int iLen = 0;
	int i = 0, j = 0;

	//Save  Msg (ItemCode is enough, ItemName is not necessary, so set it to "NA".)
	iLen= sprintf(g_pTmpBuff,"NA, %d, %d, %d, %d, %d, ", \
		m_ucTestItemCode, Row, Col, m_iStartLine, ItemCount);
	memcpy(g_pMsgAreaLine2+g_lenMsgAreaLine2, g_pTmpBuff, iLen);
	g_lenMsgAreaLine2 += iLen;

	m_iStartLine += Row;
	m_iTestDataCount++;

	//Save Data 
	for(i = 0+iArrayIndex; i < Row+iArrayIndex; i++)
	{
		for(j = 0; j < Col; j++)
		{
			if(j == (Col -1))//The Last Data of the Row, add "\n"
				iLen= sprintf(g_pTmpBuff,"%d, ",  iData[i][j]);	
			else
				iLen= sprintf(g_pTmpBuff,"%d, ", iData[i][j]);	

			memcpy(g_pStoreDataArea+g_lenStoreDataArea, g_pTmpBuff, iLen);
			g_lenStoreDataArea += iLen;		
		}
	}

}

/************************************************************************
* Name: GetChannelNum
* Brief:  Get Channel Num(Tx and Rx)
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; //= new unsigned char;

	//m_strCurrentTestMsg = "Get Tx Num...";
	ReCode = GetPanelRows(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iTxNum = rBuffer[0];	
		if(g_ScreenSetParam.iTxNum+4 > g_ScreenSetParam.iUsedMaxTxNum)
		{
			FTS_TEST_DBG("Failed to get Tx number, Get num = %d, UsedMaxNum = %d", 
				g_ScreenSetParam.iTxNum, g_ScreenSetParam.iUsedMaxTxNum);
			g_ScreenSetParam.iTxNum = 0;			
			return ERROR_CODE_INVALID_PARAM;
		}
//g_ScreenSetParam.iTxNum = 26;
	}
	else
	{
		FTS_TEST_DBG("Failed to get Tx number");
	}

	///////////////m_strCurrentTestMsg = "Get Rx Num...";

	ReCode = GetPanelCols(rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iRxNum = rBuffer[0];
		if(g_ScreenSetParam.iRxNum > g_ScreenSetParam.iUsedMaxRxNum)
		{
			FTS_TEST_DBG("Failed to get Rx number, Get num = %d, UsedMaxNum = %d", 
				g_ScreenSetParam.iRxNum, g_ScreenSetParam.iUsedMaxRxNum);
			g_ScreenSetParam.iRxNum = 0;			
			return ERROR_CODE_INVALID_PARAM;
		}
		//g_ScreenSetParam.iRxNum = 28;
	}
	else
	{
		FTS_TEST_DBG("Failed to get Rx number");
	}

	return ReCode;

}
/************************************************************************
* Name: GetRawData
* Brief:  Get Raw Data of MCAP
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetRawData(void)
{
	unsigned char ReCode = ERROR_CODE_OK;
	int iRow = 0;
	int iCol = 0;		

	//--------------------------------------------Enter Factory Mode
	ReCode = EnterFactory();	
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Enter Factory Mode...");
		return ReCode;
	}


	//--------------------------------------------Check Num of Channel 
	if(0 == (g_ScreenSetParam.iTxNum + g_ScreenSetParam.iRxNum)) 
	{
		ReCode = GetChannelNum();
		if( ERROR_CODE_OK != ReCode ) 
		{
			FTS_TEST_DBG("Error Channel Num...");
			return ERROR_CODE_INVALID_PARAM;
		}
	}

	//--------------------------------------------Start Scanning
	FTS_TEST_DBG("Start Scan ...");
	ReCode = StartScan();
	if(ERROR_CODE_OK != ReCode) 
	{
		FTS_TEST_DBG("Failed to Scan ...");
		return ReCode;
	}

	//--------------------------------------------Read RawData, Only MCAP
	memset(m_RawData, 0, sizeof(m_RawData));
	ReCode = ReadRawData( 1, 0xAA, ( g_ScreenSetParam.iTxNum * g_ScreenSetParam.iRxNum )*2, m_iTempRawData );
	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			m_RawData[iRow][iCol] = m_iTempRawData[iRow*g_ScreenSetParam.iRxNum + iCol];
		}
	}
	return ReCode;
}
/************************************************************************
* Name: ShowRawData
* Brief:  Show RawData
* Input: none
* Output: none
* Return: none.
***********************************************************************/
static void ShowRawData(void)
{
	int iRow, iCol;
	//----------------------------------------------------------Show RawData
	for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; iRow++)
	{
		FTS_TEST_DBG("Tx%2d:  ", iRow+1);
		for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; iCol++)
		{
			FTS_TEST_PRINT("%5d    ", m_RawData[iRow][iCol]);
		}
		FTS_TEST_PRINT("\n ");
	}
}

/************************************************************************
* Name: GetChannelNumNoMapping
* Brief:  get Tx&Rx num from other Register
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNumNoMapping(void)
{
	unsigned char ReCode;
	unsigned char rBuffer[1]; //= new unsigned char;


	FTS_TEST_DBG("Get Tx Num...");
	ReCode =ReadReg( REG_TX_NOMAPPING_NUM,  rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iTxNum= rBuffer[0];	
	}
	else
	{
		FTS_TEST_DBG("Failed to get Tx number");
	}


	FTS_TEST_DBG("Get Rx Num...");
	ReCode = ReadReg( REG_RX_NOMAPPING_NUM,  rBuffer);
	if(ReCode == ERROR_CODE_OK)
	{
		g_ScreenSetParam.iRxNum = rBuffer[0];
	}
	else
	{
		FTS_TEST_DBG("Failed to get Rx number");
	}

	return ReCode;
}
/************************************************************************
* Name: SwitchToNoMapping
* Brief:  If it is V3 pattern, Get Tx/Rx Num again
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char SwitchToNoMapping(void)
{
	unsigned char chPattern = -1;
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char RegData = -1;
	ReCode = ReadReg( REG_PATTERN_5422, &chPattern );//

	if(1 == chPattern)// 1: V3 Pattern
	{	
		RegData = -1;
		ReCode =ReadReg( REG_MAPPING_SWITCH, &RegData );
		if( 1 != RegData ) 
		{
			ReCode = WriteReg( REG_MAPPING_SWITCH, 1 );  //0-mapping 1-no mampping
			focal_msleep(20);
			GetChannelNumNoMapping();
		}
	}

	if( ReCode != ERROR_CODE_OK )
	{
		FTS_TEST_DBG("Switch To NoMapping Failed!");
	}
	return ReCode;
}
/************************************************************************
* Name: GetTestCondition
* Brief:  Check whether Rx or TX need to test, in Waterproof ON/OFF Mode.
* Input: none
* Output: none
* Return: true: need to test; false: Not tested.
***********************************************************************/
static boolean GetTestCondition(int iTestType, unsigned char ucChannelValue)
{
	boolean bIsNeeded = false;
	switch(iTestType)
	{
	case WT_NeedProofOnTest://Bit5:  0������ˮģʽ;  1��������ˮģʽ
		bIsNeeded = !( ucChannelValue & 0x20 );
		break;
	case WT_NeedProofOffTest://Bit7: 0 ��ͨģʽ��⣻ 1����ͨģʽ�����
		bIsNeeded = !( ucChannelValue & 0x80 );
		break;
	case WT_NeedTxOnVal:
		//Bit6:  0 : ����ˮRx+Tx�� 1��ֻ���һ��ͨ�� 
		//Bit2:  0: ֻ����ˮTx;  1:  ֻ����ˮRx
		bIsNeeded = !( ucChannelValue & 0x40 ) || !( ucChannelValue & 0x04 );
		break;			
	case WT_NeedRxOnVal:
		//Bit6:  0 : ����ˮRx+Tx�� 1��ֻ���һ��ͨ�� 
		//Bit2:  0: ֻ����ˮTx;  1:  ֻ����ˮRx
		bIsNeeded = !( ucChannelValue & 0x40 ) || ( ucChannelValue & 0x04 );
		break;			
	case WT_NeedTxOffVal://Bit1,Bit0:  00:��ͨģʽTx; 10: ��ͨģʽRx+Tx 
		bIsNeeded = (0x00 == (ucChannelValue & 0x03)) || (0x02 == ( ucChannelValue & 0x03 ));
		break;			
	case WT_NeedRxOffVal://Bit1,Bit0:  01: ��ͨģʽRx;    10: ��ͨģʽRx+Tx
		bIsNeeded = (0x01 == (ucChannelValue & 0x03)) || (0x02 == ( ucChannelValue & 0x03 ));
		break;
	default:break;
	}
	return bIsNeeded;
}


//��Ƶ��FIR��0������
unsigned char FT5822_TestItem_UniformityTest(bool * bTestResult)
{
	unsigned char ReCode = ERROR_CODE_OK;
	bool btmpresult = true;
	unsigned char ucFre = 0;
	unsigned char FirValue = 0;
	//unsigned char FirValues = 0;
	//char str[512] = {0};

	int iMin = 100000;
	int iMax = -100000;
	int iDeviation = 0;
	int iRow = 0;
	int iCol = 0;	
	int iUniform = 0;
	int index = 0; 

	FTS_TEST_DBG("\n\n==============================Test Item: --------RawData Uniformity Test \n");
	//-------1.Preparatory work	
	//in Factory Mode
	ReCode = EnterFactory(); 
	if(ReCode != ERROR_CODE_OK)		
	{
		FTS_TEST_DBG("\n\n// Failed to Enter factory Mode. Error Code: %d", ReCode);
		goto TEST_END;
	}	

	ReCode =  ReadReg( REG_FREQUENCY, &ucFre );
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}

	//��Ƶ 
	ReCode = WriteReg( REG_FREQUENCY, 0x81 );
	SysDelay(100);
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}

	ReCode =ReadReg(REG_FIR, &FirValue);
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}

	// fir = 0
	ReCode = WriteReg(REG_FIR, 0);
	SysDelay(100);
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}

	//��ǰ�ı��˼Ĵ��� �趪��֡����
	for (index = 0; index < 3; ++index )
	{
		ReCode = GetRawData();		
	}

	if( g_stCfg_FT5822_BasicThreshold.Uniformity_CheckTx )
	{
		FTS_TEST_DBG("\n\n=========Check Tx Linearity \n");

		memset(TxLinearity,  0, sizeof(TxLinearity));
		
		for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for (iCol = 1; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				iDeviation = focal_abs( m_RawData[iRow][iCol] - m_RawData[iRow][iCol-1] );
				iMax = m_RawData[iRow][iCol]>m_RawData[iRow][iCol-1] ?m_RawData[iRow][iCol]:m_RawData[iRow][iCol-1] ;
				iMax = iMax ? iMax : 1;
				TxLinearity[iRow][iCol] = 100 * iDeviation / iMax;			
			}
		}

		for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for (iCol = 1; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				if(0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) continue;
				if(2 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) continue;

				if(TxLinearity[iRow][iCol]<MIN_HOLE_LEVEL || 
					TxLinearity[iRow][iCol]>g_stCfg_FT5822_BasicThreshold.Uniformity_Tx_Hole)
				{
					FTS_TEST_DBG("Tx Linearity Out Of Range, TX=%d, RX=%d, TxLinearity=%d, Tx_Hole=%d.",   iCol, iRow, TxLinearity[iRow][iCol], g_stCfg_FT5822_BasicThreshold.Uniformity_Tx_Hole);
			
					btmpresult = false;					
				}					
			}
		}

		Save_Test_Data(TxLinearity,  0, g_ScreenSetParam.iTxNum,  g_ScreenSetParam.iRxNum, 1);	
		
	}
	
	if( g_stCfg_FT5822_BasicThreshold.Uniformity_CheckRx )
	{
		FTS_TEST_DBG("\n\n=========Check Rx Linearity \n");
		
		for (iRow = 1; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				iDeviation = focal_abs( m_RawData[iRow][iCol] - m_RawData[iRow-1][iCol] );
				iMax = m_RawData[iRow][iCol]>m_RawData[iRow-1][iCol] ?m_RawData[iRow][iCol] : m_RawData[iRow-1][iCol] ;
				iMax = iMax ? iMax : 1;
				RxLinearity[iRow][iCol] = 100 * iDeviation / iMax;
			}
		}
		
		for (iRow = 1; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				if(0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) continue;
				if(2 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) continue;

				if(RxLinearity[iRow][iCol]<MIN_HOLE_LEVEL || 
					RxLinearity[iRow][iCol]>g_stCfg_FT5822_BasicThreshold.Uniformity_Rx_Hole)
				{
					FTS_TEST_DBG("Rx Linearity Out Of Range, TX=%d, RX=%d, RxLinearity=%d, Rx_Hole=%d.",   iCol, iRow, RxLinearity[iRow][iCol], g_stCfg_FT5822_BasicThreshold.Uniformity_Rx_Hole);
			
					btmpresult = false;					
				}					
			}
		}

		Save_Test_Data(RxLinearity,  0, g_ScreenSetParam.iTxNum,  g_ScreenSetParam.iRxNum, 2);
		
	}
	
	if( g_stCfg_FT5822_BasicThreshold.Uniformity_CheckMinMax )
	{
		FTS_TEST_DBG("\n\n=========Check Min/Max \n");
		iMin = 100000;
		iMax = -100000;
	
		for (iRow = 0; iRow < g_ScreenSetParam.iTxNum; ++iRow )
		{
			for (iCol = 0; iCol < g_ScreenSetParam.iRxNum; ++iCol )
			{
				if( 0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] ){
					continue;
				}
				if( 2 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] ){
					continue;
				}
				if( iMin> m_RawData[iRow][iCol] ) iMin= m_RawData[iRow][iCol] ;
				if( iMax< m_RawData[iRow][iCol] ) iMax= m_RawData[iRow][iCol] ;
			}
		}
		iMax = !iMax ? 1 : iMax;
		iUniform = 100 * focal_abs(iMin) / focal_abs(iMax);

		FTS_TEST_DBG("\n\n Min: %d, Max: %d, , Get Value of Min/Max: %d.",   iMin, iMax, iUniform );

		if( iUniform < g_stCfg_FT5822_BasicThreshold.Uniformity_MinMax_Hole )
		{
			btmpresult = false;
			FTS_TEST_DBG("\n\n MinMax Out Of Range, Set Value: %d.",  g_stCfg_FT5822_BasicThreshold.Uniformity_MinMax_Hole );			
		}
	}

	//�ָ�ԭ��Ƶ��
	ReCode =  WriteReg( REG_FREQUENCY, ucFre );
	SysDelay(100);
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}
	//�ָ�ԭ��Ƶ��,fir
	ReCode =  WriteReg( REG_FIR, FirValue );
	SysDelay(100);
	if (ReCode != ERROR_CODE_OK)
	{
		goto TEST_END;
	}


TEST_END:

	if( btmpresult && ReCode == ERROR_CODE_OK )
	{
		*bTestResult = true;
		FTS_TEST_DBG("\n\n\n\n//Uniformity Test is OK!\n");
	}
	else
	{
		*bTestResult = false;
		FTS_TEST_DBG("\n\n\n\n//Uniformity Test is NG!\n");
	}
	return ReCode;
}
	
