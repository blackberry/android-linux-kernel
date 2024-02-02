/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: Test_FT8607.c
*
* Author: Software Development Team, AE
*
* Created: 2016-03-15
*
* Abstract: test item for FT8607
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
#include "focaltech_test_ft8607.h"
#include "focaltech_test_config_ft8607.h"


/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define IC_TEST_VERSION  "Test version: V1.0.0--2016-03-15, (sync version of FT_MultipleTest: V2.7.0.3--2015-07-13)"
/*buff length*/
#define BUFF_LEN_STORE_MSG_AREA		1024*10
#define BUFF_LEN_MSG_AREA_LINE2		1024*4
#define BUFF_LEN_STORE_DATA_AREA		1024*80
#define BUFF_LEN_TMP_BUFFER 			1024*16

#define DEVIDE_MODE_ADDR	0x00

// copy from 8606
#define REG_LINE_NUM	0x01
#define REG_TX_NUM	0x02
#define REG_RX_NUM	0x03

#define REG_RawBuf0			0x6A	
#define REG_RawBuf1			0x6B	
#define REG_OrderBuf0		0x6C	
#define REG_CbBuf0			0x6E	

#define REG_CbAddrH  		0x18	 
#define REG_CbAddrL			0x19	
#define REG_OrderAddrH		0x1A	
#define REG_OrderAddrL		0x1B	

#define pre 1

//	}} from 8606


#define SHEILD_NODE -1
#define MAX_CAP_GROUP_OF_8607   4
#define FT_8607_CONFIG_START_ADDR 0x7a80

#define FT_8607_LEFT_KEY_REG    0X1E
#define FT_8607_RIGHT_KEY_REG   0X1F

#define REG_8607_LCD_NOISE_FRAME  0X12
#define REG_8607_LCD_NOISE_START  0X11
#define REG_8607_LCD_NOISE_NUMBER 0X13


#define OUTPUT_MAXMIN

#define MAX_NOISE_FRAMES    32

enum { MODE_FREERUN, MODE_MONITOR, MODE_ACTIVE, MODE_COMPENSATION };

enum NOISE_TYPE
{
	NT_AvgData = 0,
	NT_MaxData = 1,
	NT_MaxDevication = 2,
	NT_DifferData = 3,
};

/*******************************************************************************
* Static variables
*******************************************************************************/

static int m_RawData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
//static int m_NoiseData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static int m_CBData[TX_NUM_MAX][RX_NUM_MAX] = {{0}};
static BYTE m_ucTempData[TX_NUM_MAX * RX_NUM_MAX*2] = {0};//One-dimensional
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};


static unsigned char GetChannelNum(void);
static int InitTest(void);


//---------------------About Store Test Dat
//static char g_pStoreAllData[TEST_DATA_BUF_SIZE] = {0};
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
* Static function prototypes
*******************************************************************************/
/////////////////////////////////////////////////////////////
static int StartScan(void);
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer);
static unsigned char GetPanelRows(unsigned char *pPanelRows);
static unsigned char GetPanelCols(unsigned char *pPanelCols);
static unsigned char GetTxRxCB(unsigned short StartNodeNo, unsigned short ReadNum, unsigned char *pReadBuffer);
/////////////////////////////////////////////////////////////
static unsigned char GetRawData(void);
static unsigned char GetChannelNum(void);
////////////////////////////////////////////////////////////
//////////////////////////////////////////////
static int InitTest(void);
static void FinishTest(void);
static void Save_Test_Data(int iData[TX_NUM_MAX][RX_NUM_MAX], int iArrayIndex, unsigned char Row, unsigned char Col, unsigned char ItemCount);
static void InitStoreParamOfTestData(void);
static void MergeAllTestData(void);
//////////////////////////////////////////////Others 
static int AllocateMemory(void);
static void FreeMemory(void);
static unsigned int SqrtNew(unsigned int n) ;
//static void ShowRawData(void);

static unsigned char FT8607_TestItem_EnterFactoryMode(void);
static unsigned char FT8607_TestItem_RawDataTest(bool * bTestResult);
//static unsigned char FT8607_TestItem_NoiseTest(bool* bTestResult);
static unsigned char FT8607_TestItem_CbTest(bool * bTestResult);
static unsigned char FT8607_TestItem_LCDNoiseTest(bool* bTestResult);
static unsigned char FT8607_TestItem_ShortCircuitTest(bool* bTestResult);


/************************************************************************
* Name: _StartTest
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
boolean FT8607_StartTest()
{
	bool bTestResult = true, bTempResult = 1;
//	unsigned char ucTempResult = 1;
	unsigned char ReCode;
	unsigned char ucDevice = 0;
	int iItemCount=0;
//	CString str;
//	SYSTEMTIME st = {0};	

	FTS_TEST_DBG("");

//	theDevice.m_cTest_FT8607[ucDevice]->InitFT8607Test();
	//--------------1. Init part	if(InitTest() < 0)
	{
		FTS_TEST_DBG("[focal] Failed to init test.");
		return false;
	}
	
	//--------------2. test item
	if(0 == g_TestItemNum)
		bTestResult = false;

	////////���Թ��̣�����˳��ִ��g_stTestItem�ṹ���Ĳ�����
	for(iItemCount = 0; iItemCount < g_TestItemNum; iItemCount++)
	{

		m_ucTestItemCode = g_stTestItem[ucDevice][iItemCount].ItemCode;
		FTS_TEST_DBG(" ItemCode:%d. ",  g_stTestItem[ucDevice][iItemCount].ItemCode);

		///////////////////////////////////////////////////////FT8607_ENTER_FACTORY_MODE
		if(Code_FT8607_ENTER_FACTORY_MODE == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			FTS_TEST_DBG(" Code_FT8607_ENTER_FACTORY_MODE. ");

			ReCode = FT8607_TestItem_EnterFactoryMode();
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		/* not need now.
		///////////////////////////////////////////////////////FT8607_CHANNEL_NUM_TEST
		if(Code_FT8607_CHANNEL_NUM_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT8607_TestItem_ChannelsTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;

		}
		*/
		///////////////////////////////////////////////////////FT8607_RAWDATA_TEST
		if(Code_FT8607_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			FTS_TEST_DBG(" Code_FT8607_RAWDATA_TEST. ");

			ReCode = FT8607_TestItem_RawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT8607_SHORT_TEST
		if(Code_FT8607_SHORT_CIRCUIT_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			FTS_TEST_DBG(" Code_FT8607_SHORT_CIRCUIT_TEST.");

			ReCode = FT8607_TestItem_ShortCircuitTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FT8607_LCD_NOISE_TEST
		if(Code_FT8607_LCD_NOISE_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			FTS_TEST_DBG(" Code_FT8607_LCD_NOISE_TEST.");

			ReCode =  FT8607_TestItem_LCDNoiseTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}
		
		/*
		///////////////////////////////////////////////////////FT8607_NOISE_TEST
		if(Code_FT8607_NOISE_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FT8607_TestItem_NoiseTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}
		*/

		///////////////////////////////////////////////////////FT8607_CB_TEST
		if(Code_FT8607_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			FTS_TEST_DBG(" Code_FT8607_CB_TEST.");

			ReCode = FT8607_TestItem_CbTest(&bTempResult); //
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
	FTS_TEST_DBG("");

	ret = AllocateMemory();//Allocate pointer Memory
	if(ret < 0)
		return -1;

	InitStoreParamOfTestData();

	g_stSCapConfEx.ChannelXNum = 0;
	g_stSCapConfEx.ChannelYNum = 0;
	g_stSCapConfEx.KeyNum = 0;
	g_stSCapConfEx.KeyNumTotal = 6;

	FTS_TEST_DBG("[focal] %s.", IC_TEST_VERSION);	//show lib version
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
* Name: FT8607_get_test_data
* Brief:  get data of test result
* Input: none
* Output: pTestData, the returned buff
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int FT8607_get_test_data(char *pTestData)
{
	FTS_TEST_DBG("");

	if(NULL == pTestData)
	{
		FTS_TEST_DBG("[focal] %s pTestData == NULL \n", __func__);	
		return -1;
	}
	memcpy(pTestData, g_pStoreAllData, (g_lenStoreMsgArea+g_lenStoreDataArea));
	return (g_lenStoreMsgArea+g_lenStoreDataArea);	
}

/************************************************************************
* Name: FT8607_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT8607_TestItem_EnterFactoryMode(void)

{	
	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;	//�������ɹ�,�ظ�����5��
	int i ;
	SysDelay(150);
	FTS_TEST_DBG("Enter factory mode...\n");
	for(i = 1; i <= iRedo; i++)
	{
		ReCode = EnterFactory();
		if(ERROR_CODE_OK != ReCode)
		{
			FTS_TEST_DBG("Failed to Enter factory mode...\n");
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

	if(ReCode == ERROR_CODE_OK)	//������ģʽ�ɹ���,�Ͷ��ͨ����
	{	
		ReCode = GetChannelNum();

		////////////����FIR,0���ر�,1������
		//WriteReg(0xFB, 0);
		//SysDelay(50);
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
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3] = {0};
	unsigned char pReadData[ByteNum];
	//unsigned char pReadDataTmp[ByteNum*2];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1=0;

	FTS_TEST_DBG("");
	
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

	if(ReCode != ERROR_CODE_OK)
	{	
		FTS_TEST_DBG("Failed to write REG_LINE_NUM! ");
		goto READ_ERR;
	}

	//***********************************************************Read raw data in test mode1		
	I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
	if(ReCode == ERROR_CODE_OK)
	{
		focal_msleep(10);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
		if(ReCode != ERROR_CODE_OK)
		{	
			FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!1 ");
			goto READ_ERR;
		}
	}

	for(i=1; i<iReadNum; i++)
	{
		if(ReCode != ERROR_CODE_OK) break;

		if(i==iReadNum-1)//last packet
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, pReadData+BYTES_PER_TIME*i, ByteNum-BYTES_PER_TIME*i);
			if(ReCode != ERROR_CODE_OK)
			{	
				FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!2 ");
				goto READ_ERR;
			}
		}
		else
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, pReadData+BYTES_PER_TIME*i, BYTES_PER_TIME);	
			if(ReCode != ERROR_CODE_OK)
			{	
				FTS_TEST_DBG("read rawdata Comm_Base_IIC_IO Failed!3 ");
				goto READ_ERR;
			}
		}

	}

	if(ReCode == ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (pReadData[i<<1]<<8)+pReadData[(i<<1)+1];
			//if(pRevBuffer[i] & 0x8000)//?з???λ
			//{
			//	pRevBuffer[i] -= 0xffff + 1;
			//}
		}
	}

READ_ERR:
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
	FTS_TEST_DBG("");

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
	
	/*g_pStoreAllData =NULL;	
	if(NULL == g_pStoreAllData)
	g_pStoreAllData = fts_malloc(1024*8);*/
	
	g_pTmpBuff =NULL;	
	if(NULL == g_pTmpBuff)
		g_pTmpBuff = fts_malloc(BUFF_LEN_TMP_BUFFER);
	if(NULL == g_pTmpBuff)
		goto ERR;
	
	return 0;

	FTS_TEST_PRINT("[FTS] %s end. \n", __func__);
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
	FTS_TEST_DBG("");
	
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
	FTS_TEST_DBG("");
	
	g_lenStoreMsgArea = 0;
	//Msg Area, Add Line1
	g_lenStoreMsgArea += sprintf(g_pStoreMsgArea,"ECC, 85, 170, IC Name, %s, IC Code, %x\n", g_strIcName,  g_ScreenSetParam.iSelectedIC);

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

	FTS_TEST_DBG("");

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

	FTS_TEST_DBG("[focal] %s lenStoreMsgArea=%d,  lenStoreDataArea = %d\n", __func__, g_lenStoreMsgArea, g_lenStoreDataArea);
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

	FTS_TEST_DBG("");
	
	//Save  Msg (ItemCode is enough, ItemName is not necessary, so set it to "NA".)
	iLen= sprintf(g_pTmpBuff,"NA, %d, %d, %d, %d, %d, ", \
		m_ucTestItemCode, Row, Col, m_iStartLine, ItemCount);
	memcpy(g_pMsgAreaLine2+g_lenMsgAreaLine2, g_pTmpBuff, iLen);
	g_lenMsgAreaLine2 += iLen;

	m_iStartLine += Row;
	m_iTestDataCount++;

	//Save Data 
	for(i = 0+iArrayIndex; (i < Row+iArrayIndex) && (i < TX_NUM_MAX); i++)
	{
		for(j = 0; (j < Col) && (j < RX_NUM_MAX); j++)
		{
			if(j == (Col -1))//The Last Data of the Row, add "\n"
				iLen= sprintf(g_pTmpBuff,"%d, \n", iData[i][j]);	
			else
				iLen= sprintf(g_pTmpBuff,"%d, ", iData[i][j]);	

			memcpy(g_pStoreDataArea+g_lenStoreDataArea, g_pTmpBuff, iLen);
			g_lenStoreDataArea += iLen;		
		}
	}
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

	FTS_TEST_DBG("");

	ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
	if(ReCode == ERROR_CODE_OK)
	{
		RegVal |= 0x80;		//����λ��1������ɨ��
		ReCode = WriteReg(DEVIDE_MODE_ADDR, RegVal);
		if(ReCode == ERROR_CODE_OK)
		{
			while(times++ < MaxTimes)		//�ȴ�ɨ������
			{
				SysDelay(8);	//8ms
				ReCode = ReadReg(DEVIDE_MODE_ADDR, &RegVal);
				if(ReCode == ERROR_CODE_OK)
				{
					if((RegVal>>7) == 0)	break;
				}
				else
				{
					FTS_TEST_DBG("StartScan read DEVIDE_MODE_ADDR error.\n");
					break;
				}
			}
			if(times < MaxTimes)	ReCode = ERROR_CODE_OK;
			else ReCode = ERROR_CODE_COMM_ERROR;
		}
		else 
			FTS_TEST_DBG("StartScan write DEVIDE_MODE_ADDR error.\n");
	}
	else
		FTS_TEST_DBG("StartScan read DEVIDE_MODE_ADDR error.\n");
	return ReCode;

}	

/************************************************************************
* Name: ShowRawData
* Brief:  Show RawData
* Input: none
* Output: none
* Return: none.
***********************************************************************/
/*static void ShowRawData(void)
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
}*/

/************************************************************************
* Name: GetTxRxCB(Same function name as FT_MultipleTest)
* Brief:  get CB of Tx/Rx
* Input: StartNodeNo, ReadNum
* Output: pReadBuffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetTxRxCB(unsigned short StartNodeNo, unsigned short ReadNum, unsigned char *pReadBuffer)
{
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned short usReturnNum = 0;//ÿ��Ҫ���صĸ���
	unsigned short usTotalReturnNum = 0;//�ܷ��ظ���
	unsigned char wBuffer[4] = {0};	
	int i = 0, iReadNum = 0;

	FTS_TEST_DBG("");
	
	iReadNum = ReadNum/BYTES_PER_TIME;

	if(0 != (ReadNum%BYTES_PER_TIME)) iReadNum++;

	wBuffer[0] = REG_CbBuf0;

	usTotalReturnNum = 0;

	for(i = 1; i <= iReadNum; i++)
	{
		if(i*BYTES_PER_TIME > ReadNum)
			usReturnNum = ReadNum - (i-1)*BYTES_PER_TIME;
		else
			usReturnNum = BYTES_PER_TIME;	

		wBuffer[1] = (StartNodeNo+usTotalReturnNum) >>8;//��ַƫ�����8λ
		wBuffer[2] = (StartNodeNo+usTotalReturnNum)&0xff;//��ַƫ�����8λ

		ReCode = WriteReg(REG_CbAddrH, wBuffer[1]);
		ReCode = WriteReg(REG_CbAddrL, wBuffer[2]);
		//ReCode = fts_i2c_read_test(wBuffer, 1, pReadBuffer+usTotalReturnNum, usReturnNum);
		ReCode = Comm_Base_IIC_IO(wBuffer, 1, pReadBuffer+usTotalReturnNum, usReturnNum);

		usTotalReturnNum += usReturnNum;

		if(ReCode != ERROR_CODE_OK)return ReCode;

		//if(ReCode < 0) return ReCode;
	}

	return ReCode;
}

//***********************************************
//��ȡPanelRows
//***********************************************
static unsigned char GetPanelRows(unsigned char *pPanelRows)
{
	return ReadReg(REG_TX_NUM, pPanelRows);
}

//***********************************************
//��ȡPanelCols
//***********************************************
static unsigned char GetPanelCols(unsigned char *pPanelCols)
{
	return ReadReg(REG_RX_NUM, pPanelCols);
}

/************************************************************************
* Name: GetChannelNum
* Brief:  Get Num of Ch_X, Ch_Y and key
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	//int TxNum, RxNum;
	int i ;
	unsigned char rBuffer[1]; //= new unsigned char;

	//FTS_TEST_DBG("Enter GetChannelNum...\n");
	//--------------------------------------------"Get Channel X Num...";
	for(i = 0; i < 3; i++)
	{
		ReCode = GetPanelRows(rBuffer);
		if(ReCode == ERROR_CODE_OK)
		{
			if(0 < rBuffer[0] && rBuffer[0] < 80)
			{
				g_stSCapConfEx.ChannelXNum = rBuffer[0];
				if(g_stSCapConfEx.ChannelXNum > g_ScreenSetParam.iUsedMaxTxNum)
				{
					FTS_TEST_DBG("Failed to get Channel X number, Get num = %d, UsedMaxNum = %d\n",
						g_stSCapConfEx.ChannelXNum, g_ScreenSetParam.iUsedMaxTxNum);
					return ERROR_CODE_INVALID_PARAM;
				}			
				break;
			}
			else
			{
				SysDelay(150);
				continue;
			}
		}
		else
		{
			FTS_TEST_DBG("Failed to get Channel X number\n");
			SysDelay(150);
		}
	}

	//--------------------------------------------"Get Channel Y Num...";
	for(i = 0; i < 3; i++)
	{
		ReCode = GetPanelCols(rBuffer);
		if(ReCode == ERROR_CODE_OK)
		{
			if(0 < rBuffer[0] && rBuffer[0] < 80)
			{
				g_stSCapConfEx.ChannelYNum = rBuffer[0];
				if(g_stSCapConfEx.ChannelYNum > g_ScreenSetParam.iUsedMaxRxNum)
				{
					FTS_TEST_DBG("Failed to get Channel Y number, Get num = %d, UsedMaxNum = %d\n",
						g_stSCapConfEx.ChannelYNum, g_ScreenSetParam.iUsedMaxRxNum);
					return ERROR_CODE_INVALID_PARAM;
				}				
				break;
			}
			else
			{
				SysDelay(150);
				continue;
			}
		}
		else
		{
			FTS_TEST_DBG("Failed to get Channel Y number\n");
			SysDelay(150);
		}
	}

	//--------------------------------------------"Get Key Num...";
	for(i = 0; i < 3; i++)
	{
		unsigned char regData = 0;
		g_stSCapConfEx.KeyNum = 0;
		ReCode = ReadReg( FT_8607_LEFT_KEY_REG, &regData );
		if(ReCode == ERROR_CODE_OK)
		{
			if( ( (regData >> 0 ) & 0x01 ) ) { g_stSCapConfEx.bLeftKey1 = true; ++g_stSCapConfEx.KeyNum; }
			if( ( (regData >> 1 ) & 0x01 ) ) { g_stSCapConfEx.bLeftKey2 = true; ++g_stSCapConfEx.KeyNum; }
			if( ( (regData >> 2 ) & 0x01 ) ) { g_stSCapConfEx.bLeftKey3 = true; ++g_stSCapConfEx.KeyNum; }
		}
		else
		{
			FTS_TEST_DBG("Failed to get Key number\n");
			SysDelay(150);
			continue;
		}
		ReCode = ReadReg( FT_8607_RIGHT_KEY_REG, &regData );
		if(ReCode == ERROR_CODE_OK)
		{
			if( ( (regData >> 0 ) & 0x01 ) ) { g_stSCapConfEx.bRightKey1 = true; ++g_stSCapConfEx.KeyNum; }
			if( ( (regData >> 1 ) & 0x01 ) ) { g_stSCapConfEx.bRightKey2 = true; ++g_stSCapConfEx.KeyNum; }
			if( ( (regData >> 2 ) & 0x01 ) ) { g_stSCapConfEx.bRightKey3 = true; ++g_stSCapConfEx.KeyNum; }
			break;
		}
		else
		{
			FTS_TEST_DBG("Failed to get Key number\n");
			SysDelay(150);
			continue;
		}
	}


	FTS_TEST_DBG("CH_X = %d, CH_Y = %d, Key = %d\n", g_stSCapConfEx.ChannelXNum ,g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNum );
	return ReCode;
}

//=================================================================
//
//Color Type: 0-Red, 1-Green 2-Blue
//
//================================================================
/*
void TestResultInfo(CString strInfo, unsigned char ucColorType)
{
	if( 0 == ucColorType ){           
		//NG
		m_strTestResult += ColorText( strInfo, RGB( 255, 0, 0 ), TEXT_SIZE_LARGE, true );
	}
	else if( 1 == ucColorType ){
		//PASS
		m_strTestResult += ColorText( strInfo, RGB( 0, 255, 0 ), TEXT_SIZE_LARGE, true );
	}
	else{
		m_strTestResult += strInfo;
	}
}
*/
/*
void TestResultInfo( const ColorText& text )
{
    m_strTestResult += text;
}
*/

/*
void TestResult(int bIsPass)
{
	CString strTemp;
	strTemp = "Graphics Test: ";
	if(0 == bIsPass) //pass
	{
		//m_strCurrentTestMsg = "Test result: PASS";
		m_strCurrentTestMsg = strTemp + "PASS";
	}
	else if(1 == bIsPass) //NG
	{
		//m_strCurrentTestMsg = "Test result: NG";
		m_strCurrentTestMsg = strTemp + "NG";
	}
	else if(2 == bIsPass)
	{
		//m_strCurrentTestMsg = "Test result: Testing";
		m_strCurrentTestMsg = strTemp + "Testing";
	}

	CString str;
	SYSTEMTIME st = {0};
	GetLocalTime(&st);//���õ�ǰ����ʱ��,�����Խ���ʱ��
	str.Format("\r\n\r\n========== Finish Test Date: %d-%02d-%02d %02d:%02d:%02d (+%03dms ) \r\n",
		st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	TestResultInfo(str);
}
*/
/************************************************************************
* Name: GetRawData
* Brief:  Get Raw Data of VA area and Key area
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char GetRawData(void)
{
	int ReCode = ERROR_CODE_OK;
	int iRow = 0, iCol = 0;

	//--------------------------------------------Enter Factory Mode
	ReCode = EnterFactory();	
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Enter Factory Mode...\n");
		return ReCode;
	}

	//--------------------------------------------Check Num of Channel 
	if(0 == (g_stSCapConfEx.ChannelXNum + g_stSCapConfEx.ChannelYNum)) 
	{
		ReCode = GetChannelNum();
		if( ERROR_CODE_OK != ReCode ) 
		{
			FTS_TEST_DBG("Error Channel Num...\n");
			return ERROR_CODE_INVALID_PARAM;
		}
	}
	//--------------------------------------------Start Scanning
	//FTS_TEST_DBG("Start Scan ...\n");
	ReCode = StartScan();
	if(ERROR_CODE_OK != ReCode) 
	{
		FTS_TEST_DBG("Failed to Scan ...\n");
		return ReCode;
	}

	//--------------------------------------------Read RawData for Channel Area
	//FTS_TEST_DBG("Read RawData...\n");
	memset(m_RawData, 0, sizeof(m_RawData));
	memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
	ReCode = ReadRawData(0, 0xAD, g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum * 2, m_iTempRawData);
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Get RawData of channel.\n");
		return ReCode;
	}
	for (iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow)
	{
		for (iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol)
		{
			m_RawData[iRow][iCol] = m_iTempRawData[iRow * g_stSCapConfEx.ChannelYNum + iCol];
		}
	}

	memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
	ReCode = ReadRawData( 0, 0xAE, g_stSCapConfEx.KeyNumTotal* 2, m_iTempRawData );
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Get RawData of keys.\n");
		return ReCode;
	}

	for (iCol = 0; iCol < g_stSCapConfEx.KeyNumTotal; ++iCol)
	{
		m_RawData[g_stSCapConfEx.ChannelXNum][iCol] = m_iTempRawData[iCol];
	}
	//m_strCurrentTestMsg = "Finish Get RawData...";

	return ReCode;
}

unsigned char FT8607_TestItem_RawDataTest(bool * bTestResult)
{
	unsigned char ReCode = ERROR_CODE_OK;
	bool btmpresult = true;
	//int iMax, iMin, iAvg;
	int RawDataMin = 0;
	int RawDataMax = 0;
	int iValue = 0;
	int i=0;
	int iRow = 0, iCol = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Raw Data Test\n\n");

	for(i = 0 ; i < 3; i++)//Lost 3 Frames, In order to obtain stable data
		ReCode = GetRawData();//��ȡRawData�ͼ���Differֵ
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to get Raw Data!! Error Code: %d\n", ReCode);
		return ReCode;
	}
	//----------------------------------------------------------Show RawData
	FTS_TEST_DBG("\nVA Channels: ");
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
	//	FTS_TEST_DBG("\nCh_%02d:  ", iRow+1);
		FTS_TEST_PRINT("\nCh_%02d:  ", iRow+1);
		for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
		{
		//	FTS_TEST_DBG("%5d, ", m_RawData[iRow][iCol]);
			FTS_TEST_PRINT("%5d, ", m_RawData[iRow][iCol]);
		}
	}
	FTS_TEST_DBG("\nKeys:  ");
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; iCol++ )
	{
	//	FTS_TEST_DBG("%5d, ",  m_RawData[g_stSCapConfEx.ChannelXNum][iCol]);
		FTS_TEST_PRINT("%5d, ",  m_RawData[g_stSCapConfEx.ChannelXNum][iCol]);
	}
	FTS_TEST_PRINT("\n");
	
	//----------------------------------------------------------�жϳ�����Χ��rawData
	//----------------------------------------------------------To Determine RawData if in Range or not
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{

		for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
		{
			if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
			RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_Min[iRow][iCol];
			RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_Max[iRow][iCol];
			iValue = m_RawData[iRow][iCol];
			if(iValue < RawDataMin || iValue > RawDataMax)
			{
				btmpresult = false;
				/*
				FTS_TEST_PRINT("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
					iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
				*/
			}
		}
	}
	
	iRow = g_stSCapConfEx.ChannelXNum;
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; iCol++ )
	{
		if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
		RawDataMin = g_stCfg_MCap_DetailThreshold.RawDataTest_Min[iRow][iCol];
		RawDataMax = g_stCfg_MCap_DetailThreshold.RawDataTest_Max[iRow][iCol];
		iValue = m_RawData[iRow][iCol];
		if(iValue < RawDataMin || iValue > RawDataMax)
		{
			btmpresult = false;
			/*
			FTS_TEST_PRINT("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", \
				iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
			*/
		}	
	}

	//////////////////////////////Save Test Data
	Save_Test_Data(m_RawData, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);
	//----------------------------------------------------------Return Result
	if(btmpresult)
	{
		* bTestResult = true;
		FTS_TEST_DBG("\n\n//RawData Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//RawData Test is NG!\n");
	}
	return ReCode;
}

/************************************************************************
* Name: SqrtNew
* Brief:  calculate sqrt of input.
* Input: unsigned int n
* Output: none
* Return: sqrt of n.
***********************************************************************/
static unsigned int SqrtNew(unsigned int n) 
{        
    unsigned int  val = 0, last = 0; 
    unsigned char i = 0;;
    
    if (n < 6)
    {
        if (n < 2)
        {
            return n;
        }
        return n/2;
    }   
    val = n;
    i = 0;
    while (val > 1)
    {
        val >>= 1;
        i++;
    }
    val <<= (i >> 1);
    val = (val + val + val) >> 1;
    do
    {
      last = val;
      val = ((val + n/val) >> 1);
    }while(focal_abs(val-last) > pre);
    return val; 
}

//static float iRawDataAvr[TX_NUM_MAX][RX_NUM_MAX] = { {0} };
static int LCD_Noise[TX_NUM_MAX][RX_NUM_MAX] = { {0} };	// float
//static int minHole[TX_NUM_MAX][RX_NUM_MAX] = { {0} };
//static int maxHole[TX_NUM_MAX][RX_NUM_MAX] = { {0} };
/************************************************************************
* Name: FT8607_TestItem_LCDNoiseTest
* Brief:  TestItem: LCD NoiseTest. Check if MCAP Noise is within the range.
* Input: bTestResult
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT8607_TestItem_LCDNoiseTest(bool* bTestResult)
{
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;
	bool bResultFlag = true;
	unsigned char regData = 0, oldMode = 0;

	int iRow = 0;
	int iCol = 0;

	int iValueMin = 0;
	int iValueMax = 0;
	int iValue = 0;

	FTS_TEST_DBG("==============================Test Item: -------- LCD Noise Test \r\n");

	//is differ mode
	ReadReg( 0x06, &oldMode );
	WriteReg( 0x06, 1 );

	FTS_TEST_DBG(" iLCDNoiseTestFrame:%d. ",  g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestFrame );

	regData = (unsigned char)(g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestFrame & 0XFF );
	ReCode = WriteReg( REG_8607_LCD_NOISE_FRAME, regData);

	ReCode = WriteReg( REG_8607_LCD_NOISE_START, 0x01 );

	ReCode = StartScan();

	GetRawData();

	ReCode = ReadReg( REG_8607_LCD_NOISE_NUMBER, &regData );
	if( ERROR_CODE_OK != regData ) 
	{
		FTS_TEST_DBG(" ReadReg error:%d. ", ReCode);	
		return ReCode;
	}
	FTS_TEST_DBG("regData get:%d. ", regData);
		
	if( regData <= 0 ) regData = 1;
	FTS_TEST_DBG("regData set to:%d. ", regData);

	WriteReg( 0x06, oldMode );

//return ReCode;

	for ( iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow )
	{
		for ( iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol )
		{
		//	LCD_Noise[iRow][iCol] = ::sqrtf((float)m_RawData[iRow][iCol] / regData);
			LCD_Noise[iRow][iCol] = SqrtNew(m_RawData[iRow][iCol] / regData);			
		}
	}

	//show data of LCD_Noise
#if 1
	FTS_TEST_PRINT("\nVA Channels: ");
	for ( iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow )
	{
		FTS_TEST_PRINT("\nCh_%02d:  ", iRow+1);
		for ( iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol )
		{
		//	FTS_TEST_PRINT("%d, ", m_RawData[iRow][iCol] / regData);		
			FTS_TEST_PRINT("%4d, ", LCD_Noise[iRow][iCol]);
		}
	}
	FTS_TEST_PRINT("\n");
#endif

/*
	{
		NodeVal nodeOutRange;
		memset( minHole, MIN_HOLE_LEVEL, sizeof( minHole ) );
		ArrayFillWithVal( maxHole, g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestMax );
		AnalyzeInfoIncell info( g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNumTotal, false );
		bResult = AnalyzeTestResultInCell( LCD_Noise, minHole, maxHole, 
			g_stCfg_Incell_DetailThreshold.InvalidNode, info, textTemp, nodeOutRange );

		if( !bResult )
		{
			FTS_TEST_DBG("\r\n//========= Out of Threshold in LCD Noise Test: \r\n");
			bResultFlag = false;
		}

		SaveTestData( GetSaveMatrixData( LCD_Noise, g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum ),
			g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, 1 );
	}
*/

	//////////////////////// analyze
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
		for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
		{
			if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
			
			iValueMin = MIN_HOLE_LEVEL;
			iValueMax = g_stCfg_FT8607_BasicThreshold.iLCDNoiseTestMax;
			iValue = LCD_Noise[iRow][iCol];
			if(iValue < iValueMin || iValue > iValueMax)
			{
				bResultFlag = false;
				FTS_TEST_PRINT(" LCD Noise test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d). \n", 
					iRow+1, iCol+1, iValue, iValueMin, iValueMax);
			}
		}
	}

	if( !bResultFlag )
	{
		FTS_TEST_DBG("//========= Out of Threshold in LCD Noise Test.");
	}
	////////////////////////		

	//	Save Test Data
	Save_Test_Data(LCD_Noise, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);

//TEST_END:

	if(bResultFlag)
	{
		FTS_TEST_DBG("		//LCD Noise Test is OK!");
		* bTestResult = true;
	}
	else
	{
		FTS_TEST_DBG("		//LCD Noise Test is NG!");
		* bTestResult = false;
	}
	return ReCode;
}


/************************************************************************
* Name: FT8607_TestItem_CbTest
* Brief:  TestItem: Cb Test. Check if Cb is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FT8607_TestItem_CbTest(bool * bTestResult)
{
	bool btmpresult = true;
	unsigned char ReCode = ERROR_CODE_OK;
	int iRow = 0;
	int iCol = 0;
//	int readlen = g_stSCapConf.ChannelsNum + g_stSCapConf.KeyNum;
	int iMaxValue = 0;
	int iMinValue = 0;
//	int i=0;

	FTS_TEST_DBG("\n\n==============================Test Item: --------  CB Test\n\n");

	/*
	unsigned char bClbResult = 0;
	for (int i=0; i<10; i++)
	{
		ReCode = ChipClb( &bClbResult );
		SysDelay(50);
		if( 0 != bClbResult) 
		{
			break;
		} 		
	} 
	if( FALSE == bClbResult) 
	{
		TestResultInfo( _T("\r\nReCalib Failed\r\n") );
		btmpresult = false;
	}
	*/

//	ReCode = GetTxRxCB( 0, (UINT16)g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum + g_stSCapConfEx.KeyNumTotal, chCBData );
	ReCode = GetTxRxCB( 0, (short)(g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum + g_stSCapConfEx.KeyNumTotal), m_ucTempData );

	if( ERROR_CODE_OK != ReCode )
	{
		btmpresult = false;
		FTS_TEST_DBG("Failed to get CB value...\n");
		goto TEST_ERR;
	}

	memset(m_CBData, 0, sizeof(m_CBData));

	for ( iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow )
	{
		for ( iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol )
		{
			m_CBData[iRow][iCol] = m_ucTempData[ iRow * g_stSCapConfEx.ChannelYNum + iCol ];
		}
	}
	
	///key
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNumTotal; ++iCol )
	{
		m_CBData[g_stSCapConfEx.ChannelXNum][iCol] = m_ucTempData[ g_stSCapConfEx.ChannelXNum*g_stSCapConfEx.ChannelYNum + iCol ];
	}

	//------------------------------------------------Show CbData	
	FTS_TEST_PRINT("\nVA Channels:  \n");
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
		FTS_TEST_PRINT("\nCh_%02d:  ", iRow+1);
		for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
		{
			FTS_TEST_PRINT("%3d, ", m_CBData[iRow][iCol]);
		}
	}
	
	FTS_TEST_PRINT("\nKeys:  \n");
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNumTotal; iCol++ )
	{
		FTS_TEST_PRINT("%3d, ",  m_CBData[g_stSCapConfEx.ChannelXNum][iCol]);
	}
	FTS_TEST_PRINT("\n");

	iMinValue = g_stCfg_FT8607_BasicThreshold.CbTest_Min;
	iMaxValue = g_stCfg_FT8607_BasicThreshold.CbTest_Max;	

	for(iRow = 0;iRow < (g_stSCapConfEx.ChannelXNum + 1);iRow++)
	{
		for(iCol = 0;iCol < g_stSCapConfEx.ChannelYNum;iCol++)
		{
			if( (0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) )  
			{
				continue;
			}
//			if( iRow >= g_stSCapConfEx.ChannelXNum && iCol >= g_stSCapConfEx.KeyNum ) 
			if( iRow >= g_stSCapConfEx.ChannelXNum && iCol >= g_stSCapConfEx.KeyNumTotal) 				
			{
				continue;
			}

			if(focal_abs(m_CBData[iRow][iCol]) < iMinValue || focal_abs(m_CBData[iRow][iCol]) > iMaxValue)
			{
				btmpresult = false;
			//	FTS_TEST_PRINT("CB test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) \n", 
			//		iRow+1, iCol+1, m_CBData[iRow][iCol], iMinValue, iMaxValue);
			}
		}
	}

	//////////////////////////////Save Test Data
	Save_Test_Data(m_CBData, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);

	if(btmpresult)
	{
		* bTestResult = true;
		FTS_TEST_DBG("\n\n//CB Test is OK!\n");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//CB Test is NG!\n");
	}

	return ReCode;
TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//CB Test is NG!\n");
	return ReCode;	

}

static unsigned char pReadBuffer[80 * 80 * 2] = {0};
static unsigned char WeakShort_GetAdcData( int AllAdcDataLen, int *pRevBuffer  )
{
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char RegMark = 0;
	int index = 0;
	int i = 0;
	int usReturnNum = 0;
	unsigned char wBuffer[2] = {0};
	unsigned short usTotalReturnNum = 0;//�ܷ��ظ���
	int iReadNum = AllAdcDataLen / 342;

	memset( wBuffer, 0, sizeof(wBuffer) );
	wBuffer[0] = 0x89;

	FTS_TEST_DBG("");
		
	if((AllAdcDataLen % 342) > 0) ++iReadNum;

	ReCode = WriteReg( 0x0F, 1 );  //����ADC����
	if( ERROR_CODE_OK != ReCode )
	{
		FTS_TEST_DBG("WriteReg failed.");
		return 3;
	}

	for ( index = 0; index < 50; ++index )
	{
		SysDelay( 50 );
		ReCode = ReadReg( 0x10, &RegMark );  //��ѯ�����������
		if( ERROR_CODE_OK == ReCode && 0 == RegMark )
			break;
	}
	if( index >= 50)
	{
		FTS_TEST_DBG("ReadReg failed.");
		return 6;
	}

	/////
//	if(HY_I2C_INTERFACE == iCommMode)
	{
		usTotalReturnNum = 0;

		usReturnNum = 342;
		if(ReCode == ERROR_CODE_OK)
		{
			ReCode = Comm_Base_IIC_IO(wBuffer, 1, pReadBuffer, usReturnNum);
		}

		for( i=1; i<iReadNum; i++)
		{
			if(ReCode != ERROR_CODE_OK) 
			{
				FTS_TEST_DBG("Comm_Base_IIC_IO  error.   !!!");				
				break;
			}

			if(i==iReadNum-1)//last packet
			{
				usReturnNum = AllAdcDataLen-342*i;
				ReCode = Comm_Base_IIC_IO(NULL, 0, pReadBuffer+342*i, usReturnNum);
			}
			else
			{
				usReturnNum = 342;
				ReCode = Comm_Base_IIC_IO(NULL, 0, pReadBuffer+342*i, usReturnNum);		
			}	
		}
	}

	for ( index = 0; index < AllAdcDataLen/2; ++index )
	{
		pRevBuffer[index] = (pReadBuffer[index * 2] << 8) + pReadBuffer[index * 2 + 1];
	}

	FTS_TEST_DBG(" END.\n");
	return ReCode;
}

static int iAdcData[TX_NUM_MAX * RX_NUM_MAX] =  {0};
//static int minHole[TX_NUM_MAX][RX_NUM_MAX] = { {0} };
//static int maxHole[TX_NUM_MAX][RX_NUM_MAX] = { {100000000}};	//	ArrayFillWithVal( maxHole, 100000000 );
static int shortRes[TX_NUM_MAX][RX_NUM_MAX] = { {0} };		//	static float shortRes[TX_NUM_MAX][RX_NUM_MAX] = {0.0};
unsigned char FT8607_TestItem_ShortCircuitTest(bool* bTestResult)
{
	unsigned char ReCode = ERROR_CODE_OK;
	bool bTempResult=true;

	int ResMin = g_stCfg_FT8607_BasicThreshold.ShortCircuit_ResMin;

	int iAllAdcDataNum = 0;
	unsigned char iTxNum = 0, iRxNum = 0, iChannelNum = 0;

	int iRow = 0;
	int iCol = 0;
	int i=0;
	int tmpAdc = 0;
	int iValueMin = 0;
	int iValueMax = 0;
	int iValue = 0;

	FTS_TEST_DBG("");
	FTS_TEST_DBG("==============================Test Item: -------- Short Circuit Test \r\n");
	ReCode = EnterFactory();
	if (ERROR_CODE_OK != ReCode)
	{
		bTempResult = false;
		FTS_TEST_DBG(" Failed to Enter factory mode. Error Code: %d", ReCode);
		goto TEST_END;
	}

	ReCode = ReadReg(0x02, &iTxNum);
	ReCode = ReadReg(0x03, &iRxNum);
	if (ERROR_CODE_OK != ReCode)
	{
		bTempResult = false;
		FTS_TEST_DBG("// Failed to read reg. Error Code: %d", ReCode);
		goto TEST_END;
	}

	FTS_TEST_DBG(" iTxNum:%d.  iRxNum:%d.", iTxNum, iRxNum);

	iChannelNum = iTxNum + iRxNum;
	iAllAdcDataNum = iTxNum * iRxNum + g_stSCapConfEx.KeyNumTotal;
	memset(iAdcData, 0, sizeof(iAdcData));

	for (i=0; i<1; i++)
	{
		ReCode = WeakShort_GetAdcData(iAllAdcDataNum*2, iAdcData);
		SysDelay(50);
		if (ERROR_CODE_OK != ReCode)
		{
			bTempResult = false;
			FTS_TEST_DBG(" // Failed to get AdcData. Error Code: %d", ReCode);
			goto TEST_END;
		}
	}
	
	//show ADCData 
#if 1
	FTS_TEST_DBG("ADCData:\n");
	for (i=0; i<iAllAdcDataNum; i++)
	{
		FTS_TEST_PRINT("%-4d  ",iAdcData[i]);  		
		if (0 == (i+1)%iRxNum)
		{
			FTS_TEST_PRINT("\n");  		
		}
	}
	FTS_TEST_PRINT("\n");  		
#endif

	for ( iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; ++iRow )
	{
		for ( iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol )
		{
			tmpAdc = iAdcData[iRow *iRxNum + iCol];			
			if(tmpAdc > 2007) tmpAdc = 2007;
			shortRes[iRow][iCol] = (tmpAdc * 100) / (2047 - tmpAdc);
		}
	}

	/*
	NodeVal nodeOutRange;
	ArrayFillWithVal( minHole, ResMin );
	ArrayFillWithVal( maxHole, 100000000 );
	*/	
	/*
	AnalyzeInfoIncell info( g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNumTotal, true );
	bool bResult = AnalyzeTestResultInCell( shortRes, minHole, maxHole, 
		g_stCfg_Incell_DetailThreshold.InvalidNode, info, textTemp, nodeOutRange );
	*/
	/*
	if(!bResult)
	{
		FTS_TEST_DBG("\r\n//========= Out of Threshold in Short Circuit Test: \r\n");
		bTempResult = false;
	}
	*/

	//////////////////////// analyze
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
		for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
		{
			if(g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol] == 0)continue;//Invalid Node
			
			iValueMin = ResMin;
			iValueMax = 100000000;		//	ArrayFillWithVal( maxHole, 100000000 );
			iValue = shortRes[iRow][iCol];
			if(iValue < iValueMin || iValue > iValueMax)
			{
				bTempResult = false;
				FTS_TEST_PRINT(" Short Circuit test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d). \n", \
					iRow+1, iCol+1, iValue, iValueMin, iValueMax);
			}
		}
	}

	if( !bTempResult )
	{
		FTS_TEST_DBG("//========= Out of Threshold in Short Circuit Test:.");
	}
	////////////////////////		

TEST_END:

	if(bTempResult)
	{
		FTS_TEST_DBG("		//Short Circuit Test is OK!");
		* bTestResult = true;
	}
	else
	{
		FTS_TEST_DBG("		//Short Circuit Test is NG!");
		* bTestResult = false;
	}

	return ReCode;
}

/************************************************************************
* Name: FT8607_TestItem_NoiseTest
* Brief:  TestItem: NoiseTest. Check if MCAP Noise is within the range.
* Input: bTestResult
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
/*
//	not use noise test. but use LCD noise test
unsigned char FT8607_TestItem_NoiseTest(bool* bTestResult)
{
	unsigned char ReCode;
	unsigned char chNoiseValue = 0xff;
	bool btmpresult = true;
	int iNoiseFrames = 0;
	int i = j = iRow = iCol = 0;
	int iNgNum = 0;
	int iValue = 0;
	int iMinValue = 0, iMaxValue = 0;
	int n = 0,temp = 0;
	int iSize  = 0;

	int *pTempNext=NULL;
	int *pTempPrev=NULL;

	unsigned char chOldRawDataMod = 0, chNewRawDataMod = 0;

	int index = 0;

	float iRawDataAvr[TX_NUM_MAX][RX_NUM_MAX]={0};

	int iNoiseFrames = g_stCfg_FT8607_BasicThreshold.NoiseTest_Frames;
	int iNoiseTime = g_stCfg_FT8607_BasicThreshold.NoiseTest_Time;
	bool bSampModeFrame = g_stCfg_FT8607_BasicThreshold.NoiseTest_SampeMode == 0;
	bool bNoiseTip = g_stCfg_FT8607_BasicThreshold.NoiseTest_ShowTip == 1;

	vector<int*> vecDataSampled;
	memset(m_NoiseData, 0, sizeof(m_NoiseData));

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Noise Test  \n\n");

	//////////////////////////////////////////////////////////��Noise -Start
	if( g_stCfg_FT8607_BasicThreshold.IsDifferMode )
	{
		//IsDifferģʽ����
		chOldRawDataMod = 0;
		chNewRawDataMod = 0;
		ReCode = ReadReg( 0x06, &chOldRawDataMod );
		ReCode = WriteReg( 0x06, 0x01 );
		SysDelay(10);
		ReCode = ReadReg( 0x06, &chNewRawDataMod );
		if( ReCode != ERROR_CODE_OK || chNewRawDataMod != 1 ){
			btmpresult = false;
			FTS_TEST_DBG( "Switch Mode Failed!" );
			goto TEST_END;
		}
	}


	//����һ������
	for ( index = 0; index < 3; index++ )
	{
		ReCode = GetRawData();
	}

	if( bSampModeFrame )
	{
		SampeleByFrame( iNoiseFrames, vecDataSampled );
	}
	else
	{
		SampleByTime( iNoiseTime, vecDataSampled );
	} 

	//////////////////////////////////////////calculate average Start

	for(i=0; i<g_stSCapConfEx.ChannelXNum + 1; i++)
	{
		for(j=0; j<g_stSCapConfEx.ChannelYNum; j++)
		{
			iSize = vecDataSampled.size();
			if( iSize ) m_NoiseData[i][j] /= iSize;
		}
	}
	memcpy(iRawDataAvr, m_NoiseData, sizeof(m_NoiseData));
	memset(m_NoiseData, 0, sizeof(m_NoiseData));
	//////////////////////////////////////////calculate average End

	if( 0 == g_stCfg_FT8607_BasicThreshold.NoiseTest_NoiseMode )
	{
		CaculateNoiseBaseOnAve( iRawDataAvr, vecDataSampled );
	}
	else if( 1 == g_stCfg_FT8607_BasicThreshold.NoiseTest_NoiseMode)
	{
		CaculateNoiseBaseOnMax( iRawDataAvr, vecDataSampled );
	}
	else if( 2 == g_stCfg_FT8607_BasicThreshold.NoiseTest_NoiseMode)
	{
		CaculateNoiseBaseOnMaxMin( iRawDataAvr, vecDataSampled );
	}
	else if( 3 == g_stCfg_FT8607_BasicThreshold.NoiseTest_NoiseMode )
	{
		CaculateNoiseBaseOnDiffer( iRawDataAvr, vecDataSampled );
	}

	unsigned char chNoiseValue = 0xff;
	ReCode = theDevice.m_cHidDev[m_NumDevice]->EnterWork();
	SysDelay(100);
	ReCode = ReadReg( 0x80, &chNoiseValue );
	ReCode = EnterFactory();
	SysDelay(100);

	{
		NodeVal nodeOutRange;
		AnalyzeInfoIncell info( g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNumTotal, true );
		int minHole[TX_NUM_MAX][RX_NUM_MAX];
		int maxHole[TX_NUM_MAX][RX_NUM_MAX];
		memset( minHole, MIN_HOLE_LEVEL, sizeof(minHole) );

		ArrayFillWithVal( maxHole, g_stCfg_FT8607_BasicThreshold.NoiseTest_Coefficient * chNoiseValue * 32 / 100 );

		bool bResult = AnalyzeTestResultInCell( m_NoiseData, minHole, maxHole,
			g_stCfg_MCap_DetailThreshold.InvalidNode, info, textTemp, nodeOutRange );

	//	TestResultInfo( textTemp );

#ifdef  OUTPUT_MAXMIN
		//����noise��max,minֵ
		float MaxVal=0;
		float MinVal=65535;
		CalArrayMaxMin(m_NoiseData,g_stCfg_Incell_DetailThreshold.InvalidNode,info,MaxVal,MinVal);

		str.Format("\r\nNoise Max:%.2f, Min:%.2f\r\n",MaxVal,MinVal);
		TestResultInfo(str);
#endif

		if( !bResult )
		{
			CString strHead = _T("\r\n Out of Threshold in Noise Test:\r\n");
			PrintNodeValue( &nodeOutRange, strHead, 2 );
			btmpresult = false;
		}
	}

	////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�ռ���������,����CSV�ļ�
// 	strTemp = "";
// 	for(int i = 0; i < g_stSCapConfEx.ChannelXNum; i++)
// 	{		
// 		strTemp += "\r\n";
// 		for(int j = 0; j < g_stSCapConfEx.ChannelYNum; j++)
// 		{	
// 			str.Format("%0.2lf,", m_NoiseData[i][j]);
// 			strTemp += str;
// 		}
// 	}

	SaveTestData(GetSaveMatrixData( m_NoiseData, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 2 ),
		g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum);

	////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<�ռ���������,����CSV�ļ�

TEST_END:
	ReCode = WriteReg( 0x06, 0x00 );
	SysDelay(20);
	for ( int index = 0; index < 2; index++ )
	{
		StartScan(1);
	}

	if(btmpresult)
	{
		TestResultInfo("\r\n\r\n//Noise Test is OK!\r\n",1);
		* bTestResult = true;			
	}
	else
	{
		TestResultInfo("\r\n\r\n//Noise Test is NG!\r\n",0);
		* bTestResult = false;			
	}

	for ( int index = 0; index < (int)vecDataSampled.size(); index++ )
	{
		if( NULL != vecDataSampled.at( index ) )  delete[] vecDataSampled.at( index );
	}
	vecDataSampled.clear();
	//if(NULL != pFrameDataBuf) free(pFrameDataBuf);

	return ReCode;
}
*/

