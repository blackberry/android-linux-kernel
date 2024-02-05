/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: Test_FTE736.c
*
* Author: Software Development 
*
* Created: 2016-05-19
*
* Abstract: test item for FTE736
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "focaltech_test_global.h"
#include "focaltech_test_ftE736.h"
#include "focaltech_test_detail_threshold.h"
#include "focaltech_test_config_ftE736.h"
//#include "Comm_FTE736.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define IC_TEST_VERSION  "Test version: V1.0.0--2015-12-24, (sync version of FT_MultipleTest: V2.9.0.1--2015-12-22)"

/*buff length*/
#define BUFF_LEN_STORE_MSG_AREA		1024*10
#define BUFF_LEN_MSG_AREA_LINE2		1024*4
#define BUFF_LEN_STORE_DATA_AREA		1024*80
#define BUFF_LEN_TMP_BUFFER 			1024*16

#define MAX_NOISE_FRAMES    32
/////////////////////////////////////////////////Reg FTE736
#define DEVIDE_MODE_ADDR	0x00
#define REG_LINE_NUM	0x01
#define REG_TX_NUM	0x02
#define REG_RX_NUM	0x03
#define FTE736_LEFT_KEY_REG    0X1E
#define FTE736_RIGHT_KEY_REG   0X1F

#define REG_CbAddrH  		0x18	 
#define REG_CbAddrL			0x19	
#define REG_OrderAddrH		0x1A	
#define REG_OrderAddrL		0x1B	

#define REG_RawBuf0			0x6A	
#define REG_RawBuf1			0x6B	
#define REG_OrderBuf0		0x6C	
#define REG_CbBuf0			0x6E	

#define REG_K1Delay			0x31	
#define REG_K2Delay			0x32	
#define REG_SCChannelCf		0x34

#define pre 1


/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
/*
struct structSCapConfEx 
{
	unsigned char ChannelXNum;
	unsigned char ChannelYNum;
	unsigned char KeyNum;
	unsigned char KeyNumTotal;
	bool bLeftKey1;
	bool bLeftKey2;
	bool bLeftKey3;
	bool bRightKey1;
	bool bRightKey2;
	bool bRightKey3;	
};
struct structSCapConfEx g_stSCapConfEx;
*/
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

static int m_RawData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_NoiseData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_CBData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_AvgData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};
static int m_iTempData[TX_NUM_MAX][RX_NUM_MAX] = {{0,0}};//Two-dimensional
static BYTE m_ucTempData[TX_NUM_MAX * RX_NUM_MAX*2] = {0};//One-dimensional
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};
static int m_TempNoiseData[MAX_NOISE_FRAMES][RX_NUM_MAX * TX_NUM_MAX] = {{0,0}};


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

/************************************************************************
* Name: FTE736_StartTest
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
boolean FTE736_StartTest()
{
	bool bTestResult = true, bTempResult = 1;
	unsigned char ReCode;
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

	////////���Թ��̣�����˳��ִ��g_stTestItem�ṹ��Ĳ�����
	for(iItemCount = 0; iItemCount < g_TestItemNum; iItemCount++)
	{
		m_ucTestItemCode = g_stTestItem[ucDevice][iItemCount].ItemCode;

		///////////////////////////////////////////////////////FTE736_ENTER_FACTORY_MODE
		if(Code_FTE736_ENTER_FACTORY_MODE == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{			
			ReCode = FTE736_TestItem_EnterFactoryMode();
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}

		///////////////////////////////////////////////////////FTE736_CHANNEL_NUM_TEST
		if(Code_FTE736_CHANNEL_NUM_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FTE736_TestItem_ChannelsTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
				break;//if this item FAIL, no longer test.				
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;

		}	


		///////////////////////////////////////////////////////FTE736_RAWDATA_TEST

		if(Code_FTE736_RAWDATA_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FTE736_TestItem_RawDataTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}


		///////////////////////////////////////////////////////FTE736_NOISE_TEST
		if(Code_FTE736_NOISE_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{

			ReCode = FTE736_TestItem_NoiseTest(&bTempResult);
			if(ERROR_CODE_OK != ReCode || (!bTempResult))
			{
				bTestResult = false;
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_NG;
			}
			else
				g_stTestItem[ucDevice][iItemCount].TestResult = RESULT_PASS;
		}


		///////////////////////////////////////////////////////FTE736_CB_TEST

		if(Code_FTE736_CB_TEST == g_stTestItem[ucDevice][iItemCount].ItemCode
			)
		{
			ReCode = FTE736_TestItem_CbTest(&bTempResult); //
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

	g_stSCapConfEx.ChannelXNum = 0;
	g_stSCapConfEx.ChannelYNum = 0;
	g_stSCapConfEx.KeyNum = 0;
	g_stSCapConfEx.KeyNumTotal = 6;

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
* Name: FTE736_get_test_data
* Brief:  get data of test result
* Input: none
* Output: pTestData, the returned buff
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int FTE736_get_test_data(char *pTestData)
{
	if(NULL == pTestData)
	{
		FTS_TEST_DBG("[focal] %s pTestData == NULL ",  __func__);	
		return -1;
	}
	memcpy(pTestData, g_pStoreAllData, (g_lenStoreMsgArea+g_lenStoreDataArea));
	return (g_lenStoreMsgArea+g_lenStoreDataArea);	
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

#if 1
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
#else
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
#endif	

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
#if 1
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
#else
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
#endif
	
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

////////////////////////////////////////////////////////////
/************************************************************************
* Name: StartScan(Same function name as FT_MultipleTest)
* Brief:  Scan TP, do it before read Raw Data
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0x00;
	unsigned char times = 0;
	const unsigned char MaxTimes = 20;	//��ȴ�160ms
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	//if(hDevice == NULL)		return ERROR_CODE_NO_DEVICE;

	ReCode = ReadReg(DEVIDE_MODE_ADDR,&RegVal);
	if(ReCode == ERROR_CODE_OK)
	{
		RegVal |= 0x80;		//���λ��1������ɨ��
		ReCode = WriteReg(DEVIDE_MODE_ADDR,RegVal);
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
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3] = {0};
	unsigned char pReadData[ByteNum];
	//unsigned char pReadDataTmp[ByteNum*2];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1=0;

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


	//***********************************************************Read raw data in test mode1		
	I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
	if(ReCode == ERROR_CODE_OK)
	{
		focal_msleep(10);
		ReCode = Comm_Base_IIC_IO(I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
	}

	for(i=1; i<iReadNum; i++)
	{
		if(ReCode != ERROR_CODE_OK) break;

		if(i==iReadNum-1)//last packet
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, pReadData+BYTES_PER_TIME*i, ByteNum-BYTES_PER_TIME*i);
		}
		else
		{
			focal_msleep(10);
			ReCode = Comm_Base_IIC_IO(NULL, 0, pReadData+BYTES_PER_TIME*i, BYTES_PER_TIME);	
		}

	}

	if(ReCode == ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (pReadData[i<<1]<<8)+pReadData[(i<<1)+1];
			//if(pRevBuffer[i] & 0x8000)//�з���λ
			//{
			//	pRevBuffer[i] -= 0xffff + 1;
			//}
		}
	}


	return ReCode;

}
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
	unsigned char wBuffer[4];	
	int i, iReadNum;

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

		wBuffer[1] = (StartNodeNo+usTotalReturnNum) >>8;//��ַƫ������8λ
		wBuffer[2] = (StartNodeNo+usTotalReturnNum)&0xff;//��ַƫ������8λ

		ReCode = WriteReg(REG_CbAddrH, wBuffer[1]);
		ReCode = WriteReg(REG_CbAddrL, wBuffer[2]);
		//ReCode = fts_i2c_read(wBuffer, 1, pReadBuffer+usTotalReturnNum, usReturnNum);
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



/////////////////////////////////////////////////////////////


/************************************************************************
* Name: FTE736_TestItem_EnterFactoryMode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FTE736_TestItem_EnterFactoryMode(void)
{	

	unsigned char ReCode = ERROR_CODE_INVALID_PARAM;
	int iRedo = 5;	//������ɹ����ظ�����5��
	int i ;
	SysDelay(150);
	FTS_TEST_DBG("Enter factory mode...");
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
			//break;
		}

	}
	SysDelay(300);

	if(ReCode == ERROR_CODE_OK)	//������ģʽ�ɹ��󣬾Ͷ���ͨ����
	{	
		ReCode = GetChannelNum();
	}
	return ReCode;
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

	//FTS_TEST_DBG("Enter GetChannelNum...");
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
					FTS_TEST_DBG("Failed to get Channel X number, Get num = %d, UsedMaxNum = %d", 
						g_stSCapConfEx.ChannelXNum, g_ScreenSetParam.iUsedMaxTxNum);
					g_stSCapConfEx.ChannelXNum = 0;
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
			FTS_TEST_DBG("Failed to get Channel X number");
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
					FTS_TEST_DBG("Failed to get Channel Y number, Get num = %d, UsedMaxNum = %d", 
						g_stSCapConfEx.ChannelYNum, g_ScreenSetParam.iUsedMaxRxNum);
					g_stSCapConfEx.ChannelYNum = 0;
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
			FTS_TEST_DBG("Failed to get Channel Y number");
			SysDelay(150);
		}
	}

	//--------------------------------------------"Get Key Num...";
	for(i = 0; i < 3; i++)
	{
		unsigned char regData = 0;
		g_stSCapConfEx.KeyNum = 0;
		ReCode = ReadReg( FTE736_LEFT_KEY_REG, &regData );
		if(ReCode == ERROR_CODE_OK)
		{
			if(((regData >> 0) & 0x01)) { g_stSCapConfEx.bLeftKey1 = true; ++g_stSCapConfEx.KeyNum;}
			if(((regData >> 1) & 0x01)) { g_stSCapConfEx.bLeftKey2 = true; ++g_stSCapConfEx.KeyNum;}
			if(((regData >> 2) & 0x01)) { g_stSCapConfEx.bLeftKey3 = true; ++g_stSCapConfEx.KeyNum;}
		}
		else
		{
			FTS_TEST_DBG("Failed to get Key number");
			SysDelay(150);
			continue;
		}
		ReCode = ReadReg( FTE736_RIGHT_KEY_REG, &regData );
		if(ReCode == ERROR_CODE_OK)
		{
			if(((regData >> 0) & 0x01)) {g_stSCapConfEx.bRightKey1 = true; ++g_stSCapConfEx.KeyNum;}
			if(((regData >> 1) & 0x01)) {g_stSCapConfEx.bRightKey2 = true; ++g_stSCapConfEx.KeyNum;}
			if(((regData >> 2) & 0x01)) {g_stSCapConfEx.bRightKey3 = true; ++g_stSCapConfEx.KeyNum;}
			break;
		}
		else
		{
			FTS_TEST_DBG("Failed to get Key number");
			SysDelay(150);
			continue;
		}
	}

	//g_stSCapConfEx.KeyNumTotal = g_stSCapConfEx.KeyNum;

	FTS_TEST_DBG("CH_X = %d, CH_Y = %d, Key = %d",  g_stSCapConfEx.ChannelXNum ,g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNum );
	return ReCode;
}
/************************************************************************
* Name: FTE736_TestItem_ChannelsTest
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FTE736_TestItem_ChannelsTest(bool * bTestResult)
{
	unsigned char ReCode;

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Channel Test ");

	ReCode = GetChannelNum();
	if(ReCode == ERROR_CODE_OK)
	{
		if((g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelXNum == g_stSCapConfEx.ChannelXNum)
			&& (g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelYNum == g_stSCapConfEx.ChannelYNum)
			&& (g_stCfg_FTE736_BasicThreshold.ChannelNumTest_KeyNum == g_stSCapConfEx.KeyNum))
		{
			* bTestResult = true;
			FTS_TEST_DBG("\n\nGet channels: (CHx: %d, CHy: %d, Key: %d), Set channels: (CHx: %d, CHy: %d, Key: %d)",
				g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNum, 
				g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelXNum, g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelYNum, g_stCfg_FTE736_BasicThreshold.ChannelNumTest_KeyNum);

			FTS_TEST_DBG("\n//Channel Test is OK!");
		}
		else
		{
			* bTestResult = false;
			FTS_TEST_DBG("\n\nGet channels: (CHx: %d, CHy: %d, Key: %d), Set channels: (CHx: %d, CHy: %d, Key: %d)",
				g_stSCapConfEx.ChannelXNum, g_stSCapConfEx.ChannelYNum, g_stSCapConfEx.KeyNum, 
				g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelXNum, g_stCfg_FTE736_BasicThreshold.ChannelNumTest_ChannelYNum, g_stCfg_FTE736_BasicThreshold.ChannelNumTest_KeyNum);

			FTS_TEST_DBG("\n//Channel Test is NG!");
		}
	}
	return ReCode;
}
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
	int iRow, iCol;

	//--------------------------------------------Enter Factory Mode
	ReCode = EnterFactory();	
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Enter Factory Mode...");
		return ReCode;
	}


	//--------------------------------------------Check Num of Channel 
	if(0 == (g_stSCapConfEx.ChannelXNum + g_stSCapConfEx.ChannelYNum)) 
	{
		ReCode = GetChannelNum();
		if( ERROR_CODE_OK != ReCode ) 
		{
			FTS_TEST_DBG("Error Channel Num...");
			return ERROR_CODE_INVALID_PARAM;
		}
	}

	//--------------------------------------------Start Scanning
	//FTS_TEST_DBG("Start Scan ...");
	ReCode = StartScan();
	if(ERROR_CODE_OK != ReCode) 
	{
		FTS_TEST_DBG("Failed to Scan ...");
		return ReCode;
	}


	//--------------------------------------------Read RawData for Channel Area
	//FTS_TEST_DBG("Read RawData...");
	memset(m_RawData, 0, sizeof(m_RawData));	
	memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
	ReCode = ReadRawData(0, 0xAD, g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum * 2, m_iTempRawData);
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to Get RawData");
		return ReCode;
	}

	for (iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow)
	{
		for (iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol)
		{
			m_RawData[iRow][iCol] = m_iTempRawData[iRow * g_stSCapConfEx.ChannelYNum + iCol];
		}
	}

	//--------------------------------------------Read RawData for Key Area
	memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
	ReCode = ReadRawData( 0, 0xAE, g_stSCapConfEx.KeyNum * 2, m_iTempRawData );
	if(ERROR_CODE_OK != ReCode) 
	{
		FTS_TEST_DBG("Failed to Get RawData");
		return ReCode;
	}

	for (iCol = 0; iCol < g_stSCapConfEx.KeyNum; ++iCol)
	{
		m_RawData[g_stSCapConfEx.ChannelXNum][iCol] = m_iTempRawData[iCol];
	}

	return ReCode;

}
/************************************************************************
* Name: FTE736_TestItem_RawDataTest
* Brief:  TestItem: RawDataTest. Check if MCAP RawData is within the range.
* Input: bTestResult
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FTE736_TestItem_RawDataTest(bool * bTestResult)
{
	unsigned char ReCode;
	bool btmpresult = true;
	//int iMax, iMin, iAvg;
	int RawDataMin;
	int RawDataMax;
	int iValue = 0;
	int i=0;
	int iRow, iCol;

	FTS_TEST_DBG("\n\n==============================Test Item: -------- Raw Data Test\n");


	//----------------------------------------------------------Read RawData
	for(i = 0 ; i < 3; i++)//Lost 3 Frames, In order to obtain stable data
		ReCode = GetRawData();
	if( ERROR_CODE_OK != ReCode ) 
	{
		FTS_TEST_DBG("Failed to get Raw Data!! Error Code: %d",  ReCode);
		return ReCode;
	}
	//----------------------------------------------------------Show RawData

	
	FTS_TEST_DBG("\nVA Channels: ");
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
	FTS_TEST_DBG("\nCh_%02d:  ", iRow+1);
	for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
	{
	FTS_TEST_DBG("%5d, ", m_RawData[iRow][iCol]);
	}
	}
	FTS_TEST_DBG("\nKeys:  ");
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; iCol++ )
	{
	FTS_TEST_DBG("%5d, ",  m_RawData[g_stSCapConfEx.ChannelXNum][iCol]);
	}
	

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
				FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
					iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
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
			FTS_TEST_DBG("rawdata test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
				iRow+1, iCol+1, iValue, RawDataMin, RawDataMax);
		}	
	}

	//////////////////////////////Save Test Data
	Save_Test_Data(m_RawData, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);
	//----------------------------------------------------------Return Result
	if(btmpresult)
	{
		* bTestResult = true;		
		FTS_TEST_DBG("\n\n//RawData Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//RawData Test is NG!");
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
/************************************************************************
* Name: FTE736_TestItem_NoiseTest
* Brief:  TestItem: NoiseTest. Check if MCAP Noise is within the range.
* Input: bTestResult
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FTE736_TestItem_NoiseTest(bool* bTestResult)
{
	unsigned char ReCode;
	unsigned char chNoiseValue = 0xff;
	bool btmpresult = true;

	int iNoiseFrames = 0;
	int i,iRow,iCol;
	int iValue = 0;
	int iMinValue = 0, iMaxValue = 0;
	int n,temp;

	int *pTempNext=NULL;
	int *pTempPrev=NULL;


	FTS_TEST_DBG("\n\n==============================Test Item: -------- Noise Test  \n");

	iNoiseFrames = g_stCfg_FTE736_BasicThreshold.NoiseTest_Frames;
	if(iNoiseFrames > MAX_NOISE_FRAMES)
		iNoiseFrames = MAX_NOISE_FRAMES;


	//Lost 3 frames, for data stability
	for (i = 0; i < 3; i++)
	{
		ReCode = GetRawData();
	}
	if( ReCode != ERROR_CODE_OK ) goto TEST_ERR;

	//Get RawData
	memset(m_TempNoiseData, 0, sizeof(m_TempNoiseData));
	memset(m_NoiseData, 0, sizeof(m_NoiseData));
	for(i = 0; i < iNoiseFrames; i++)
	{
		ReCode = GetRawData();	
		if( ReCode != ERROR_CODE_OK ) goto TEST_ERR;

		for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
		{					
			//memcpy(m_TempNoiseData[index] + iRow * g_stSCapConfEx.ChannelYNum, &m_RawData[iRow], g_stSCapConfEx.ChannelYNum * sizeof(int));
			for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
			{
				m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol] = m_RawData[iRow][iCol];	
			}
		}	

	}

	/////////////avg
		memset(m_NoiseData, 0, sizeof(m_NoiseData));
		//total
		for(i = 0; i < iNoiseFrames; i++)
		{
			for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
			{
				for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
				{
					iValue = m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol]; 
					m_NoiseData[iRow][iCol] += iValue;
				}
			}
		}
		//avg
		for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
		{							
			for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
			{
				m_AvgData[iRow][iCol]  = m_NoiseData[iRow][iCol]  / iNoiseFrames;	
			}
		}
		
	//Caculate noise by Noise Mode
	if(NT_AvgData == g_stCfg_FTE736_BasicThreshold.NoiseTest_NoiseMode)
	{
		//Caculate the Avg Value of all nodes
		//sqrt
		memset(m_NoiseData, 0, sizeof(m_NoiseData));
		for(i = 0; i < iNoiseFrames; i++)
		{
			for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
			{
				for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
				{
					iValue = m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol]; 
					m_NoiseData[iRow][iCol] += (iValue -m_AvgData[iRow][iCol])*(iValue -m_AvgData[iRow][iCol]);
				}
			}
		}		
		
		for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
		{							
			for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
			{
				m_NoiseData[iRow][iCol]  = SqrtNew(m_NoiseData[iRow][iCol]  / iNoiseFrames);	
			}
		}

	}
	else if(NT_MaxData == g_stCfg_FTE736_BasicThreshold.NoiseTest_NoiseMode)
	{
		//Find the Max Value of all nodes
		memset(m_NoiseData, 0, sizeof(m_NoiseData));
		for(i = 0; i < iNoiseFrames; i++)
		{
			for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
			{
				for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
				{
					iValue = focal_abs(m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol]); 
					iValue = focal_abs(iValue - m_AvgData[iRow][iCol]);
					if(iValue > m_NoiseData[iRow][iCol])
						m_NoiseData[iRow][iCol] = iValue;
				}
			}
		}		
	}
	else if(NT_MaxDevication == g_stCfg_FTE736_BasicThreshold.NoiseTest_NoiseMode)
	{
		//CaculateNoiseBaseOnMaxMin(iRawDataAvr, iNoiseFrames);
		memset(m_iTempData, 0xffff, sizeof(m_iTempData));//Save The Min Value	
		memset(m_NoiseData, 0, sizeof(m_NoiseData));	//Save The Max Value
		for(i = 0; i < iNoiseFrames; i++)
		{
			for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
			{
				for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
				{
					iValue = m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol]; 
					if(iValue < m_iTempData[iRow][iCol])
						m_iTempData[iRow][iCol] = iValue;
					if(iValue > m_NoiseData[iRow][iCol])					
						m_NoiseData[iRow][iCol] = iValue;
				}
			}
		}
		//Caculate Devication value(Max -Min)	
		for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
		{							
			for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
			{
				m_NoiseData[iRow][iCol]  -= m_iTempData[iRow][iCol];	
			}
		}	

	}
	else if(NT_DifferData == g_stCfg_FTE736_BasicThreshold.NoiseTest_NoiseMode)
	{
		//Caculate the Avg Value of all nodes
		memset(m_NoiseData, 0, sizeof(m_NoiseData));
		for(n = 1; n< iNoiseFrames; n++)
		{
			pTempNext = m_TempNoiseData[n];
			pTempPrev = m_TempNoiseData[n - 1];
			for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
			{
				for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
				{
					//iValue = m_TempNoiseData[i][iRow*g_stSCapConfEx.ChannelYNum + iCol]; 
					//m_NoiseData[iRow][iCol] += iValue;
					temp = focal_abs( pTempNext[iRow*g_stSCapConfEx.ChannelYNum+iCol] - pTempPrev[iRow*g_stSCapConfEx.ChannelYNum+iCol]);

					if(m_NoiseData[iRow][iCol] < temp)
						m_NoiseData[iRow][iCol] = temp;
				}
			}
		}
		/*
		for(iRow = 0; iRow < g_stSCapConfEx.ChannelXNum + 1; iRow++)
		{							
			for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
			{
				m_NoiseData[iRow][iCol]  /= iNoiseFrames;	
			}
		}
		*/
	}
	//------------------------------------------------Show NoiseData

	
	FTS_TEST_DBG("\nVA Channels: ");
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
	FTS_TEST_DBG("\nCh_%02d:  ", iRow+1);
	for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
	{
	FTS_TEST_DBG("%5d, ", m_NoiseData[iRow][iCol]);
	}
	}
	FTS_TEST_DBG("\nKeys:  ");
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; iCol++ )
	{
	FTS_TEST_DBG("%5d, ",  m_NoiseData[g_stSCapConfEx.ChannelXNum][iCol]);
	}

	/////////////
	SysDelay(150);
	ReCode = EnterWork();
	if( ReCode != ERROR_CODE_OK ) goto TEST_ERR;
	SysDelay(50);

	ReCode = ReadReg(0x80, &chNoiseValue);
	if( ReCode != ERROR_CODE_OK ) goto TEST_ERR;	

	ReCode = EnterFactory();
	if( ReCode != ERROR_CODE_OK ) goto TEST_ERR;



		iMinValue = 0;
		iMaxValue = g_stCfg_FTE736_BasicThreshold.NoiseTest_Coefficient * chNoiseValue * 32 / 100;
		FTS_TEST_DBG("");
		for(iRow = 0;iRow < (g_stSCapConfEx.ChannelXNum + 1);iRow++)
		{
			for(iCol = 0;iCol < g_stSCapConfEx.ChannelYNum;iCol++)
			{
				if( (0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) )  
				{
					continue;
				}
				if( iRow >= g_stSCapConfEx.ChannelXNum && iCol >= g_stSCapConfEx.KeyNum ) 
				{
					continue;
				}

				
				if(m_NoiseData[iRow][iCol] < iMinValue || m_NoiseData[iRow][iCol] > iMaxValue)
				{
					btmpresult = false;
					FTS_TEST_DBG("noise test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d)  ",  \
						iRow+1, iCol+1, m_NoiseData[iRow][iCol], iMinValue, iMaxValue);
				}
			}
			FTS_TEST_DBG("");
		}

	

	//////////////////////////////Save Test Data
	Save_Test_Data(m_NoiseData, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);

	if(btmpresult)
	{
		* bTestResult = true;
		FTS_TEST_DBG("\n\n//Noise Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//Noise Test is NG!");
	}

	return ReCode;
TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//Noise Test is NG!");	
	return ReCode;
}
/************************************************************************
* Name: FTE736_TestItem_CbTest
* Brief:  TestItem: Cb Test. Check if Cb is within the range.
* Input: none
* Output: bTestResult, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
unsigned char FTE736_TestItem_CbTest(bool* bTestResult)
{
	bool btmpresult = true;
	unsigned char ReCode = ERROR_CODE_OK;
	int iRow = 0;
	int iCol = 0;
	int iMaxValue = 0;
	int iMinValue = 0;

	FTS_TEST_DBG("\n\n==============================Test Item: --------  CB Test\n");

	ReCode = GetTxRxCB( 0, (short)(g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum + g_stSCapConfEx.KeyNum), m_ucTempData );

	ReCode = GetTxRxCB( 0, (short)(g_stSCapConfEx.ChannelXNum * g_stSCapConfEx.ChannelYNum + g_stSCapConfEx.KeyNum), m_ucTempData );
	if( ERROR_CODE_OK != ReCode )
	{
		btmpresult = false;
		FTS_TEST_DBG("Failed to get CB value...");
		goto TEST_ERR;
	}

	memset(m_CBData, 0, sizeof(m_CBData));
	///VA area
	for ( iRow = 0; iRow < g_stSCapConfEx.ChannelXNum; ++iRow )
	{
		for ( iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; ++iCol )
		{
			m_CBData[iRow][iCol] = m_ucTempData[ iRow * g_stSCapConfEx.ChannelYNum + iCol ];
		}
	}
	///key
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; ++iCol )
	{
		m_CBData[g_stSCapConfEx.ChannelXNum][iCol] = m_ucTempData[ g_stSCapConfEx.ChannelXNum*g_stSCapConfEx.ChannelYNum + iCol ];
	}

	//------------------------------------------------Show CbData

	
	FTS_TEST_DBG("\nVA Channels: ");
	for(iRow = 0; iRow<g_stSCapConfEx.ChannelXNum; iRow++)
	{
	FTS_TEST_DBG("\nCh_%02d:  ", iRow+1);
	for(iCol = 0; iCol < g_stSCapConfEx.ChannelYNum; iCol++)
	{
	FTS_TEST_DBG("%3d, ", m_CBData[iRow][iCol]);
	}
	}
	FTS_TEST_DBG("\nKeys:  ");
	for ( iCol = 0; iCol < g_stSCapConfEx.KeyNum; iCol++ )
	{
	FTS_TEST_DBG("%3d, ",  m_CBData[g_stSCapConfEx.ChannelXNum][iCol]);
	}

	iMinValue = g_stCfg_FTE736_BasicThreshold.CbTest_Min;
	iMaxValue = g_stCfg_FTE736_BasicThreshold.CbTest_Max;	
	for(iRow = 0;iRow < (g_stSCapConfEx.ChannelXNum + 1);iRow++)
	{
		for(iCol = 0;iCol < g_stSCapConfEx.ChannelYNum;iCol++)
		{
			if( (0 == g_stCfg_MCap_DetailThreshold.InvalidNode[iRow][iCol]) )  
			{
				continue;
			}
			if( iRow >= g_stSCapConfEx.ChannelXNum && iCol >= g_stSCapConfEx.KeyNum ) 
			{
				continue;
			}

			if(focal_abs(m_CBData[iRow][iCol]) < iMinValue || focal_abs(m_CBData[iRow][iCol]) > iMaxValue)
			{
				btmpresult = false;
				FTS_TEST_DBG("CB test failure. Node=(%d,  %d), Get_value=%d,  Set_Range=(%d, %d) ",  \
					iRow+1, iCol+1, m_CBData[iRow][iCol], iMinValue, iMaxValue);
			}
		}
	}

	//////////////////////////////Save Test Data
	Save_Test_Data(m_CBData, 0, g_stSCapConfEx.ChannelXNum+1, g_stSCapConfEx.ChannelYNum, 1);

	if(btmpresult)
	{
		* bTestResult = true;
		FTS_TEST_DBG("\n\n//CB Test is OK!");
	}
	else
	{
		* bTestResult = false;
		FTS_TEST_DBG("\n\n//CB Test is NG!");
	}

	return ReCode;

TEST_ERR:

	* bTestResult = false;
	FTS_TEST_DBG("\n\n//CB Test is NG!");
	return ReCode;	
}
