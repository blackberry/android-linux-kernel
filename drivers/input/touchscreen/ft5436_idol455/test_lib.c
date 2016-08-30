#include <linux/kernel.h>
//#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
//#include <linux/gpio.h>
//#include <linux/platform_device.h>
#include <linux/string.h>

//#include <linux/mount.h>
#include <linux/unistd.h>

#include "test_lib.h"
#include "ini.h"

#define FT5X46_TEST 		1 /*1: Enable, 0: Disable*/
//zax 20141114+++++++++++++++++++++++
/*-----------------------------------------------------------
Error Code Define
-----------------------------------------------------------*/
#define		ERROR_CODE_OK						0x00
#define		ERROR_CODE_CHECKSUM_ERROR			0x01
#define		ERROR_CODE_INVALID_COMMAND			0x02
#define		ERROR_CODE_INVALID_PARAM			0x03
#define		ERROR_CODE_IIC_WRITE_ERROR			0x04
#define		ERROR_CODE_IIC_READ_ERROR			0x05
#define		ERROR_CODE_WRITE_USB_ERROR			0x06
#define		ERROR_CODE_WAIT_RESPONSE_TIMEOUT	0x07
#define		ERROR_CODE_PACKET_RE_ERROR			0x08
#define		ERROR_CODE_NO_DEVICE				0x09
#define		ERROR_CODE_WAIT_WRITE_TIMEOUT		0x0a
#define		ERROR_CODE_READ_USB_ERROR			0x0b
#define		ERROR_CODE_COMM_ERROR				0x0c
#define		ERROR_CODE_ALLOCATE_BUFFER_ERROR	0x0d
#define		ERROR_CODE_DEVICE_OPENED			0x0e
#define		ERROR_CODE_DEVICE_CLOSED			0x0f


struct Test_ConfigParam_FT5X46{
	short RawDataTest_Low_Min;
	short RawDataTest_Low_Max;
	short RawDataTest_High_Min;
	short RawDataTest_High_Max;

	boolean RawDataTest_LowFreq;
	boolean RawDataTest_HighFreq;
	boolean SCapCbTest_SetWaterproof_ON;
	boolean SCapCbTest_SetWaterproof_OFF;

	boolean SCapRawDataTest_SetWaterproof_ON;
	boolean SCapRawDataTest_SetWaterproof_OFF;

	boolean RawDataTest;
	boolean SCAPCbTest;
	boolean SCAPRawDataTest;

	
	short WeakShortTest_CG_Min;
	short WeakShortTest_CC_Min;

	short valid_node[TX_NUM_MAX][RX_NUM_MAX];
	short InvalidNodes[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_Low_Min_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_Low_Max_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_High_Min_node[TX_NUM_MAX][RX_NUM_MAX];	
	short RawDataTest_High_Max_node[TX_NUM_MAX][RX_NUM_MAX];	


	short SCapCbTest_ON_Min_Value;
	short SCapCbTest_ON_Max_Value;
	short SCapCbTest_OFF_Min_Value;
	short SCapCbTest_OFF_Max_Value;

	short SCapRawDataTest_ON_Max_Value;
	short SCapRawDataTest_ON_Min_Value;
	short SCapRawDataTest_OFF_Max_Value;
	short SCapRawDataTest_OFF_Min_Value;

	
	short SCapCbTest_ON_Max[TX_NUM_MAX][RX_NUM_MAX];
	short SCapCbTest_ON_Min[TX_NUM_MAX][RX_NUM_MAX];
	short SCapCbTest_OFF_Max[TX_NUM_MAX][RX_NUM_MAX];
	short SCapCbTest_OFF_Min[TX_NUM_MAX][RX_NUM_MAX];


	short SCapRawDataTest_ON_Max[TX_NUM_MAX][RX_NUM_MAX];
	short SCapRawDataTest_ON_Min[TX_NUM_MAX][RX_NUM_MAX];
	short SCapRawDataTest_OFF_Max[TX_NUM_MAX][RX_NUM_MAX];
	short SCapRawDataTest_OFF_Min[TX_NUM_MAX][RX_NUM_MAX];

	
	short SCapCbTest_Min;
	short SCapCbTest_Max;

	short SCapRawDataTest_OffMin;
	short SCapRawDataTest_OffMax;
	short SCapRawDataTest_OnMin;
	short SCapRawDataTest_OnMax;

};

/*test section*/
#define Section_TestItem 	"TestItem"
#define Section_BaseSet "Basic_Threshold"
#define Section_SpecialSet "SpecialSet"
#define Section_INVALID_NODE "INVALID_NODE"

#define Item_RawDataTest_Low_Min "RawDataTest_Low_Min"
#define Item_RawDataTest_Low_Max "RawDataTest_Low_Max"
#define Item_RawDataTest_High_Min "RawDataTest_High_Min"
#define Item_RawDataTest_High_Max "RawDataTest_High_Max"
#define Item_RawDataTest_LowFreq "RawDataTest_LowFreq"
#define Item_RawDataTest_HighFreq "RawDataTest_HighFreq"

#define Item_WeakShortTest_CG_Min "WeakShortTest_CG"
#define Item_WeakShortTest_CC_Min "WeakShortTest_CC"/*综合测试软件命名有误*/

#define Special_RawDataTest_Low_Min "RawData_Min_Low_Tx"
#define Special_RawDataTest_Low_Max "RawData_Max_Low_Tx"
#define Special_RawDataTest_High_Min "RawData_Min_High_Tx"
#define Special_RawDataTest_High_Max "RawData_Max_High_Tx"
#define SCapCbTest_SetWaterproof_on "ScapCBTest_SetWaterproof_ON"
#define SCapCbTest_SetWaterproof_off "ScapCBTest_SetWaterproof_OFF"

#define SCapRawDataTest_SetWaterproof_on "SCapRawDataTest_SetWaterproof_ON"
#define SCapRawDataTest_SetWaterproof_off "SCapRawDataTest_SetWaterproof_OFF"


#define RawData_Test "RAWDATA_TEST"

#define SCAP_Cb_Test "SCAP_CB_TEST"
#define SCAP_RawData_Test "SCAP_RAWDATA_TEST"

#define SCapCbTest_ON_max "SCapCbTest_ON_Max"
#define SCapCbTest_ON_min "SCapCbTest_ON_Min"
#define SCapCbTest_OFF_max "SCapCbTest_OFF_Max"
#define SCapCbTest_OFF_min "SCapCbTest_OFF_Min"

#define SCapRawDataTestONMax "SCapRawDataTest_ON_Max"
#define SCapRawDataTestONMin "SCapRawDataTest_ON_Min"
#define SCapRawDataTestOFFMax "SCapRawDataTest_OFF_Max"
#define SCapRawDataTestOFFMin "SCapRawDataTest_OFF_Min"


#define SCapCbTest_min "SCapCbTest_Min"
#define SCapCbTest_max "SCapCbTest_Max"



#define SCapRawDataTest_Offmin "SCapRawDataTest_OFF_Min"
#define SCapRawDataTest_Offmax "SCapRawDataTest_OFF_Max"
#define SCapRawDataTest_Onmin "SCapRawDataTest_ON_Min"
#define SCapRawDataTest_Onmax "SCapRawDataTest_ON_Max"

#define ScapCB_ON_Max "ScapCB_ON_Max_"
#define ScapCB_ON_Min "ScapCB_ON_Min_"
#define ScapCB_OFF_Max "ScapCB_OFF_Max_"
#define ScapCB_OFF_Min "ScapCB_OFF_Min_"

#define ScapRawData_OFF_Min "ScapRawData_OFF_Min_"
#define ScapRawData_OFF_Max "ScapRawData_OFF_Max_"
#define ScapRawData_ON_Min "ScapRawData_ON_Min_"
#define ScapRawData_ON_Max "ScapRawData_ON_Max_"

#define InvalidNode "InvalidNode"
#define		REG_ScCbBuf0	0x4E
#define		REG_ScCbBuf1	0x4F
#define		REG_ScCbAddrR	0x45
#define		REG_LINE_NUM	0x01
#define		REG_RawBuf0	0x36
//zax 20141114-----------------------------------------
/*
static inline void kernel_fpu_begin(void)
{
struct thread_info *me = current_thread_info();
preempt_disable();
if (me->status & TS_USEDFPU)
__save_init_fpu(me->task);
else
clts();
}

static inline void kernel_fpu_end(void)
{
stts();
preempt_enable();
}*/


static FTS_I2c_Read_Function focal_I2C_Read;
static FTS_I2c_Write_Function focal_I2C_write;

static boolean bRawdataTest = true;
//zax 20141114+++++++++++++++++++++++
static short iTxNum = 0;
static short iRxNum = 0;
static int SCap_iTxNum = 0;
static int SCap_iRxNum = 0;
//static unsigned char Reg_VolAddr = 0x05;

static struct Test_ConfigParam_FT5X46 g_TestParam_FT5X46;

static char *g_testparamstring = NULL;
//zax 20141116 ++++++++++++++
#if 0
extern int Save_rawData1[TX_NUM_MAX][RX_NUM_MAX];
extern int TX_NUM;
extern int RX_NUM;
extern int SCab_1;
extern int SCab_2;
extern int SCab_3;
extern int SCab_4;
extern int SCab_5;
extern int SCab_6;
extern int SCab_7;
extern int SCab_8;
#endif
int TX_NUM;
int RX_NUM;
int SCab_1;
int SCab_2;
int SCab_3;
int SCab_4;
int SCab_5;
int SCab_6;
int SCab_7;
int SCab_8;
int Save_rawData1[TX_NUM_MAX][RX_NUM_MAX];
//zax 20141116 -------------------
static short rawdata[TX_NUM_MAX][RX_NUM_MAX];
static int SCap_rawData[TX_NUM_MAX][RX_NUM_MAX];
//zax 20141116+++++++++++++++++++++++
static int Save_rawData[TX_NUM_MAX][RX_NUM_MAX];
//zax 20141116-------------------------------
static void TestTp(void);
static void TestTp1(void);
void Ft5336_TestRawDataAndDiff(void);
static int StartScan(void);
//static int SetDriverVol(unsigned char vol);
//static char GetDriverVol(void);
//static void SetTxRxNum(short txnum,short rxnum);
static short GetTxNum(void);
static short GetRxNum(void);
//static char GetOffsetTx(unsigned char txindex);
//static char GetOffsetRx(unsigned char rxindex);
//static void SetOffsetTx(unsigned char txindex,unsigned char offset);
//static void SetOffsetRx(unsigned char rxindex,unsigned char offset);
static void GetRawData(short RawData[TX_NUM_MAX][RX_NUM_MAX]);
//static boolean StartTestTP(void);

static boolean FT5X46_TestItem(void);
static boolean FT5X46_TestItem1(void);
static boolean TestItem_RawDataTest_FT5X46(void);
static boolean TestItem_SCapCbTest(void);
static boolean TestItem_SCapRawDataTest(void);

//zax 20141114------------------------------------
//static boolean TestItem_WeakShortTest(void);
//static int WeakShort_GetAdcData( int AllAdcDataLen, int *pRevBuffer );

static void GetTestParam(void);

static void focal_msleep(int ms)
{
	msleep(ms);
}
int SetParamData(char * TestParamData)
{
	g_testparamstring = TestParamData;

	GetTestParam();
	return 0;
}
void FreeTestParamData(void)
{
	if(g_testparamstring)
		kfree(g_testparamstring);

	g_testparamstring = NULL;
}
/*
static short focal_abs(short value)
{
short absvalue = 0;
if(value > 0)
absvalue = value;
else
absvalue = 0 - value;

return absvalue;
}
*/
//zax 20141116 ++++++++++++++
void focal_save_scap_sample1(void)
{
	int i=0;
	int j=0;
	
	for(i=0;i<iTxNum+8;i++)
	{
		for(j=0;j<iRxNum;j++)
		{
			Save_rawData1[i][j]=Save_rawData[i][j];
			//databuf[i*TX_NUM_MAX+j]=SCap_rawData[i][j];
			//printk("zax SCap_rawData[i][j] %d\n",SCap_rawData[i][j]);
		}
	}
}
//zax 20141116 --------------------
void focal_save_scap_sample(int *databuf, int num)
{
	int i=0;
	int j=0;
	for(i=0;i<TX_NUM_MAX;i++)
	{
		for(j=0;j<RX_NUM_MAX;j++)
		{
			databuf[i*RX_NUM_MAX+j]=Save_rawData[i][j];
			//databuf[i*TX_NUM_MAX+j]=SCap_rawData[i][j];
			//printk("zax SCap_rawData[i][j] %d\n",SCap_rawData[i][j]);
		}
	}
}

static int GetParamValue(char *section, char *ItemName, int defaultvalue) 
{
	int paramvalue = defaultvalue;
	char value[512];
	memset(value , 0x00, sizeof(value));
	if(ini_get_key(g_testparamstring, section, ItemName, value) < 0) {
		return paramvalue;
	} else {
		paramvalue = atoi(value);
	}

	return paramvalue;
}

static int GetParamString(char *section, char *ItemName, char *defaultvalue) {
	char value[512];
	int len = 0;
	memset(value , 0x00, sizeof(value));
	if(ini_get_key(g_testparamstring, section, ItemName, value) < 0) {
		return 0;
	} else {
		len = sprintf(defaultvalue, "%s", value);
	}

	return len;
}
//zax 20141114+++++++++++++++++++++++
static void GetTestParam(void)
{
	char str_tmp[128], str_node[64], str_value[512];
	int j, index, valuelen = 0, i = 0, k = 0;
	int iLenght = 0;
	memset(str_tmp, 0, sizeof(str_tmp));
	memset(str_node, 0, sizeof(str_node));
	memset(str_value, 0, sizeof(str_value));
	//zax 20141116+++++++++++++++++++++++
	memset(Save_rawData, 0, sizeof(Save_rawData));
	//zax 20141116-----------------------------
#if (FT5X46_TEST == 1)
	g_TestParam_FT5X46.RawDataTest_Low_Min = GetParamValue(Section_BaseSet, Item_RawDataTest_Low_Min, 3000);
	g_TestParam_FT5X46.RawDataTest_Low_Max = GetParamValue(Section_BaseSet, Item_RawDataTest_Low_Max, 15000);
	g_TestParam_FT5X46.RawDataTest_High_Min = GetParamValue(Section_BaseSet, Item_RawDataTest_High_Min, 3000);
	g_TestParam_FT5X46.RawDataTest_High_Max = GetParamValue(Section_BaseSet, Item_RawDataTest_High_Max, 15000);

	g_TestParam_FT5X46.RawDataTest_LowFreq = GetParamValue(Section_BaseSet, Item_RawDataTest_LowFreq, 1);
	g_TestParam_FT5X46.RawDataTest_HighFreq = GetParamValue(Section_BaseSet, Item_RawDataTest_HighFreq, 1);

	g_TestParam_FT5X46.WeakShortTest_CG_Min = GetParamValue(Section_BaseSet, Item_WeakShortTest_CG_Min, 2000);
	g_TestParam_FT5X46.WeakShortTest_CC_Min = GetParamValue(Section_BaseSet, Item_WeakShortTest_CC_Min, 2000);

	
	g_TestParam_FT5X46.SCapCbTest_SetWaterproof_ON=GetParamValue(Section_BaseSet, SCapCbTest_SetWaterproof_on, 0);
	g_TestParam_FT5X46.SCapCbTest_SetWaterproof_OFF=GetParamValue(Section_BaseSet, SCapCbTest_SetWaterproof_off, 0);

	g_TestParam_FT5X46.SCapRawDataTest_SetWaterproof_ON=GetParamValue(Section_BaseSet, SCapRawDataTest_SetWaterproof_on, 0);
	g_TestParam_FT5X46.SCapRawDataTest_SetWaterproof_OFF=GetParamValue(Section_BaseSet, SCapRawDataTest_SetWaterproof_off, 0);

	
	
	g_TestParam_FT5X46.SCapCbTest_ON_Min_Value=GetParamValue(Section_BaseSet,SCapCbTest_ON_min , 0);
	g_TestParam_FT5X46.SCapCbTest_ON_Max_Value=GetParamValue(Section_BaseSet,SCapCbTest_ON_max , 240);


	g_TestParam_FT5X46.SCapCbTest_OFF_Min_Value=GetParamValue(Section_BaseSet,SCapCbTest_OFF_min , 0);
	g_TestParam_FT5X46.SCapCbTest_OFF_Max_Value=GetParamValue(Section_BaseSet,SCapCbTest_OFF_max , 240);




	g_TestParam_FT5X46.SCapRawDataTest_ON_Max_Value=GetParamValue(Section_BaseSet,SCapRawDataTestONMax , 8500);
	g_TestParam_FT5X46.SCapRawDataTest_ON_Min_Value=GetParamValue(Section_BaseSet,SCapRawDataTestONMin , 5000);
	g_TestParam_FT5X46.SCapRawDataTest_OFF_Max_Value=GetParamValue(Section_BaseSet,SCapRawDataTestOFFMax , 8500);
	g_TestParam_FT5X46.SCapRawDataTest_OFF_Min_Value=GetParamValue(Section_BaseSet,SCapRawDataTestOFFMin , 5000);


	
	g_TestParam_FT5X46.RawDataTest=GetParamValue(Section_TestItem, RawData_Test, 0);
	g_TestParam_FT5X46.SCAPCbTest=GetParamValue(Section_TestItem, SCAP_Cb_Test, 0);
	g_TestParam_FT5X46.SCAPRawDataTest=GetParamValue(Section_TestItem, SCAP_RawData_Test, 0);
	printk("zax init   %d %d %d\n",g_TestParam_FT5X46.RawDataTest,g_TestParam_FT5X46.SCAPCbTest,g_TestParam_FT5X46.SCAPRawDataTest);


	g_TestParam_FT5X46.SCapCbTest_Min=GetParamValue(Section_BaseSet, SCapCbTest_min, 0);
	g_TestParam_FT5X46.SCapCbTest_Max=GetParamValue(Section_BaseSet, SCapCbTest_max, 240);

	
	g_TestParam_FT5X46.SCapRawDataTest_OffMin=GetParamValue(Section_BaseSet, SCapRawDataTest_Offmin, 150);
	g_TestParam_FT5X46.SCapRawDataTest_OffMax=GetParamValue(Section_BaseSet, SCapRawDataTest_Offmax, 1000);
	g_TestParam_FT5X46.SCapRawDataTest_OnMin=GetParamValue(Section_BaseSet, SCapRawDataTest_Onmin, 150);
	g_TestParam_FT5X46.SCapRawDataTest_OnMax=GetParamValue(Section_BaseSet, SCapRawDataTest_Onmax, 1000);

	for (i = 0; i < TX_NUM_MAX; i++) 
	{
		for (j = 0; j < RX_NUM_MAX; j++) 
		{
			g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Min;
			g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Max;
			g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Min;
			g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Max;

			g_TestParam_FT5X46.SCapCbTest_ON_Max[i][j] = g_TestParam_FT5X46.SCapCbTest_Max;
			g_TestParam_FT5X46.SCapCbTest_ON_Min[i][j] = g_TestParam_FT5X46.SCapCbTest_Min;
			g_TestParam_FT5X46.SCapCbTest_OFF_Max[i][j] = g_TestParam_FT5X46.SCapCbTest_Max;
			g_TestParam_FT5X46.SCapCbTest_OFF_Min[i][j] = g_TestParam_FT5X46.SCapCbTest_Min;

			g_TestParam_FT5X46.SCapRawDataTest_ON_Max[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OnMax;
			g_TestParam_FT5X46.SCapRawDataTest_ON_Min[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OnMin;
			g_TestParam_FT5X46.SCapRawDataTest_OFF_Max[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OffMax;
			g_TestParam_FT5X46.SCapRawDataTest_OFF_Min[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OffMin;

			g_TestParam_FT5X46.valid_node[i][j] = 1;
			g_TestParam_FT5X46.InvalidNodes[i][j]=1;
		}
	}
/*
	for (i = 0; i < 25; i++) 
	{
		for (j = 0; j < 25; j++) 
		{
			memset(str_tmp, 0x00, sizeof(str_tmp));
			sprintf(str_tmp, "%s[%d][%d]",InvalidNode,(i+1),(j+1));
			g_TestParam_FT5X46.valid_node[i][j] = GetParamValue(Section_INVALID_NODE, str_tmp, 1);
		}
	}

*/
	/////////////////////////////////RawDataTest_Low_Min_node
	printk(" Search INI files \n");

	for (i = 0; i < TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_Low_Min,(i+1));
		
		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for (j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	/////////////////////////////////RawDataTest_Low_Max_node
	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_Low_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_Low_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	/////////////////////////////////RawDataTest_High_Min_node
	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_High_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_High_Min_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	/////////////////////////////////RawDataTest_High_Max_node
	for(i=0; i<TX_NUM_MAX; i++) 
	{		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",Special_RawDataTest_High_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.RawDataTest_High_Max_node[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j] = g_TestParam_FT5X46.RawDataTest_High_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	//////读取阈值，若无特殊设置，则以Basic_Threshold替代
	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapCB_ON_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapCbTest_ON_Max[i][k] = (short)(atoi(str_tmp));
					//printk("zax8999  %d\n",g_TestParam_FT5X46.SCapCbTest_ON_Max[i][k] );
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapCbTest_ON_Max[i][j] = g_TestParam_FT5X46.SCapCbTest_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapCB_ON_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapCbTest_ON_Min[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapCbTest_ON_Min[i][j] = g_TestParam_FT5X46.SCapCbTest_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapCB_OFF_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapCbTest_OFF_Max[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapCbTest_OFF_Max[i][j] = g_TestParam_FT5X46.SCapCbTest_Max;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapCB_OFF_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapCbTest_OFF_Min[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapCbTest_OFF_Min[i][j] = g_TestParam_FT5X46.SCapCbTest_Min;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	
	
	//////读取阈值，若无特殊设置，则以Basic_Threshold替代
	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapRawData_OFF_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapRawDataTest_OFF_Max[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapRawDataTest_OFF_Max[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OffMax;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapRawData_OFF_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapRawDataTest_OFF_Min[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapRawDataTest_OFF_Min[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OffMin;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	

	//////读取阈值，若无特殊设置，则以Basic_Threshold替代
	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapRawData_ON_Max,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapRawDataTest_ON_Max[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapRawDataTest_ON_Max[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OnMax;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	
	for(i=0; i<2; i++) 
	{		

		if(0 == i)iLenght = RX_NUM_MAX;
		if(1 == i)iLenght = TX_NUM_MAX;
		
		memset(str_value, 0x00, sizeof(str_value));
		memset(str_tmp, 0x00, sizeof(str_tmp));
		sprintf(str_tmp, "%s%d",ScapRawData_ON_Min,(i+1));

		valuelen = GetParamString(Section_SpecialSet, str_tmp, str_value);
		
		if (valuelen > 0) 
		{
			index = 0;
			k = 0;
			memset(str_tmp, 0x00, sizeof(str_tmp));
			for(j=0; j<valuelen; j++) 
			{
				if(',' == str_value[j]) 
				{
					g_TestParam_FT5X46.SCapRawDataTest_ON_Min[i][k] = (short)(atoi(str_tmp));
					index = 0;
					memset(str_tmp, 0x00, sizeof(str_tmp));
					k++;
				} 
				else 
				{
					if(' ' == str_value[j])
						continue;
					str_tmp[index] = str_value[j];
					index++;
				}
			}
		} 
		else 
		{
			for(j = 0; j < RX_NUM_MAX; j++) 
			{
				g_TestParam_FT5X46.SCapRawDataTest_ON_Min[i][j] = g_TestParam_FT5X46.SCapRawDataTest_OnMin;
#ifdef FOCAL_DBG
#endif
			}
		}			
	}	
#endif

}
//zax 20141114---------------------------------------------
int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read)
{
	focal_I2C_Read = fpI2C_Read;
	return 0;
}

int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write)
{
	focal_I2C_write = fpI2C_Write;
	return 0;
}

static int ReadReg(unsigned char RegAddr, unsigned char *RegData)
{
	return focal_I2C_Read(&RegAddr, 1, RegData, 1);
}

static int WriteReg(unsigned char RegAddr, unsigned char RegData)
{
	unsigned char cmd[2] = {0};
	cmd[0] = RegAddr;
	cmd[1] = RegData;
	return focal_I2C_write(cmd, 2);
}

static int StartScan(void)
{
	int err = 0, i = 0;
	unsigned char regvalue = 0x00;

	/*scan*/
	//if(WriteReg(0x00,0x40) < 0) {
	//	FTS_DBG("Enter factory failure\n");
	//}
	//focal_msleep(100);

	err = ReadReg(0x00,&regvalue);
	//printk("zax StartScan1 %d\n",err);
	if (err < 0) 
	{
		FTS_DBG("Enter StartScan Err. RegValue: %d \n", regvalue);
		//printk("zax StartScan2 %d\n");
		return err;
	}
	else 
	{
		regvalue |= 0x80;
		err = WriteReg(0x00,regvalue);
		if (err < 0) 
		{	//printk("zax StartScan3 \n");
			return err;
		}
		else 
		{
			for(i=0; i<20; i++) {
				focal_msleep(8);
				err = ReadReg(0x00,&regvalue);
				if (err < 0) 
				{//printk("zax StartScan4 \n");
					return err;
				} 
				else 
				{
					if (0 == (regvalue >> 7)) {
						break;
					}
				}
			}
			if (i >= 20) 
			{//printk("zax StartScan5 \n");
				return (-5);
			}
		}
	}
	//printk("zax StartScan6\n");
	return 0;
}	
#if 0
static int SetDriverVol(unsigned char vol)
{
	return WriteReg(Reg_VolAddr,vol);
}

static char GetDriverVol(void)
{
	char vol = 0;
	unsigned char regvalue = 0x00;

	ReadReg(Reg_VolAddr,&regvalue);
	vol = (char)regvalue;

	return vol;
}

static void SetTxRxNum(short txnum,short rxnum)
{
	iTxNum = txnum;
	iRxNum = rxnum;
}
#endif
static short GetTxNum(void)
{
	short txnum = 0;
	unsigned char regvalue = 0x00;

	if(WriteReg(0x00, 0x40) >= 0)
	{
		ReadReg(0x02,&regvalue);
		txnum = (short)regvalue;
	}
	else
	{
		return TX_NUM_MAX;
	}

	return txnum;
}

static short GetRxNum(void)
{
	short rxnum = 0;
	unsigned char regvalue = 0x00;

	if(WriteReg(0x00, 0x40) >= 0)
	{
		ReadReg(0x03,&regvalue);
		rxnum = (short)regvalue;
	}
	else
	{
		return RX_NUM_MAX;
	}

	return rxnum;
}
#if 0
static char GetOffsetTx(unsigned char txindex)
{
	char txoffset = 0;
	char regvalue = 0x00;

	ReadReg((0xad + txindex),&regvalue);
	txoffset = regvalue;

	return txoffset;
}

static char GetOffsetRx(unsigned char rxindex)
{
	char rxoffset = 0;
	char regvalue = 0x00;

	ReadReg((0xd6 + rxindex),&regvalue);
	rxoffset = regvalue;

	return rxoffset;
}

static void SetOffsetTx(unsigned char txindex,unsigned char offset)
{
	WriteReg((0xad + txindex),offset);
}

static void SetOffsetRx(unsigned char rxindex,unsigned char offset)
{  
	WriteReg((0xd6 + rxindex),offset);
}
#endif
static void GetRawData(short RawData[TX_NUM_MAX][RX_NUM_MAX])
{
        //unsigned char LineNum = 0;
        unsigned char I2C_wBuffer[3];
        //unsigned char *rrawdata = NULL;
        unsigned char rrawdata[iTxNum*iRxNum*2];
        short len = 0, i = 0, j = 0; 
        short ByteNum = 0;
        int ReCode = 0;
        //rrawdata = kmalloc(iTxNum*iRxNum*2, GFP_KERNEL);

        if(WriteReg(0x00,0x40) >= 0)
        {
                if(StartScan() >= 0)
                {       
                        I2C_wBuffer[0] = 0x01;
                        I2C_wBuffer[1] = 0xaa;
                        focal_msleep(10);
                        ReCode = focal_I2C_write(I2C_wBuffer, 2);
                        I2C_wBuffer[0] = (unsigned char)(0x36);
                        focal_msleep(10);
                        ReCode = focal_I2C_write(I2C_wBuffer, 1);          

                        for (j = 0; j < iTxNum; j++)
                        {
                                ByteNum = iRxNum*2;
                                if (ReCode >= 0) {                                    
                                        len = ByteNum;

                                        memset(rrawdata, 0x00, sizeof(rrawdata));
                                        focal_msleep(10);
                                        ReCode = focal_I2C_Read(NULL, 0, rrawdata, len);      
                                        if (ReCode >= 0) 
                                        {
                                                for (i = 0; i < (len >> 1); i++) 
                                                {                                               
                                                        RawData[j][i%iRxNum] = (short)((unsigned short)(rrawdata[i << 1]) << 8) \
                                                                + (unsigned short)rrawdata[(i << 1) + 1];
                                                }
                                        }
                                        else 
                                        {
                                                FTS_DBG("Get Rawdata failure\n");
                                        }                                       
                                }
                        }                                       
                }
        }
        //kfree(rrawdata);        
}

//zax 20141114+++++++++++++++++++++++
boolean StartTestTP1(void) 
{
	bRawdataTest = true;
	printk("[focal] %s start \n", __func__);
	TestTp1();

	return bRawdataTest;
}

static void TestTp1(void) {
	int i = 0;//min = 0, max = 0;
	//unsigned char regvalue = 0x00;

	bRawdataTest = true;

	if(WriteReg(0x00, 0x40) < 0) {
		printk("Enter factory failure\n");
		bRawdataTest = false;
		goto Enter_WorkMode;
	}
	else
	{
		printk("Enter factory Successful\n");
	}
	focal_msleep(200);

	iTxNum = GetTxNum();
	focal_msleep(100);
	iRxNum = GetRxNum();

	bRawdataTest = FT5X46_TestItem1();

Enter_WorkMode:	
	//the end, return work mode
	for (i = 0; i < 3; i++) {
		//if (WriteReg(0x00, 0x00) >=0)
		//	break;
		//else {
		//	focal_msleep(200);
		//}
	}
	
}

boolean FT5X46_TestItem1(void)
{
	boolean bTestReuslt = true;

	//zax bTestReuslt = bTestReuslt & TestItem_RawDataTest_FT5X46();
	bTestReuslt = bTestReuslt & TestItem_SCapRawDataTest();
	//bTestReuslt = bTestReuslt & TestItem_SCapCbTest();
	printk("[focal] %s result = %d  \n", __func__, bTestReuslt);
	focal_msleep(1000);
	//bTestReuslt = bTestReuslt & TestItem_WeakShortTest();

	return bTestReuslt;		
}
boolean StartTestTP(void) 
{
	bRawdataTest = true;
	printk("[focal] %s start \n", __func__);
	TestTp();

	return bRawdataTest;
}

static void TestTp(void) {
	int i = 0;//min = 0, max = 0;
	//unsigned char regvalue = 0x00;

	bRawdataTest = true;

	if(WriteReg(0x00, 0x40) < 0) {
		printk("Enter factory failure\n");
		bRawdataTest = false;
		goto Enter_WorkMode;
	}
	else
	{
		printk("Enter factory Successful\n");
	}
	focal_msleep(200);

	iTxNum = GetTxNum();
	focal_msleep(100);
	iRxNum = GetRxNum();
	//zax 20141116 ++++++++++++++
	TX_NUM=iTxNum;
	RX_NUM=iRxNum;
	//zax 20141116 -----------------
	bRawdataTest = FT5X46_TestItem();

Enter_WorkMode:	
	//the end, return work mode
	for (i = 0; i < 3; i++) {
		if (WriteReg(0x00, 0x00) >=0)
			break;
		else {
			focal_msleep(200);
		}
	}
	
}

boolean FT5X46_TestItem(void)
{
	boolean bTestReuslt = true;

	if(g_TestParam_FT5X46.RawDataTest)
		bTestReuslt = bTestReuslt & TestItem_RawDataTest_FT5X46();
	focal_msleep(1000);
	
	if(g_TestParam_FT5X46.SCAPCbTest)
		bTestReuslt = bTestReuslt & TestItem_SCapCbTest();
	focal_msleep(1000);
	if(g_TestParam_FT5X46.SCAPRawDataTest)
		bTestReuslt = bTestReuslt & TestItem_SCapRawDataTest();
	
	printk("[focal] %s result = %d  \n", __func__, bTestReuslt);
	focal_msleep(1000);
	//bTestReuslt = bTestReuslt & TestItem_WeakShortTest();

	return bTestReuslt;		
}
//zax 20141114--------------------------
boolean TestItem_RawDataTest_FT5X46(void)
{
	boolean bTestReuslt = true;
	boolean bUse = false;
	int i = 0, j = 0;
	short min_value, max_value;
	FTS_DBG("==============================Test Item: -----  RawDataTest_FT5X46\n");

	//设置低频点
	if(g_TestParam_FT5X46.RawDataTest_LowFreq == 1)
	{
		bUse = true;
		FTS_DBG("=========Set Frequecy Low\n");
		WriteReg(0x0a, 0x80);
		FTS_DBG("=========FIR State: ON\n");	
		WriteReg(0xFB, 0x01);
		focal_msleep(10);

		if( iTxNum == 0 || iRxNum == 0)
		{
			bTestReuslt  = false;
			FTS_DBG("Tx: %2d, Rx: %2d  \n", iTxNum, iRxNum);
			goto TEST_END;
		}

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);
		/////////////////Print RawData Start	
		printk("[FocalTech]dump rawdata, Tx: %2d, Rx: %2d \n", iTxNum, iRxNum);
		for(i = 0;i < iTxNum;i++)
		{
			printk("\n");
			for(j = 0;j < iRxNum;j++)
			{
				printk("%5d  ",rawdata[i][j]);

			}
		}
		printk("\n");
		/////////////////Print RawData End

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;
				min_value = g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					FTS_DBG("rawdata test failure. min_value=%d max_value=%d rawdata[%d][%d]=%d \n",  min_value, max_value, i+1, j+1, rawdata[i][j]);
				}
			}
		}

		FTS_DBG("=========FIR State: OFF\n");	
		WriteReg(0xFB, 0x00);	
		focal_msleep(10);

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_Low_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_Low_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					FTS_DBG("rawdata test failure. min_value=%d max_value=%d rawdata[%d][%d]=%d \n", \
						min_value, max_value, i+1, j+1, rawdata[i][j]);
				}
			}
		}	

	}

	//设置高频点
	if(g_TestParam_FT5X46.RawDataTest_HighFreq== 1)
	{
		bUse = true;
		FTS_DBG("=========Set Frequecy High\n");
		WriteReg(0x0a, 0x81);
		FTS_DBG("=========FIR State: ON\n");	
		WriteReg(0xFB, 0x01);
		focal_msleep(10);

		if( iTxNum == 0 || iRxNum == 0)
		{
			bTestReuslt  = false;
			FTS_DBG("Tx: %2d, Rx: %2d  \n", iTxNum, iRxNum);
			goto TEST_END;
		}

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);

		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					FTS_DBG("rawdata test failure. min_value=%d max_value=%d rawdata[%d][%d]=%d \n", \
						min_value, max_value, i+1, j+1, rawdata[i][j]);
				}
			}
		}

		FTS_DBG("=========FIR State: OFF\n");	
		WriteReg(0xFB, 0x00);	
		focal_msleep(10);

		//Read Raw Data
		for(i = 0; i < 2; i++)
		{
			focal_msleep(10);
			StartScan();
		}
		GetRawData(rawdata);
		printk("[FocalTech]zax1111111111 dump rawdata, Tx: %2d, Rx: %2d \n", iTxNum, iRxNum);
		//zax 20141116+++++++++++++++++++++++
		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				Save_rawData[i][j]=rawdata[i][j];
				printk("%5d  ",Save_rawData[i][j]);

			}
		}
		//zax 20141116------------------------------
		for(i = 0;i < iTxNum;i++)
		{
			for(j = 0;j < iRxNum;j++)
			{
				if(0 == g_TestParam_FT5X46.valid_node[i][j])  continue;

				min_value = g_TestParam_FT5X46.RawDataTest_High_Min_node[i][j];
				max_value = g_TestParam_FT5X46.RawDataTest_High_Max_node[i][j];

				if(rawdata[i][j] < min_value || rawdata[i][j] > max_value)
				{
					bTestReuslt  = false;
					FTS_DBG("rawdata test failure. min_value=%d max_value=%d rawdata[%d][%d]=%d \n", \
						min_value, max_value, i+1, j+1, rawdata[i][j]);
				}
			}
		}	

	}

TEST_END:

	if(bUse)
	{
		if( bTestReuslt)
		{
			printk("RawData Test is OK.\n");
		}
		else
		{
			printk("RawData Test is NG.\n");
		}
	}

	return bTestReuslt;
}

/*==================================================================================

微短路量产FW方面相关寄存器及功能如下：
ShortTestEn（0x07）：     微短路测试使能寄存器
ValLBuf0 （0xF4）：     ValL数据寄存器0
ValLBuf0 （0xF5）：     ValL数据寄存器1

微短路量产测试时Host端所需操作：
1. 微短路测试使能
Host:  W 07 01          // 主机发送使能后微短路测试一次；然后重新恢复到工厂模式

2. 测试数据读取
Host:  W F4  R FF     // 将自短和互短测试的所有ValL数据读出：GroundValL[64]和MutualValL[64] 
3. 数据说明：

Offset:  0x00~0x01

Ground_ShortTest:
地短路校准：2   Byte          
地短路数据：(TxNum+RxNum)*2   Byte  // Tx在前，Rx在后

Mutual_ShortTest:
互短路校准：2  Byte
互短路数据：(TxNum+RxNum)*2   Byte  // Tx在前，Rx在后
==================================================================================*/
#if 0
static boolean TestItem_WeakShortTest(void)
{
	//kernel_fpu_begin();	
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;
	boolean bRet = true;
	int i;
	int iAllAdcDataNum = 63;	
	int iMaxTx = 35;
	unsigned char iTxNum, iRxNum, iChannelNum;
	int iClbData_Ground, iClbData_Mutual, iOffset;
	int iAdcData[iAllAdcDataNum];
	int fKcal = 0;
	int fMShortResistance[80*2], fGShortResistance[80*2];
	//float fKcal = 0;
	//float fMShortResistance[80*2], fGShortResistance[80*2];
	int iDoffset = 0, iDsen = 0, iDrefn = 0, iMaxD = 0;
	int iMin_CG = g_TestParam_FT5X46.WeakShortTest_CG_Min;
	int iCount = 0;
	int iMin_CC = g_TestParam_FT5X46.WeakShortTest_CC_Min;


	FTS_DBG("==============================Test Item: -----  Weak Short-Circuit Test \n");
	//TxNum寄存器：0x02(Read Only)
	//RxNum寄存器：0x03(Read Only)	

	focal_msleep(100);
	ReCode = ReadReg(0x02, &iTxNum);//Get Tx
	focal_msleep(100);

	ReCode = ReadReg(0x03, &iRxNum);//Get Rx

	if(ReCode < ERROR_CODE_OK)
	{
		FTS_DBG("Read Register Error!\n");
		bRet = false;
		goto TEST_END;
	}

	iChannelNum = iTxNum + iRxNum;
	iMaxTx = iTxNum;
	iAllAdcDataNum = 1 + (1 + iTxNum + iRxNum)*2;//总通道数 + 对地校准数据 + 通道间校准数据 + Offset

	//iAdcData = kmalloc(sizeof(int)*iAllAdcDataNum, GFP_KERNEL);// = new int[iAllAdcDataNum];
	memset(iAdcData, 0, sizeof(iAdcData));
	for(i = 0; i < 2; i++)
	{
		ReCode = WeakShort_GetAdcData( iAllAdcDataNum*2, iAdcData );
		//	SysDelay(50);
		if(ReCode < ERROR_CODE_OK)
		{
			bRet = false;
			goto TEST_END;
		}
	}

	iOffset = iAdcData[0];
	iClbData_Ground = iAdcData[1];
	iClbData_Mutual = iAdcData[2 + iChannelNum];

	printk("Offset: %4d,	\n", iAdcData[0]);

	printk("\nGround Data:\n");
	for(i = 0; i <= iChannelNum; i++)
	{
		if(i == 0)
			printk("ClbData_Ground: %4d", iAdcData[i + 1]);			
		else if(i <= iMaxTx)
			printk("Tx%02d: %4d, ",i, iAdcData[i+1]);
		else
			printk("Rx%02d: %4d, ", i - iMaxTx, iAdcData[i+1]);

		if(i % 5 == 0)
			printk("\n");
	}

	printk("\nChannel Data:\n");	
	for(i = 0; i <= iChannelNum; i++)
	{	
		if(i == 0)
			printk("ClbData_Mutual: %4d", iAdcData[i + 2 + iChannelNum]);			
		else if(i <= iMaxTx)
			printk("Tx%02d: %4d, ", i, iAdcData[i + 2 + iChannelNum]);
		else
			printk("Rx%02d: %4d, ", i - iMaxTx, iAdcData[i + 2 + iChannelNum]);

		if(i % 5 == 0)
			printk("\n");

	}

	//FTS_DBG("CC: %d, CG: %d, Tx: %d, Rx: %d\n", iMin_CC, iMin_CG, iTxNum, iRxNum);

	/////////////////////////////////////////////对地数据
	//::GetPrivateProfileString("BaseSet","Weak_Short_Min","992", str.GetBuffer(MAX_PATH),MAX_PATH,g_strIniFile);
	//int iMin_CG = atoi(str);

	iDoffset = iOffset - 1024;
	iDrefn = iClbData_Ground;

	fKcal = 1;
	//strAdc = "\r\n\r\nShort Circuit (Channel and Ground):\r\n";
	//str.Format("\r\nKcal: %.02f\r\n", fKcal);
	//strAdc += str;

	for(i = 0; i < iChannelNum; i++)
	{
		iDsen = iAdcData[i+2];
		if((2047+iDoffset) - iDsen <= 0)//小于等于0，直接判PASS
		{
			continue;
		}

		//采用新的公式进行计算
		/*fGShortResistance[i] = (float)( iDsen - iDoffset + 410 ) * 251 * fKcal / (10*( 2047 + iDoffset - iDsen )) - 3;*/
		fGShortResistance[i] = ( iDsen - iDoffset + 410 ) * 251 * fKcal / (10*( 2047 + iDoffset - iDsen )) - 3;

		if(fGShortResistance[i] < 0) fGShortResistance[i] = 0;
		if((iMin_CG > fGShortResistance[i]) || (iDsen - iDoffset < 0))//小于等于0，直接判0欧短路
		{
			if(iCount == 0)
				FTS_DBG("Short Circuit (Channel and Ground):\n");	

			iCount++;
			if(i+1 <= iMaxTx)
				FTS_DBG("Tx%02d: %d(kOhm),\n", i+1, fGShortResistance[i]);
			else
				FTS_DBG("Rx%02d: %d(kOhm),\n", i+1 - iMaxTx, fGShortResistance[i]);
		}
	}
	if(iCount > 0)
	{
		//TestResultInfo(strAdc);
		bRet = false;
	}

	/////////////////////////////////////////////通道间数据
	//::GetPrivateProfileString("BaseSet","Weak_Short_Min_CC","992", str.GetBuffer(MAX_PATH),MAX_PATH,g_strIniFile);
	//int iMin_CC = atoi(str);

	iDoffset = iOffset - 1024;
	iDrefn = iClbData_Mutual;	
	fKcal = 1.0;
	//strAdc = "\r\n\r\nShort Circuit (Channel and Channel):\r\n";
	iCount = 0;
	for(i = 0; i < iChannelNum; i++)
	{
		iDsen = iAdcData[i+iChannelNum + 3];
		iMaxD = iDrefn;
		if(iMaxD<(iDoffset + 116)) iMaxD = iDoffset + 116;
		//if(iDsen - iDrefn <= 0)  continue;
		if(iDsen - iMaxD<= 0)  continue;

		//采用新的公式
		/*fMShortResistance[i] = ( (float)1931 / ( iDsen - iDoffset - 116 ) * 16  - 19 ) *  fKcal - 6;*/
		//fMShortResistance[i] = ( (1931* 16) / ( iDsen - iDoffset - 116 )   - 19 ) *  fKcal - 6;//last fomula
		fMShortResistance[i] = ( ((2047 + iDoffset - iMaxD) / ( iDsen - iMaxD) ) *24 - 27 ) *  fKcal - 6;
		//if(fMShortResistance[i] < 0)continue;
		if(fMShortResistance[i] < 0 && fMShortResistance[i] >= -240 ) fMShortResistance[i] = 0;
		else if( fMShortResistance[i] < -240 )  continue;

		if( fMShortResistance[i] <= 0  || fMShortResistance[i] < iMin_CC )
		{
			if(iCount == 0)
				FTS_DBG("\nShort Circuit (Channel and Channel):\n");

			iCount++;
			if(i+1 <= iMaxTx)
				FTS_DBG("Tx%02d: %d(kOhm),\n", i+1, fMShortResistance[i]);
			else
				FTS_DBG("Rx%02d: %d(kOhm),\n", i+1 - iMaxTx, fMShortResistance[i]);

		}
	}
	if(iCount > 0)
	{
		//TestResultInfo(strAdc);
		bRet = false;
	}

TEST_END:
	if(ReCode < ERROR_CODE_OK) bRet = false;

	if( bRet )
	{
		FTS_DBG("//Weak Short Test is OK.\n");
	}
	else
	{
		FTS_DBG("//Weak Short Test is NG.\n");
	}

	//kfree(iAdcData);

	return bRet;
	//kernel_fpu_end();
	//return true;
}
#endif
/*==================================================================================

微短路量产FW方面相关寄存器及功能如下：
ShortTestEn（0x07）：     微短路测试使能寄存器
ValLBuf0 （0xF4）：     ValL数据寄存器0
ValLBuf0 （0xF5）：     ValL数据寄存器1

微短路量产测试时Host端所需操作：
1. 微短路测试使能
Host:  W 07 01          // 主机发送使能后微短路测试一次；然后重新恢复到工厂模式

2. 测试数据读取
Host:  W F4  R FF     // 将自短和互短测试的所有ValL数据读出：GroundValL[64]和MutualValL[64] 
3. 数据说明：

Offset:  0x00~0x01

Ground_ShortTest:
地短路校准：2   Byte          
地短路数据：(TxNum+RxNum)*2   Byte  // Tx在前，Rx在后

Mutual_ShortTest:
互短路校准：2  Byte
互短路数据：(TxNum+RxNum)*2   Byte  // Tx在前，Rx在后

==================================================================================*/
#if 0
static int WeakShort_GetAdcData( int AllAdcDataLen, int *pRevBuffer )
{
	int i = 0;
	unsigned char ReCode = ERROR_CODE_OK;
	int iReadDataLen = AllAdcDataLen;//Offset*2 + (ClbData + TxNum + RxNum)*2*2
	//unsigned char *pDataSend = NULL;	
	unsigned char Data = 0xff;

	unsigned char pDataSend[iReadDataLen+1];
	//if(pDataSend == NULL)	return 0x0d;


	ReCode = WriteReg(0x07, 0x01);// 主机发送使能后微短路测试一次
	if(ReCode < ERROR_CODE_OK)return ReCode;

	focal_msleep(200);

	for(i = 0; i < 40; i++)//准备好数据后，FW将0x07寄存器置为0
	{
		focal_msleep(50);//SysDelay(5);
		ReCode = ReadReg(0x07, &Data);
		if(ReCode >= ERROR_CODE_OK)
		{
			if(Data == 0)break;
		}
	}

	if(Data != 0)//if Data != 0, no ready
	{
		FTS_DBG("//Check Data: %d.\n", Data);
		return -1;
	}

	pDataSend[0] = 0xF4;
	//ReCode = HY_IIC_IO(hDevice, pDataSend, 1, pDataSend + 1, iReadDataLen);
	//focal_I2C_write(pDataSend, 1);	
	ReCode = focal_I2C_Read(pDataSend, 1, pDataSend + 1, iReadDataLen);

	if(ReCode >= ERROR_CODE_OK)
	{
		for(i = 0; i < iReadDataLen/2; i++)
		{
			pRevBuffer[i] = (pDataSend[1 + 2*i]<<8) + pDataSend[1 + 2*i + 1];
			//printk("Ch%02d: %02d, \n", i+1, pRevBuffer[i]);
		}
	}

	//kfree(pDataSend);

	return ReCode;
}
#endif
	//zax 20141114+++++++++++++++++++++++
boolean NeedProofOnTest(unsigned char chVal)
{
        return !( chVal & 0x20 );
}
boolean NeedProofOffTest(unsigned char chVal)
{
       return !( chVal & 0x80 );
}
bool NeedTxOnVal(unsigned char chVal)
{
       return !( chVal & 0x40 ) || !( chVal & 0x04 );
}
bool NeedRxOnVal(unsigned char chVal)
{
       return !( chVal & 0x40 ) || ( chVal & 0x04 );
}
bool NeedTxOffVal(unsigned char chVal)
{
       return 0x00 == (chVal & 0x03) || 0x10 == ( chVal & 0x03 );
}
bool NeedRxOffVal(unsigned char chVal)
{ 
       return 0x01 == (chVal & 0x03) || 0x10 == ( chVal & 0x03 );
}
unsigned char GetChannelNumNoMapping(void)
{
	//CString strTestResult, str, strTemp;
	unsigned char ReCode;
	//int TxNum, RxNum;

	unsigned char rBuffer[1]; //= new unsigned char;


	printk("Get Tx Num...\n");
	ReCode =ReadReg( 0x55,  rBuffer);
	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{
		SCap_iTxNum = rBuffer[0];	
	}
	else
	{
		
		printk("Failed to get Tx number\n");
	}

	
	printk("Get Rx Num...\n");
	ReCode = ReadReg( 0x56,  rBuffer);
	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{
		SCap_iRxNum = rBuffer[0];
	}
	else
	{
		
		printk("Failed to get Rx number\n");
	}

	return ReCode;
}
boolean SwitchToNoMapping(void)
{
	unsigned char chPattern = -1;
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char RegData = -1;
	ReCode = ReadReg( 0x53, &chPattern );

	//printk("zax1 Switch To NoMapping ReCode %d\n",ReCode);

	//printk("zax1 Switch To NoMapping chPattern %d\n",chPattern);
	if(1 == chPattern)
	{
		RegData = -1;
		ReCode =ReadReg( 0x54, &RegData );
		//printk("zax2 Switch To NoMapping RegData %d    ReCode   %d\n",RegData,ReCode);
		if( 1 != RegData ) 
		{
			ReCode = WriteReg( 0x54, 1 );  //0-mapping 1-no mampping
			//printk("zax2 Switch To NoMapping   ReCode   %d\n",ReCode);
			focal_msleep(20);
			GetChannelNumNoMapping();
		}
	}
	else
	{
		SCap_iTxNum=iTxNum;
		SCap_iRxNum=iRxNum;

	}
	//if( ReCode != ERROR_CODE_OK )
	if(ReCode < ERROR_CODE_OK)
	{
		printk("Switch To NoMapping Failed!\n");
		return false;
	}
	return true;
}
unsigned char GetTxSC_CB(unsigned char index, unsigned char *pcbValue)
{
	unsigned char ReCode = ERROR_CODE_OK;
	unsigned char wBuffer[4];
	if(index<128)//单个读取
	{	
		*pcbValue = 0;

		WriteReg(REG_ScCbAddrR, index);
		ReCode = ReadReg(REG_ScCbBuf0, pcbValue);
	}
	else//连续读取，长度为index-128
	{
		
		//unsigned char rBuffer[2];

		WriteReg(REG_ScCbAddrR, 0);

		wBuffer[0] = REG_ScCbBuf0;
		//if(HY_I2C_INTERFACE == iCommMode)
		//{			
			//ReCode = Comm_Base_IIC_IO(hDevice, wBuffer, 1, pcbValue, index-128);
			ReCode = focal_I2C_Read(wBuffer, 1, pcbValue, index-128);
		//}
		//else if(HY_SPI_INTERFACE == iCommMode)
		//{
			//placeholder
		//}
		//else if(HY_USB_INTERFACE == iCommMode)
		//{
		//	ReCode = GetUsbDataByCMD(this, CMD_READ_CB_ALL, pcbValue, index-128);
		//}
	}	

	return ReCode;
}

boolean TestItem_SCapCbTest()
{
	//CString str, strTemp;
	int i, index,Value,CBMin,CBMax;
	boolean bFlag = true;
	unsigned char ReCode;
	boolean btmpresult = true;
	int iMax, iMin, iAvg;
	unsigned char pReadData[300] = {0};
	unsigned char I2C_wBuffer[1];

	//TestResultInfo("\r\n\r\n==============================Test Item: -----  Scap CB Test \r\n\r\n");	

	//PrintInvalidDataScap();

	//unsigned char regAddr = 0x06, regData = 0;
	unsigned char regAddr = 0x09, regData = 0;

   	 ReCode = ReadReg( regAddr, &regData );
	//printk("zaxzax 123  %d  %d \n",ReCode,regData);
	

	btmpresult= SwitchToNoMapping();
	//printk("zax StartScan btmpresult %d   \n",btmpresult);
	if( !btmpresult ) goto TEST_END;

	ReCode=StartScan();
	//printk("zax StartScan ReCode %d   \n",ReCode);
	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{			
		for(i = 0; i < 1; i++)
		{
			memset(SCap_rawData, 0, sizeof(SCap_rawData));
			memset(pReadData, 0, sizeof(pReadData));
			
			//防水CB
			I2C_wBuffer[0] = REG_ScCbBuf0;
			ReCode = WriteReg( 0x44, 1 );//自容工作方式选择:  1：防水 0:非防水
			StartScan();
			ReCode = WriteReg( REG_ScCbAddrR, 0 );
			
			ReCode = GetTxSC_CB( SCap_iTxNum + SCap_iRxNum + 128, pReadData );
			for ( index = 0; index < SCap_iRxNum; ++index )
			{
				SCap_rawData[0 + SCap_iTxNum][index]= pReadData[index];
				printk("zax SCap_rawData0  %d   \n",pReadData[index]);
			}
			for ( index = 0; index < SCap_iTxNum; ++index )
			{
				SCap_rawData[1 + SCap_iTxNum][index] = pReadData[index + SCap_iRxNum];
				printk("zax SCap_rawData1  %d   \n",pReadData[index + SCap_iRxNum]);
			}

			//非防水rawdata
			I2C_wBuffer[0] = REG_ScCbBuf0;
			ReCode = WriteReg( 0x44, 0 );//自容工作方式选择:  1：防水 0:非防水
			StartScan();
			ReCode = WriteReg( REG_ScCbAddrR, 0 );
			//ReCode = theDevice.m_cHidDev[m_NumDevice]->HY_IIC_IO( theDevice.m_cHidDev[m_NumDevice]->hDevice, I2C_wBuffer, 1, pReadData, g_ScreenSetParam.iRxNum + g_ScreenSetParam.iTxNum );
			ReCode = GetTxSC_CB( SCap_iRxNum + SCap_iTxNum + 128, pReadData );
			for ( index = 0; index < SCap_iRxNum; ++index )
			{
				SCap_rawData[2 + SCap_iTxNum][index]= pReadData[index];
				printk("zax SCap_rawData2  %d   \n",pReadData[index]);
			}
			for ( index = 0; index < SCap_iTxNum; ++index )
			{
				SCap_rawData[3 + SCap_iTxNum][index] = pReadData[index + SCap_iRxNum];
				printk("zax SCap_rawData3  %d   \n",pReadData[index + SCap_iRxNum]);
			}

			//if( ReCode != ERROR_CODE_OK )	
			if( ReCode < ERROR_CODE_OK )
			{
				
				printk("Failed to Get SCap CB!\n");
			}		
		}
	}
	else
	{
		printk("Failed to Scan SCap CB!!\n");
	}

	//----------------------------------------------------------判断超过范围的rawData


	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{	

		bFlag=NeedProofOnTest(regData);
		//printk("zax NeedProofOnTest %d		%d\n", bFlag,g_TestParam_FT5X46.SCapCbTest_SetWaterproof_ON);
		if(g_TestParam_FT5X46.SCapCbTest_SetWaterproof_ON && bFlag)
		{
			CBMin = g_TestParam_FT5X46.SCapCbTest_ON_Min_Value;
			CBMax = g_TestParam_FT5X46.SCapCbTest_ON_Max_Value;
	
			iMax = -SCap_rawData[0+SCap_iTxNum][0];
			iMin = 2 * SCap_rawData[0+SCap_iTxNum][0];
			iAvg = 0;
			Value = 0;
			//strTemp = "\r\n////////////////////// WaterProof On Mode:";
			//strTemp += "\r\nSCap CB_Rx:	";
			printk("WaterProof On Mode:  \n");
			printk("SCap CB_Rx:  \n");
			bFlag=NeedRxOnVal(regData);
			SCab_1=bFlag;//zax 20141116+++++++++++++++++++++++
			//printk("zaxzaxzax NeedRxOnVal  %d   \n",bFlag);
			for( i = 0;bFlag && i < SCap_iRxNum; i++ )
			{
				//if( m_ScapInvalide[0][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[0][i] == 0 )      continue;

				Value = SCap_rawData[0+SCap_iTxNum][i];
				Save_rawData[0+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				iAvg += Value;
				//str.Format("%5d	", Value);

				CBMin = g_TestParam_FT5X46.SCapCbTest_ON_Min[0][i];
				CBMax = g_TestParam_FT5X46.SCapCbTest_ON_Max[0][i];

				//strTemp += str;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > CBMax || Value < CBMin) btmpresult = false;
			}
			//strTemp += "\r\nSCap CB_Tx:	";
			printk("SCap CB_Tx:  \n");
			bFlag=NeedTxOnVal(regData) ;
			SCab_2=bFlag;//zax 20141116+++++++++++++++++++++++
			//printk("zaxzaxzax NeedTxOnVal  %d   \n",bFlag);
			for(i = 0;bFlag &&  i < SCap_iTxNum; i++)
			{
				//if( m_ScapInvalide[1][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[1][i] == 0 )      continue;

				Value = SCap_rawData[1+SCap_iTxNum][i];
				Save_rawData[1+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				//str.Format("%5d	", Value);
				//strTemp += str;

				CBMin = g_TestParam_FT5X46.SCapCbTest_ON_Min[1][i];
				CBMax = g_TestParam_FT5X46.SCapCbTest_ON_Max[1][i];

				iAvg += Value;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > CBMax || Value < CBMin) btmpresult = false;
			}
			//TestResultInfo(strTemp);

			iAvg = iAvg/(SCap_iTxNum + SCap_iRxNum);
			//str.Format("\r\n\r\n// Max SCap CB: %d, Min SCap CB: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);

			printk("Max SCap CB: %d, Min SCap CB: %d, Deviation Value: %d, Average Value: %d\n", iMax, iMin, iMax - iMin, iAvg);
			//TestResultInfo(str);

			////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>收集测试数据，存入CSV文件
			/*
			strTemp = "";
			for(int i = 0; i < 2; i++)
			{		
				strTemp += "\r\n";
				for(int j = 0; j < SCap_iRxNum; j++)
				{	
					str.Format("%d,", SCap_rawData[i + SCap_iTxNum][j]);
					strTemp += str;
				}
			}
			SaveTestData( strTemp, 2, SCap_iRxNum );
			*/
			////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<收集测试数据，存入CSV文件
		}
		bFlag=NeedProofOffTest(regData);
		printk("NeedProofOffTest %d		%d\n", bFlag,g_TestParam_FT5X46.SCapCbTest_SetWaterproof_OFF);
		if(g_TestParam_FT5X46.SCapCbTest_SetWaterproof_OFF && bFlag)
		{
			CBMin = g_TestParam_FT5X46.SCapCbTest_OFF_Min_Value;
			CBMax = g_TestParam_FT5X46.SCapCbTest_OFF_Max_Value;
			//iMax = iMin = RawData[0+g_ScreenSetParam.iTxNum][0];
			iMax = -SCap_rawData[2+SCap_iTxNum][0];
			iMin = 2 * SCap_rawData[2+SCap_iTxNum][0];
			iAvg = 0;
			Value = 0;

			//strTemp = "\r\n\r\n////////////////////// WaterProof Off Mode:";
			//strTemp += "\r\nSCap CB_Rx:	";
			printk("WaterProof Off Mode:  \n");
			printk("SCap CB_Rx:  \n");
			bFlag=NeedRxOffVal(regData);
			SCab_3=bFlag;//zax 20141116+++++++++++++++++++++++
			//printk("zaxzaxzax NeedRxOffVal  %d   \n",bFlag);
			for(i = 0;bFlag &&  i < SCap_iRxNum; i++)
			{
				//if( m_ScapInvalide[0][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[0][i] == 0 )      continue;

				Value = SCap_rawData[2+SCap_iTxNum][i];
				Save_rawData[2+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				iAvg += Value;
				//str.Format("%5d	", Value);

				CBMin = g_TestParam_FT5X46.SCapCbTest_OFF_Min[0][i];
				CBMax = g_TestParam_FT5X46.SCapCbTest_OFF_Max[0][i];

				//strTemp += str;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if( Value > CBMax || Value < CBMin ) btmpresult = false;
			}
			//strTemp += "\r\nSCap CB_Tx:	";
			printk("SCap CB_Tx:  \n");
			bFlag=NeedTxOffVal(regData);
			SCab_4=bFlag;//zax 20141116+++++++++++++++++++++++
			//printk("zaxzaxzaxNeedTxOffVal  %d   \n",bFlag);
			for(i = 0; bFlag && i < SCap_iTxNum; i++)
			{
				//if( m_ScapInvalide[1][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[1][i] == 0 )      continue;

				Value = SCap_rawData[3+SCap_iTxNum][i];
				Save_rawData[3+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				//str.Format("%5d	", Value);
				//strTemp += str;

				CBMin = g_TestParam_FT5X46.SCapCbTest_OFF_Min[1][i];
				CBMax = g_TestParam_FT5X46.SCapCbTest_OFF_Max[1][i];

				iAvg += Value;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > CBMax || Value < CBMin) btmpresult = false;
			}
			//TestResultInfo(strTemp);

			iAvg = iAvg/(SCap_iTxNum + SCap_iRxNum);
			//str.Format("\r\n\r\n// Max SCap CB: %d, Min SCap CB: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);
			printk("Max SCap CB: %d, Min SCap CB: %d, Deviation Value: %d, Average Value: %d\n", iMax, iMin, iMax - iMin, iAvg);
			//TestResultInfo(str);

			////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>收集测试数据，存入CSV文件
			/*
			strTemp = "";
			for(int i = 0; i < 2; i++)
			{		
				strTemp += "\r\n";
				for(int j = 0; j < SCap_iRxNum; j++)
				{	
					str.Format("%d,", SCap_rawData[i + 2 + SCap_iTxNum][j]);
					strTemp += str;
				}
			}
			SaveTestData(strTemp, 2, SCap_iRxNum, 2);
			*/
			////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<收集测试数据，存入CSV文件
		}
	}
	else
	{
		btmpresult = false;
	}

TEST_END:
	if(btmpresult)
	{
		//TestResultInfo("\r\n\r\n//SCap CB Test is OK.\r\n",1);
		printk("SCap CB Test is OK\n");
		//* bTestResult = true;
	}
	else
	{
		//TestResultInfo("\r\n\r\n//SCap CB Test is NG.\r\n",0);
		printk("SCap CB Test is NG\n");
		//* bTestResult = false;
	}
	return btmpresult;
}

unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned short BytesNumInTestMode3, BytesNumInTestMode2,BytesNumInTestMode1;
	unsigned short BytesNumInTestMode6, BytesNumInTestMode5,BytesNumInTestMode4;
	
	unsigned char ReCode=ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3];
	//unsigned char *pReadData = new unsigned char[ByteNum];
	//unsigned char *pReadDataTmp = new unsigned char[ByteNum*2];
	int i,iReadNum;
	unsigned char *pReadData = NULL;
	unsigned char *pReadDataTmp = NULL;
	pReadData =kmalloc(ByteNum, GFP_KERNEL);
	pReadDataTmp =kmalloc(ByteNum*2, GFP_KERNEL);

	BytesNumInTestMode3=0;
	BytesNumInTestMode2=0;
	BytesNumInTestMode1=0;
	BytesNumInTestMode6=0;
	BytesNumInTestMode5=0;
	BytesNumInTestMode4=0;

	//if(hDevice == NULL)		return ERROR_CODE_NO_DEVICE;
	if(pReadData == NULL)		return ERROR_CODE_ALLOCATE_BUFFER_ERROR;

	iReadNum=ByteNum/342;

	if(0 != (ByteNum%342)) 
		iReadNum++;

	if(ByteNum <= 342)
	{
		BytesNumInTestMode1 = ByteNum;		
	}
	else
	{
		BytesNumInTestMode1 = 342;
	}
	
	ReCode = WriteReg(REG_LINE_NUM, LineNum);//Set row addr;

	//printk("zax ReadRawData1 %d\n",ReCode);
		//***********************************************************Read raw data in test mode1
		

		I2C_wBuffer[0] = REG_RawBuf0;	//set begin address
		//if(ReCode == ERROR_CODE_OK)
		if(ReCode >= ERROR_CODE_OK)
		{
			//ReCode = Comm_Base_IIC_IO(hDevice, I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
			focal_msleep(10);
			ReCode =focal_I2C_Read(I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
			
		}
		//printk("zax ReadRawData1 %d\n",ReCode);
		//printk("zax ReadRawData1 iReadNum  %d\n",iReadNum);
		for(i=1; i<iReadNum; i++)
		{//printk("zax ReadRawData  conti\n");
			//if(ReCode != ERROR_CODE_OK) 
			if(ReCode < ERROR_CODE_OK) 
			{
				//printk("zax ReadRawData  break\n");
				break;
			}
			if(i==iReadNum-1)//last packet
			{
				//ReCode = Comm_Base_IIC_IO(hDevice, NULL, 0, pReadData+342*i, ByteNum-342*i);
				focal_msleep(10);
				ReCode =focal_I2C_Read(NULL, 0, pReadData+342*i, ByteNum-342*i);
			}
			else
			{
				//ReCode = Comm_Base_IIC_IO(hDevice, NULL, 0, pReadData+342*i, 342);
				focal_msleep(10);
				ReCode =focal_I2C_Read(NULL, 0, pReadData+342*i, 342);
			}

		}
		
	
	//printk("zax final ReadRawData %d\n",ReCode);
	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{
		for(i=0; i<(ByteNum>>1); i++)
		{
			pRevBuffer[i] = (pReadData[i<<1]<<8)+pReadData[(i<<1)+1];
			printk("zaxzax ReadRawData %d\n",pRevBuffer[i]);
			
		}
	}

	//delete []pReadData;
	//delete []pReadDataTmp;
	kfree(pReadData);
	kfree(pReadDataTmp);
	return ReCode;
}

static int g_prawData[300] = {0};
boolean TestItem_SCapRawDataTest()
{
	//CString str, strTemp;
	int i,RawDataMin,RawDataMax,Value;
	boolean bFlag = true;
	unsigned char ReCode;
	boolean btmpresult = true;
	int iMax, iMin, iAvg;
	//int prawData[300] = {0};
	int *prawData = g_prawData;

	//TestResultInfo("\r\n\r\n==============================Test Item: -----  Scap RawData Test \r\n\r\n");	

	//PrintInvalidDataScap();

	//unsigned char regAddr = 0x06, regData = 0;
	unsigned char regAddr = 0x09, regData = 0;

   	ReCode = ReadReg( regAddr, &regData );
	//printk("zaxzax 123  %d   \n",regData);

	btmpresult= SwitchToNoMapping();
	if( !btmpresult ) goto TEST_END;

	ReCode = StartScan();
	//if(ReCode == ERROR_CODE_OK)
	//printk("zax StartScan %d\n",ReCode);
	if(ReCode >= ERROR_CODE_OK)
	{			
		for(i = 0; i < 1; i++)
		{
			memset(SCap_rawData, 0, sizeof(SCap_rawData));

			memset(prawData, 0, sizeof(*prawData));
 
			
			//防水rawdata
			//printk("zax TestItem_SCapRawDataTest (SCap_iRxNum + SCap_iTxNum)*2 %d %d 	%d\n",SCap_iTxNum,SCap_iRxNum,(SCap_iRxNum + SCap_iTxNum)*2);
			
			for(i=0;i<2;i++)
				ReCode = ReadRawData(0, 0xAC, (SCap_iRxNum + SCap_iTxNum)*2, prawData );
			//printk("zax TestItem_SCapRawDataTest ReadRawData1 %d\n",ReCode);

			
			memcpy( SCap_rawData[0+SCap_iTxNum], prawData, sizeof(int)*SCap_iRxNum );
			memcpy( SCap_rawData[1+SCap_iTxNum], prawData + SCap_iRxNum, sizeof(int)*SCap_iTxNum );

            		//非防水rawdata
            		for(i=0;i<2;i++)
				ReCode = ReadRawData(0, 0xAB, (SCap_iRxNum + SCap_iTxNum)*2, prawData );

			//printk("zax TestItem_SCapRawDataTest ReadRawData2 %d\n",ReCode);

			
			memcpy( SCap_rawData[2+SCap_iTxNum], prawData, sizeof(int)*SCap_iRxNum );
			memcpy( SCap_rawData[3+SCap_iTxNum], prawData + SCap_iRxNum, sizeof(int)*SCap_iTxNum );

			//if( ReCode != ERROR_CODE_OK )	
			if( ReCode < ERROR_CODE_OK )	
			{
				//str.Format("Error Code: %s",g_ErrorMsg[ReCode]);
				//TestResultInfo("\r\nFailed to Get SCap RawData! " + str);
				printk("Failed to Get SCap RawData!\n");
			}		
		}
	}
	else
	{
		//str.Format("Error Code: %s",g_ErrorMsg[ReCode]);
		//TestResultInfo("\r\nFailed to Scan SCap RawData! " + str);

		printk("Failed to Scan SCap RawData! \n");
	}

	//----------------------------------------------------------判断超过范围的rawData


	//if(ReCode == ERROR_CODE_OK)
	if(ReCode >= ERROR_CODE_OK)
	{	

		bFlag=NeedProofOnTest(regData);
		
		if(g_TestParam_FT5X46.SCapRawDataTest_SetWaterproof_ON && bFlag )
		{
			RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_ON_Min_Value;
			RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_ON_Max_Value;
			//iMax = iMin = RawData[0+g_ScreenSetParam.iTxNum][0];
			iMax = -SCap_rawData[0+SCap_iTxNum][0];
			iMin = 2 * SCap_rawData[0+SCap_iTxNum][0];
			iAvg = 0;
			Value = 0;
			//strTemp = "\r\n////////////////////// WaterProof On Mode:";
			//strTemp += "\r\nSCap RawData_Rx:	";
			printk("SCap RawData_Rx:\n");
			bFlag=NeedRxOnVal(regData);
			SCab_5=bFlag;//zax 20141116+++++++++++++++++++++++
			for( i = 0; bFlag && i < SCap_iRxNum; i++ )
			{
				if( g_TestParam_FT5X46.InvalidNodes[0][i] == 0 )      continue;

				Value = SCap_rawData[0+SCap_iTxNum][i];
				Save_rawData[4+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				iAvg += Value;
				//str.Format("%5d	", Value);
				RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_ON_Min[0][i];
				RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_ON_Max[0][i];

				printk("zaxzax1 Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
				
				//strTemp += str;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > RawDataMax || Value < RawDataMin) btmpresult = false;
			}
			//strTemp += "\r\nSCap RawData_Tx:	";

			printk("SCap RawData_Tx:\n");
			bFlag=NeedTxOnVal(regData);
			SCab_6=bFlag;//zax 20141116+++++++++++++++++++++++
			for(i = 0;bFlag && i < SCap_iTxNum; i++)
			{
				//if( m_ScapInvalide[1][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[1][i] == 0 )      continue;

				Value = SCap_rawData[1+SCap_iTxNum][i];
				Save_rawData[5+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				//str.Format("%5d	", Value);
				//strTemp += str;
				RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_ON_Min[1][i];
				RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_ON_Max[1][i];
				printk("zaxzaxzax Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
				iAvg += Value;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > RawDataMax || Value < RawDataMin) btmpresult = false;
			}
			//TestResultInfo(strTemp);

			iAvg = iAvg/(SCap_iTxNum + SCap_iRxNum);
			//str.Format("\r\n\r\n// Max SCap RawData: %d, Min SCap RawData: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);
			printk("Max SCap RawData: %d, Min SCap RawData: %d, Deviation Value: %d, Average Value: %d\n", iMax, iMin, iMax - iMin, iAvg);
			//TestResultInfo(str);

			////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>收集测试数据，存入CSV文件
			/*
			strTemp = "";
			for(int i = 0; i < 2; i++)
			{		
				strTemp += "\r\n";
				for(int j = 0; j < SCap_iRxNum; j++)
				{	
					str.Format("%d,", SCap_rawData[i + SCap_iTxNum][j]);
					strTemp += str;
				}
			}
			SaveTestData(strTemp, 2, SCap_iRxNum);
			*/
			////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<收集测试数据，存入CSV文件
		}
		bFlag=NeedProofOffTest(regData);
		if(g_TestParam_FT5X46.SCapRawDataTest_SetWaterproof_OFF && bFlag)
		{
			RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_OFF_Min_Value;
			RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_OFF_Max_Value;
			//iMax = iMin = RawData[0+g_ScreenSetParam.iTxNum][0];
			iMax = -SCap_rawData[2+SCap_iTxNum][0];
			iMin = 2 * SCap_rawData[2+SCap_iTxNum][0];
			iAvg = 0;
			Value = 0;

			//strTemp = "\r\n\r\n////////////////////// WaterProof Off Mode:";
			//strTemp += "\r\nSCap RawData_Rx:	";
			printk("SCap RawData_Rx:\n");
			bFlag=NeedRxOffVal(regData);
			SCab_7=bFlag;//zax 20141116+++++++++++++++++++++++
			for(i = 0; bFlag && i < SCap_iRxNum; i++)
			{
				//if( m_ScapInvalide[0][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[0][i] == 0 )      continue;

				Value = SCap_rawData[2+SCap_iTxNum][i];
				Save_rawData[6+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				iAvg += Value;
				//str.Format("%5d	", Value);

				RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_OFF_Min[0][i];
				RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_OFF_Max[0][i];
				printk("zaxzax3 Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
				//strTemp += str;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > RawDataMax || Value < RawDataMin) btmpresult = false;
			}
			//strTemp += "\r\nSCap RawData_Tx:	";
			printk("SCap RawData_Tx:\n");
			bFlag=NeedTxOffVal(regData);
			SCab_8=bFlag;//zax 20141116+++++++++++++++++++++++
			for(i = 0; bFlag && i < SCap_iTxNum; i++)
			{
				//if( m_ScapInvalide[1][i] == 0 )      continue;
				if( g_TestParam_FT5X46.InvalidNodes[1][i] == 0 )      continue;

				Value = SCap_rawData[3+SCap_iTxNum][i];
				Save_rawData[7+SCap_iTxNum][i]=Value;//zax 20141116+++++++++++++++++++++++
				//str.Format("%5d	", Value);
				//strTemp += str;

				RawDataMin = g_TestParam_FT5X46.SCapRawDataTest_OFF_Min[1][i];
				RawDataMax = g_TestParam_FT5X46.SCapRawDataTest_OFF_Max[1][i];
				printk("zaxzax4 Value %d RawDataMin %d  RawDataMax %d  \n", Value, RawDataMin, RawDataMax);
				iAvg += Value;
				if(iMax < Value) iMax = Value;
				if(iMin > Value) iMin = Value;
				if(Value > RawDataMax || Value < RawDataMin) btmpresult = false;
			}
			//TestResultInfo(strTemp);

			iAvg = iAvg/(SCap_iTxNum + SCap_iRxNum);
			//str.Format("\r\n\r\n// Max SCap RawData: %d, Min SCap RawData: %d, Deviation Value: %d, Average Value: %d", iMax, iMin, iMax - iMin, iAvg);

			printk("Max SCap RawData: %d, Min SCap RawData: %d, Deviation Value: %d, Average Value: %d\n", iMax, iMin, iMax - iMin, iAvg);
			//TestResultInfo(str);

			////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>收集测试数据，存入CSV文件
			/*
			strTemp = "";
			for(int i = 0; i < 2; i++)
			{		
				strTemp += "\r\n";
				for(int j = 0; j < SCap_iRxNum; j++)
				{	
					str.Format("%d,", SCap_rawData[i + 2 + SCap_iTxNum][j]);
					strTemp += str;
				}
			}
			SaveTestData(strTemp, 2, SCap_iRxNum, 2);
			*/
			////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<收集测试数据，存入CSV文件
		}
	}
	else
	{
		btmpresult = false;
	}

TEST_END:
	if(btmpresult)
	{
		//TestResultInfo("\r\n\r\n//SCap RawData Test is OK.\r\n",1);
		printk("SCap RawData Test is OK\n");
		//* bTestResult = true;
	}
	else
	{
		//TestResultInfo("\r\n\r\n//SCap RawData Test is NG.\r\n",0);
		printk("SCap RawData Test is NG\n");
		//* bTestResult = false;
	}
	return btmpresult;
}
//zax 20141114---------------------------------