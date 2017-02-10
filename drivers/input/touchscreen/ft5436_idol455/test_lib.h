#ifndef _FOCAL_MCAPTESTLIB_H
#define _FOCAL_MCAPTESTLIB_H
//zax 20141116+++++++++++++++++++++++
//#define TX_NUM_MAX 80
//#define RX_NUM_MAX 80
#define TX_NUM_MAX 80
#define RX_NUM_MAX 80

//zax 20141116-----------------------------
/*enum boolean {false = 0, true = 1,};*/
#define boolean unsigned char
#define false 0
#define true  1
/*
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
*/
typedef int (*FTS_I2c_Read_Function)(unsigned char *, int , unsigned char *, int);
typedef int (*FTS_I2c_Write_Function)(unsigned char *, int);

int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read);
int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write);
int SetParamData(char *TestParamData);
void FreeTestParamData(void);
//zax 20141114+++++++++++++++++++++++++
void focal_save_scap_sample(int *databuf, int num);
//zax 20141116 ++++++++++++++
void focal_save_scap_sample1(void);
//zax 20141116 -----------------
boolean StartTestTP(void);		/*return true pass,else ng*/
boolean StartTestTP1(void);
//zax 20141114-----------------------------
//boolean GetData_RawDataTest(short **TestData, int *iTxNumber, int *iRxNumber, unsigned char FreqType, unsigned char FirState);/*FreqType: 0xff: default, 1:HighFreq, 0: LowFreq; FirState: 0xff: default, 1: On, 0: Off;*/

#endif
