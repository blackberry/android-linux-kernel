/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Test_FTE736.c
*
* Author: Software Development Team, AE
*
* Created: 2016-05-19
*
* Abstract: test item for FTE736
*
************************************************************************/
#ifndef _TEST_FTE736_H
#define _TEST_FTE736_H

#include "focaltech_test_main.h"

boolean FTE736_StartTest(void);
int FTE736_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

unsigned char FTE736_TestItem_RawDataTest(bool * bTestResult);
unsigned char FTE736_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FTE736_TestItem_NoiseTest(bool* bTestResult);
unsigned char FTE736_TestItem_CbTest(bool* bTestResult);
unsigned char FTE736_TestItem_EnterFactoryMode(void);



#endif
