/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: Test_FT8606.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT8606
*
************************************************************************/
#ifndef _TEST_FT8606_H
#define _TEST_FT8606_H

#include "focaltech_test_main.h"

boolean FT8606_StartTest(void);
int FT8606_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

unsigned char FT8606_TestItem_RawDataTest(bool * bTestResult);
unsigned char FT8606_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FT8606_TestItem_NoiseTest(bool* bTestResult);
unsigned char FT8606_TestItem_CbTest(bool* bTestResult);
unsigned char FT8606_TestItem_EnterFactoryMode(void);



#endif
