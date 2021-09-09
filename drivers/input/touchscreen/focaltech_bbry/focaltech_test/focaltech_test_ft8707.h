/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Test_FT8707.c
*
* Author: Software Development Team, AE
*
* Created: 2015-07-14
*
* Abstract: test item for FT8707
*
************************************************************************/
#ifndef _TEST_FT8707_H
#define _TEST_FT8707_H

#include "focaltech_test_main.h"

boolean FT8707_StartTest(void);
int FT8707_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

unsigned char FT8707_TestItem_RawDataTest(bool * bTestResult);
unsigned char FT8707_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FT8707_TestItem_NoiseTest(bool* bTestResult);
unsigned char FT8707_TestItem_CbTest(bool* bTestResult);
unsigned char FT8707_TestItem_EnterFactoryMode(void);



#endif
