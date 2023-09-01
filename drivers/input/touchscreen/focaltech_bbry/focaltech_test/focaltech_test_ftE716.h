/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: Test_FTE716.c
*
* Author: Software Development Team, AE
*
* Created: 2016-05-19
*
* Abstract: test item for FTE716
*
************************************************************************/
#ifndef _TEST_FTE716_H
#define _TEST_FTE716_H

#include "focaltech_test_main.h"

boolean FTE716_StartTest(void);
int FTE716_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

unsigned char FTE716_TestItem_RawDataTest(bool * bTestResult);
unsigned char FTE716_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FTE716_TestItem_NoiseTest(bool* bTestResult);
unsigned char FTE716_TestItem_CbTest(bool* bTestResult);
unsigned char FTE716_TestItem_EnterFactoryMode(void);



#endif
