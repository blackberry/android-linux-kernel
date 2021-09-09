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
#ifndef _TEST_FT8607_H
#define _TEST_FT8607_H

#include "focaltech_test_main.h"
#define		MAX_DROP_FRAMES_FT8607		1


boolean FT8607_StartTest(void);
int FT8607_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
/*
unsigned char FT8607_TestItem_RawDataTest(bool * bTestResult);
unsigned char FT8607_TestItem_ChannelsTest(bool * bTestResult);
unsigned char FT8607_TestItem_ShortCircuitTest(bool* bTestResult);
unsigned char FT8607_TestItem_LCDNoiseTest(bool* bTestResult);

unsigned char FT8607_TestItem_CbTest(bool * bTestResult);
unsigned char FT8607_TestItem_EnterFactoryMode(void);
*/	
#endif
