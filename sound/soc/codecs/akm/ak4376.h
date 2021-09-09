/* Copyright (C) 2016 Tcl Corporation Limited */
/*
 * ak4376.h  --  audio driver for ak4376
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/06/24	   1.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _AK4376_H
#define _AK4376_H

//#define PLL_BICK_MODE

#define SRC_OUT_FS_48K			//Enable:48kHz system, Disable:44.1kHz system

#ifdef SRC_OUT_FS_48K
#define XTAL_OSC_FS 24576000	//48kHz System: 512fs
#else
#define XTAL_OSC_FS 22579200	//44.1kHz System: 512fs
#endif

#define AK4376_00_POWER_MANAGEMENT1			0x00
#define AK4376_01_POWER_MANAGEMENT2			0x01
#define AK4376_02_POWER_MANAGEMENT3			0x02
#define AK4376_03_POWER_MANAGEMENT4			0x03
#define AK4376_04_OUTPUT_MODE_SETTING		0x04
#define AK4376_05_CLOCK_MODE_SELECT			0x05
#define AK4376_06_DIGITAL_FILTER_SELECT		0x06
#define AK4376_07_DAC_MONO_MIXING			0x07
#define AK4376_08_JITTER_CLEANER_SETTING1	0x08
#define AK4376_09_JITTER_CLEANER_SETTING2	0x09
#define AK4376_0A_JITTER_CLEANER_SETTING3	0x0A
#define AK4376_0B_LCH_OUTPUT_VOLUME			0x0B
#define AK4376_0C_RCH_OUTPUT_VOLUME			0x0C
#define AK4376_0D_HP_VOLUME_CONTROL			0x0D
#define AK4376_0E_PLL_CLK_SOURCE_SELECT		0x0E
#define AK4376_0F_PLL_REF_CLK_DIVIDER1		0x0F
#define AK4376_10_PLL_REF_CLK_DIVIDER2		0x10
#define AK4376_11_PLL_FB_CLK_DIVIDER1		0x11
#define AK4376_12_PLL_FB_CLK_DIVIDER2		0x12
#define AK4376_13_SRC_CLK_SOURCE			0x13
#define AK4376_14_DAC_CLK_DIVIDER			0x14
#define AK4376_15_AUDIO_IF_FORMAT			0x15
#define AK4376_16_DUMMY						0x16
#define AK4376_17_DUMMY						0x17
#define AK4376_18_DUMMY						0x18
#define AK4376_19_DUMMY						0x19
#define AK4376_1A_DUMMY						0x1A
#define AK4376_1B_DUMMY						0x1B
#define AK4376_1C_DUMMY						0x1C
#define AK4376_1D_DUMMY						0x1D
#define AK4376_1E_DUMMY						0x1E
#define AK4376_1F_DUMMY						0x1F
#define AK4376_20_DUMMY						0x20
#define AK4376_21_DUMMY						0x21
#define AK4376_22_DUMMY						0x22
#define AK4376_23_DUMMY						0x23
#define AK4376_24_MODE_CONTROL				0x24
/* MODIFIED-BEGIN by hongwei.tian, 2018-01-04,BUG-5854780*/
#define AK4376_25_DUMMY						0x25
#define AK4376_26_DUMMY						0x26
#define AK4376_27_DUMMY						0x27
#define AK4376_28_DUMMY						0x28
#define AK4376_29_DUMMY						0x29
#define AK4376_2a_DUMMY						0x2a

#define AK4376_MAX_REGISTERS	(AK4376_2a_DUMMY + 1)
/* MODIFIED-END by hongwei.tian,BUG-5854780*/

/* Bitfield Definitions */

/* AK4376_15_AUDIO_IF_FORMAT (0x15) Fields */
#define AK4376_DIF					0x04
#define AK4376_DIF_I2S_MODE		(0 << 2)
#define AK4376_DIF_MSB_MODE		(1 << 2)

/* AK4376_05_CLOCK_MODE_SELECT (0x05) Fields
   AK4376_08_JITTER_CLEANER_SETTING1 (0x08) Fields */
#define AK4376_FS				0x1F
#define AK4376_FS_8KHZ			(0x00 << 0)
#define AK4376_FS_11_025KHZ		(0x01 << 0)
#define AK4376_FS_16KHZ			(0x04 << 0)
#define AK4376_FS_22_05KHZ		(0x05 << 0)
#define AK4376_FS_32KHZ			(0x08 << 0)
#define AK4376_FS_44_1KHZ		(0x09 << 0)
#define AK4376_FS_48KHZ			(0x0A << 0)
#define AK4376_FS_88_2KHZ		(0x0D << 0)
#define AK4376_FS_96KHZ			(0x0E << 0)
#define AK4376_FS_176_4KHZ		(0x11 << 0)
#define AK4376_FS_192KHZ		(0x12 << 0)

#define AK4376_CM		(0x03 << 5)
#define AK4376_CM_0		(0 << 5)
#define AK4376_CM_1		(1 << 5)
#define AK4376_CM_2		(2 << 5)
#define AK4376_CM_3		(3 << 5)

/*Defined Sentence for Timer */
#define LVDTM_HOLD_TIME		30	// (msec)
#define VDDTM_HOLD_TIME		500	// (msec)
#define HPTM_HOLD_TIME		15	// (msec)

/*Defined Sentence for Soft Mute Cycle(ms)*/
#define SMUTE_TIME_MODE  0    // 0:22msec 1:43msec

#endif
