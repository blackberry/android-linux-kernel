/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /************************************************************************
*
* File Name: focaltech_apk_node.c
*
* Author:	  Xu YF & ZR,  Software Department, FocalTech
*
* Created: 2016-04-08
*   
* Modify:
*
* Abstract: i2c communication with TP
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include "focaltech_comm.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_I2C_COMMUNICATION_INFO  "File Version of  focaltech_i2c_communication.c:  V1.0.0 2016-04-08"

/*i2c flag for write*/
#define I2C_M_WRITE    0

/*dma declare, allocate and release*/
#define FTS_DMA_BUFF_SIZE     128
#define FTS_DMA_EN      0//0:disable, 1:enable 




/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
/*如果要定义一个静态mutex型变量，应该使用DEFINE_MUTEX*/
static DEFINE_MUTEX(i2c_rw_access);

#if FTS_DMA_EN
static u8 *g_dma_buff_va = NULL;
static u8 *g_dma_buff_pa = NULL;
#endif

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static void msg_dma_alloct(void);
static void msg_dma_release(void);
/*******************************************************************************
* functions body
*******************************************************************************/

/*******************************************************************************
*  Name: fts_i2c_read
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_read_universal(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);

	if(readlen > 0)
	{
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = I2C_M_WRITE,
					 .len = writelen,
					 .buf = writebuf,
				 },
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret < 0)
				FTS_COMMON_DBG("i2c read error.");
		} else {
			struct i2c_msg msgs[] = {
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = readlen,
					 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0)
				FTS_COMMON_DBG("i2c read error.");
		}
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

/*******************************************************************************
*  Name: fts_i2c_write
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_write_universal(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = I2C_M_WRITE,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	mutex_lock(&i2c_rw_access);

	if(writelen > 0)
	{
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			FTS_COMMON_DBG("i2c write error.");
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}


/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read_dma(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;

#if FTS_DMA_EN
	// for DMA I2c transfer
	
	mutex_lock(&i2c_rw_access);
	
	if(writelen!=0)
	{
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
			printk("i2c write failed\n");
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}

	//DMA Read 

	if(readlen!=0)

	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	
	mutex_unlock(&i2c_rw_access);
#endif

	return ret;

}

/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write_dma(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;
	
#if FTS_DMA_EN
	mutex_lock(&i2c_rw_access);
	
 	//client->addr = client->addr & I2C_MASK_FLAG;

	//ret = i2c_master_send(client, writebuf, writelen);
	memcpy(g_dma_buff_va, writebuf, writelen);
	
	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
		printk("i2c write failed\n");
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
		
	mutex_unlock(&i2c_rw_access);
#endif

	return ret;

}



static void msg_dma_alloct(void)
{
#if FTS_DMA_EN
	g_dma_buff_va = (u8 *)dma_alloc_coherent(NULL, FTS_DMA_BUFF_SIZE, &g_dma_buff_pa, GFP_KERNEL);//DMA size 4096 for customer
	if(!g_dma_buff_va)
	{
	        FTS_COMMON_DBG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
	}
	else
	{
		FTS_COMMON_DBG("[DMA] Allocate DMA I2C Buffer succeeded!\n");
	}
#endif
	return;
}
static void msg_dma_release(void)
{
#if FTS_DMA_EN
	if(g_dma_buff_va)
	{
	     	dma_free_coherent(NULL, FTS_DMA_BUFF_SIZE, g_dma_buff_va, g_dma_buff_pa);
	        g_dma_buff_va = NULL;
	        g_dma_buff_pa = NULL;
		FTS_COMMON_DBG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	 }
#endif

	return;
}

int fts_i2c_communication_init(void)
{	
	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_I2C_COMMUNICATION_INFO);	//show version
	FTS_COMMON_DBG("");//default print: current function name and line number
	msg_dma_alloct();
	
	return 0;
}
/************************************************************************
* Name: fts_sysfs_exit
* Brief:  remove sysfs
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_i2c_communication_exit(void)
{
	FTS_COMMON_DBG("");//default print: current function name and line number
	msg_dma_release();	
	
	return 0;
}

