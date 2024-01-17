/*
*	test function for fw team
*
*/
#include "focaltech_comm.h"
#include<linux/random.h>

int ErrorCount=0;
static unsigned int test_count=0;
int Save_LOG_TXT(void)
{
	int count = 0;
	//int err = 0;
	//int i = 0, j = 0;	
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	loff_t pos;
	mm_segment_t old_fs;
	char *databuf = NULL;
	ssize_t err1;
	
	mutex_lock(&fts_input_dev->mutex);

	databuf = kmalloc(10 * 1024, GFP_ATOMIC);
	if (databuf == NULL) {
		FTS_COMMON_DBG("alloc data buf fail!\n");
		return -1;
	}
	memset(databuf, 0, sizeof(databuf));	
	//sprintf(databuf, "Project Code: %s\n", g_projectcode);	
	if (NULL == pfile)
		pfile = filp_open("/mnt/sdcard/output_error.txt", O_WRONLY|O_CREAT|O_TRUNC, 0777);

	if (IS_ERR(pfile)) {
		FTS_COMMON_DBG("error occured while opening file %s.\n", "");
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	pos = 0;		
	count=0;					
	count += sprintf(databuf + count,"NG ErrorCount= %d,",ErrorCount);
			
	count += sprintf(databuf + count,"\n");
	
	err1 = vfs_write(pfile, databuf, count, &pos);
	if (err1 < 0)
		FTS_COMMON_DBG("write scap sample fail!\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	kfree(databuf);	
	mutex_unlock(&fts_input_dev->mutex);
	FTS_COMMON_DBG("[Focal][%s]	finish!\n", __func__);
	return 1;
}

ssize_t fts_tptesti2cloop_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/*read register for loop 

*write example:echo 8801 > ftstptptesti2c ---read/write register 0x88 for 10000 count
*
*note:the number of input must be 4
*/
ssize_t fts_tptesti2cloop_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);	
	ssize_t num_read_chars = 0;
	int retval=0,i=0;
	
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 loop_count=0;
	u8 valbuf[5]={0};
	ErrorCount=0;
	memset(valbuf, 0, sizeof(valbuf));

	mutex_lock(&fts_input_dev->mutex);
	num_read_chars = count - 1;
	//FTS_COMMON_DBG("reg1 fail %d\n",num_read_chars);
	if(num_read_chars!=4)
	{
		
		//dev_err(dev, "please input 7 character\n");
		//ErrorCount=ErrorCount+1;
		goto error_return;
		
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval)
	{
		FTS_COMMON_DBG("reg fail\n");
		//dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		//ErrorCount=ErrorCount+1;
		goto error_return;
	}
	
		//read register
		regaddr = wmreg>>8;
		loop_count=wmreg;

		//FTS_COMMON_DBG("reg test %d	%d\n",regaddr, loop_count);
		for(i=0;i<(loop_count*10000);i++)	
		{
			if(fts_read_reg(client, regaddr, &regvalue) < 0)
			{
				FTS_COMMON_DBG("reg fail\n");
				ErrorCount=ErrorCount+1;
				//dev_err(dev, "Could not read the register(0x%02x) , error count= %d\n", regaddr,ErrorCount);
				break;
			}
			else
			{
				//FTS_COMMON_DBG("reg success %d   %d\n",regaddr,regvalue);
				//dev_dbg(dev, "the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
				regvalue++;
				if(regvalue==256)
				{
					regvalue=0;
				}
				if(fts_write_reg(client, regaddr, regvalue)<0)
				{
					FTS_COMMON_DBG("reg fail\n");
					ErrorCount=ErrorCount+1;
					//dev_err(dev, "Could not write the register(0x%02x)\n", regaddr);
				}
				else
				{
					//FTS_COMMON_DBG("reg success\n");
					//dev_dbg(dev, "Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
				}
			}
		}
		
		FTS_COMMON_DBG("stop loop= %d, error= %d   \n",(loop_count*10000),ErrorCount);
error_return:
	mutex_unlock(&fts_input_dev->mutex);
	Save_LOG_TXT();
	return count;
}


ssize_t fts_writerandsleep_show(struct device *dev,
struct device_attribute *attr, char *buf)
{	
	/* place holder for future use */
	int ii=0;
   	//time_t t;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);	
   	//srandom32((unsigned) time(&t));
  
	//ii=(int)(random32()%140+10);
	srandom32((uint)jiffies);	
	for(;;)
	{
		ii = (int)(random32()%140+10);
		
		fts_write_reg(client, 0xd0, 0x01); 

		//if(test_zax==1)
		//	break;
		msleep(1000);
		//fts_ctpm_hw_reset();
		msleep(ii);
		fts_write_reg(client, 0xd0, 0x00); 
		//if(test_zax==1)
		//	break;
		msleep(2000);
		FTS_COMMON_DBG("count %d\n",test_count++);
		FTS_COMMON_DBG("time %d\n",ii);

		

		
	}
	//Save_LOG_TXT();
	FTS_COMMON_DBG("I2C error for loop stop\n");
	return -EPERM;
}

ssize_t fts_writerandsleep_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	return count;
}




