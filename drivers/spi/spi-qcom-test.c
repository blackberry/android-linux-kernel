#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#include <linux/sysfs.h>

#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id qcom_spi_test_table[] = {
	{ .compatible = "qcom,spi-test",}, //Compatible node must match dts
	{ },
};
#else
#define qcom_spi_test_table NULL
#endif

u8 fpc_id_read_cmd[] = {0xFC, 00, 00};
//u8 fpc_id_read_cmd[] = {0xFC, 00, 00};
#define BUFFER_SIZE 4<<10
struct spi_message spi_msg;
struct spi_transfer spi_xfer[5];
u8 *tx_buf; //This needs to be DMA friendly buffer
u8 *rx_buf; //This needs to be DMA friendly buffer



static int spi_test_transfer(struct spi_device *spi)
{
	dev_err(&spi->dev, "%s:Entry \n", __func__);

	spi_message_init(&spi_msg);

	tx_buf[0] = 0xfc;
	spi_xfer[0].tx_buf = tx_buf;
	spi_xfer[0].rx_buf = rx_buf;
	spi_xfer[0].len = 1;
	spi_xfer[0].bits_per_word = 8;
	spi_xfer[0].speed_hz = spi->max_speed_hz;

	tx_buf[1] = 0x00;
	tx_buf[2] = 0x00;
	spi_xfer[1].tx_buf = tx_buf + 1;
	spi_xfer[1].rx_buf = rx_buf + 1;
	spi_xfer[1].len = 2;
	spi_xfer[1].bits_per_word = 8;
	spi_xfer[1].speed_hz = spi->max_speed_hz;

	tx_buf[3] = 0xfc;
	tx_buf[4] = 0x00;
	tx_buf[5] = 0x00;
	spi_xfer[2].tx_buf = tx_buf + 3;
	spi_xfer[2].rx_buf = rx_buf + 3;
	spi_xfer[2].len = 3;
	spi_xfer[2].bits_per_word = 8;
	spi_xfer[2].speed_hz = spi->max_speed_hz;

	tx_buf[6] = 0xfc;
	tx_buf[7] = 0x00;
	tx_buf[8] = 0x00;
	spi_xfer[3].tx_buf = tx_buf + 6 ;
	spi_xfer[3].rx_buf = rx_buf + 6 ;
	spi_xfer[3].len = 3;
	spi_xfer[3].bits_per_word = 8;
	spi_xfer[3].speed_hz = spi->max_speed_hz;

	spi_message_add_tail(&spi_xfer[0], &spi_msg);
	spi_message_add_tail(&spi_xfer[1], &spi_msg);
	spi_message_add_tail(&spi_xfer[2], &spi_msg);
	spi_message_add_tail(&spi_xfer[3], &spi_msg);
	return spi_sync(spi, &spi_msg);
}

struct spi_device *org_spi = NULL;

static ssize_t spi_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
//	struct spi_device *spi = dev_get_drvdata(dev);
	struct spi_device *spi = container_of(
			dev, struct spi_device, dev);

	int  retval;
	int  idx;
	u8   *tmp_buf;

	pr_err("dev=%p, spi=%p org_spi=%p, org spi->dev=%p\n",
		dev, spi, org_spi, &org_spi->dev);
	retval = spi_test_transfer(org_spi);
	dev_err(&org_spi->dev, "SPI sync returned rc=%d\n", retval);

	for (idx = 0; idx < 4; idx++) {
		tmp_buf = (u8 *)(spi_xfer[idx].rx_buf);
		dev_err(&spi->dev, "xfer=%d, len=%d, data:[0x%02x, 0x%02x, 0x%02x]",
			idx, spi_xfer[idx].len, tmp_buf[0], tmp_buf[1], tmp_buf[2]);
	};
	return snprintf(buf, PAGE_SIZE, "rc=%d, rx_data=0x%02x 0x%02x 0x%02x\n",
		retval, rx_buf[0], rx_buf[1], rx_buf[2]);
}

static inline ssize_t spi_test_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	dev_warn(dev, "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static struct device_attribute attrs[] = {
	__ATTR(spi_test, (S_IRUSR | S_IRGRP),
			spi_test_show,
			spi_test_store_error)
};

static int spi_test_probe(struct spi_device *spi)
{
	int cs;
	int cpha,cpol,cs_high;
	u32 max_speed;
	int attr_count;
	int retval;

	dev_err(&spi->dev, "%s\n", __func__);

	org_spi = spi;

	//allocate memory for transfer
	tx_buf = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
	if(tx_buf == NULL){
		dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
		return -ENOMEM;
	}
	rx_buf = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
	if(rx_buf == NULL){
		dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
		return -ENOMEM;
	}
	cs = spi->chip_select;
	cpha = ( spi->mode & SPI_CPHA ) ? 1:0;
	cpol = ( spi->mode & SPI_CPOL ) ? 1:0;
	cs_high = ( spi->mode & SPI_CS_HIGH ) ? 1:0;
	max_speed = spi->max_speed_hz;
	dev_err(&spi->dev, "cs [%x] CPHA [%x] CPOL [%x] CS_HIGH [%x]\n",
		cs, cpha, cpol, cs_high);
	dev_err(&spi->dev, "Max_speed [%d]\n", max_speed );

	//Once you have a spi_device structure you can do a transfer anytime
	spi->bits_per_word = 8;
	dev_err(&spi->dev, "SPI sync returned [%d]\n",
	retval = spi_test_transfer(spi));

	dev_err(&spi->dev, "SPI sync returned rc=%d, rx=[0x%02x, 0x%02x, 0x%02x]\n", 
		retval, rx_buf[0], rx_buf[1], rx_buf[2]);
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = device_create_file(&spi->dev,
				&attrs[attr_count]);
		if (retval < 0) {
			dev_err(&spi->dev,
				"%s: Failed to create sysfs attributes\n",
				__func__);
			break;
		}
	}

	return 0;
}

//SPI Driver Info
static struct spi_driver spi_test_driver = {
	.driver = {
		.name = "qcom_spi_test",
		.owner = THIS_MODULE,
		.of_match_table = qcom_spi_test_table,
	},
	.probe = spi_test_probe,
};

static int __init spi_test_init(void)
{
	pr_err("spi_test_init\n");
	return spi_register_driver(&spi_test_driver);
}

static void __exit spi_test_exit(void)
{
	spi_unregister_driver(&spi_test_driver);
}

module_init(spi_test_init);
module_exit(spi_test_exit);
MODULE_DESCRIPTION("SPI TEST");
MODULE_LICENSE("GPL v2");
