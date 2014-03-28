#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h> 
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/slab.h>

/* RTC DS1343 Registers */
#define DS1343_SECONDS_REG	0x00
#define DS1343_MINUTES_REG	0x01
#define DS1343_HOURS_REG	0x02
#define DS1343_DAY_REG		0x03
#define DS1343_DATE_REG		0x04
#define DS1343_MONTH_REG	0x05
#define DS1343_YEAR_REG		0x06
#define DS1343_ALM0_SEC_REG	0x07
#define DS1343_ALM0_MIN_REG	0x08
#define DS1343_ALM0_HOUR_REG	0x09
#define DS1343_ALM0_DAY_REG	0x0A
#define DS1343_ALM1_SEC_REG	0x0B
#define DS1343_ALM1_MIN_REG	0x0C
#define DS1343_ALM1_HOUR_REG	0x0D
#define DS1343_ALM1_DAY_REG	0x0E
#define DS1343_CONTROL_REG	0x0F
#define DS1343_STATUS_REG	0x10
#define DS1343_TRICKLE_REG	0x11

struct ds1343_priv {
	struct spi_device *spi;
	struct rtc_device *rtc;
	struct mutex;
};

static int ds1343_get_reg(struct device *dev, unsigned char address,
				unsigned char *buf)
{
	struct spi_device *spi = to_spi_device();

	*buf = address;

	return spi_write_then_read(spi, buf, 1, buf, 1);
}

static int ds1343_set_reg(struct device *dev, unsigned char address,
				unsigned char data)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[2];
	
	buf[0] = address | 0x80;
	buf[1] = data;
	
	return spi_write_then_read(spi, buf, 2, NULL, 0);
}

static const struct rtc_class_ops ds1343_rtc_ops = {
	.read_time = ds1343_read_time,
	.set_time = ds1343_set_time,
};

static int ds1343_probe(struct spi_device *spi)
{
	struct ds1343_priv *priv;
	int data;
	int res;

	priv = devm_kzalloc(&spi->dev, sizeof(struct ds1343_priv), GFP_KERNEL);
	if(!priv)
		return -ENOMEM;
	
	priv->spi = spi;
	mutex_init(&priv->mutex);
	
	/* RTC DS1347 works in spi mode 3 and
	 * its chip select is active high
	 */
	spi->mode = SPI_MODE_3 | SPI_CS_HIGH;
	spi->bits_per_word = 8;
	spi_setup(spi);
	spi_set_drvdata(spi, priv);

	res = ds1343_get_reg(&spi->dev, DS1343_SECONDS_REG, &data);
	if(res)
		return res;
	
	

	return 0;
}

static struct spi_driver ds1343_driver = {
	.driver = {
		.name = "ds1343",
		.owner = THIS_MODULE,
	},
	.probe = ds1343_probe,
};

module_spi_driver(ds1343_driver);

MODULE_DESCRIPTION("DS1343 RTC SPI Driver");
MODULE_AUTHOR("Raghavendra Chandra Ganiga <ravi23ganiga@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("rtc:ds1343");
