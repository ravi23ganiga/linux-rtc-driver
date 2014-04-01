#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h> 
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/slab.h>

#define DS1343_DRV_VERSION	"01.00"

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

/* DS1343 Control Registers bits */
#define DS1343_EOSC		0x80
#define DS1343_INTCN		0x04
#define DS1343_A1IE		0x02
#define DS1343_A0IE		0x01

/* DS1343 Status Registers bits */
#define DS1343_OSF	0x80
#define DS1343_IRQF1	0x20
#define DS1343_IRQF0	0x01

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

static int ds1343_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int ds1343_read_time(struct device *dev, struct rtc_time *dt)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char buf[7];
	int res;

	buf[0] = DS1343_SECONDS_REG;

	res = spi_write_then_read(spi, buf, 1, buf, 7);
	if(res)
		return res;

	dt->tm_sec	= bcd2bin(buf[0]);
	dt->tm_min	= bcd2bin(buf[1]);
	dt->tm_hour	= bcd2bin(buf[2] & 0x3F);
	dt->tm_wday	= bcd2bin(buf[3]) - 1;
	dt->tm_mday	= bcd2bin(buf[4]);
	dt->tm_mon	= bcd2bin(buf[5] & 0x1F) - 1;
	dt->tm_year	= bcd2bin(buf[6]) + 100;
	
	return rtc_valid_tm(dt);
}

static int ds1343_set_time(struct device *dev, struct rtc_time *dt)
{
	int res;
	res = ds1343_set_reg(dev, DS1343_SECONDS_REG, bin2bcd(dt->tm_sec));
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_MINUTES_REG, bin2bcd(dt->tm_min));
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_HOURS_REG, bin2bcd(dt->tm_hour) & 0x3F);
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_DAY_REG, bin2bcd(dt->tm_wday + 1));
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_DATE_REG, bin2bcd(dt->tm_mday));
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_MONTH_REG, bin2bcd(dt->tm_mon + 1));
	if(res)
		return res;

	dt->tm_year %= 100;

	res = ds1343_set_reg(dev, DS1343_YEAR_REG, bin2bcd(dt->tm_year));
	if(res)
		return res;
	
	return 0;
}

static const struct rtc_class_ops ds1343_rtc_ops = {
	.ioctl		= ds1343_ioctl,
	.proc		= ds1343_proc,
	.read_time 	= ds1343_read_time,
	.set_time 	= ds1343_set_time,
	.read_alarm	= ds1343_read_alarm,
	.set_alarm	= ds1343_set_alarm,
	.alarm_irq_enable = ds1343_alarm_irq_enable,
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
	res = spi_setup(spi);
	if(res)
		return res;

	spi_set_drvdata(spi, priv);

	res = ds1343_get_reg(&spi->dev, DS1343_SECONDS_REG, &data);
	if(res)
		return res;

	ds1343_read_reg(&spi->dev, DS1343_CONTROL_REG, &data);
	data |= DS1343_INTCN;
	data &= ~(DS1343_EOSC | DS1343_A1IE | DS1343_A0IE);
	ds1343_set_reg(&spi->dev, DS1343_CONTROL_REG, data);

	ds1343_read_reg(&spi->dev, DS1343_STATUS_REG, &data);
	data &= ~(DS1343_OSF | DS1343_IRQF1 | DS1343_IRQF0);
	ds1343_set_reg(&spi->dev, DS1343_STATUS_REG, data);

	priv->rtc = devm_rtc_device_register(&spi->dev, "ds1343",
					&ds1343_rtc_ops, THIS_MODULE);
	if(IS_ERR(priv->rtc)) {
		dev_err(&spi->dev, "unable to register rtc ds1343\n");
		return PTR_ERR(priv->rtc);
	}

	if(spi->irq >= 0) {
		res = devm_request_irq(&spi->dev, spi->irq, ds1343_irq, 0,
				"ds1343", priv);
		if(res) {
			dev_err(&spi->dev, "unable to request irq for rtc ds1343\n");
			return res;
		}
	}

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
MODULE_VERSION(DS1343_DRV_VERSION);
MODULE_ALIAS("rtc:ds1343");
