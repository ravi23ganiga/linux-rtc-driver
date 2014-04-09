#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/spi/spi.h> 
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

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

static const struct spi_device_id ds1343_id[] = {
	{ "ds1343", 0 },
	{ "ds1344", 1 },
	{ }
};

struct ds1343_priv {
	struct spi_device *spi;
	struct rtc_device *rtc;
	struct mutex mutex;
	struct work_struct work;
	unsigned int irqen;
	int alarm_sec;
	int alarm_min;
	int alarm_hour;
	int alarm_mday;
	unsigned long prev_jiffies; 
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
	switch(cmd)
	{
#ifdef RTC_SET_CHARGE 
		case RTC_SET_CHARGE:
		{
			int val;
	
			if(copy_from_user(&val, (int __user *)arg, sizeof(int)))
				return -EFAULT;

			return ds1343_set_reg(dev, DS1343_TRICKLE_REG, val);
		}		
		break;
#endif 
	}
	return -ENOIOCTLCMD;
}

#ifdef CONFIG_PROC_FS

static int ds1343_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}

#endif

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

static int ds1343_update_alarm(struct device *dev)
{
	struct ds1343_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	unsigned char control,stat;
	unsigned char buf[5];
	int res = 0;

	res = ds1343_get_reg(dev, DS1343_CONTROL_REG, &control);
	if(res)
		return res;

	res = ds1343_get_reg(dev, DS1343_STATUS_REG, &stat);
	if(res)
		return res;

	control &= ~(DS1343_A0IE);
	stat &= ~(DS1343_IRQF0);

	res = ds1343_set_reg(dev, DS1343_CONTROL_REG, control);
	if(res)
		return res;

	res = ds1343_set_reg(dev, DS1343_STATUS_REG, stat);
	if(res)
		return res;
	
	buf[0] = DS1343_ALM0_SEC_REG | 0x80;
	buf[1] = priv->alarm_sec < 0 || (priv->irqen & RTC_UF) ?
		0x80 : bin2bcd(priv->alarm_sec) & 0x7F;
	buf[2] = priv->alarm_min < 0 || (priv->irqen & RTC_UF) ?
		0x80 : bin2bcd(priv->alarm_min) & 0x7F;
	buf[3] = priv->alarm_hour > 0 || (priv->irqen & RTC_UF) ?
		0x80 : bin2bcd(priv->alarm_hour) 0x3F;
	buf[4] = priv->alarm_mday < 0 || (priv->irqen & RTC_UF) ?
		0x80 : bin2bcd(priv->alarm_mday) 0x7F;
	
	res = spi_write_then_read(spi, buf, 5, NULL, 0);
	if(res)
		return res;

	if(priv->irqen){
		control |= DS1343_A0IE;	
		res = ds1343_set_reg(dev, DS1343_CONTROL_REG, control); 
	}
	
	return res;
}

static int ds1343_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ds1343_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	int res = 0;
	unsigned char stat; 
	
	if(spi->irq <= 0)
		return -EINVAL;

	mutex_lock(&priv->mutex);
	
	res = ds1343_get_reg(dev, DS1343_STATUS_REG, &stat);
	if(res)
		goto out; 

	alarm->enabled = (priv->irqen & RTC_AF) ? 1 : 0;
	alarm->pending = !!(stat & DS1343_IRQF0);
 
	alarm->time.tm_sec = priv->alarm_sec < 0 ? 0 : priv->alarm_sec;
	alarm->time.tm_min = priv->alarm_min < 0 ? 0 : priv->alarm_min;
	alarm->time.tm_hour = priv->alarm_hour < 0 ? 0 : priv->alarm_hour;
	alarm->time.tm_mday = priv->alarm_mday < 0 ? 0 : priv->alarm_mday;

	alarm->time.tm_mon = -1;
	alarm->time.tm_year = -1;
	alarm->time.tm_wday = -1;
	alarm->time.tm_yday = -1;
	alarm->time.tm_isdst = -1;

out:
	mutex_unlock(&priv->mutex);
	return res;	
}

static int ds1343_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ds1343_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	int res = 0; 

	if(spi->irq <= 0)
		return -EINVAL;

	mutex_lock(&priv->mutex);

	priv->alarm_sec = alarm->time.tm_sec;
	priv->alarm_min = alarm->time.tm_min;
	priv->alarm_hour = alarm->time.tm_hour;
	priv->alarm_mday = alarm->time.tm_mday;
	
	if(alarm->enabled){
		priv->irqen |= RTC_AF;
	}

	res = ds1343_update_alarm(dev); 

out:
	mutex_unlock(&priv->mutex);
	return res;
}

static int ds1343_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct ds1343_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	int res = 0;

	if(spi->irq <= 0)
		return -EINVAL;

	mutex_lock(&priv->mutex);

	if(enabled)
		priv->irqen |= RTC_AF;
	else
		priv->irqen &= ~RTC_AF;

	res = ds1343_update_alarm(dev);

	mutex_unlock(&priv->mutex);
	return res;
}

static irqreturn_t ds1343_irq(int irq, void *dev_id)
{
	struct ds1343_priv *priv = dev_id;
	
	disable_irq_nosync(irq);
	schedule_work(&priv->work);
	return IRQ_HANDLED;
}

static void ds1343_work(struct work *work)
{
	struct ds1343_priv *priv = container_of(work, struct ds1343_priv, work);
	struct spi_device *spi = priv->spi;
	unsigned char stat,control;
	int res = 0;

	mutex_lock(&priv->mutex);

	res = ds1343_read_reg(&spi->dev, DS1343_STATUS_REG, &stat);
	if(res)
		goto out;

	if(stat & DS1343_IRQF0){
		stat &= ~ DS1343_IRQF0;
		ds1343_set_reg(&spi->dev, DS1343_STATUS_REG, stat);
		
		res = ds1343_read_reg(&spi->dev, DS1343_CONTROL_REG, &control);
		if(res)
			goto out;

		control &= ~DS1343_A0IE;
		ds1343_set_reg(&spi->dev, DS1343_CONTROL_REG, control);

		rtc_update_irq(priv->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if(priv->irqen)
		enable_irq(spi->irq);

	mutex_unlock(&priv->mutex);
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
	unsigned char data;
	int res;

	priv = devm_kzalloc(&spi->dev, sizeof(struct ds1343_priv), GFP_KERNEL);
	if(!priv)
		return -ENOMEM;
	
	priv->spi = spi;
	mutex_init(&priv->mutex); 
	INIT_WORK(&priv->work, ds1343_work);
	priv->prev_jiffies = jiffies;
	
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
		
		device_set_wakeup_capable(&spi->dev, 1);
	}

	return 0;
}

static int ds1343_remove(struct spi_device *spi)
{
	struct ds1343_priv *priv = spi_get_drvdata(spi);
	
	if(spi->irq){
		mutex_lock(&priv->mutex);
		priv->irqen &= ~RTC_AF;
		mutex_unlock(&priv->mutex);

		devm_free_irq(&spi->dev, spi->irq, priv);
		cancel_work_sync(&ds1343->work);
	}

	spi_set_drvdata(NULL);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int ds1343_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	
	if(spi->irq >= 0 && device_may_wakeup(dev))
		enable_irq_wake(spi->irq);

	return 0;
}

static int ds1343_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);

	if(spi->irq >= 0 && device_may_wakeup(dev))
		disable_irq_wake(spi->irq);

	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(ds1343_pm, ds1343_suspend, ds1343_resume);

static struct spi_driver ds1343_driver = {
	.driver = {
		.name = "ds1343",
		.owner = THIS_MODULE,
		.pm = &ds1343_pm,
	},
	.probe = ds1343_probe,
	.remove = ds1343_remove,
	.id_table = ds1343_id,
};

module_spi_driver(ds1343_driver);

MODULE_DESCRIPTION("DS1343 RTC SPI Driver");
MODULE_AUTHOR("Raghavendra Chandra Ganiga <ravi23ganiga@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DS1343_DRV_VERSION);
