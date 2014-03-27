#include <linux/init.h>
#include <linux/spi.h>
#include <linux/module.h>
#include <linux/rtc.h>

static const struct rtc_class_ops ds1343_rtc_ops = {
	.read_time = ds1343_read_time,
	.set_time = ds1343_set_time,
};

static int ds1343_probe(struct spi_device *spi)
{

}

static struct spi_driver ds1343_driver = {
	.driver = {
		.name = "ds1343",
		.owner = THIS_MODULE,
	},
	.probe = ds1343_probe,
};

module_spi_driver(ds1343_driver);
