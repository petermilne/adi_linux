// SPDX-License-Identifier: GPL-2.0
/*
 * AD7191 ADC driver
 *
 * Copyright 2011-2015 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/ad_sigma_delta.h>

// TO BE USED
/* Mode Register: AD7191_MODE_CLKSRC options */
#define AD7191_CLK_EXT_MCLK1_2		0 /* External 4.92 MHz Clock connected*/
					  /* from MCLK1 to MCLK2 */
#define AD7191_CLK_EXT_MCLK2		1 /* External Clock applied to MCLK2 */
#define AD7191_CLK_INT			2 /* Internal 4.92 MHz Clock not */
					  /* available at the MCLK2 pin */
#define AD7191_CLK_INT_CO		3 /* Internal 4.92 MHz Clock available*/
					  /* at the MCLK2 pin */

/* ID Register Bit Designations (AD7191_REG_ID) */
#define CHIPID_AD7191		0x0
#define AD7191_ID_MASK		GENMASK(3, 0)

#define AD7191_EXT_FREQ_MHZ_MIN	2457600
#define AD7191_EXT_FREQ_MHZ_MAX	5120000
#define AD7191_INT_FREQ_MHZ	4915200

#define AD7191_CH_AIN1_AIN2		0
#define AD7191_CH_AIN3_AIN4		1
#define AD7191_CH_TEMP			2

/* NOTE:
 * The AD7190/2/5 features a dual use data out ready DOUT/RDY output.
 * In order to avoid contentions on the SPI bus, it's therefore necessary
 * to use spi bus locking.
 *
 * The DOUT/RDY output must also be wired to an interrupt capable GPIO.
 */

enum {
	ID_AD7191,
};

struct ad7191_chip_info {
	unsigned int			chip_id;
	const char			*name;
	const struct iio_chan_spec	*channels;
	u8				num_channels;
	const struct iio_info		*info;
};

struct ad7191_state {
	const struct ad7191_chip_info	*chip_info;
	struct regulator		*avdd;
	struct regulator		*vref;
	u32				fclk;
	struct mutex			lock;	/* protect sensor state */

	struct ad_sigma_delta		sd;

	struct gpio_desc			*odr1_gpio;
	struct gpio_desc			*odr2_gpio;
	struct gpio_desc			*pga1_gpio;
	struct gpio_desc			*pga2_gpio;
	struct gpio_desc			*temp_gpio;
	struct gpio_desc			*chan_gpio;
	struct gpio_desc			*clksel_gpio;

	u16							int_vref_mv;
	u8							gain_index;
	u32							scale_avail[4][2];
	u8							samp_freq_index;
	u32							samp_freq_avail[4];
};

static struct ad7191_state *ad_sigma_delta_to_ad7191(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct ad7191_state, sd);
}

static int ad7191_set_channel(struct ad_sigma_delta *sd, unsigned int channel)
{
	struct ad7191_state *st = ad_sigma_delta_to_ad7191(sd);
	u8 temp_gpio_val, chan_gpio_val;

	dev_info(&sd->spi->dev, "setting channel %d\n", channel);

	chan_gpio_val = 0x1 & channel;
	temp_gpio_val = (0x2 & channel) >> 1;

	dev_info(&st->sd.spi->dev, "Setting chan gpio to %d\n", chan_gpio_val);
	dev_info(&st->sd.spi->dev, "Setting temp gpio to %d\n", temp_gpio_val);

	gpiod_set_value(st->chan_gpio, chan_gpio_val);
	gpiod_set_value(st->temp_gpio, temp_gpio_val);

	return 0;
}

static int ad7191_set_mode(struct ad_sigma_delta *sd, enum ad_sigma_delta_mode mode)
{
	// struct ad7191_state *st = ad_sigma_delta_to_ad7191(sd);

	dev_info(&sd->spi->dev, "setting mode %d\n", mode);

	// to set mode ??

	return 0;
}

static int ad7191_postprocess_sample(struct ad_sigma_delta *sd, enum ad_sigma_delta_mode mode)
{
	// struct ad7191_state *st = ad_sigma_delta_to_ad7191(sd);

	dev_info(&sd->spi->dev, "postprocess sample, mode = %d\n", mode);

	// to postprocess sample ??

	return 0;
}

static const struct ad_sigma_delta_info ad7191_sigma_delta_info = {
	.set_channel = ad7191_set_channel,
	.postprocess_sample = ad7191_postprocess_sample,
	.set_mode = ad7191_set_mode,
	.has_registers = false,
	.irq_flags = IRQF_TRIGGER_FALLING,
};

static inline bool ad7191_valid_external_frequency(u32 freq)
{
	return (freq >= AD7191_EXT_FREQ_MHZ_MIN &&
		freq <= AD7191_EXT_FREQ_MHZ_MAX);
}

static int ad7191_setup(struct iio_dev *indio_dev, struct device *dev)
{
	struct ad7191_state *st = iio_priv(indio_dev);
	u64 scale_uv;
	int gain[4] = {1, 8, 64, 128};
	int i;

	st->samp_freq_avail[0] = 120;
	st->samp_freq_avail[1] = 60;
	st->samp_freq_avail[2] = 50;
	st->samp_freq_avail[3] = 10;

	for (i = 0; i < ARRAY_SIZE(st->scale_avail); i++) {
		scale_uv = ((u64)st->int_vref_mv * 100000000) >>
				   indio_dev->channels[0].scan_type.realbits;
		do_div(scale_uv, gain[i]);

		st->scale_avail[i][1] = do_div(scale_uv, 100000000) * 10;
		st->scale_avail[i][0] = scale_uv;
	}

	return 0;
}

static unsigned int ad7191_get_temp_scale(bool unipolar)
{
	return unipolar ? 2815 * 2 : 2815;
}

static int ad7191_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{ 
	struct ad7191_state *st = iio_priv(indio_dev);
	bool unipolar = true; // no bipolar mode

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return ad_sigma_delta_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			mutex_lock(&st->lock);
			*val = st->scale_avail[st->gain_index][0];
			*val2 = st->scale_avail[st->gain_index][1];
			mutex_unlock(&st->lock);
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			*val = 0;
			*val2 = 1000000000 / ad7191_get_temp_scale(unipolar);
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		if (!unipolar)
			*val = -(1 << (chan->scan_type.realbits - 1));
		else
			*val = 0;
		/* Kelvin to Celsius */
		if (chan->type == IIO_TEMP)
			*val -= 273 * ad7191_get_temp_scale(unipolar);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		// *val = DIV_ROUND_CLOSEST(ad7191_get_f_adc(st), 1024);
		*val = st->samp_freq_avail[st->samp_freq_index];
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad7191_set_gain(struct ad7191_state *st, u8 gain_index)
{
	u8 pga1_gpio_val, pga2_gpio_val;
	st->gain_index = gain_index;

	// pga2, pga1
	// 0, 0 => 1
	// 0, 1 => 8
	// 1, 0 => 64
	// 1, 1 => 128

	pga1_gpio_val = 0x1 & gain_index;
	pga2_gpio_val = (0x2 & gain_index) >> 1;

	dev_info(&st->sd.spi->dev, "Setting pga1 to %d\n", pga1_gpio_val);
	dev_info(&st->sd.spi->dev, "Setting pga2 to %d\n", pga2_gpio_val);

	gpiod_set_value(st->pga1_gpio, pga1_gpio_val);
	gpiod_set_value(st->pga2_gpio, pga2_gpio_val);

	return 0;
}

static int ad7191_set_samp_freq(struct ad7191_state *st, u8 samp_freq_index)
{
	u8 odr1_gpio_val, odr2_gpio_val;
	st->samp_freq_index = samp_freq_index;
	
	// odr2, odr1
	// 0, 0 => 120
	// 0, 1 => 60
	// 1, 0 => 50
	// 1, 1 => 10

	odr1_gpio_val = 0x1 & samp_freq_index;
	odr2_gpio_val = (0x2 & samp_freq_index) >> 1;

	dev_info(&st->sd.spi->dev, "Setting odr1 to %d\n", odr1_gpio_val);
	dev_info(&st->sd.spi->dev, "Setting odr2 to %d\n", odr2_gpio_val);

	gpiod_set_value(st->odr1_gpio, odr1_gpio_val);
	gpiod_set_value(st->odr2_gpio, odr2_gpio_val);

	return 0;
}

static int ad7191_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	struct ad7191_state *st = iio_priv(indio_dev);
	int ret, i;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = -EINVAL;
		mutex_lock(&st->lock);
		for (i = 0; i < ARRAY_SIZE(st->scale_avail); i++)
			if (val2 == st->scale_avail[i][1]) {
				ret = ad7191_set_gain(st, i);
				break;
			}
		mutex_unlock(&st->lock);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!val) {
			ret = -EINVAL;
			break;
		}
		mutex_lock(&st->lock);
		for (i = 0; i < ARRAY_SIZE(st->samp_freq_avail); i++)
			if (val == st->samp_freq_avail[i]) {
				ret = ad7191_set_samp_freq(st, i);
				break;
			}
		mutex_unlock(&st->lock);
		break;
	default:
		ret = -EINVAL;
	}

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad7191_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7191_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *length, long mask)
{
	struct ad7191_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (int *)st->scale_avail;
		*type = IIO_VAL_INT_PLUS_NANO;
		*length = ARRAY_SIZE(st->scale_avail) * 2;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (int *)st->samp_freq_avail;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(st->samp_freq_avail);

		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}

static const struct iio_info ad7191_info = {
	.read_raw = ad7191_read_raw,
	.write_raw = ad7191_write_raw,
	.write_raw_get_fmt = ad7191_write_raw_get_fmt,
	.read_avail = ad7191_read_avail,
	.validate_trigger = ad_sd_validate_trigger,
};

#define __AD719x_CHANNEL(_si, _channel1, _channel2, _address, _type, \
	_mask_all, _mask_type_av, _mask_all_av) \
	{ \
		.type = (_type), \
		.differential = ((_channel2) == -1 ? 0 : 1), \
		.indexed = 1, \
		.channel = (_channel1), \
		.channel2 = (_channel2), \
		.address = (_address), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_OFFSET), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			(_mask_all), \
		.info_mask_shared_by_type_available = (_mask_type_av), \
		.info_mask_shared_by_all_available = (_mask_all_av), \
		.scan_index = (_si), \
		.scan_type = { \
			.sign = 'u', \
			.realbits = 24, \
			.storagebits = 24, \
			.endianness = IIO_BE, \
		}, \
	}

#define AD719x_DIFF_CHANNEL(_si, _channel1, _channel2, _address) \
	__AD719x_CHANNEL(_si, _channel1, _channel2, _address, IIO_VOLTAGE, 0, \
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_SAMP_FREQ), 0)

#define AD719x_TEMP_CHANNEL(_si, _address) \
	__AD719x_CHANNEL(_si, 0, -1, _address, IIO_TEMP, 0, 0, 0)

static const struct iio_chan_spec ad7191_channels[] = {
	AD719x_TEMP_CHANNEL(0, AD7191_CH_TEMP),
	AD719x_DIFF_CHANNEL(1, 1, 2, AD7191_CH_AIN1_AIN2),
	AD719x_DIFF_CHANNEL(2, 3, 4, AD7191_CH_AIN3_AIN4),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct ad7191_chip_info ad7191_chip_info_tbl[] = {
	[ID_AD7191] = {
		.chip_id = CHIPID_AD7191,
		.name = "ad7191",
		.channels = ad7191_channels,
		.num_channels = ARRAY_SIZE(ad7191_channels),
		.info = &ad7191_info,
	},
};

static void ad7191_reg_disable(void *reg)
{
	regulator_disable(reg);
}

static int ad7191_init_gpios(struct iio_dev *indio_dev)
{
	struct ad7191_state *st = iio_priv(indio_dev);
	struct device *dev = &st->sd.spi->dev;

	st->odr1_gpio = devm_gpiod_get_optional(dev, "odr1", GPIOD_OUT_LOW);
	if (IS_ERR(st->odr1_gpio))
		return dev_err_probe(dev, PTR_ERR(st->odr1_gpio),
				     "Failed to get odr1 gpio.\n");

	if (st->odr1_gpio)
		gpiod_set_value(st->odr1_gpio, 0);

	st->odr2_gpio = devm_gpiod_get_optional(dev, "odr2", GPIOD_OUT_LOW);
	if (IS_ERR(st->odr2_gpio))
		return dev_err_probe(dev, PTR_ERR(st->odr2_gpio),
				     "Failed to get odr2 gpio.\n");

	if (st->odr2_gpio)
		gpiod_set_value(st->odr2_gpio, 0);

	st->pga1_gpio = devm_gpiod_get_optional(dev, "pga1", GPIOD_OUT_LOW);
	if (IS_ERR(st->pga1_gpio))
		return dev_err_probe(dev, PTR_ERR(st->pga1_gpio),
				     "Failed to get pga1 gpio.\n");

	if (st->pga1_gpio)
		gpiod_set_value(st->pga1_gpio, 0);

	st->pga2_gpio = devm_gpiod_get_optional(dev, "pga2", GPIOD_OUT_LOW);
	if (IS_ERR(st->pga2_gpio))
		return dev_err_probe(dev, PTR_ERR(st->pga2_gpio),
				     "Failed to get pga2 gpio.\n");

	if (st->pga2_gpio)
		gpiod_set_value(st->pga2_gpio, 0);

	st->temp_gpio = devm_gpiod_get_optional(dev, "temp", GPIOD_OUT_LOW);
	if (IS_ERR(st->temp_gpio))
		return dev_err_probe(dev, PTR_ERR(st->temp_gpio),
				     "Failed to get temp gpio.\n");

	if (st->temp_gpio)
		gpiod_set_value(st->temp_gpio, 0);

	st->chan_gpio = devm_gpiod_get_optional(dev, "chan", GPIOD_OUT_LOW);
	if (IS_ERR(st->chan_gpio))
		return dev_err_probe(dev, PTR_ERR(st->chan_gpio),
				     "Failed to get chan gpio.\n");

	if (st->chan_gpio)
		gpiod_set_value(st->chan_gpio, 0);

	st->clksel_gpio = devm_gpiod_get_optional(dev, "clksel", GPIOD_OUT_LOW);
	if (IS_ERR(st->clksel_gpio))
		return dev_err_probe(dev, PTR_ERR(st->clksel_gpio),
				     "Failed to get clksel gpio.\n");

	if (st->clksel_gpio)
		gpiod_set_value(st->clksel_gpio, 0);

	return 0;
}

static int ad7191_probe(struct spi_device *spi)
{
	struct ad7191_state *st;
	struct iio_dev *indio_dev;
	int ret;

	dev_info(&spi->dev, "ad7191: probing\n");

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);

	st->avdd = devm_regulator_get(&spi->dev, "avdd");
	if (IS_ERR(st->avdd))
		return PTR_ERR(st->avdd);

	ret = regulator_enable(st->avdd);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable specified AVdd supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad7191_reg_disable, st->avdd);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(&spi->dev, "dvdd");
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to enable specified DVdd supply\n");

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable specified Vref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad7191_reg_disable, st->vref);
	if (ret)
		return ret;

	ret = regulator_get_voltage(st->vref);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret,
						"Device tree error, Vref voltage undefined\n");
	st->int_vref_mv = ret / 1000;
	dev_info(&spi->dev, "int_vref_mv = %d\n", st->int_vref_mv);

	st->chip_info = spi_get_device_match_data(spi);
	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = st->chip_info->info;

	ad_sd_init(&st->sd, indio_dev, spi, &ad7191_sigma_delta_info);

	ret = ad7191_init_gpios(indio_dev);
	if (ret)
		return ret;

	ret = devm_ad_sd_setup_buffer_and_trigger(&spi->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&spi->dev, "ad7191: before setup\n");

	ret = ad7191_setup(indio_dev, &spi->dev);
	if (ret)
		return ret;

	dev_info(&spi->dev, "ad7191: after setup\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad7191_of_match[] = {
	{ .compatible = "adi,ad7191", .data = &ad7191_chip_info_tbl[ID_AD7191] },
	{}
};
MODULE_DEVICE_TABLE(of, ad7191_of_match);

static const struct spi_device_id ad7191_ids[] = {
	{ "ad7191", (kernel_ulong_t)&ad7191_chip_info_tbl[ID_AD7191] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7191_ids);

static struct spi_driver ad7191_driver = {
	.driver = {
		.name	= "ad7191",
		.of_match_table = ad7191_of_match,
	},
	.probe		= ad7191_probe,
	.id_table	= ad7191_ids,
};
module_spi_driver(ad7191_driver);

MODULE_AUTHOR("Alisa-Dariana Roman <alisa.roman@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7191 ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_AD_SIGMA_DELTA);
