/*
 * LED class driver for Maxim MAX20096, a Dual-Channel Synchronous Buck,
 * High-Brightness LED Controller with SPI interface
 *
 * (c) 2022 Aquabyte, Inc.
 * Developed by Zachary T Welch <zach@aquabyte.ai>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

//#define MAX20096_DEBUG

#define MAX20096_REG_NO_OP		0x00
#define MAX20096_REG_CNFG_SPI		0x01
#define MAX20096_REG_CNFG_GEN		0x02
#define MAX20096_REG_CNFG_CRNT1		0x03
#define MAX20096_REG_CNFG_CRNT2		0x04
#define MAX20096_REG_CNFG_TMNG		0x05
#define MAX20096_REG_CNFG_PWM1		0x06
#define MAX20096_REG_CNFG_PWM2		0x07

#define MAX20096_REG_MON_VBUCK1		0x0a
#define MAX20096_REG_MON_VBUCK2		0x0b
#define MAX20096_REG_MON_LED1		0x0c
#define MAX20096_REG_MON_LED2		0x0d
#define MAX20096_REG_MON_TEMP		0x0e
#define MAX20096_REG_GEN_STAT		0x0f

#define MAX20096_NUM_LEDS		2
#define MAX20096_MAX_BRIGHTNESS		0x07f
#define MAX20096_MAX_PWM		0x3ff

#define MAX20096_NAME_FMT		"max20096-%d"

/* CNFG_SPI register bit definitions */
#define MAX20096_CNFG_SPI_SFT_SDI	(1 << 0)
#define MAX20096_CNFG_SPI_SFT_CLK	(1 << 1)
#define MAX20096_CNFG_SPI_SFT_CSB	(1 << 2)
#define MAX20096_CNFG_SPI_SFT_RB	(1 << 3)
#define MAX20096_CNFG_SPI_ST_AB		(1 << 4)
#define MAX20096_CNFG_SPI_DCHN		(1 << 5)
#define MAX20096_CNFG_SPI_HW_RST	(1 << 6)
#define MAX20096_CNFG_SPI_RW_ERR	(1 << 7)
#define MAX20096_CNFG_SPI_PAR_ERR	(1 << 8)
#define MAX20096_CNFG_SPI_CLK_ERR	(1 << 9)

/* CNFG_GEN register bit definitions */
#define MAX20096_CNFG_GEN_CNFG_SEL	(1 << 8)
#define MAX20096_CNFG_GEN_PWM1_SEL	(1 << 7)
#define MAX20096_CNFG_GEN_PWM2_SEL	(1 << 6)
#define MAX20096_CNFG_GEN_BUCK1_EN	(1 << 5)
#define MAX20096_CNFG_GEN_BUCK2_EN	(1 << 4)
#define MAX20096_CNFG_GEN_PWM_FREQ	(0x7 << 0)

enum max20096_cnfg_gen_pwm_freq {
	MAX20096_CNFG_GEN_PWM_FREQ_200HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_333HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_400HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_500HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_667HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_1000HZ,
	MAX20096_CNFG_GEN_PWM_FREQ_2000HZ,
};

/* SPI command definitions */

#define MAX20096_REG_READ_BIT		(1 << 15)
#define MAX20096_REG_WRITE_BIT		(0 << 15)
#define MAX20096_REG_ADDR(a)		((a & 0xf) << 11)
#define MAX20096_REG_PARITY(p)		((p & 1) << 10)
#define MAX20096_REG_VALUE		0x3ff

#define MAX20096_CMD_READ(reg) \
		(MAX20096_REG_READ_BIT | \
		MAX20096_REG_ADDR(reg))

#define MAX20096_CMD_WRITE(reg, value) \
		(MAX20096_REG_WRITE_BIT | \
		MAX20096_REG_ADDR(reg) | \
		((value) & MAX20096_REG_VALUE))


struct max20096_led {
	int			id;
	char			name[sizeof(MAX20096_NAME_FMT)];

	u16			crnt;
	u16			pwm;

	struct led_classdev	ldev;
	struct max20096_device	*pdata;
};

struct max20096_device {
	bool			setup;
	struct mutex		lock;
	struct spi_device	*spi;
	struct gpio_desc	*resetb_gpio;
	struct max20096_led	leds[MAX20096_NUM_LEDS];
};


static u16 max20096_calculate_parity(u16 word)
{
	u16 i, parity = 1;

	word &= ~MAX20096_REG_PARITY(1);

	for (i = 0; i < 16; i++) {
		parity ^= word & 1;
		word >>= 1;
	}
	return parity;
}

static int max20096_setup(struct max20096_device *pdata);

static int max20096_write(struct max20096_device *pdata, u16 word)
{
	int ret;

	if (!pdata->setup) {
		ret = max20096_setup(pdata);
		if (ret < 0)
			return ret;
	}

	word |= MAX20096_REG_PARITY(max20096_calculate_parity(word));

#ifdef MAX20096_DEBUG
	dev_info(&pdata->spi->dev, "sent cmd=%04x", word);
#endif

	word = cpu_to_be16(word);

	mutex_lock(&pdata->lock);
	ret = spi_write(pdata->spi, &word, sizeof(word));
	mutex_unlock(&pdata->lock);

	return ret;
}

static int max20096_write_reg(struct max20096_led *led, u16 reg, u16 value)
{
	return max20096_write(led->pdata, MAX20096_CMD_WRITE(reg, value));
}

static int max20096_led_update(struct max20096_led *led)
{
	u16 addr;
	int ret;

	addr = MAX20096_REG_CNFG_CRNT1 + led->id;
	ret = max20096_write_reg(led, addr, led->crnt);
	if (ret < 0) return ret;

	addr = MAX20096_REG_CNFG_PWM1 + led->id;
	ret = max20096_write_reg(led, addr, led->pwm);
	if (ret < 0) return ret;

	return 0;
}

static int max20096_set_brightness(struct led_classdev *ldev,
				      enum led_brightness brightness)
{
	struct max20096_led *led;
	led = container_of(ldev, struct max20096_led, ldev);

	led->crnt = brightness;

	return max20096_led_update(led);
}

static int max20096_read(struct max20096_device *pdata, int reg, u16 *result)
{
	u16 word = MAX20096_CMD_READ(reg);
	u16 parity, bit = MAX20096_REG_PARITY(1);
	int ret;

	ret = max20096_write(pdata, word);
	if (ret < 0) return ret;

	ret = spi_read(pdata->spi, &word, sizeof(word));
	if (ret < 0) return ret;

	word = be16_to_cpu(word);
	parity = word & bit;
	*result = word;

#ifdef MAX20096_DEBUG
	dev_info(&pdata->spi->dev, "read data=%04x", word);
#endif

	return max20096_calculate_parity(word & ~bit) == !!parity ? 0 : -EIO;
}

static void max20096_reset(struct max20096_device *pdata)
{
	int i;

	mutex_lock(&pdata->lock);

	gpiod_direction_output(pdata->resetb_gpio, 1);
	udelay(1);
	gpiod_direction_output(pdata->resetb_gpio, 0);
	udelay(1);

	for (i = 0; i < MAX20096_NUM_LEDS; i++)
		pdata->setup = false;

	mutex_unlock(&pdata->lock);
}

static int max20096_setup(struct max20096_device *pdata)
{
	struct max20096_led *led;
	int ret, i;
	u16 cmd;

	pdata->setup = true;

	cmd = MAX20096_CMD_WRITE(MAX20096_REG_CNFG_GEN,
				MAX20096_CNFG_GEN_CNFG_SEL |
				MAX20096_CNFG_GEN_PWM1_SEL |
				MAX20096_CNFG_GEN_PWM2_SEL |
				MAX20096_CNFG_GEN_BUCK1_EN |
				MAX20096_CNFG_GEN_BUCK2_EN |
				MAX20096_CNFG_GEN_PWM_FREQ_500HZ);
	ret = max20096_write(pdata, cmd);
	if (ret < 0) return ret;

	for (i = 0; i < MAX20096_NUM_LEDS; i++) {
		led = pdata->leds + i;
		led->crnt = 0;
		led->pwm = 0x100;

		ret = max20096_led_update(led);
		if (ret < 0) return ret;
	}
	return 0;
}

const int max20096_pwm_freqs[] = { 200, 333, 400, 500, 667, 1000, 2000, 200 };

static ssize_t max20096_config_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *ldev;
	struct max20096_led *led;
	u16 cmd, cnfg_spi, cnfg_gen;
	int ret;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);

	cmd = MAX20096_CMD_READ(MAX20096_REG_CNFG_SPI);
	ret = max20096_read(led->pdata, cmd, &cnfg_spi);
	if (ret < 0) return -EIO;

	cmd = MAX20096_CMD_READ(MAX20096_REG_CNFG_GEN);
	ret = max20096_read(led->pdata, cmd, &cnfg_gen);
	if (ret < 0) return -EIO;

	return sprintf(buf, "CNFG_SPI: CLK_ERR=%d PAR_ERR=%d RW_ERR=%d "
		"HW_RST=%d DCHN=%dST_AB=%d\n\tSFT_RB=%d SFT_CSB=%d "
		"SFT_CLK=%d SFT_SDI=%d\n"
		"CNFG_GEN: CNFG_SEL=%d PWM1_SEL=%d PWM2_SEL=%d\n\t"
		"BUCK1_EN=%d BUCK2_EN=%d PWM_FREQ=%d\n",
		!!(cnfg_spi & MAX20096_CNFG_SPI_CLK_ERR),
		!!(cnfg_spi & MAX20096_CNFG_SPI_PAR_ERR),
		!!(cnfg_spi & MAX20096_CNFG_SPI_RW_ERR),
		!!(cnfg_spi & MAX20096_CNFG_SPI_HW_RST),
		!!(cnfg_spi & MAX20096_CNFG_SPI_DCHN),
		!!(cnfg_spi & MAX20096_CNFG_SPI_ST_AB),
		!!(cnfg_spi & MAX20096_CNFG_SPI_SFT_RB),
		!!(cnfg_spi & MAX20096_CNFG_SPI_SFT_CSB),
		!!(cnfg_spi & MAX20096_CNFG_SPI_SFT_CLK),
		!!(cnfg_spi & MAX20096_CNFG_SPI_SFT_SDI),

		!!(cnfg_gen & MAX20096_CNFG_GEN_CNFG_SEL),
		!!(cnfg_gen & MAX20096_CNFG_GEN_PWM1_SEL),
		!!(cnfg_gen & MAX20096_CNFG_GEN_PWM2_SEL),
		!!(cnfg_gen & MAX20096_CNFG_GEN_BUCK1_EN),
		!!(cnfg_gen & MAX20096_CNFG_GEN_BUCK2_EN),
		max20096_pwm_freqs[cnfg_gen & MAX20096_CNFG_GEN_PWM_FREQ]
		);
}
static DEVICE_ATTR(config, 0444, max20096_config_get, NULL);

static ssize_t max20096_pwm_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *ldev;
	struct max20096_led *led;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);

	return sprintf(buf, "0x%04x\n", led->pwm);
}

static ssize_t max20096_pwm_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *ldev;
	struct max20096_led *led;
	unsigned long word;
	int ret;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);

	ret = kstrtoul(buf, 16, &word);
	if (ret)
		return ret;
	if (word > MAX20096_MAX_PWM)
		return -EINVAL;

	led->pwm = word;
	return size;
}
static DEVICE_ATTR(pwm, 0644, max20096_pwm_get, max20096_pwm_set);

static ssize_t max20096_raw_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *ldev;
	struct max20096_led *led;
	unsigned long cmd;
	u16 word;
	int ret;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);

	ret = kstrtoul(buf, 16, &cmd);
	if (ret)
		return ret;
	if (cmd > 0xf)
		return -EINVAL;

	ret = max20096_read(led->pdata, cmd, &word);
	return ret ? sprintf(buf, "%#04x\n", word) : -EIO;
}

static ssize_t max20096_raw_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *ldev;
	struct max20096_led *led;
	int ret;
	unsigned long word;

	ret = kstrtoul(buf, 16, &word);
	if (ret)
		return ret;
	if (word > 0xffff)
		return -EINVAL;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);
	ret = max20096_write(led->pdata, word);
	return ret < 0 ? ret : size;
}

static DEVICE_ATTR(raw, 0644, max20096_raw_get, max20096_raw_set);

static ssize_t max20096_reset_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *ldev;
	struct max20096_led *led;
	int ret;

	if (strncmp(buf, "1\n", size))
		return -EINVAL;

	ldev = dev_get_drvdata(dev);
	led = container_of(ldev, struct max20096_led, ldev);

	max20096_reset(led->pdata);
	ret = max20096_setup(led->pdata);
	return ret ? : size;
}
static DEVICE_ATTR(reset, 0200, NULL, max20096_reset_set);

static struct attribute *max20096_attrs[] = {
	&dev_attr_config.attr,
	&dev_attr_pwm.attr,
	&dev_attr_raw.attr,
	&dev_attr_reset.attr,
	NULL
};
ATTRIBUTE_GROUPS(max20096);

static int max20096_probe(struct spi_device *spi)
{
	struct max20096_device *pdata = NULL;
	struct max20096_led *led;
	int ret, i;

	spi->bits_per_word = 16;

	pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->resetb_gpio = devm_gpiod_get(&spi->dev, "resetb", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->resetb_gpio)) {
		dev_err(&spi->dev, "unable to find reset gpio");
		return PTR_ERR(pdata->resetb_gpio);
	}

	for (i = 0; i < ARRAY_SIZE(pdata->leds); i++) {
		led = pdata->leds + i;

		led->id = i;
		snprintf(led->name, sizeof(led->name), MAX20096_NAME_FMT, i);

		led->ldev.name = led->name;
		led->ldev.brightness = LED_OFF;
		led->ldev.max_brightness = MAX20096_MAX_BRIGHTNESS;
		led->ldev.brightness_set_blocking = max20096_set_brightness;
		led->ldev.groups = max20096_groups;

		ret = led_classdev_register(&spi->dev, &led->ldev);
		if (ret < 0)
			goto errout;

		led->pdata = pdata;
	}

	mutex_init(&pdata->lock);
	pdata->spi = spi;

	spi_set_drvdata(spi, pdata);

	max20096_reset(pdata);

	return 0;

errout:
	while (i--)
		led_classdev_unregister(&pdata->leds[i].ldev);

	return ret;
}

static int max20096_remove(struct spi_device *spi)
{
	struct max20096_device *pdata = spi_get_drvdata(spi);
	int i;

	for (i = 0; i < ARRAY_SIZE(pdata->leds); i++)
		led_classdev_unregister(&pdata->leds[i].ldev);

	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct of_device_id max20096_of_match[] = {
	{ .compatible = "maxim,max20096", },
	{ }
};
MODULE_DEVICE_TABLE(of, max20096_of_match);

static struct spi_driver max20096_driver = {
	.driver = {
		.name		= "max20096",
		.of_match_table = max20096_of_match,
	},
	.probe		= max20096_probe,
	.remove		= max20096_remove,
};

module_spi_driver(max20096_driver);

MODULE_DESCRIPTION("Driver for MAX20096 LED controller");
MODULE_AUTHOR("Zachary T Welch");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:max20096");
