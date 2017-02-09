#include <stdint.h>
#include <stdio.h>

#include <unistd.h>

#include "rtlsdr_i2c.h"
#include "tuner_tua9001.h"

#define RETURN_OK  0
#define RETURN_ERR -1

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define SYS_DVBT 0 /* as we know only one */

struct reg_val {
	uint8_t reg;
	uint16_t val;
};

/* write register */
int tua9001_wr_reg(void *dev, uint8_t reg, uint16_t val)
{
	int ret;
	uint8_t buf[3] = { reg, (val >> 8) & 0xff, (val >> 0) & 0xff };

	ret = rtlsdr_i2c_tunb_write_fn(dev, TUA9001_I2C_ADDR, buf, 2);
	if (ret < 0) {
		fprintf(stderr, "%s: i2c wr failed=%d reg=%02x\n","tuner_tua9001.c", ret, reg);
	}

	return ret;
}

int tua9001_release(void *dev)
{
	return 0;
}

int tua9001_init(void *dev)
{
	int ret = 0;
	uint8_t i;
	struct reg_val data[] = {
		{ 0x1e, 0x6512 },
		{ 0x25, 0xb888 },
		{ 0x39, 0x5460 },
		{ 0x3b, 0x00c0 },
		{ 0x3a, 0xf000 },
		{ 0x08, 0x0000 },
		{ 0x32, 0x0030 },
		{ 0x41, 0x703a },
		{ 0x40, 0x1c78 },
		{ 0x2c, 0x1c00 },
		{ 0x36, 0xc013 },
		{ 0x37, 0x6f18 },
		{ 0x27, 0x0008 },
		{ 0x2a, 0x0001 },
		{ 0x34, 0x0a40 },
	};
	
	rtlsdr_set_gpio_bit_fn(dev,TUA9001_RESETN_PIN,0);

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = tua9001_wr_reg(dev, data[i].reg, data[i].val);
	        if (ret < 0)
			goto err;
	}
err:
	if (ret < 0)
		fprintf(stderr, "%s: failed=%d\n","tua9001_init", ret);

	return RETURN_OK;
}

int tua9001_set_params(void *dev, uint32_t frequency_hz, uint32_t bandwidth_hz)
{
	int ret = 0; 
	unsigned int i;
	uint16_t val;
	uint32_t frequency;
	struct reg_val data[2];
	
	uint8_t delivery_system = SYS_DVBT;

	//fprintf(stderr, "%s: delivery_system=%d frequency=%d bandwidth_hz=%d\n", "tua9001_set_params", delivery_system ,frequency_hz, bandwidth_hz);

	switch (SYS_DVBT) {
	case SYS_DVBT:
		switch (bandwidth_hz) {
		case 8000000:
			val  = 0x0000;
			break;
		case 7000000:
			val  = 0x1000;
			break;
		case 6000000:
			val  = 0x2000;
			break;
		case 5000000:
			val  = 0x3000;
			break;
		default:
			ret = -1;
			goto err;
		}
		break;
	default:
		ret = -1;
		goto err;
	}

	data[0].reg = 0x04;
	data[0].val = val;

	frequency = (frequency_hz - 150000000);
	frequency /= 100;
	frequency *= 48;
	frequency /= 10000;

	data[1].reg = 0x1f;
	data[1].val = frequency;

	rtlsdr_set_gpio_bit_fn(dev,TUA9001_RXEN_PIN,0);

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		ret = tua9001_wr_reg(dev, data[i].reg, data[i].val);
		if (ret < 0)
			goto err;
	}
	
	rtlsdr_set_gpio_bit_fn(dev,TUA9001_RXEN_PIN,1);
	
err:
	if (ret < 0)
		fprintf(stderr, "%s: failed=%d\n", "tua9001_set_params", ret);

	return ret;
}