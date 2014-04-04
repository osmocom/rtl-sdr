#ifndef __TUNER_TUA9001_H
#define __TUNER_TUA9001_H

#define TUA9001_I2C_ADDR	0xc0
#define TUA9001_CHECK_ADDR	0x7e
#define TUA9001_CHECK_VAL	0x2328

#define TUA9001_RESETN_PIN  3
#define TUA9001_RXEN_PIN    1

int tua9001_init(void *dev);
int tua9001_release(void *dev);
int tua9001_set_params(void *dev, uint32_t frequency_hz, uint32_t bandwidth_hz);

#endif /* __TUNER_TUA9001_H */