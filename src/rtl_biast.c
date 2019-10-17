/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * rtl_eeprom, EEPROM modification tool
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define EEPROM_SIZE	256
#define MAX_STR_SIZE	256
#define STR_OFFSET	0x09

static rtlsdr_dev_t *dev = NULL;

typedef struct rtlsdr_config {
	uint16_t vendor_id;
	uint16_t product_id;
	char manufacturer[MAX_STR_SIZE];
	char product[MAX_STR_SIZE];
	char serial[MAX_STR_SIZE];
	int have_serial;
	int enable_ir;
	int remote_wakeup;
} rtlsdr_config_t;


void usage(void)
{
	fprintf(stderr,
		"rtl_biast, a tool for turning the RTL-SDR.com \n"
		"bias tee or any GPIO ON and OFF. Example to turn on the \n"
		"bias tee: rtl_biast -d 0 -b 1\n"
                "Any GPIO: rtl_biast -d 0 -g 1 -b 1\n\n"
		"Usage:\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-b bias_on (default: 0)]\n"
                "\t[-g GPIO select (default: 0)]\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int i, r, opt;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t bias_on = 0;
        uint32_t gpio_pin = 0;
	int device_count;
	char *filename = NULL;
	FILE *file = NULL;
	char *manuf_str = NULL;
	char *product_str = NULL;
	char *serial_str = NULL;
	uint8_t buf[EEPROM_SIZE];
	rtlsdr_config_t conf;
	int flash_file = 0;
	int default_config = 0;
	int change = 0;
	int ir_endpoint = 0;
	char ch;

	while ((opt = getopt(argc, argv, "d:b:g:h?")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'b':
			bias_on = atoi(optarg);
			break;
                case 'g':
                       gpio_pin = atoi(optarg);
                       break;
		default:
			usage();
			break;
		}
	}

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, dev_index);

        rtlsdr_set_gpio(dev, gpio_pin, bias_on);
	//rtlsdr_set_bias_tee(dev, bias_on);

	//rtlsdr_set_direct_sampling(dev, 1);

exit:
	rtlsdr_close(dev);

	return r >= 0 ? r : -r;
}

