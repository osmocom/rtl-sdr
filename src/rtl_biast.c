/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * rtl_biast, tool to set bias tee gpio output
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

static rtlsdr_dev_t *dev = NULL;

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
	rtlsdr_set_bias_tee_gpio(dev, gpio_pin, bias_on);

exit:
	/*
	 * Note - rtlsdr_close() in this tree does not clear the bias tee
	 * GPIO line, so it leaves the bias tee enabled if a client program
	 * doesn't explictly disable it.
	 *
	 * If that behaviour changes then another rtlsdr_close() will be
	 * needed that takes some extension flags, and one of them should
	 * be to either explicitly close the biast or leave it alone.
	 */
	rtlsdr_close(dev);

	return r >= 0 ? r : -r;
}
