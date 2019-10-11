/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#else
#include <winsock2.h>
#include "getopt/getopt.h"
#endif

#include <pthread.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

static SOCKET s;

static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;

static pthread_cond_t cond;

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

static rtlsdr_dev_t *dev = NULL;

static int enable_biastee = 0;

// Ring Buffer declarations
// 8MB appears to cover several seconds at high bitrates -- about as much lag as you'd want
#define RINGBUFSZ_INIT (8*1024*1024)
static int ringbuf_sz = RINGBUFSZ_INIT;
static int ringbuf_trimsz = 512*1024;
static unsigned char *ringbuf = NULL;
static volatile unsigned int ringbuf_head = 0;
static volatile unsigned int ringbuf_tail = 0;
static unsigned int total_radio_bytes = 0;
static unsigned int max_bytes_in_flight = 0;

static volatile int do_exit = 0;

void usage(void)
{
	printf("rtl_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t[-a listen address]\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-b number of buffers (default: 15, set by library)]\n"
		"\t[-n max number of linked list buffers to keep (default: 500)]\n"
		"\t[-d device index (default: 0)]\n"
		"\t[-P ppm_error (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-D enable direct sampling (default: off)]\n");
	exit(1);
}

#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
#ifdef _MSC_VER
		tmp -= 11644473600000000Ui64;
#else
		tmp -= 11644473600000000ULL;
#endif
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	rtlsdr_cancel_async(dev);
	do_exit = 1;
}
#endif

void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    static time_t lasttime = 0;
	static int lastbytes = 0;
	time_t curtime;

	if(!do_exit) {
		unsigned int bufferleft;

		if (ringbuf == NULL)
		{
			printf("Allocate %d bytes for ringbuf.\n", ringbuf_sz);
			ringbuf = (unsigned char*)malloc(ringbuf_sz);
		}

		bufferleft = ringbuf_sz - ((ringbuf_head < ringbuf_tail) ? (ringbuf_head - ringbuf_tail + ringbuf_sz) : (ringbuf_head - ringbuf_tail));
		if (len < bufferleft)
		{
			if ((ringbuf_head+len) < (unsigned int)ringbuf_sz)
			{
				memcpy(((unsigned char*)(ringbuf+ringbuf_head)), buf, len);
			}
			else
			{
				memcpy(((unsigned char*)ringbuf+ringbuf_head), buf, ringbuf_sz-ringbuf_head);
				memcpy((unsigned char*)ringbuf, buf+(ringbuf_sz-ringbuf_head), len-(ringbuf_sz-ringbuf_head));
			}
			ringbuf_head = (ringbuf_head + len) % ringbuf_sz;
		}
		else
		{
			printf("overrun: head=%d tail=%d, Trimming %d bytes from tail of buffer\n", ringbuf_head, ringbuf_tail, ringbuf_trimsz);
			ringbuf_tail = (ringbuf_tail + ringbuf_trimsz) % ringbuf_sz;
		}

		total_radio_bytes += len;
		curtime = time (NULL);
		if ((curtime - lasttime) > 30)
		{
		   int nsecs = curtime - lasttime;
		   int nbytes = total_radio_bytes - lastbytes;
		   int bytes_in_flight = (ringbuf_head - ringbuf_tail);
		   if (bytes_in_flight < 0)
			  bytes_in_flight = ringbuf_sz + bytes_in_flight;
		   lasttime=curtime;
		   lastbytes=total_radio_bytes;
		   printf(">> [ %3.2fMB/s ]  [ bytes_in_flight(cur/max) = %4dK / %4dK ]\n",
			  (float)nbytes/(float)nsecs/1000.0/1000.0, bytes_in_flight/1024, max_bytes_in_flight/1024);
		   max_bytes_in_flight=0;
		}
	}
}

static void *tcp_worker(void *arg)
{
	int bytesleft, bytessent;
	struct timeval tv= {1,0};
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;

	while(1) {
		if(do_exit)
			pthread_exit(0);

		bytesleft = (ringbuf_head < ringbuf_tail) ?
		            (ringbuf_head - ringbuf_tail + ringbuf_sz) :
					(ringbuf_head - ringbuf_tail);
		while (bytesleft > 0)
		{
		   FD_ZERO(&writefds);
		   FD_SET(s, &writefds);
		   tv.tv_sec = 1;
		   tv.tv_usec = 0;
		   r = select(s+1, NULL, &writefds, NULL, &tv);
		   if(r) {
			  unsigned int sendchunk;
			  if (ringbuf_tail < ringbuf_head)
				 sendchunk = ringbuf_head - ringbuf_tail;
			  else
				 sendchunk = ringbuf_sz - ringbuf_tail;
			  if (sendchunk > max_bytes_in_flight)
				 max_bytes_in_flight = sendchunk;
			  bytessent = send(s,  (unsigned char*)(ringbuf+ringbuf_tail), sendchunk, 0);
			  bytesleft -= bytessent;
			  ringbuf_tail = (ringbuf_tail + bytessent) % ringbuf_sz;
		   }
		   if(bytessent == SOCKET_ERROR || do_exit) {
			  printf("worker socket bye\n");
			  sighandler(0);
			  pthread_exit(NULL);
		   }
		}
	}
}

static int set_gain_by_index(rtlsdr_dev_t *_dev, unsigned int index)
{
	int res = 0;
	int* gains;
	int count = rtlsdr_get_tuner_gains(_dev, NULL);

	if (count > 0 && (unsigned int)count > index) {
		gains = malloc(sizeof(int) * count);
		count = rtlsdr_get_tuner_gains(_dev, gains);

		res = rtlsdr_set_tuner_gain(_dev, gains[index]);

		free(gains);
	}

	return res;
}

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif
static void *command_worker(void *arg)
{
	int left, received = 0;
	fd_set readfds;
	struct command cmd={0, 0};
	struct timeval tv= {1, 0};
	int r = 0;
	uint32_t tmp;

	while(1) {
		left=sizeof(cmd);
		while(left >0) {
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s+1, &readfds, NULL, NULL, &tv);
			if(r) {
				received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
				left -= received;
			}
			if(received == SOCKET_ERROR || do_exit) {
				printf("comm recv bye\n");
				sighandler(0);
				pthread_exit(NULL);
			}
		}
		switch(cmd.cmd) {
		case 0x01:
			printf("set freq %d\n", ntohl(cmd.param));
			rtlsdr_set_center_freq(dev,ntohl(cmd.param));
			break;
		case 0x02:
			printf("set sample rate %d\n", ntohl(cmd.param));
			rtlsdr_set_sample_rate(dev, ntohl(cmd.param));
			break;
		case 0x03:
			printf("set gain mode %d\n", ntohl(cmd.param));
			rtlsdr_set_tuner_gain_mode(dev, ntohl(cmd.param));
			break;
		case 0x04:
			printf("set gain %d\n", ntohl(cmd.param));
			rtlsdr_set_tuner_gain(dev, ntohl(cmd.param));
			break;
		case 0x05:
			printf("set freq correction %d\n", ntohl(cmd.param));
			rtlsdr_set_freq_correction(dev, ntohl(cmd.param));
			break;
		case 0x06:
			tmp = ntohl(cmd.param);
			printf("set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
			rtlsdr_set_tuner_if_gain(dev, tmp >> 16, (short)(tmp & 0xffff));
			break;
		case 0x07:
			printf("set test mode %d\n", ntohl(cmd.param));
			rtlsdr_set_testmode(dev, ntohl(cmd.param));
			break;
		case 0x08:
			printf("set agc mode %d\n", ntohl(cmd.param));
			rtlsdr_set_agc_mode(dev, ntohl(cmd.param));
			break;
		case 0x09:
			printf("set direct sampling %d\n", ntohl(cmd.param));
			rtlsdr_set_direct_sampling(dev, ntohl(cmd.param));
			break;
		case 0x0a:
			printf("set offset tuning %d\n", ntohl(cmd.param));
			rtlsdr_set_offset_tuning(dev, ntohl(cmd.param));
			break;
		case 0x0b:
			printf("set rtl xtal %d\n", ntohl(cmd.param));
			rtlsdr_set_xtal_freq(dev, ntohl(cmd.param), 0);
			break;
		case 0x0c:
			printf("set tuner xtal %d\n", ntohl(cmd.param));
			rtlsdr_set_xtal_freq(dev, 0, ntohl(cmd.param));
			break;
		case 0x0d:
			printf("set tuner gain by index %d\n", ntohl(cmd.param));
			set_gain_by_index(dev, ntohl(cmd.param));
			break;
		case 0x0e:
			printf("set bias tee %d\n", ntohl(cmd.param));
			rtlsdr_set_bias_tee(dev, (int)ntohl(cmd.param));
			break;
		default:
			break;
		}
		cmd.cmd = 0xff;
	}
}

int main(int argc, char **argv)
{
	int r, opt, i;
	char* addr = "127.0.0.1";
	int port = 1234;
	uint32_t frequency = 100000000, samp_rate = 2048000;
	struct sockaddr_in local, remote;
	uint32_t buf_num = 0;
	int dev_index = 0;
	int dev_given = 0;
	int gain = 0;
	int ppm_error = 0;
	int direct_sampling = 0;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	u_long blockmode = 1;
	dongle_info_t dongle_info;
#ifdef _WIN32
	WSADATA wsd;
	i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	while ((opt = getopt(argc, argv, "a:p:f:g:s:b:d:P:T:D")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'a':
			addr = optarg;
			break;
		case 'p':
			port = atoi(optarg);
			break;
		case 'b':
			buf_num = atoi(optarg);
			break;
		case 'P':
			ppm_error = atoi(optarg);
			break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'D':
			direct_sampling = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc < optind)
		usage();

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	rtlsdr_open(&dev, (uint32_t)dev_index);
	if (NULL == dev) {
	fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	/* Set direct sampling */
        if (direct_sampling) {
                verbose_direct_sampling(dev, 2);
        }

	/* Set the tuner error */
	verbose_ppm_set(dev, ppm_error);

	/* Set the sample rate */
	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");

	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	else
		fprintf(stderr, "Tuned to %i Hz.\n", frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		r = rtlsdr_set_tuner_gain_mode(dev, 0);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to enable automatic gain.\n");
	} else {
		/* Enable manual gain */
		r = rtlsdr_set_tuner_gain_mode(dev, 1);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to enable manual gain.\n");

		/* Set the tuner gain */
		r = rtlsdr_set_tuner_gain(dev, gain);
		if (r < 0)
			fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
		else
			fprintf(stderr, "Tuner gain set to %f dB.\n", gain/10.0);
	}

	rtlsdr_set_bias_tee(dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	/* Reset endpoint before we start reading from it (mandatory) */
	r = rtlsdr_reset_buffer(dev);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");

	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_cond_init(&exit_cond, NULL);

	memset(&local,0,sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	bind(listensocket,(struct sockaddr *)&local,sizeof(local));

#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	while(1) {
		printf("listening...\n");
		printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
			   "(gr-osmosdr) source\n"
			   "to receive samples in GRC and control "
			   "rtl_tcp parameters (frequency, gain, ...).\n",
			   addr, port);
		listen(listensocket,1);

		while(1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket+1, &readfds, NULL, NULL, &tv);
			if(do_exit) {
				goto out;
			} else if(r) {
				rlen = sizeof(remote);
				s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
				break;
			}
		}

		setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("client accepted!\n");

		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "RTL0", 4);

		r = rtlsdr_get_tuner_type(dev);
		if (r >= 0)
			dongle_info.tuner_type = htonl(r);

		r = rtlsdr_get_tuner_gains(dev, NULL);
		if (r >= 0)
			dongle_info.tuner_gain_count = htonl(r);

		r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
		if (sizeof(dongle_info) != r)
			printf("failed to send dongle information\n");

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, 0);

		pthread_join(tcp_worker_thread, &status);
		pthread_join(command_thread, &status);

		closesocket(s);

		printf("all threads dead..\n");
		
		// Clear stale data for next client
		ringbuf_head = ringbuf_tail = 0;
		memset(ringbuf, 0, ringbuf_sz);

		do_exit = 0;
	}

out:
	rtlsdr_close(dev);
	closesocket(listensocket);
	closesocket(s);
	if (ringbuf)
	   free(ringbuf);
#ifdef _WIN32
	WSACleanup();
#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}
