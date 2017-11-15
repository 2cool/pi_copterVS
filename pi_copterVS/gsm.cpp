#include "gsm.h"
#include <cstdio>
#include <cstdio>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include "I2Cdev.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include "debug.h"

/*
https://blog.jokielowie.com/en/2016/10/raspberry-pi-oraz-siec-gsm-wysylamy-sms-laczymy-sie-z-internetem-iot/
https://www.rhydolabz.com/wiki/?p=16325
pon rnet
poff rnet
cat /var/log/syslog | grep pppd
tail -f /var/log/syslog


picocom --baud 9600 /dev/tnt0



wget --output-document=/dev/null http://speedtest.wdc01.softlayer.com/downloads/test500.zip
*/

#define  ARDUINO_ADDR 9

int f_ser;
int f_i2c;

int GsmClass::init()
{
	f_ser = open("/dev/tnt1", O_RDWR | O_NOCTTY | O_SYNC);
	if (f_ser < 0)
	{
		fprintf(Debug.out_stream, "error %d opening /dev/tnt0: %s", errno, strerror(errno));
		return -1;
	}

	fprintf(Debug.out_stream, "arduino connection test\n");
	if ((f_i2c = open("/dev/i2c-0", O_RDWR)) < 0) {
		fprintf(Debug.out_stream, "Failed to open the bus.\n");
		return -1;
	}
	if (ioctl(f_i2c, I2C_SLAVE, ARDUINO_ADDR) < 0) {
		fprintf(Debug.out_stream, "Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
	return 0;

}

int GsmClass::getsim(char * str) {
	char reg = 3;
	char sim_count;
	write(f_i2c, &reg, 1);
	int ret = read(f_i2c, &sim_count, 1);
	int res = 0;
	if (ret == 1 && sim_count > 0) {
		res = read(f_i2c, str, sim_count);
		//for (int i = 0; i < sim_count; i++)
		//	printf("%c", str[i+ shift]);
	}
	return res;
}

char buf[32];
int GsmClass::send2sim(char *str, int len) {
	buf[0] = 2;
	memcpy(buf + 1, str, len);
	//usleep(50);
	write(f_i2c, buf, len + 1);
	//	for (int i = 1; i<len+1; i++)
	//	printf("%c", buf[i]);
}

int GsmClass::loop()
{
	char buf1[1024];
	int a_in;
	ioctl(f_ser, FIONREAD, &a_in);
	if (a_in) {
		//printf("----\n");
		if (a_in > 16)
			a_in = 16;

		int av = read(f_ser, &buf1, a_in);
		send2sim(buf1, a_in);
	}
	int res = getsim(buf1);
	if (res) {
		write(f_ser, buf1, res);
	}

	return 0;

}

GsmClass GSM;