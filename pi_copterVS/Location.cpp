// 
// 
// 
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
//#include <thread>

#include "Location.h"
#include "define.h"
#include "debug.h"
#include "Stabilization.h"
#include "Autopilot.h"

#define gps Serial2
#define DELTA_ANGLE_C 0.001f
#define DELTA_A_RAD (DELTA_ANGLE_C*GRAD2RAD)
#define DELTA_A_E7 (DELTA_ANGLE_C*10000000)



int fd4S;


int set_interface_attribs(int fd4S, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd4S, &tty) != 0)
	{
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
													// disable IGNBRK for mismatched speed tests; otherwise receive break
													// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd4S, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd4S, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd4S, &tty) != 0)
	{
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr(fd4S, TCSANOW, &tty) != 0)
		printf("error %d setting term attributes", errno);
}















void LocationClass::xy(){

	
	
	//------------------------------------------------------------
	
	float t;
	if (lon_home == 0){
		return;
	}

	t = form_lon2Y((float)(lon_home - lon_));
	speedY = (t - y2home) *rdt;
	y2home = t;
	dY = form_lon2Y((float)(lon_needV_ - (float)lon_)) + (speedY*0.5f);

	t = from_lat2X((float)(lat_home - lat_));
	speedX = (t - x2home) * rdt;
	x2home = t;
	dX = from_lat2X((float)(lat_needV_ - (float)lat_)) + (speedX*0.5f);
	
	set_cos_sin_dir();


#ifdef XY_SAFE_AREA
	if (Autopilot.motors_is_on() && sqrt(x2home*x2home+y2home*y2home)>XY_SAFE_AREA)
		Autopilot.control_falling(e_OUT_OF_PER_H);
#endif
}
void LocationClass::update(){

	
	float bearing, distance;

#ifdef DEBUG_MODE
	Out.println("upd");
	Debug.dump(lat_, lon_, 0, 0);
#endif
	float lat = 1.74532925199433e-9f * (float)lat_;  //radians
	float lon = 1.74532925199433e-9f * (float)lon_;

	bearing = bearing_(lat, lon, lat + DELTA_A_RAD, lon + DELTA_A_RAD);
	distance = distance_(lat, lon, lat + DELTA_A_RAD, lon + DELTA_A_RAD);
//	Debug.dump(lat_, lon_, lat + 0.01*GRAD2RAD, lon + 0.01*GRAD2RAD);
//	Debug.dump(lat, lon, distance, bearing);
	float y = distance*(float)sin(bearing);
	float x = distance*(float)cos(bearing);
	
	//kd_lat = -x *0.00001;
	//kd_lon = -y *0.00001;

	kd_lat_ = x / (DELTA_A_E7);
	r_kd_lat = 1.0f / kd_lat_;
	kd_lon_ = y / (DELTA_A_E7);
	r_kd_lon = 1.0f / kd_lon_;



	//Debug.load(0, x, y);
	//Debug.dump();
	//Out.print("3UPD "); Out.print(x); Out.print(" "); Out.println(y);
}
#define MAX_DIST2UPDATE 1000000
void LocationClass::updateXY(){
	dist2home_2 = x2home*x2home + y2home*y2home;
	//Out.println(dist2home_2);
	if (accuracy_hor_pos<5 &&  abs(dist2home_2 - oldDist) > MAX_DIST2UPDATE){
		oldDist = dist2home_2;
		
		update();
		
	}

	xy();
	//Out.print("N  "); Out.print(x2home); Out.print(" "); Out.println(y2home);
}







void LocationClass::calcChecksum(unsigned char* CK) {
	memset(CK, 0, 2);
	for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
		CK[0] += ((unsigned char*)(&posllh))[i];
		CK[1] += CK[0];
	}
}

bool LocationClass::processGPS() {

	int available;
	ioctl(fd4S, FIONREAD, &available);
	//printf("available %i\n",available);

	if (available < 36){
		return false;
	}
	static int fpos = 0;
	static unsigned char checksum[2];
	const int payloadSize = sizeof(NAV_POSLLH);
	char buf[4096];
	int buf_ind = 0;
#define c (buf[buf_ind])
	available=read(fd4S, &buf[buf_ind], 4096 - buf_ind);
	while (available) {
		if (fpos < 2) {
			if (c == UBX_HEADER[fpos])
				fpos++;
			else
				fpos = 0;
		}
		else {
			if ((fpos - 2) < payloadSize)
				((unsigned char*)(&posllh))[fpos - 2] = c;

			fpos++;

			if (fpos == (payloadSize + 2)) {
				calcChecksum(checksum);
			}
			else if (fpos == (payloadSize + 3)) {
				if (c != checksum[0])
					fpos = 0;
			}
			else if (fpos == (payloadSize + 4)) {
				fpos = 0;
				if (c == buf[buf_ind]) {

					
					accuracy_hor_pos = DELTA_ANGLE_C*(float)posllh.hAcc;
					if (accuracy_hor_pos > 99)accuracy_hor_pos =  99;
					accuracy_ver_pos = DELTA_ANGLE_C*(float)posllh.vAcc;
					if (accuracy_ver_pos > 99)accuracy_ver_pos =  99;
					mseconds = posllh.iTOW;
					dt = DELTA_ANGLE_C*(float)(posllh.iTOW - old_iTOW);
					dt=constrain(dt, 0.1f, 1);
					rdt = 1.0f / dt;
					old_iTOW = posllh.iTOW;
					last_gps_data_time = micros();
					
					lat_ = posllh.lat;
					lon_ = posllh.lon;

					//Debug.load(0, lat_, lon_);
					//Debug.dump();

					buf_ind++;
					available--;
					if (available) {
						memcpy(buf, &buf[buf_ind], available);
						buf_ind -= available;
					}
					return true;
				}
			}
			else if (fpos > (payloadSize + 4)) {
				fpos = 0;
			}
		}
		available--;
		buf_ind++;
		//ioctl(fd4S, FIONREAD, &available);
	}
	return false;
}





void LocationClass::add2NeedLoc(const float speedX, const float speedY, const float dt){
	float t = (add_lat_need+= from_X2Lat(speedX*dt));
	add_lat_need -= t;
	lat_needR_ -= t;
	lat_needV_ = lat_needR_ - from_X2Lat(Stabilization.getDist_XY(speedX));
	t = (add_lon_need += from_Y2Lon(speedY*dt));
	add_lon_need -= t;
	lon_needR_ -= t;
	lon_needV_ = lon_needR_ - from_Y2Lon(Stabilization.getDist_XY(speedY));
}



//lat 0.001 грудус = 111.2 метра
//lon 0.001 грудус = 74.6 метра

//lat 899.2805755396
//lon 1340.482573727

int LocationClass::init(){
#ifndef FALSE_GPS
	fd4S = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_SYNC);
	if (fd4S < 0)
	{
		printf("error %d opening /dev/ttyS3: %s", errno, strerror(errno));
		return -1;
	}

	set_interface_attribs(fd4S, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(fd4S, 0);                // set no blocking
#endif
	mspeedx =  mspeedy = 0;
	old_iTOW = 0;
	oldDist = MAX_DIST2UPDATE + MAX_DIST2UPDATE;

	dt = 0.1f;
	add_lat_need = add_lon_need = 0;
	//kd_lon = 0;// -0.000812690982;
	//kd_lat = 0;// -0.001112000712;

	x2home = y2home = speedX = speedY = 0;
	lon_home = lat_home = 0;
	accuracy_hor_pos = 99.99f;
	accuracy_ver_pos = 99.99f;
	altitude = 0;
	lat_ = 0;  //radians
	lon_ = 0;
	dt = 0.1f;
	rdt = 10;
	speedX = speedY = 0;
	last_gps_data_time = 0;
	printf("loc init\n");
}

void LocationClass::setHomeLoc(){
	lat_home = lat_;
	lon_home = lon_;
	//Debug.dump(lat_, lon_, 0, 0);
	speedX = speedY = x2home = y2home = 0;
	setNeedLoc2HomeLoc();
}
void LocationClass::setNeedLoc(long lat, long lon){
	lat_needR_ = lat_needV_ = (float)lat;
	lon_needR_ = lon_needV_ = (float)lon;
	xy();
	//set_cos_sin_dir();

}

void LocationClass::setNeedLoc2HomeLoc(){
	setNeedLoc(lat_home, lon_home);
}

float LocationClass::set_cos_sin_dir(){

	if (dX == 0){
		cosDirection = 0;
		sinDirection = 1;
	}
	else{
		float angle =  (float)atan(dY / dX);
		//ErrorLog.println(angle*RAD2GRAD);
		cosDirection = (float)abs(cos(angle));
		sinDirection = (float)abs(sin(angle));
	}
	
}

float LocationClass::bearing_(const float lat, const float lon, const float lat2, const float lon2){
	float x, y;
	sin_cos(x, y, lat, lon, lat2, lon2);
	return (float)atan2(y, x);  //minus и как у гугла

}
void LocationClass::sin_cos(float &x, float &y, const float lat, const float lon, const float lat2, const float lon2){
	float rll = (lon2 - lon);
	float rlat = (lat);
	float rlat2 = (lat2);

	y = (float)(sin(rll)*cos(rlat2));
	x = (float)(cos(rlat)*sin(rlat2) - sin(rlat)*cos(rlat2)*cos(rll));

}
float LocationClass::distance_(const float lat, const float lon, const float lat2, const float lon2){
	float R = 6371000;
	float f1 = (lat);
	float f2 = (lat2);
	float df = (lat2 - lat);
	float dq = (lon2 - lon);

	float a = (float)(sin(df / 2)*sin(df / 2) + cos(f1)*cos(f2)*sin(dq / 2)*sin(dq / 2));
	return (float)(R * 2 * atan2(sqrt(a), sqrt(1 - a)));

}

LocationClass Location;

