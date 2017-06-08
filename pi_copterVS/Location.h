// Location.h

#ifndef _LOCATION_h
#define _LOCATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
	unsigned char cls;
	unsigned char id;
	unsigned short len;
	unsigned long iTOW;
	long lon;
	long lat;
	long height;
	long hMSL;
	unsigned long hAcc;
	unsigned long vAcc;
};


class LocationClass
{


public:
	bool processGPS();
	float cosDirection, sinDirection;
	float dt, rdt;
	float add_lat_need, add_lon_need;
	long lat_, lon_, lat_home, lon_home;
	float accuracy_hor_pos, altitude;
	float accuracy_ver_pos;
	unsigned long mseconds;
	void setSpeedZero(){ lat_needV_ = lat_needR_; lon_needV_ = lon_needR_; }
	int init();
	void setNeedLoc2HomeLoc();
	
	void setNeedLoc(const long lat, const long lon);
	void setHomeLoc();
	void add2NeedLoc(const float speedX, const float speedY, const float dt);


	void updateXY();
	void bearing_dist(float &bearing, float & distance);
	
	float x2home, y2home, dX, dY, speedX, speedY;
	float dist2home_2;
	//---------------
	uint64_t last_gps_data_time;
	
	float bearing_(const float lat, const float lon, const float lat2, const float lon2);
	void sin_cos(float &x, float &y, const float lat, const float lon, const float lat2, const float lon2);

	float distance_(const float lat, const float lon, const float lat2, const float lon2);
	//---------------

	void clearSpeedCorrection(){ lat_needV_ = lat_needR_; lon_needV_ = lon_needR_; }

	float from_lat2X(const float lat){
		return lat*kd_lat_;
	}
	float from_X2Lat(const float x){ 
		return x *r_kd_lat; 
	}

	float form_lon2Y(const float lon){
		return lon*kd_lon_;
	}
	float from_Y2Lon(const float y){
		return y * r_kd_lon;
	}

private:
	float set_cos_sin_dir();
	void xy();
	float lat_needV_, lon_needV_, lat_needR_, lon_needR_;
	unsigned long old_iTOW;
	float mspeedx, mspeedy;
	void calcChecksum(unsigned char* CK);
	NAV_POSLLH posllh;
	float oldDist;
	void update();
	float kd_lon_, kd_lat_;
	float r_kd_lon, r_kd_lat;


};

extern LocationClass Location;

#endif

