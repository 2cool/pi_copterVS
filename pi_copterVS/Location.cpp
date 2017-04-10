// 
// 
// 

#include "Location.h"
#include "define.h"
#include "debug.h"
#include "Stabilization.h"
#define gps Serial2
#define DELTA_ANGLE_C 0.001
#define DELTA_A_RAD (DELTA_ANGLE_C*GRAD2RAD)
#define DELTA_A_E7 (DELTA_ANGLE_C*10000000)


void LocationClass::xy(){

	
	
	//------------------------------------------------------------
	
	float t;
	if (lon_home == 0){
		return;
	}

	t = form_lon2Y((float)(lon_home - lon_));
	speedY = (t - y2home) *rdt;
	y2home = t;
	dY = form_lon2Y((float)(lon_needV_ - lon_)) + (speedY*0.5);

	t = from_lat2X((float)(lat_home - lat_));
	speedX = (t - x2home) * rdt;
	x2home = t;
	dX = from_lat2X((float)(lat_needV_ - lat_)) + (speedX*0.5);
	
	set_cos_sin_dir();
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
	float y = distance*sin(bearing);
	float x = distance*cos(bearing);
	
	//kd_lat = -x *0.00001;
	//kd_lon = -y *0.00001;

	kd_lat_ = x / (DELTA_A_E7);
	r_kd_lat = 1.0 / kd_lat_;
	kd_lon_ = y / (DELTA_A_E7);
	r_kd_lon = 1.0 / kd_lon_;



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
	//Serial.println(gps.available());
	if (gps.available() < 36){
		return false;
	}
	static int fpos = 0;
	static unsigned char checksum[2];
	const int payloadSize = sizeof(NAV_POSLLH);

	while (gps.available()) {
		byte c = gps.read();
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
				if (c == checksum[1]) {

					
					accuracy_hor_pos = DELTA_ANGLE_C*posllh.hAcc;
					accuracy_ver_pos = DELTA_ANGLE_C*posllh.vAcc;
					mseconds = posllh.iTOW;
					dt = DELTA_ANGLE_C*(posllh.iTOW - old_iTOW);
					rdt = 1.0 / dt;
					old_iTOW = posllh.iTOW;
					last_gps_data_time = millis();
					
					lat_ = posllh.lat;
					lon_ = posllh.lon;

					return true;
				}
			}
			else if (fpos > (payloadSize + 4)) {
				fpos = 0;
			}
		}
	}
	return false;
}





void LocationClass::add2NeedLoc(const float speedX, const float speedY, const float dt){
	long t = (add_lat_need+= from_X2Lat(speedX*dt));
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

void LocationClass::init(){
	mspeedx =  mspeedy = 0;
	old_iTOW = 0;
	oldDist = MAX_DIST2UPDATE + MAX_DIST2UPDATE;

	dt = 0.1;
	add_lat_need = add_lon_need = 0;
	//kd_lon = 0;// -0.000812690982;
	//kd_lat = 0;// -0.001112000712;

	x2home = y2home = speedX = speedY = 0;
	lon_home = lat_home = 0;
	accuracy_hor_pos = 99.99;
	accuracy_ver_pos = 99.99;
	altitude = 0;
	lat_ = 0;  //radians
	lon_ = 0;
	dt = 0.1;
	rdt = 10;
	speedX = speedY = 0;
	Out.println("loc init");
}

void LocationClass::setHomeLoc(){
	lat_home = lat_;
	lon_home = lon_;
	//Debug.dump(lat_, lon_, 0, 0);
	speedX = speedY = x2home = y2home = 0;
	setNeedLoc2HomeLoc();
}
void LocationClass::setNeedLoc(long lat, long lon){
	lat_needR_ = lat_needV_ = lat;
	lon_needR_ = lon_needV_ = lon;
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
		float angle =  atan(dY / dX);
		//ErrorLog.println(angle*RAD2GRAD);
		cosDirection = abs(cos(angle));
		sinDirection = abs(sin(angle));
	}
	
}

float LocationClass::bearing_(const float lat, const float lon, const float lat2, const float lon2){
	float x, y;
	sin_cos(x, y, lat, lon, lat2, lon2);
	return atan2(y, x);  //minus и как у гугла

}
void LocationClass::sin_cos(float &x, float &y, const float lat, const float lon, const float lat2, const float lon2){
	float rll = (lon2 - lon);
	float rlat = (lat);
	float rlat2 = (lat2);

	y = sin(rll)*cos(rlat2);
	x = cos(rlat)*sin(rlat2) - sin(rlat)*cos(rlat2)*cos(rll);

}
float LocationClass::distance_(const float lat, const float lon, const float lat2, const float lon2){
	float R = 6371000;
	float f1 = (lat);
	float f2 = (lat2);
	float df = (lat2 - lat);
	float dq = (lon2 - lon);

	float a = sin(df / 2)*sin(df / 2) + cos(f1)*cos(f2)*sin(dq / 2)*sin(dq / 2);
	return R * 2 * atan2(sqrt(a), sqrt(1 - a));

}

LocationClass Location;

