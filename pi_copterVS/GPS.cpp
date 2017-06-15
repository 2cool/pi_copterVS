 
// 
// 
#include "Hmc.h"
#include "define.h"
#include "GPS.h"
#include <math.h>
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"


void GPSClass::init()
{	

#ifndef	FALSE_GPS
	//gps.begin(9600);
	//gps.begin(115200);
	loc.last_gps_data_time = millis();

#endif
	
	if (loc.init() == -1) {
		printf("GPS ERROR\n");
		return;
	}
	errors = 0;
	printf("GPS INIT\n");
}


#ifdef FALSE_GPS

#include "Stabilization.h"


#include "Balance.h"
uint32_t gpsttime = 0;
//float distX = 0, distY = 0;
//float speedX = 0, speedY = 0;
float fdt = 0;
bool gps_foo1 = true;
uint32_t tkskks = 0;

//#define FALSE_LAT 479001194
//#define FALSE_LON 332531662


#define FALSE_LAT 479059400
#define FALSE_LON 333368000

long lat = FALSE_LAT;
long lon = FALSE_LON;




float fullX = 0, fullY = 0;
void GPSClass::loop(){
	//dataRedy = false;
	if (Mpu.mpu_calibrated==false){
		
		return;
	}

	//home 47.9078 33.3318
	//next 47.908233 33.3319

	//distX = Mpu.fdistX;
	//distY = Mpu.fdistY;
	//speedX = Mpu.fspeedX;
	//speedY = Mpu.fspeedY;

	gpsttime = millis()/100;
	
	if (loc.mseconds != gpsttime)
		loc.mseconds = gpsttime;
	else
		return;
	loc.accuracy_hor_pos = 0;
	loc.accuracy_ver_pos = 1;
	loc.altitude = MS5611.altitude();// +4 * ((float)rand()) / RAND_MAX;





	loc.dt = 0.1;

	
	
	if (Autopilot.motors_is_on() == false){
		//speedX = speedY = 0;
		
	}

#ifdef FALSE_BAROMETR
		if (MS5611.altitude() <= 0){
			//speedX = speedY = 0;
	
			//distX = distY = 0;
		}
#endif


		//Debug.dump(lat*1000, lon*1000, loc.x2home, loc.y2home);
		

		

#ifndef FASLE_GPS_STILL
		lat = FALSE_LAT + (long)(loc.from_X2Lat(Mpu.fdistX));
		lon = FALSE_LON + (long)(loc.from_Y2Lon(Mpu.fdistY));
#endif
		loc.lat_ = lat;
		loc.lon_ = lon;
		loc.updateXY();

		//loc.lat_ = lat;
		//loc.lon_ = lon;



		

		/*
		fullX += distX;
		fullY += distY;
		Out.print(fullX);
		Out.print(" ");
		Out.println(fullY);
		*/

		//distX = distY = 0;
		//Out.println("---------");
		//Out.println( (loc.get_kd_lat()));
	//	Debug.dump(lat, lon, 0, 0);
		//Out.println(Location.get_kd_lat());
}

#else

int errors__ =0;
uint64_t last_gps_time1 = 0;

uint64_t max_time = 0;
void GPSClass::loop(){

	uint64_t ttt = micros();
	if (micros() - last_gps_time1 >= 33000) {
		last_gps_time1 = micros();
		if (loc.processGPS()) {
			//printf("pgs %i\n", micros() - ttt);
			if (loc.accuracy_hor_pos < (MIN_ACUR_HOR_POS_TO_FLY)) {
				errors = 0;

				loc.updateXY();
			}
			else {
				errors++;
				if (errors > 50) {
					Autopilot.control_falling(e_GPS_ERRORS_M_50);
					
				}
			}
		}
		//loc.accuracy_hor_pos = errors__/100;
		//loc.accuracy_ver_pos = errors__ % 100;
		if ((last_gps_time1 > loc.last_gps_data_time) && (last_gps_time1 - loc.last_gps_data_time) > 1000000){//NO_GPS_TIME_TO_FALL) {
			printf("gps update error  %i\n",millis()/1000);
			//init();
			loc.last_gps_data_time = micros();
			errors__++;
			Autopilot.control_falling(e_GPS_NO_UPDATE);
		}

	}
	uint64_t dt=micros() - ttt;
	if (dt > max_time) {
		max_time = dt;
	//	printf("max=%i\n", max_time);
	}
}













#endif

GPSClass GPS;

