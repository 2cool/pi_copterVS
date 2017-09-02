 
#include "Hmc.h"
#include "define.h"
#include "GPS.h"
#include <math.h>
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"
#include "Log.h"

void GPSClass::init()
{	
#ifndef	FALSE_WIRE
	loc.last_gps_data_time = millis();
#endif
	if (loc.init() == -1) {
		fprintf(Debug.out_stream,"GPS ERROR\n");
		return;
	}
	fprintf(Debug.out_stream,"GPS INIT\n");
}


#ifdef FALSE_WIRE


uint32_t gpsttime = 0;
//float distX = 0, distY = 0;
//float speedX = 0, speedY = 0;
float fdt = 0;
bool gps_foo1 = true;
uint32_t tkskks = 0;

//#define FALSE_LAT 479001194
//#define FALSE_LON 332531662


//park gag
//#define FALSE_LAT 479059400
//#define FALSE_LON 333368000

//pole
#define FALSE_LAT 479001188
#define FALSE_LON 332530612

long lat = FALSE_LAT;
long lon = FALSE_LON;




float fullX = 0, fullY = 0;
void GPSClass::loop(){




	gpsttime = millis()/100;
	
	if (loc.mseconds != gpsttime)
		loc.mseconds = gpsttime;
	else
		return;
	loc.accuracy_hor_pos_ = 0;
	loc.accuracy_ver_pos_ = 1;
	loc.altitude = MS5611.altitude();





	loc.dt = 0.1;

	
	
	if (Autopilot.motors_is_on() == false){
		//speedX = speedY = 0;
		
	}

#ifdef FALSE_WIRE
		if (MS5611.altitude() <= 0){
			//speedX = speedY = 0;
	
			//distX = distY = 0;
		}
#endif


		//Debug.dump(lat*1000, lon*1000, loc.x2home, loc.y2home);
		

		

#ifndef FASLE_GPS_STILL
		lat = FALSE_LAT + (long)(loc.from_X2Lat(Emu.get_x()));
		lon = FALSE_LON + (long)(loc.from_Y2Lon(Emu.get_y()));
#endif
		loc.lat_ = lat;
		loc.lon_ = lon;
		loc.updateXY();

			if (Log.writeTelemetry && Autopilot.motors_is_on()) {

				NAV_POSLLH posllh;
				posllh.lat = lat;
				posllh.lon = lon;
				Log.loadByte(LOG::GpS);
				Log.loadGPS_full(&posllh);
				Log.loadFloat((float)loc.x2home);
				Log.loadFloat((float)loc.y2home);
				Log.loadFloat((float)loc.dX);
				Log.loadFloat((float)loc.dY);
				Log.loadFloat((float)loc.speedX);
				Log.loadFloat((float)loc.speedY);
				Log.loadFloat((float)loc.accX);
				Log.loadFloat((float)loc.accY);

			}


		//loc.lat_ = lat;
		//loc.lon_ = lon;



		

		/*
		fullX += distX;
		fullY += distY;
		Out.fprintf(Debug.out_stream,fullX);
		Out.fprintf(Debug.out_stream," ");
		Out.println(fullY);
		*/

		//distX = distY = 0;
		//Out.println("---------");
		//Out.println( (loc.get_kd_lat()));
	//	Debug.dump(lat, lon, 0, 0);
		//Out.println(Location.get_kd_lat());
}

#else


uint64_t last_gps_time1 = 0;

void GPSClass::loop(){

	uint64_t ttt = micros();
	if (micros() - last_gps_time1 >= 33000) {
		last_gps_time1 = micros();
		if (loc.processGPS()) {
			loc.updateXY();
		}
		if ((last_gps_time1 > loc.last_gps_data_time) && (last_gps_time1 - loc.last_gps_data_time) > 1000000){//NO_GPS_TIME_TO_FALL) {
			fprintf(Debug.out_stream,"gps update error  %i\n",millis()/1000);
			loc.last_gps_data_time = micros();
			//Autopilot.control_falling(e_GPS_NO_UPDATE);
		}
	}
}
#endif

GPSClass GPS;

