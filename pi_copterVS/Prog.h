// Prog.h

#ifndef _PROG_h
#define _PROG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "define.h"
#include "GeoDot.h"

/*

lat,
lon,
speed,//positive - speed ; negative delay in seconds;
direction

*/

class ProgClass
{
#define PROG_MEMORY_SIZE 20000
 protected:
	 bool go_next, distFlag, altFlag;
	 //float preX, preY;
	// float advance_dist;
	 int32_t old_lon, old_lat, lon, lat;
	 uint8_t oldTimer;
	 float old_alt,alt, oldDir, old_cam_angle;
	 uint16_t prog_data_size,prog_data_index;
	 uint16_t prog_steps_count_must_be;
	 uint8_t step_index,steps_count;
	 byte prog[PROG_MEMORY_SIZE];
	 float speed_X, speed_Y, speed_Z;
	 bool program_is_OK();
	 float r_time,time4step2done,speed_corected_delta, old_dt;
	 uint32_t begin_time;
	 float max_speed_xy,timer;
	 float max_stab_z_P, max_stab_z_M;
 public:
	 bool intersactionFlag;
	 float stabX, stabY;
	bool getIntersection(float &distX, float &distY);
	float altitude, direction, cam_pitch;
	 void init();
	// GeoDotClass gd;
	 bool add(byte*buf);
	 bool start();
	 bool load_next(bool);
	
	 void clear();
	 void loop();

	 void init(string buf){

	 }
};

extern ProgClass Prog;

#endif

