// 
// 
// 

//тайминг виставлять от начала полета. типа висеть тут 60 сек от начала. тоесть елс иприлетел в 59 сек то висеть сек.  ???
//добавить слежение за точкой.


#include "Prog.h"
#include "GPS.h"
#include "Autopilot.h"
#include "Stabilization.h"
#include "debug.h"
#include "Telemetry.h"
#include "LED.h"
enum { LAT_LON = 1, DIRECTION = 2, ALTITUDE = 4,  CAMERA_ANGLE = 8, TIMER = 16,SPEED_XY=32,SPEED_Z=64,LED_CONTROL=128};


#define MAX_HA 3
#define MAX_VA 1

void ProgClass::init(){

	intersactionFlag = false;
	speed_X = speed_Y = speed_Z = 0;
	steps_count = 0;
	step_index=0;
	prog_steps_count_must_be = 0;
	prog_data_index = 0;
	prog_data_size = 0;
}



#define MIN_DT 0.01

//камеоа млєеь сдежиь за точкой в пространстве. которую ей надо передать
//все делать через таймер. через время за которое надо єто зделать.




void ProgClass::loop(){

	float dt = 0.001f*(float)(millis() - begin_time);


	float rdt = dt - old_dt;
	if (rdt < MIN_DT)
		return;

	old_dt = dt;

	if (go_next == false) {
		if (timer == 0) {
			if (altFlag == false)
				altFlag = (alt == old_alt) || (abs(MS5611.altitude() - Autopilot.flyAtAltitude) <= (ACCURACY_Z));

			if (distFlag == false) {
				if (lat == old_lat && lon == old_lon) {
					distFlag = true;
				}
				else {
					const float advance_dist = Stabilization.getDist_XY(max_speed_xy);//*1.1
					const float acur = max(max(ACCURACY_XY, GPS.loc.accuracy_hor_pos), advance_dist);
					distFlag = sqrt(GPS.loc.dX*GPS.loc.dX + GPS.loc.dY*GPS.loc.dY) <= acur;
				}
			}
			go_next = altFlag & distFlag;
		}
		else {
			if (timer <= dt)
				go_next = true;

		}
	}
	if (go_next){
		go_next = distFlag = altFlag = false;
		if (load_next() == false){
			//if (Autopilot.lost_conection_time == 0){
				printf("PROG END\n");
				Autopilot.start_stop_program(false);
			//}
		}
	}
	intersactionFlag = Prog.getIntersection(stabX, stabY);
}


#define MAX_TIME_LONG_FLIGHT  1200
bool ProgClass::program_is_OK(){
	if (prog_data_size >= 14){// && prog_steps_count_must_be == steps_count){
		prog_data_index = 0;
		time4step2done = 0;
		old_dt = 0;
		begin_time = 0;
		lat = GPS.loc.lat_;
		lon = GPS.loc.lon_;
		alt = MS5611.altitude();
		uint8_t step = 1;
		float fullTime = 0;
		while (load_next()){
			
			if (lat != old_lat && lon != old_lon){
				const float dx = GPS.loc.from_lat2X((float)(lat - old_lat));
				const float dy = GPS.loc.form_lon2Y((float)(lon - old_lon));
				float time = (float)(sqrt(dx*dx + dy*dy) / max_speed_xy);
				const float dAlt = alt - old_alt;
				time += dAlt / ((dAlt >= 0) ? Stabilization.max_stab_z_P : Stabilization.max_stab_z_M);
				time *= 1.25f;
				if (timer > 0){
					if (timer < time){
						printf("time error in step: %i\n",step);
						return false;
					}
					else
						time = timer;
				}
				fullTime += time;
				if (fullTime>MAX_TIME_LONG_FLIGHT){
					printf("to long fly for prog!\n");
					return false;
				}
			old_lat = lat;
			old_lon = lon;
			}
			step++;

			old_alt = alt;

		}


		const float x2 = GPS.loc.from_lat2X((float)(lat - GPS.loc.lat_));
		const float y2 = GPS.loc.form_lon2Y((float)(lon - GPS.loc.lon_));

		const float dist = (float)sqrt(x2*x2 + y2*y2);


		if (dist >= 20 || alt  >= 20){
			printf("end poitn to far from star!!!\n");
			return false;
		}

		printf("time for flyghy: %i",(int)fullTime);
		return true;
	}
else
	return false;


}





bool ProgClass::start(){
	if (program_is_OK()){
		step_index = 0;
		prog_data_index = 0;
		time4step2done = 0;
		old_dt = 0;
		begin_time = 0;
		lat = GPS.loc.lat_;
		lon = GPS.loc.lon_;
		alt = MS5611.altitude();
		go_next = distFlag = altFlag = true;
		return true;
	}
	else{
		clear();
		printf("no program\n");
	}
	return false;

}



float pDistance(float x, float y, float x1, float y1, float x2, float y2) {

	float A = x - x1;
	float B = y - y1;
	float C = x2 - x1;
	float D = y2 - y1;

	float dot = A * C + B * D;
	float len_sq = C * C + D * D;
	float param = -1;
	if (len_sq != 0) //in case of 0 length line
		param = dot / len_sq;

	float xx, yy;

	if (param < 0) {
		xx = x1;
		yy = y1;
	}
	else if (param > 1) {
		xx = x2;
		yy = y2;
	}
	else {
		xx = x1 + param * C;
		yy = y1 + param * D;

		printf("x=%f\n",xx);
		printf("y=%f\n",yy);
	}

	float dx = x - xx;
	float dy = y - yy;
	return (float)sqrt(dx * dx + dy * dy);
}



float sgn(const float x){ return (x < 0) ? -1 : 1; }

bool ProgClass::getIntersection(float &x, float &y){
	if (lat == old_lat && lon == old_lon){
		//ErrorLog.println("len=0");
		return false;
	}

	float ks = 1;
	const float dist_x = Stabilization.getDistX();
	const float dist_y = Stabilization.getDistY();
	const float dist_ = (float)sqrt(dist_x*dist_x + dist_y*dist_y);

	//speed Заменить на реальную. а то иначе...
	float r = Stabilization.getDist_XY(max_speed_xy);
	if (r > dist_){
		//ErrorLog.println("r>dist");
		return false;
	}
	//----------------------------

	const float x2 = GPS.loc.from_lat2X((float)(lat - GPS.loc.lat_));
	const float x1 = GPS.loc.from_lat2X((float)(old_lat - GPS.loc.lat_));
	const float y2 = GPS.loc.form_lon2Y((float)(lon - GPS.loc.lon_));
	const float y1 = GPS.loc.form_lon2Y((float)(old_lon - GPS.loc.lon_));
	const float dx = x2 - x1;
	const float dy = y2 - y1;
	const float l2 = dx*dx + dy*dy;
	//const float dr = sqrt(l2);
	const float D = x1*y2 - x2*y1;

	float discriminant = (r*r*l2) - (D*D);
	if (discriminant <= 0){
		//ErrorLog.println("dis<0");
    // нахождение от точки до прямой.
		{
			const float dot = -x1 * dx - y1 * dy;
			float param = -1;
			if (l2 == 0) //in case of 0 length line
				return false;

			param = dot / l2;
			float xx, yy;

			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * dx;
				yy = y1 + param * dy;
			}
			float dist2line;


			if ((x2 - xx)*(xx - x1)<0 || (y2 - yy)*(yy - y1)<0){//точка не на линии
				//ErrorLog.println("not on line");
				float dx = x2 - xx;
				float dy = y2 - yy;
				const float dist2 = dx*dx+dy*dy;
				dx = x1 - xx;
				dy = y1 - yy;
				const float dist1 = dx*dx + dy*dy;
				dist2line = (float)(1.0 + sqrt(min(dist2, dist1)));
			}else
				dist2line = (float)(1.0 + sqrt(xx * xx + yy * yy));

			if (dist2line > r){
#ifdef FALL_IF_STRONG_WIND
				if (dist2line > MAX_DIST_ERROR_TO_FALL){
					Autopilot.off_throttle(false, e_TOO_STRONG_WIND);
					return true;
				}
#endif
				ks = r / dist2line;
			}
			r = dist2line;
			discriminant = (r*r*l2) - (D*D);
		}
	//---------------------------------
	}
	if (discriminant <= 0)
		return false;
	discriminant = (float)sqrt(discriminant);
	const float rdr2 = 1.0f/l2;
	float temp = sgn(dy)*dx*discriminant;
	const float ix1 = (D*dy + temp)*rdr2;
	const float ix2 = (D*dy - temp)*rdr2;
	temp = abs(dy)*discriminant;
	const float iy1 = (-D*dx + temp)*rdr2;
	const float iy2 = (-D*dx - temp)*rdr2;

	float tx = x2 - ix1;
	float ty = y2 - iy1;
	const float dist1 = tx*tx + ty*ty;

	tx = x2 - ix2;
	ty = y2 - iy2;
	const float dist2 = tx*tx + ty*ty;

	if (dist1<dist2){
		x = ix1;
		y = iy1;
	}
	else{
		x = ix2;
		y = iy2;
	}
	
	//------------------------------------------
	x = Stabilization.getSpeed_XY(x*ks);
	y = Stabilization.getSpeed_XY(y*ks);
	//Debug.load(0, x / 30, y / 30);
	//Debug.dump();
	return true;
}


void ProgClass::clear(){
	init();
}

bool ProgClass::load_next(){
	old_lat = lat;
	old_lon = lon;
	old_alt = alt;
	if (prog_data_index >= prog_data_size || steps_count <= step_index){
		return false;
	}
	step_index++;
	//Out.println("next");

	int wi = prog_data_index+1;

	if (prog[prog_data_index] & TIMER){
		timer = prog[wi++];
		//time4step2done = (float)gd.timer;
		//r_time = 1.0 / time4step2done;
		//Out.println(timer);
	}

#define K01 0.1f
	if (prog[prog_data_index] & SPEED_XY){
		//Stabilization.max_speed_xy = K01*(float)prog[wi++];
		max_speed_xy = K01*(float)prog[wi++];
		if (max_speed_xy < 1)
			max_speed_xy = 1;
		//Out.println(max_speed_xy);
	}

	if (prog[prog_data_index] & SPEED_Z){
		const float speedZ = K01* (float)prog[wi++];
		
		if (speedZ >= 0){
			Stabilization.max_stab_z_P = max(speedZ,0.15f);
			Stabilization.max_stab_z_M = MAX_VER_SPEED_MINUS;
		}
		else {
			Stabilization.max_stab_z_P = MAX_VER_SPEED_PLUS;
			Stabilization.max_stab_z_M = speedZ;

		}

		//Out.println(speedZ);
	}

	if (prog[prog_data_index] & LAT_LON){
		byte*lb = (byte*)&lat;
		lb[0] = prog[wi++];
		lb[1] = prog[wi++];
		lb[2] = prog[wi++];
		lb[3] = prog[wi++];
		lb = (byte*)&lon;
		lb[0] = prog[wi++];
		lb[1] = prog[wi++];
		lb[2] = prog[wi++];
		lb[3] = prog[wi++];

		/*advance_dist = Stabilization.getDist_XY(max_speed_xy);
		const float dX = lat - GPS.loc.lat_;
		const float dY = lon - GPS.loc.lon_;
		const float distX = GPS.loc.from_X2Lat(dX);
		const float distY = GPS.loc.from_Y2Lon(dY);
		float track_dist = sqrt(distX*distX + distY*distY);
		if (track_dist < (advance_dist * 4)){
			advance_dist = track_dist*0.25;
		}*/
		GPS.loc.setNeedLoc(lat, lon);
		//Out.println(lat); Out.println(lon);  Out.println(advance_dist);
	}


	if (prog[prog_data_index] & DIRECTION){
		oldDir = 1.4173228f*(float)prog[wi++];
		Autopilot.setYaw(-oldDir);
	}

	if (prog[prog_data_index] & ALTITUDE){
		int16_t ialt;
		byte*lb = (byte*)&ialt;
		lb[0] = prog[wi++];
		lb[1] = prog[wi++];
		alt = ialt;
		Autopilot.set_new_altitude(alt);
	}


	if (prog[prog_data_index] & CAMERA_ANGLE){
		old_cam_angle = -1.4173228f*(float)prog[wi++];
		Pwm.gimagl_pitch(old_cam_angle);
	}

	if (prog[prog_data_index] & LED_CONTROL){
		LED.prog_index = prog[wi++];
	}


	//==============================================================
	
	
	
	
	prog_data_index = wi;

	//==============================================================

	begin_time = millis();

	old_dt = 0;

	stabX = stabY = 0;
	return true;

}




bool ProgClass::add(byte*buf)
{
	
	uint16_t pi = prog_data_size;

	uint8_t i = 1;
	
	prog[pi++] = buf[0];


	//Out.println(buf[0]);
	

	
	if (steps_count != buf[i++]){
		clear();
		printf("PROG INDEX ERROR\n");
		return false;
	}
	if (i + 17 > PROG_MEMORY_SIZE){
		clear();
		printf("PROG_MEMORY_OVERFLOW\n");
		return false;
	}
//	Out.print("mask:"); Out.println(buf[0]);
	
	if (buf[0] & TIMER){
		prog[pi++] = buf[i++];
	//	Out.print("timer="); Out.println(timer);
	}

	if (buf[0] & SPEED_XY){
		prog[pi++] = buf[i++];
	//	Out.println("speed="); Out.println(K01*speedXY);
	}
	if (buf[0] & SPEED_Z){
		prog[pi++] = buf[i++];
	//	Out.println("speedZ="); Out.println(K01*speedZ);
	}

	if (buf[0] & LAT_LON){

		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];

		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
	}
	

	if (buf[0] & DIRECTION){
		prog[pi++] = buf[i++];
	}
	

	if (buf[0] & ALTITUDE){

		prog[pi++] = buf[i++];
		prog[pi++] = buf[i++];
	}

	if (buf[0] & CAMERA_ANGLE){
		prog[pi++] = buf[i++];

	}

	if (buf[0] & LED_CONTROL){
		prog[pi++] = buf[i++];
		printf("led prog=%i\n",buf[i-1]);
	}

	if (steps_count == 0){
		byte*lb = (byte*)&prog_steps_count_must_be;
		lb[0] = buf[i++];
		lb[1] = buf[i++];
		//Out.print("prog steps="); Out.println(prog_steps_count_must_be);
	}
	
	

	prog_data_size = pi;
	steps_count++;
	printf("%i dot added! %i\n", steps_count, prog_data_size); 
	return true;
}

ProgClass Prog;

