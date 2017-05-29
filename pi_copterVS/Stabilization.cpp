// 
// 
// 

#include "define.h"
#include "WProgram.h"
#include "Stabilization.h"
#include "AP_PID.h"
#include "mpu.h"
#include "MS5611.h"
#include "Autopilot.h"

#include "GPS.h"
#include "debug.h"
#include "Balance.h"
#include "Prog.h"

void StabilizationClass::init(){

	
	accXY_stabKP = 0.2f;
	accXY_stabKP_Rep = 1.0f / accXY_stabKP;
	set_acc_xy_speed_kp(5);
	set_acc_xy_speed_kI(1);
	//set_acc_xy_speed_kD(6); зачем вичислять ускорение если мі его итак заем.
	set_acc_xy_speed_imax(MAX_ANGLE);
	max_speed_xy = MAX_HOR_SPEED;


	XY_KF_DIST = 0.1f;  
	XY_KF_SPEED = 0.1f;
	
	
	//--------------------------------------------------------------------------
	last_accZ = 1;
	//Z_CF_DIST = 0.03;
	//Z_CF_SPEED = 0.005;



	Z_CF_DIST = 0.1f;//єкспиримент
	Z_CF_SPEED = 0.016f;


	accZ_stabKP = 0.5f;
	accZ_stabKP_Rep = 1.0f / accZ_stabKP;


	pids[ACCZ_SPEED].kP(0.1f);//?
	pids[ACCZ_SPEED].kI(0.05f);//?
	pids[ACCZ_SPEED].imax(0.3f);
	max_stab_z_P =  MAX_VER_SPEED_PLUS;
	max_stab_z_M = MAX_VER_SPEED_MINUS;

	sX=sY=sZ = 0;
	speedZ = speedX = speedY = 0;
	printf("stab init\n");

}
//bool flx = false, fly = false;





//33 max speed on pressureat 110000
const float air_drag_k = 9.8f / (float)(PRESSURE_AT_0 * 33 * 33);
float air_drag(const float speed){
	return abs(speed)*speed*MS5611.pressure*air_drag_k;
}
float air_drag_wind(const float a){
	float w=(float)sqrt(abs(a / (MS5611.pressure*air_drag_k)));
	return (a < 0) ? -w : w;
}
void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	max_speed_xy = MAX_HOR_SPEED;
	max_stab_z_P = MAX_VER_SPEED_PLUS;
	max_stab_z_M = MAX_VER_SPEED_MINUS;
}
void StabilizationClass::init_XY(const float sx, const float speedx, const float sy, const float speedy){

	sX = sx;
	sY = sy;
	speedX = speedx;
	speedY = speedy;
	resset_xy();
//	gps_sec = GPS.loc.mseconds;

}

int cnnnnn = 0;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::XY(float &pitch, float&roll){

//#ifdef FALSE_GPS
//	const float ax = -Mpu.accX;
//	const float ay = -Mpu.accY;
//#else
	const float ax = (-Mpu.cosYaw*Mpu.accX + Mpu.sinYaw*Mpu.accY);
	const float ay = (-Mpu.cosYaw*Mpu.accY - Mpu.sinYaw*Mpu.accX);
//#endif
	//--------------------------------------------------------prediction
	sX += Mpu.dt*(speedX + ax*Mpu.dt*0.5f);
	speedX += (ax*Mpu.dt);
	sY += Mpu.dt*(speedY + ay*Mpu.dt*0.5f);
	speedY += (ay*Mpu.dt);
	// -------------------------------------------------------corection



	sX += (GPS.loc.dX - sX)*XY_KF_DIST;
	speedX += (GPS.loc.speedX - speedX)*XY_KF_SPEED;
	//--------------------------------------------------------
	sY += (GPS.loc.dY - sY)*XY_KF_DIST;
	speedY += (GPS.loc.speedY - speedY)*XY_KF_SPEED;
	

	float stabX, stabY;
	if (Autopilot.progState() && Prog.intersactionFlag){
		stabX = Prog.stabX;
		stabY = Prog.stabY;
	}
	else{
		const float dist = (float)sqrt(sX*sX + sY*sY);
		const float max_speed = min(getSpeed_XY(dist),max_speed_xy);
		stabX = abs((GPS.loc.cosDirection)*max_speed);
		if (sX < 0)
			stabX *= -1.0f;
		stabY = abs((GPS.loc.sinDirection)*max_speed);
		if (sY < 0)
			stabY *= -1.0f;
	}
	const float glob_pitch = -pids[ACCX_SPEED].get_pid(stabX + speedX, Mpu.dt);
	
	const float glob_roll = pids[ACCY_SPEED].get_pid(stabY + speedY, Mpu.dt);

	//----------------------------------------------------------------преобр. в относительную систему координат
	pitch = Mpu.cosYaw*glob_pitch - Mpu.sinYaw*glob_roll;
	roll = Mpu.cosYaw*glob_roll + Mpu.sinYaw*glob_pitch;

	
	/*
	if ((cnnnnn & 3) == 0)	{
		Debug.load(0, (sX )*0.1, (sY)*0.1);
		//Debug.load(1, speedX, speedY);
		Debug.dump();
		
		
	}
	cnnnnn++;
	*/
	










	
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float old_altitude = 0;




void StabilizationClass::init_Z(){
	sZ = Autopilot.corectedAltitude();
	//speedZ = speedz;
	Stabilization.resset_z();

}
float deltaZ = 0;


int tttcnt = 0;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



float StabilizationClass::Z(){/////////////////////////////////////////////////////////////


	sZ += Mpu.dt*(speedZ + Mpu.accZ*Mpu.dt*0.5f);
	sZ += (Autopilot.corectedAltitude() - sZ)*Z_CF_DIST;

	speedZ += Mpu.accZ*Mpu.dt;
	speedZ += (MS5611.speed - speedZ)*Z_CF_SPEED;

	float stab = getSpeed_Z(Autopilot.flyAtAltitude - sZ);
	stab = constrain(stab, max_stab_z_M, max_stab_z_P);

	float fZ = HOVER_THROTHLE + pids[ACCZ_SPEED].get_pid(stab - speedZ, Mpu.dt)*Balance.powerK();
	//	if (++tttcnt == 3){
	//	tttcnt = 0;
		//Debug.load(0, speedZ,speedZf);// (MS5611.altitude - Autopilot.flyAtAltitude));
	//	Debug.load(0, Autopilot.flyAtAltitude - sZ, stab - speedZ);
	//Debug.dump();
		//Serial.println(pids[ACCZ_SPEED].get_integrator());
		
		//Debug.dump(fZ, sZ, Autopilot.flyAtAltitude, 0);
		//Out.println(fZ);
//	}

	return fZ;
	
}

void StabilizationClass::resset_z(){
	pids[ACCZ_SPEED].reset_I();
	pids[ACCZ_SPEED].set_integrator(max(HOVER_THROTHLE,Autopilot.get_throttle()) - HOVER_THROTHLE);
	
}
void StabilizationClass::resset_xy(){

	pids[ACCX_SPEED].reset_I();
	pids[ACCY_SPEED].reset_I();
}

string StabilizationClass::get_z_set(){

	ostringstream convert;
	convert<<\
	accZ_stabKP<<","<<pids[ACCZ_SPEED].kP()<<","<<\
	pids[ACCZ_SPEED].kI()<<","<<pids[ACCZ_SPEED].imax()<<","<<\
	max_stab_z_P<<","<<max_stab_z_M<<","<<\
	Z_CF_SPEED<<","<<Z_CF_DIST;
	string ret = convert.str();
	return string(ret);

}



void StabilizationClass::setZ(const float  *ar){


	int error = 1;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		
		error = 0;
		float t;
		uint8_t i = 0;

		error += Commander._set(ar[i++], accZ_stabKP);
		accZ_stabKP_Rep = 1.0f / accZ_stabKP;

		t = pids[ACCZ_SPEED].kP();
		if ((error += Commander._set(ar[i++],t))==0)
			pids[ACCZ_SPEED].kP(t);

		t = pids[ACCZ_SPEED].kI();
		if ((error += Commander._set(ar[i++], t))==0)
			pids[ACCZ_SPEED].kI(t);

		t = pids[ACCZ_SPEED].imax();
		if ((error += Commander._set(ar[i++], t))==0)
			pids[ACCZ_SPEED].imax(t);

		error += Commander._set(ar[i++], max_stab_z_P);
		error += Commander._set(ar[i++], max_stab_z_M);
		error += Commander._set(ar[i++], Z_CF_SPEED);
		error += Commander._set(ar[i], Z_CF_DIST);


		resset_z();
		printf("Stabilization Z set:\n");

		for (uint8_t ii = 0; ii < i; ii++){
			printf("%f,",ar[ii]);
		}
		printf("%f\n",ar[i]);
	}
	if (error>0){
		printf("Stab Z set Error\n");
	}
}


string StabilizationClass::get_xy_set(){
	ostringstream convert;
	convert<<\
	accXY_stabKP<<","<<pids[ACCX_SPEED].kP()<<","<<\
	pids[ACCX_SPEED].kI()<<","<<pids[ACCX_SPEED].imax()<<","<<\
	max_speed_xy<<","<<XY_KF_SPEED<<","<<XY_KF_DIST;
	string ret = convert.str();
	return string(ret);
}

void StabilizationClass::setXY(const float  *ar){

	int error = 1;

	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){

		error = 0;
		float t;
		uint8_t i = 0;

		error += Commander._set(ar[i++], accXY_stabKP);
		accXY_stabKP_Rep = 1.0f / accXY_stabKP;

		t = pids[ACCX_SPEED].kP();
		if ((error += Commander._set(ar[i++], t))==0)
			set_acc_xy_speed_kp(t);

		t = pids[ACCX_SPEED].kI();
		if ((error += Commander._set(ar[i++], t))==0)
			set_acc_xy_speed_kI(t);

		t = pids[ACCX_SPEED].imax();
		if ((error += Commander._set(ar[i++], t))==0)
			set_acc_xy_speed_imax(t);

		error += Commander._set(ar[i++], max_speed_xy);
		error += Commander._set(ar[i++], XY_KF_SPEED);
		error += Commander._set(ar[i++], XY_KF_DIST);


		resset_xy();
		printf("Stabilization XY set:\n");
		for (uint8_t ii = 0; ii < i; ii++){
			printf("%f,",ar[ii]);
		}
		printf("%f\n",ar[i]);
	}
	if (error>0)
	{
		printf("Stab XY set Error\n");
	}
}












StabilizationClass Stabilization;
