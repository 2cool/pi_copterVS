#include "mpu_umulator.h"



#include "debug.h"
#include "Autopilot.h"


EmuClass::EmuClass()
{
}


EmuClass::~EmuClass()
{
}



void getWindForces(float wf[4][3]) {

}



const double f_speed_k[3] = { 0.033,0.033,0.24 };  //20  grad pri 10/m/s

const double f_gyro_k[3] = { 0.05,0.05,3.8 };

const double G = 9.8;


int rand_w_cnt;
enum{HMASK=15,MMASK=63,LMASK=1023};
float maxWind[3];
float rand_L[3], rand_M[3][4], rand_H[3][4];
float low[3], mid[3][4], high[3][4];

void init_wind(float x, float y, float z) {
	maxWind[X] = x;
	maxWind[Y] = y;
	maxWind[Z] = z;
	for (int i = 0; i < 3; i++) {
		rand_L[i] = (maxWind[i] * (float)(rand()) / (float)RAND_MAX);
		for (int j = 0; j < 4; j++) {
			rand_M[i][j] = (maxWind[i] * 0.25) - 0.5*(maxWind[i] * (float)(rand()) / (float)RAND_MAX);
			rand_H[i][j] = (maxWind[i] * 0.125) - 0.25*(maxWind[i] * (float)(rand()) / (float)RAND_MAX);
		}
	}
}



void get_wind(float w[][4]) {

	rand_w_cnt++;
	if (rand_w_cnt&MMASK == MMASK) {
		for (int i=0; i<3;i++)
			for (int j = 0; j < 4; j++) {
				rand_M[i][j]=(maxWind[i]*0.25) - 0.5*(maxWind[i] * (float)(rand()) / (float)RAND_MAX);
			}
	}
	if (rand_w_cnt&HMASK == HMASK) {
		for (int i = 0; i<3; i++)
			for (int j = 0; j < 4; j++) {
				rand_H[i][j] = (maxWind[i] * 0.125) - 0.25*(maxWind[i] * (float)(rand()) / (float)RAND_MAX);
			}
	}

	if (rand_w_cnt&LMASK == LMASK) {
		for (int i = 0; i<3; i++)
			rand_L[i] = (maxWind[i] *(float)(rand()) / (float)RAND_MAX);
	}


	for (int i = 0; i < 3; i++) {
		low[i] += (rand_L[i] - low[i])*0.001;
		for (int j = 0; j < 4; j++) {
			mid[i][j] += (rand_M[i][j] - mid[i][j])*0.01;
			high[i][j] += (rand_H[i][j] - high[i][j])*0.1;
			w[i][j] = low[i] + mid[i][j] + high[i][j];
		}
	}

}


float  EmuClass::get_pitch() { return (float)(pitch*RAD2GRAD); }
float  EmuClass::get_roll() { return (float)(roll*RAD2GRAD); }
float  EmuClass::get_yaw() { return (float)(yaw*RAD2GRAD); }
float  EmuClass::get_gyroYaw() { return (float)((gyro[YAW]* RAD2GRAD)); }
float  EmuClass::get_gyroPitch() { return (float)(gyro[PITCH]* RAD2GRAD); }
float  EmuClass::get_gyroRoll() { return (float)(gyro[ROLL]* RAD2GRAD); }
float  EmuClass::get_accX() { return (float)(acc[X]); }
float  EmuClass::get_accY() { return (float)(acc[Y]); }
float  EmuClass::get_accZ() {return (float)((pos[Z]<0.01?0:acc[Z]));}
float  EmuClass::get_alt() { return (float)pos[Z]; }
float  EmuClass::get_speedZ() { return (float)speed[Z]; }
float  EmuClass::get_x() { return (float)pos[X]; }
float  EmuClass::get_y() { return (float)pos[Y]; }

double wind_f[3][4];


void EmuClass::init(float wx, float wy, float wz,float y , float p , float r ) {
	init_wind(wx, wy, wz);
	pos[Z] = pos[Y] = pos[X] = 0;
	speed[X] = speed[Y] = speed[Z] = 0;
	acc[X] = acc[Y] = acc[Z] = -G;
	gyro[X] = gyro[Y] = gyro[Z] = 0;
	yaw = y;
	pitch = p;
	roll = r;

}

void EmuClass::update(float fm_[4], double dt) {
	if (Autopilot.motors_is_on() == false)
		return;




	


	const double MCF = 1;
	//fm_[0] = 0.6;
	//fm_[1] = 0.5;
	//fm_[2] = 0.6;
	//fm_[3] = 0.5;

	if (dt > 0.02)
		dt = 0.02;
	fm[0] += ((fm_[0] + fm_[1])*0.5 - fm[0])*MCF;
	fm[1] += ((fm_[1] + fm_[3])*0.5 - fm[1])*MCF;
	fm[2] += ((fm_[2] + fm_[0])*0.5 - fm[2])*MCF;
	fm[3] += ((fm_[3] + fm_[2])*0.5 - fm[3])*MCF;


	fmr[0] += (fm_[0] - fmr[0])*MCF;
	fmr[1] += (fm_[1] - fmr[1])*MCF;
	fmr[2] += (fm_[2] - fmr[2])*MCF;
	fmr[3] += (fm_[3] - fmr[3])*MCF;



	
	float wacc[3], wgyro[3];
	{
		float w[3][4];
		get_wind(w);

		float w03[3], w12[3];
		float rot03[3], rot12[3];
		for (int i = 0; i < 3; i++) {

			rot03[i] = (w[i][0] - w[i][3]);
			rot12[i] = (w[i][2] - w[i][1]);


			if (w[i][0] >= 0 && w[i][3] >= 0) {
				w03[i] = min(w[i][0], w[i][3]);
			}
			else if (w[i][0] < 0 && w[i][3] < 0) {
				w03[i] = max(w[i][0], w[i][3]);
			}
			else
				w03[i] = 0;
			if (w[i][1] >= 0 && w[i][2] >= 0) {
				w12[i] = min(w[i][1], w[i][2]);
			}
			else if (w[i][1] < 0 && w[i][2] < 0) {
				w12[i] = max(w[i][1], w[i][2]);
			}
			else
				w12[i] = 0;

			wacc[i] = w12[i] + w03[i];
			wacc[i] =  wacc[i] * wacc[i] * f_speed_k[i];


		}
		wgyro[Z] = rot03[Y] + rot12[Y] + rot03[X] + rot12[X];
		wgyro[X] = rot03[Z];
		wgyro[Y] = rot12[Z];

		wgyro[Z] = 10*wgyro[Z] * wgyro[Z];
		wgyro[X] = 1000*wgyro[X] * wgyro[X];
		wgyro[Y] = 1000*wgyro[Y] * wgyro[Y];

}
	















	gyro_res[YAW] = gyro[YAW] * abs(gyro[YAW])*f_gyro_k[Z];
	gyro_res[ROLL] = gyro[ROLL] * abs(gyro[ROLL])*f_gyro_k[X];
	gyro_res[PITCH] = gyro[PITCH] * abs(gyro[PITCH])*f_gyro_k[Y];



	const double arm03_force = fmin(fm[0], fm[3]);
	const double arm03_pitch_rotation = 5 * G*(fm[0] - fm[3]) - gyro_res[PITCH];
	const double arm03_yaw_rot = (fmr[0] + fmr[3]);
	const double arm21_force = fmin(fm[1], fm[2]);
	const double arm21_roll_rotation = 5 * G*(fm[2] - fm[1]) - gyro_res[ROLL];
	const double arm21_yaw_rot = (fmr[2] + fmr[1]);
	const double yaw_rot_force = 10*(arm21_yaw_rot - arm03_yaw_rot) - gyro_res[YAW];
	double force = G*(arm03_force + arm21_force);


	





	gyro[YAW] += (yaw_rot_force+ wgyro[Z])*dt;
	gyro[PITCH] += (arm03_pitch_rotation+ wgyro[Y])*dt;
	gyro[ROLL] += (arm21_roll_rotation+ wgyro[X])*dt;


	yaw += gyro[YAW] * dt;
	roll += gyro[ROLL] * dt;
	pitch += gyro[PITCH] * dt;


	if (yaw > 6.2831853)
		yaw -= 6.2831853;
	if (yaw < -6.2831853)
		yaw += 6.2831853;

	









	double m[3][3];

	const double cosA = cos(yaw);
	const double sinA = sin(yaw);
	const double cosB = cos(pitch);
	const double sinB = sin(pitch);
	const double cosY = cos(roll);
	const double sinY = sin(roll);

	m[0][0] = cosA*cosB; m[0][1] = cosA*sinB*sinY - sinA*cosY; m[0][2] = cosA*sinB*cosY + sinA*sinY;
	m[1][0] = sinA*cosB; m[1][1] = sinA*sinB*sinY + cosA*cosY; m[1][2] = sinA*sinB*cosY - cosA*sinY;
	m[2][0] = -sinB;	 m[2][1] = cosB*sinY;				   m[2][2] = cosB*cosY;

	double vec[] = { 0,0,1 };

	acc[X] = m[0][0] * vec[0] + m[0][1] * vec[1] + m[0][2] * vec[2];
	acc[Y] = m[1][0] * vec[0] + m[1][1] * vec[1] + m[1][2] * vec[2];
	acc[Z] = m[2][0] * vec[0] + m[2][1] * vec[1] + m[2][2] * vec[2];


	acc[X] = -(acc[X] * force) - resistenceF[X] + wacc[X];
	acc[Y] = -(acc[Y] * force) - resistenceF[Y] + wacc[Y];
	acc[Z] = (acc[Z] * force) - resistenceF[Z] - G + wacc[Z];


	//dobavit schum

	acc[Z]+= 2.5-5.0*((float)(rand()) / (float)RAND_MAX);
	acc[X] += 1.0 - 2.0*((float)(rand()) / (float)RAND_MAX);
	acc[Y] += 1.0 - 2.0*((float)(rand()) / (float)RAND_MAX);

	



	speed[X] += acc[X] * dt;
	speed[Y] += acc[Y] * dt;
	speed[Z] += acc[Z] * dt;





	resistenceF[X] = speed[X] * abs(speed[X]) * f_speed_k[X];
	resistenceF[Y] = speed[Y] * abs(speed[Y]) * f_speed_k[Y];
	resistenceF[Z] = speed[Z] * abs(speed[Z]) * f_speed_k[Z];



	pos[X] += speed[X] * dt;
	pos[Y] += speed[Y] * dt;
	pos[Z] += speed[Z] * dt;

	
	if (pos[Z] <= 0 ) {
		pos[Z] = pos[Y] = pos[X] = 0;
		speed[X] = speed[Y] = speed[Z] = 0;
		acc[X] = acc[Y] = acc[Z] = -G;
		gyro[X] = gyro[Y] = gyro[Z] = 0;
		yaw = 0;
		pitch = 0;
		roll =0;
		//Autopilot.motors_do_on(false, "S_S");
		
	}
	Debug.load(0, pos[X], pos[Y]);
	Debug.dump();

}

EmuClass Emu;