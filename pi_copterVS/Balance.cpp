
#include "Balance.h"
#include "MPU6050.h"
#include "define.h"
#include "Stabilization.h"
#include "Telemetry.h"
#include "debug.h"
#include "GPS.h"
#include "Log.h"
#include "mpu.h"

void correct(float & f){
	if (f < 0)
		f = 0;
	else if (f>1)
		f = 1;
}




/*



���� �������� ������ � ������� �� 0.11

63 �����

2.24-3.01 �� 108.088 �����������

*/




/*

0         1

+

2    +    3

*/






float MAX_ANGLE__ = 35;

float old[2];
float force[2];
float balanceForce[2];

float dTime[2];
float nTime[2];
#define MIN_SPEED 0.2f
float const predTime = 0.009f;



//float dt = 0.009f;

long oldtttttttttttt = 0;
int cntttttttttt = 0;

//bool tempp1 = false;
//uint32_t taim0;
static const float f_constrain(const float v, const float min, const float max){
	return constrain(v, min, max);
}

void BalanceClass::init()
{
//f/speed^2/0.5=cS;
	//speed^2*0.5*cS=f
	//speed = sqrt(2f / cS)
	//cS = 0.00536;//15 ���� 
//	0.00357

	f_[0] = f_[1] = f_[2] = f_[3] = 0;
	fprintf(Debug.out_stream,"BALANCE INIT\n");
	
	c_pitch = c_roll = 0;
	

	Stabilization.init();
	throttle = MIN_THROTTLE_;
	_max_angle_= MAX_ANGLE_;


	pitch_roll_stabKP = 2;
	

	//pitch_roll_rateKP = 0.0007;
	//pitch_roll_rateKI = 0.001;
	//pitch_roll_rateIMAX = 0.05;
	

	pids[PID_PITCH_RATE].kP(0.0014);
	pids[PID_PITCH_RATE].kI(0.002);
	pids[PID_PITCH_RATE].imax(0.2);

	pids[PID_ROLL_RATE].kP(0.0014);
	pids[PID_ROLL_RATE].kI(0.002);
	pids[PID_ROLL_RATE].imax(0.2);

	yaw_stabKP = 2;

	pids[PID_YAW_RATE].kP(0.0017f);
	pids[PID_YAW_RATE].kI(0.0017f);
	pids[PID_YAW_RATE].imax(0.1);

	Mpu.init();
	Hmc.init();
	Hmc.loop();
	Mpu.initYaw(Hmc.heading*RAD2GRAD);
	mid_powerK = 1;
	power_K = 1;
#ifdef DEBUG_MODE
	fprintf(Debug.out_stream, "Heading :%i\n", (int)Hmc.get_headingGrad());
	
#endif

	
}


string BalanceClass::get_set(){
	
	ostringstream convert;
	convert << \
		pids[PID_PITCH_RATE].kP() << "," << \
		pids[PID_PITCH_RATE].kI() << "," << \
		pids[PID_PITCH_RATE].imax() << "," << \
		pitch_roll_stabKP << "," << \
		pids[PID_YAW_RATE].kP() << "," << \
		pids[PID_YAW_RATE].kI() << "," << \
		pids[PID_YAW_RATE].imax() << "," << \
		yaw_stabKP << "," << \
		_max_angle_ << "," << \
		power_K << ",";
	string ret = convert.str();
	return string(ret);
}



void BalanceClass::set(const float *ar){
	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		int error=0;
		
		float t;

		t = pids[PID_PITCH_RATE].kP();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_PITCH_RATE].kP(t);
			pids[PID_ROLL_RATE].kP(t);
		}
		t = pids[PID_PITCH_RATE].kI();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_PITCH_RATE].kI(t);
			pids[PID_ROLL_RATE].kI(t);
		}
		t = pids[PID_PITCH_RATE].imax();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_PITCH_RATE].imax(t);
			pids[PID_ROLL_RATE].imax(t);
		}

		error += Commander._set(ar[i++], pitch_roll_stabKP);

		t = pids[PID_YAW_RATE].kP();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_YAW_RATE].kP(t);
		}
		t = pids[PID_YAW_RATE].kI();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_YAW_RATE].kI(t);
		}
		t = pids[PID_YAW_RATE].imax();
		if ((error += Commander._set(ar[i++], t))==0){
			pids[PID_YAW_RATE].imax(t);
		}
		t = yaw_stabKP;
		if ((error += Commander._set(ar[i++], t)) == 0) {
			yaw_stabKP = t;
		}
		t = _max_angle_;
		if ((error += Commander._set(ar[i++], t)) == 0) {
			_max_angle_ = constrain(t,15,35);
		}

		t = power_K;
		if ((error += Commander._set(ar[i++], t))==0){

			power_K=constrain(t,1,1.2);
		}
		error += Commander._set(ar[i++], yaw_stabKP);

		error += Commander._set(ar[i], _max_angle_);

	//	error += Commander._set(ar[i], stop_throttle);

		fprintf(Debug.out_stream,"balance set:\n");
		
		if (error == 0){
			//for (ii = 0; ii < i; ii++){
			//	Out.fprintf(Debug.out_stream,ar[ii]); Out.fprintf(Debug.out_stream,",");
			//}
			//Out.println(ar[ii]);
			fprintf(Debug.out_stream,"OK\n");
		}
		else{
			fprintf(Debug.out_stream,"ERROR to big or small. P=%i",error);
		}
	}
	else{
		fprintf(Debug.out_stream,"ERROR\n");
	}
}

float BalanceClass::powerK(){
	mid_powerK +=(Telemetry.powerK*MS5611.powerK- mid_powerK)*0.001;
	return mid_powerK;
}
/////////////////////////////////////////////////////////////////////////////////////////
int motors_off_i = 0;
void BalanceClass::escCalibration() {
	if (motors_off_i == 0 && Telemetry.power_is_on() == false) {
		throttle = f_[0] = f_[1] = f_[2] = f_[3] = 1;
		fprintf(Debug.out_stream,"!!!max power!!!\n");
		motors_off_i++;
	}else
		if (motors_off_i == 1 && Telemetry.power_is_on()) {
			throttle = f_[0] = f_[1] = f_[2] = f_[3] = 0;
			fprintf(Debug.out_stream,"!!!off power!!!\n");
			motors_off_i++;
		}
}



#define MAX_ANGLE_SPEED 360
#define MAX_YAW_SPEED 60
//#define MAX_POWER_K_IF_MAX_ANGLE_30 1.12



uint64_t hmc_last_time = 0;

bool BalanceClass::loop()
{
	
	if (!Mpu.loop()) {
		MS5611.loop();
		if (micros() - hmc_last_time > 10000) {
			hmc_last_time = micros();
			Hmc.loop();
		}
		GPS.loop();
		return false;
	}
	else {
		if (Autopilot.motors_is_on()) { 
			const float pK = powerK();
			const float min_throttle = constrain(MIN_THROTTLE_*pK*power_K, MIN_THROTTLE_,0.47);
			const float max_throttle = constrain(MAX_THROTTLE_*pK*power_K, MAX_THROTTLE_,0.9);

			maxAngle = _max_angle_;
			if (Autopilot.z_stabState()) {
				throttle = pK*power_K*Stabilization.Z(false);
				throttle = constrain(throttle, min_throttle, max_throttle);

				const float thr = throttle / Mpu.tiltPower;
				if (thr > max_throttle) {
					maxAngle = constrain(RAD2GRAD*acos(throttle/ max_throttle),MIN_ANGLE, _max_angle_);
					throttle = max_throttle;
				}
				else {
					throttle = thr;
				}
			}
			else {

				Stabilization.Z(true);
				throttle = Autopilot.get_throttle();
				throttle /= Mpu.tiltPower;
				throttle = constrain(throttle, 0.3f, max_throttle);
				//	Debug.load(0, throttle, f_[0]);

			}

			if (Autopilot.xy_stabState()) {
				Stabilization.XY(c_pitch, c_roll,false);
			}
			else {
				Stabilization.XY(c_pitch, c_roll, true);
				c_pitch = Autopilot.get_Pitch();
				c_roll = Autopilot.get_Roll();
			}

			c_pitch = constrain(c_pitch, -maxAngle, maxAngle);
			c_roll = constrain(c_roll, -maxAngle, maxAngle);
			const float maxAngle07 = maxAngle*0.7f;
			if (abs(c_pitch) > maxAngle07 || abs(c_roll) > maxAngle07) {
				//	c_pitch = constrain(c_pitch, -maxAngle, maxAngle);
				//c_roll = constrain(c_roll, -maxAngle, maxAngle);
				float k = (float)(RAD2GRAD*acos(cos(c_pitch*GRAD2RAD)*cos(c_roll*GRAD2RAD)));
				if (k == 0)
					k = maxAngle;
				if (k > maxAngle) {
					k = maxAngle / k;
					c_pitch *= k;
					c_roll *= k;
				}
			}

		

#define BCF 0.1

			float pitch_stab_output = f_constrain(pitch_roll_stabKP*(wrap_180(Mpu.get_pitch() - c_pitch)), -MAX_ANGLE_SPEED, MAX_ANGLE_SPEED);
			float roll_stab_output = f_constrain(pitch_roll_stabKP*(wrap_180(Mpu.get_roll() - c_roll)), -MAX_ANGLE_SPEED, MAX_ANGLE_SPEED);
			float yaw_stab_output = f_constrain(yaw_stabKP*wrap_180(-Autopilot.get_yaw() - Mpu.get_yaw()), -MAX_YAW_SPEED, MAX_YAW_SPEED);

			// rate PIDS

			const float max_delta = 0.5;// (throttle < 0.6f) ? 0.3f : MAX_DELTA;

			float pitch_output = pK*pids[PID_PITCH_RATE].get_pid(pitch_stab_output + Mpu.gyroPitch, Mpu.dt);
			pitch_output = constrain(pitch_output, -max_delta, max_delta);
			float roll_output = pK*pids[PID_ROLL_RATE].get_pid(roll_stab_output + Mpu.gyroRoll, Mpu.dt);
			roll_output = constrain(roll_output, -max_delta, max_delta);
			float yaw_output = pK*pids[PID_YAW_RATE].get_pid(yaw_stab_output - Mpu.gyroYaw, Mpu.dt);
			yaw_output = constrain(yaw_output, -0.1f, 0.1f);

			float m_yaw_output = -yaw_output;  //���������������� ��� ������ �������� �� �����
			if ((throttle + yaw_output) < min_throttle)
				yaw_output = min_throttle - throttle;//???????????????????????????????????????????????????
			if ((throttle + m_yaw_output) < min_throttle)
				m_yaw_output = min_throttle - throttle;

			
			f_[3] = f_constrain((throttle + roll_output + pitch_output + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[1] = f_constrain((throttle + roll_output - pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[2] = f_constrain((throttle - roll_output + pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[0] = f_constrain((throttle - roll_output - pitch_output + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);

			//������� ����� ��������� � ������� �� 1.2 ��� ������ � �������.

			if (Log.writeTelemetry) {
				Log.loadByte(LOG::BAL);
				Log.loadFloat(0);
				Log.loadFloat(0);
				Log.loadFloat(c_pitch);
				Log.loadFloat(c_roll);
				Log.loadFloat(throttle);
				Log.write_bank_cnt();
				Log.loadMem((uint8_t*)f_, 16);
			}

			if (Hmc.compas_motors_calibr) {
				f_[0] = 0;
				f_[1] = 0;
				f_[2] = 0;
				f_[3] = 0;
				f_[Hmc.motor_index] = 0.5;
			}
#ifndef FALSE_WIRE
			else {
				if (Autopilot.starts_cnt_after_powers_on == 1) {
				//	if (acos(Mpu.cosPitch*Mpu.cosRoll)>0.122) {
						//Autopilot.motors_do_on(false, e_ESK_ERROR);
				//	}
				//	else {
						f_[0] = MIN_THROTTLE_;
						f_[1] = MIN_THROTTLE_;
						f_[2] = MIN_THROTTLE_;
						f_[3] = MIN_THROTTLE_;
						throttle = MIN_THROTTLE_;
				//	}
				}
			}
#endif

		}
		else
		{
			pids[PID_PITCH_RATE].reset_I();
			pids[PID_ROLL_RATE].reset_I();
			pids[PID_YAW_RATE].reset_I();
			c_pitch = c_roll = 0;

			if (Debug.escCalibr>0)
				escCalibration();

		}
		if (Log.writeTelemetry)
			Log.end();            //               Nepishetca poslednie secundy loga?..  filtruemye dannye proverat na oshibku


#ifdef MOTORS_OFF
		Pwm.throttle(0, 0, 0, 0);  //670 micros
#else
		Pwm.throttle(f_[0], f_[1], f_[2], f_[3]);  //670 micros
#endif


#ifdef FALSE_WIRE
		Emu.update(f_, Mpu.dt);
#endif
		




	}
	return true;
}



void BalanceClass::set_off_th_() { 
	f_[0] = f_[1] = f_[2] = f_[3] = 0; 
	throttle = 0; 
#ifdef FALSE_WIRE
	Emu.update(f_, Mpu.dt);
#endif
	Pwm.throttle(f_[0], f_[1], f_[2], f_[3]);
}









BalanceClass Balance;

