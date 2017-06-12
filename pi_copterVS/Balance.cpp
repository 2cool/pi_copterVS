
/*
можность для взлета при низком заряде уже равна не 0.5 а 0.6   - Зделанно
Также коєфециенти балансировки возможно надо менять при уменьшении напряжения на батарее   - Зделанно


упал при смене коєфециентов стабилизации. когда поменя комп фильтри на 0.1 и 0.1 можность на двигатели стала -1 и квадрик упал. проверять что идет на двигатель. действительное ли єто число   - Зделанно

на андроиде показівает хуйню пока еще домашнии координати неопределени


*/







#include "Balance.h"
#include "MPU6050.h"
#include "define.h"
#include "Stabilization.h"
#include "Telemetry.h"
#include "debug.h"
#include "LED.h"
#include "GPS.h"
void correct(float & f){
	if (f < 0)
		f = 0;
	else if (f>1)
		f = 1;
}




/*



если скорость радиан в секунду то 0.11

63 грама

2.24-3.01 за 108.088 милесекунды






*/




/*

0         1

+

2    +    3

*/








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
	f_[0] = f_[1] = f_[2] = f_[3] = 0;
	printf("BALANCE INIT\n");
	
	c_pitch = c_roll = 0;
	

	Stabilization.init();
	throttle = MIN_THROTTLE_;
	maxAngle = MAX_ANGLE;


	pitch_roll_stabKP = 1.5;
	

	//pitch_roll_rateKP = 0.0007;
	//pitch_roll_rateKI = 0.001;
	//pitch_roll_rateIMAX = 0.05;
	

	pids[PID_PITCH_RATE].kP(0.0014f);
	pids[PID_PITCH_RATE].kI(0.0001f);
	pids[PID_PITCH_RATE].imax(MAX_DELTA*0.1f);

	pids[PID_ROLL_RATE].kP(0.0014f);
	pids[PID_ROLL_RATE].kI(0.0001f);
	pids[PID_ROLL_RATE].imax(MAX_DELTA*0.1f);

	yaw_stabKP = 2;

	pids[PID_YAW_RATE].kP(0.0017f);
	pids[PID_YAW_RATE].kI(0.0017f);
	pids[PID_YAW_RATE].imax(MAX_YAW_DELTA);
	


	Mpu.init();
	Hmc.init();
	Hmc.loop();
	Mpu.initYaw(Hmc.heading);
#ifdef DEBUG_MODE
	Out.print("Heading :"); Out.println(Hmc.headingGrad);
#endif

	
}


string BalanceClass::get_set(){
	
	ostringstream convert;
	convert<<\
	pids[PID_PITCH_RATE].kP()<<","<<\
	pids[PID_PITCH_RATE].kI()<<","<<\
	pids[PID_PITCH_RATE].imax()<<","<<\
	pitch_roll_stabKP<<","<<\
	pids[PID_YAW_RATE].kP()<<","<<\
	pids[PID_YAW_RATE].kI()<<","<<\
	pids[PID_YAW_RATE].imax()<<","<<\
	yaw_stabKP<<","<<maxAngle;
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

		error += Commander._set(ar[i++], yaw_stabKP);

		
		error += Commander._set(ar[i], maxAngle);
	
		

	//	error += Commander._set(ar[i], stop_throttle);

		printf("balance set:\n");
		
		if (error == 0){
			//for (ii = 0; ii < i; ii++){
			//	Out.print(ar[ii]); Out.print(",");
			//}
			//Out.println(ar[ii]);
			printf("OK\n");
		}
		else{
			printf("ERROR to big or small. P=%i",error);
		}
	}
	else{
		printf("ERROR\n");
	}
}





float BalanceClass::powerK(){
	float pk = Telemetry.powerK*MS5611.powerK;
	
	return pk;
}
/////////////////////////////////////////////////////////////////////////////////////////
int motors_off_i = 0;
void BalanceClass::escCalibration() {
	if (motors_off_i == 0 && Telemetry.power_is_on() == false) {
		throttle = f_[0] = f_[1] = f_[2] = f_[3] = 1;
		printf("!!!max power!!!\n");
		motors_off_i++;
	}else
		if (motors_off_i == 1 && Telemetry.power_is_on()) {
			throttle = f_[0] = f_[1] = f_[2] = f_[3] = 0;
			printf("!!!off power!!!\n");
			motors_off_i++;
		}
}

void BalanceClass::setMaxAngle(const float ang){
	maxAngle = ang;
	
}

#define MAX_ANGLE_SPEED 300
#define MAX_YAW_SPEED 60
//#define MAX_POWER_K_IF_MAX_ANGLE_30 1.12



uint64_t hmc_last_time = 0;
bool BalanceClass::loop()
{
	if (!Mpu.loop()) {
		//usleep(1000);
		MS5611.loop();
		//usleep(1000);
		if (micros() - hmc_last_time > 10000) {
			hmc_last_time = micros();
			Hmc.loop();
		}
		//usleep(1000);
		//GPS.loop();
		//usleep(3000);
		return false;
	}
	else {

		// Do the magic
		if (Autopilot.motors_is_on()) {  // Throttle raised, turn on stablisation.

				// Stablise PIDS

			float pK = powerK();
			float min_throttle = pK*MIN_THROTTLE_;

			if (Autopilot.z_stabState()) {
				throttle = Stabilization.Z();
				throttle = constrain(throttle, min_throttle, MAX_THROTTLE_);

				const float thr = throttle / Mpu.tiltPower;
				if (thr > MAX_THROTTLE_) {
					const float angle = RAD2GRAD * (float)acos(throttle / MAX_THROTTLE_);
					if (angle < MIN_ANGLE) {
						setMaxAngle(MIN_ANGLE);
						throttle = COS_MIN_ANGLE*MAX_THROTTLE_;
					}
					else {
						setMaxAngle(angle);
						throttle = MAX_THROTTLE_;
					}


				}
				else {
					throttle = thr;
					setMaxAngle(MAX_ANGLE);
				}
			}
			else {
				throttle = Autopilot.get_throttle();
				throttle /= Mpu.tiltPower;
				throttle = constrain(throttle, 0.3f, MAX_THROTTLE_);
				//	Debug.load(0, throttle, f_[0]);
				if (throttle < 0.3f) {
					// reset yaw target so we maintain this on takeoff
					// reset PID integrals whilst on the ground
					pids[PID_PITCH_RATE].reset_I();
					pids[PID_ROLL_RATE].reset_I();
					pids[PID_YAW_RATE].reset_I();
				}

			}












			if (Autopilot.xy_stabState()) {
				Stabilization.XY(c_pitch, c_roll);
			}
			else {
				c_pitch = Autopilot.get_Pitch();
				c_roll = Autopilot.get_Roll();
			}


			const float maxPa = Mpu.get_pitch_max_a();
			c_pitch = constrain(c_pitch, -maxPa, maxPa);
			const float maxRa = Mpu.get_roll_max_a();
			c_roll = constrain(c_roll, -maxRa, maxRa);


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

			float pitch_stab_output = f_constrain(pitch_roll_stabKP*(Mpu.get_pitch() - c_pitch), -MAX_ANGLE_SPEED, MAX_ANGLE_SPEED);
			float roll_stab_output = f_constrain(pitch_roll_stabKP*(Mpu.get_roll() - c_roll), -MAX_ANGLE_SPEED, MAX_ANGLE_SPEED);
			float yaw_stab_output = f_constrain(yaw_stabKP*wrap_180(-Autopilot.get_yaw() - Mpu.yaw), -MAX_YAW_SPEED, MAX_YAW_SPEED);
			//ErrorLog.println(wrap_180(-Autopilot.get_Yaw() - Mpu.yaw));

		//	Out.print(-Autopilot.get_Yaw()); Out.print(" "); Out.println(Mpu.yaw);
			//Out.println(wrap_180(Autopilot.get_Yaw() - Mpu.yaw));
			//	Out.print("\t"); 
			//	Out.println(yaw_stab_output);

			// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)


			//Out.print(pitch_stab_output); Out.print("\t"); Out.println(roll_stab_output);


			// rate PIDS

			const float max_delta = (throttle < 0.6f) ? 0.3f : MAX_DELTA;



			float pitch_output = pK * pids[PID_PITCH_RATE].get_pid(pitch_stab_output + Mpu.gyroPitch, Mpu.dt);
			pitch_output = constrain(pitch_output, -max_delta, max_delta);
			float roll_output = pK * pids[PID_ROLL_RATE].get_pid(roll_stab_output + Mpu.gyroRoll, Mpu.dt);
			roll_output = constrain(roll_output, -max_delta, max_delta);
			float yaw_output = pK * pids[PID_YAW_RATE].get_pid(yaw_stab_output - Mpu.gyroYaw, Mpu.dt);
			yaw_output = constrain(yaw_output, -0.1f, 0.1f);


			float m_yaw_output = -yaw_output;  //антираскачивание при низкой мощности на плече
			if ((throttle + yaw_output) < min_throttle)
				yaw_output = min_throttle - throttle;
			if ((throttle + m_yaw_output) < min_throttle)
				m_yaw_output = min_throttle - throttle;


			f_[3] = f_constrain((throttle + roll_output + pitch_output + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[1] = f_constrain((throttle + roll_output - pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[2] = f_constrain((throttle - roll_output + pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[0] = f_constrain((throttle - roll_output - pitch_output + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);

			if (Hmc.compas_motors_calibr) {
				f_[0] = 0;
				f_[1] = 0;
				f_[2] = 0;
				f_[3] = 0;
				LED.light(Hmc.motor_index);
				LED.prog_index = LED.MOT_OFF_P;

				f_[Hmc.motor_index] = 0.5;

			}


			//1-1
			//2-4
			//3-2
			//4-3

		}
		else
		{
			LED.prog_index = LED.MOT_OFF_P;
			c_pitch = c_roll = 0;

			//	Pwm.throttle(0, 0, 0, 0);
				//throttle = 0;

			if (false)
				escCalibration();

		}




		LED.loop();
		//	int tt = micros();



#ifdef MOTORS_OFF
		Pwm.throttle(0, 0, 0, 0);  //670 micros
#else

		Pwm.throttle(f_[0], f_[1], f_[2], f_[3]);  //670 micros
		//Pwm.throttle(f_[0], 0, 0, f_[3]);  //670 micros

	//	Debug.load(0, f_[0], f_[3]);
	//	Debug.dump();

		//Pwm.throttle(throttle, throttle, throttle, throttle);  //400 micros

#endif
//	Debug.load(1, f_[0], f_[3]);
//	Debug.load(0, f_[1], f_[2]);
//	Debug.dump();
	//Pwm.gimagl_pitch(-40);


//	int ttt = micros() - tt;
//	ttt = tt;
	/*
	int tttttttttttttt = millis();

	if (tttttttttttttt>oldtttttttttttt){
		oldtttttttttttt = tttttttttttttt + 100;

		switch (cntttttttttt & 3)
		{

		case 0:
			Out.print(f_[0]); Out.print("\t");
			break;
		case 1:
			Out.println(f_[1]);
			break;
		case 2:
			Out.print(f_[2]); Out.print("\t");
			break;

		default:
			Out.println(f_[3]); Out.println(" ");
			break;


		}
		cntttttttttt++;
	}
	*/
	}
	return true;
}













BalanceClass Balance;

