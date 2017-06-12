#include "mpu.h"
#include "define.h"
#include "Settings.h"
#include "Hmc.h"
#include "Autopilot.h"
#include "Telemetry.h"

#include "debug.h"
#include "Stabilization.h"
#include "GPS.h"
#include "LED.h"
//#include "Mem.h"


#define delay_ms(a)    usleep(a*1000)


inline void sin_cos(const float a, float &s, float &c) {
	s = (float)sin(a);
	const float ss = s*s;
	c = (float)sqrt(1 - min(1.0f, ss));
	c = c;
}



#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//2g
//	0.00006103515625
//4g
//#define K4ACCELERATION 0.0001220703125
#define K4ACCELERATION 0.00006103515625
//3G

static const float f_constrain(const float v, const float min, const float max){
	return constrain(v, min, max);
}
#define DIM 3
//WiFiClass wi_fi;


void  MpuClass::initYaw(const float angle){
	float add_2_yaw = angle;
}


int MpuClass::ms_open() {
	dmpReady = 1;
	initialized = 0;
	for (int i = 0; i<DIM; i++) {
		lastval[i] = 10;
	}

	// initialize device
	printf("Initializing MPU...\n");
	if (mpu_init(NULL) != 0) {
		printf("MPU init failed!\n");
		return -1;
	}
	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
		printf("Failed to set sensors!\n");
		return -1;
	}
	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(2000) != 0) {
		printf("Failed to set gyro sensitivity!\n");
		return -1;
	}
	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(2) != 0) {
		printf("Failed to set accel sensitivity!\n");
		return -1;
	}
	// verify connection
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n", devStatus);

	//fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
		printf("Failed to initialize MPU fifo!\n");
		return -1;
	}

	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware() != 0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1) != 0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	//dmp_set_orientation()
	//if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0) {
		printf("Failed to enable DMP features!\n");
		return -1;
	}


	printf("Setting DMP fifo rate...\n");
	if (dmp_set_fifo_rate(rate) != 0) {
		printf("Failed to set dmp fifo rate!\n");
		return -1;
	}
	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo() != 0) {
		printf("Failed to reset fifo!\n");
		return -1;
	}

	printf("Checking... ");
	do {
		delay_ms(1000 / rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
		r = dmp_read_fifo(g, a, _q, &sensors, &fifoCount);
	} while (r != 0 || fifoCount<5); //packtets!!!
	printf("Done.\n");

	initialized = 1;
	return 0;
}
void MpuClass::init()
{
	acc_callibr_time = 0;
	rate = 100;
	yaw = add_2_yaw = 0;
	//f/speed^2/0.5=cS;
	//speed^2*0.5*cS=f
	//speed = sqrt(2f / cS)
	//cS = 0.00536;//15 град 
//	0.00357
	cS = (float)tan(NEED_ANGLE_4_SPEED_10_MS * GRAD2RAD)*0.02f;
	speedX = speedY;
	max_g_cnt = 0;
	cosYaw = 1;
	sinYaw = 0;
	temp_deb = 6;
	fx = fy = fz = 0;
	upsidedown = false;
	faccX = faccY = faccZ = 0;
	oldmpuTime = micros();
	yaw = pitch = roll = gyroPitch = gyroRoll = gyroYaw = accX = accY = accZ = 0;
	sinPitch = sinRoll = 0;
	tiltPower = cosPitch = cosRoll = 1;
	roll_max_angle = pitch_max_angle = MAX_ANGLE;
	//COMP_FILTR = 0;// 0.003;

	printf("Initializing MPU6050\n");

#ifndef FALSE_MPU



	ms_open();
/*
	accelgyro.initialize();
	printf("Testing device connections...\n");
	if (accelgyro.testConnection())
	{
		printf("MPU6050 connection successful\n");
	}
	else
	{
		printf("MPU6050 connection failed\n");
		delay(10000);
	}

	accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);
	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_188);//
	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_98);
*/
#ifdef GYRO_CALIBR
	gyro_calibratioan = false;
#else
	gyro_calibratioan = true;
#endif
	int16_t offset_[6];
	mpu_calibrated = Settings.readMpuSettings(offset_);
	gyro_calibratioan &= mpu_calibrated;

	if (mpu_calibrated) {
		/*accelgyro.setXAccelOffset(offset_[ax_offset]);
		accelgyro.setYAccelOffset(offset_[ay_offset]);
		accelgyro.setZAccelOffset(offset_[az_offset]);
		accelgyro.setXGyroOffset(offset_[gx_offset]);
		accelgyro.setYGyroOffset(offset_[gy_offset]);
		accelgyro.setZGyroOffset(offset_[gz_offset]);*/
#ifdef DEBUG_MODE
		for (int i = 0; i < 6; i++)
			Out.println(offset_[i]);
#endif
#ifndef WORK_WITH_WIFI
		//calibrated = false;
#endif
	}
	else {
		printf("MPU NOT CALIBRATED !!!\n");
	}




#else
	mpu_calibrated = gyro_calibratioan = true;
	fspeedX = fspeedY = fdistX = fdistY = 0;
	altitude_Z = speed_Z = 0;

	wind_x = wind_y=0;

#endif

}

string MpuClass::get_set(){
	float ang = atan(50*cS)*RAD2GRAD;
	
	ostringstream convert;
	convert<<
	ang<<",";
	
	string ret = convert.str();
	return string(ret);
	
}

void MpuClass::set(const float  *ar){

	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		int error = 0;

		float t;

		//f/speed^2/0.5=cS;
		//speed^2*0.5*cS=f
		//speed = sqrt(2f / cS)
		//cS = 0.00536;//15 град

		const float new_cS = (float)(tan(ar[i++] * GRAD2RAD)*0.02f);
		//Serial.println(new_cS);
		t = cS;
		if ((error += Commander._set(new_cS, t)) == 0)
			cS = t;

		printf("mpu set:\n");
		//int ii;
		if (error == 0){
			//for (ii = 0; ii < i; ii++){
			//	Out.print(ar[ii]); Out.print(",");
			//}
			//Out.println(ar[ii]);
			printf("OK\n");
		}
		else{
			printf("ERROR to big or small. P=%i\n",error);
		}
	}
	else{
		printf("ERROR\n");
	}
}

int16_t MpuClass::getGX(){
	int16_t x, y, z;
	accelgyro.getAcceleration(&x, &y, &z);
	return x;
}

const float n003 = 0.030517578f;
const float n006 =  0.061035156f;

const float n604 = 0.00006103515625f;
const float to_98g = 0.0005981445312f;

#ifdef FALSE_MPU


#include "Balance.h"


const float air_drag_kk = 9.8 * 0.00536 / (PRESSURE_AT_0);
float air_draggf(const float speed){
	return 0.5*abs(speed)*speed*MS5611.pressure*air_drag_kk;
}


float th_stat = 0;
float MpuClass::getZA(){

	float dt = Mpu.dt;
	float _throttle = (Autopilot.motors_is_on()) ? Balance.get_throttle() : 0;
	th_stat += (_throttle - th_stat)*0.2;

	float wind_a = air_draggf(speed_Z);
	float prK = MS5611.pressure / PRESSURE_AT_0;
	float a = -9.8 + 9.8*th_stat / Telemetry.powerK * (1.0 / HOVER_THROTHLE) * Mpu.tiltPower*prK - wind_a;

	//powerrr.input(a);
	//Debug.dump(throttle, a, 0, 0);
	//powerrr.input(9.8*(throttle*Mpu.tiltPower -0.5));
	//Out.println(Mpu.tiltPower);
	//a = powerrr.output();

	//	Out.println(a);
	altitude_Z += speed_Z*dt + a*dt*dt*0.5;
	if (altitude_Z <= 0){
		altitude_Z = speed_Z = 0;
		//powerrr.setup(2);
		return 0;
	}
	//graphic(0, altitude_Z*0.01, throttle);
	speed_Z += a*dt;
	//Out.println(a);
	//Debug.load(0, altitude_Z, speed_Z);
	//Debug.dump();
	return a;


}



float air_dragg(const float speed){
	return 0.5*abs(speed)*speed*MS5611.pressure*air_drag_kk;
}


long nextTime = 0;
float max_wind_speed;
#define MAX_X_WIND_SPEED 0
#define MAX_Y_WIND_SPEED 0


float MpuClass::windX(){
	if (millis() > nextTime){
		nextTime = millis() + (long)(1000.0*(1.0 + 5.0*((float)rand()) / RAND_MAX));
		max_wind_speed = MAX_X_WIND_SPEED*((float)rand()) / RAND_MAX;
	}
	wind_x += (max_wind_speed - wind_x)*0.03;
	return wind_x;

}
float MpuClass::windY(){
	if (millis() > nextTime){
		nextTime = millis() + (long)(1000.0*(1.0 + 5.0*((float)rand()) / RAND_MAX));
		max_wind_speed = MAX_Y_WIND_SPEED*((float)rand()) / RAND_MAX;
	}
	wind_y += (max_wind_speed - wind_y)*0.03;
	return wind_y;

}
void MpuClass::getFalse(float &accx, float &accy){
	if (altitude_Z == 0)
		fdistX = fdistY = fspeedX = fspeedY = accx = accy = 0;
	else{
		float pitch = sin(-get_pitch()*GRAD2RAD)*9.8 / cosPitch;
		float roll = sin(get_roll()*GRAD2RAD)*9.8 / cosRoll;
		float cosYaw = cos(yaw*GRAD2RAD);
		float sinYaw = sin(yaw*GRAD2RAD);


		//Debug.load(0, wind()/17, 0);
		//Debug.dump();
		//	Out.println(air_dragg(speedX + windX));
		accx = (cosYaw*pitch - sinYaw*roll) - air_dragg(fspeedX + windX());
		accy = (sinYaw*pitch + cosYaw*roll) - air_dragg(fspeedY + windY());
		fdistX += accx*dt*dt*0.5 + fspeedX*dt;
		fdistY += accy*dt*dt*0.5 + fspeedY*dt;
		fspeedX += accx*dt;
		fspeedY += accy*dt;
	}
	//Debug.dump(fspeedX, fspeedY, fdistX, fdistY);
	//Debug.load(0, fdistX*0.1, fdistY*0.1);
	//	Debug.dump();
}



float fpitch, froll, fyaw;
bool flagggggg = true;
float oldpitch = 0, oldRoll = 0, oldyaw = 0;
///////////////////////////////////////////////////////////////////

void MpuClass::loop(){
	uint32_t mputime = micros();
	dt = (float)(mputime - oldmpuTime)*0.000001;// *div;
	rdt = 1.0 / dt;
	oldmpuTime = mputime;

	if (flagggggg){
		flagggggg = false;
		pitch = 0;
		roll = 0;
		yaw = 0;
	}
	float ttddfdf = 0.01;
	pitch += (Balance.c_pitch - pitch)*ttddfdf;
	roll += (Balance.c_roll - roll)*ttddfdf;
	yaw += (Hmc.get_headingGrad() - yaw)*0.05;

	gyroRoll = (roll - oldRoll) * 100;
	oldRoll = roll;
	gyroPitch = (pitch - oldpitch) * 100;
	oldpitch = pitch;

	gyroYaw = (yaw - oldyaw) * 100;
	oldyaw = yaw;

	tiltPower = (cosPitch = cos(pitch*GRAD2RAD))*(cosRoll = cos(roll*GRAD2RAD));
	accZ = getZA();
	getFalse(accX, accY);
	cosYaw = cos(Mpu.yaw*GRAD2RAD);
	sinYaw = sin(Mpu.yaw*GRAD2RAD);
	delay(9);
}

#else


/////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
	v->x = 2 * (q->x*q->z - q->w*q->y);
	v->y = 2 * (q->w*q->x + q->y*q->z);
	v->z = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
	return 0;
}



float c_cosPitch = 1, c_sinPitch = 0, c_cosRoll = 1, c_sinRoll = 0, c_tiltPower=0, c_pitch=0, c_roll=0;
void MpuClass::set_max_angle(float cx,float cy) {
	roll_max_angle = pitch_max_angle = MAX_ANGLE;
	if (Autopilot.motors_is_on()) {
		//спедд икс и игрик они же недолжны вращатся вместе с const float _ax = cosYaw*GPS.loc.aX + sinYaw*GPS.loc.aY;
#ifndef MOTORS_OFF
		float rspeedX = cosYaw*speedX - sinYaw*speedY;
		float rspeedY = cosYaw*speedY + sinYaw*speedX;


		float break_fx = 0.5f*abs(rspeedX)*rspeedX*(cS + cS*abs(c_sinPitch));
		float total_ax = c_sinPitch / c_cosPitch - break_fx;
		rspeedX = 9.8f*total_ax*dt;
		total_ax *= c_cosPitch;


		float break_fy = 0.5f*abs(rspeedY)*rspeedY*(cS + cS*abs(c_sinRoll));
		float total_ay = c_sinRoll / c_cosRoll - break_fy;
		rspeedY = 9.8f*total_ay*dt;
		total_ay *= c_cosRoll;


		speedX += (cosYaw*rspeedX + sinYaw*rspeedY);
		speedY += (cosYaw*rspeedY - sinYaw*rspeedX);


		cx += total_ax;
		cy -= total_ay;

#define CF 0.007f
		const float aRoll = atan2(cy, c_tiltPower) * RAD2GRAD;
		const float aPitch = atan(cx / sqrt(cy * cy + c_tiltPower * c_tiltPower)) * RAD2GRAD;
		c_roll += gyroRoll*dt;
		c_pitch += gyroPitch*dt;
		c_roll -= (aRoll + c_roll)*CF;
		c_pitch += (aPitch - c_pitch)*CF;
		sin_cos(c_pitch*GRAD2RAD, c_sinPitch, c_cosPitch);
		sin_cos(c_roll*GRAD2RAD, c_sinRoll, c_cosRoll);

		c_tiltPower = c_cosPitch*c_cosRoll;
		c_tiltPower = constrain(c_tiltPower, 0.5, 1);

		float pk = abs(c_pitch-pitch);
		pk = constrain(pk, 0, MAX_ANGLE);
		pitch_max_angle -= pk;

		float rk = abs(c_roll - roll);
		rk = constrain(rk, 0, MAX_ANGLE);
		roll_max_angle -= rk;
#endif

	}
	else {
		speedY = speedX = 0;
		c_cosPitch = c_cosRoll =1; c_sinPitch = c_sinRoll = c_tiltPower = c_pitch = c_roll = 0;
	}
	//Debug.load(0, roll_max_angle / MAX_ANGLE, pitch_max_angle / MAX_ANGLE);
}


#define ROLL_COMPENSATION_IN_YAW_ROTTATION 0.02
#define PITCH_COMPENSATION_IN_YAW_ROTTATION 0.025

float ac_accX = 0, ac_accY = 0, ac_accZ = -0.3664f;


bool MpuClass::loop(){//-------------------------------------------------L O O P-------------------------------------------------------------

	uint64_t mputime = micros();

	//dmp
	if (dmp_read_fifo(g, a, _q, &sensors, &fifoCount) != 0) //gyro and accel can be null because of being disabled in the efeatures
		return false;
		
	dt = (float)(mputime - oldmpuTime)*0.000001f;// *div;
	rdt = 1.0f / dt;
	oldmpuTime = mputime;

	q = _q;

	//GetGravity();
	gravity.x = 2 * (q.x*q.z - q.w*q.y);
	gravity.y = 2 * (q.w*q.x + q.y*q.z);
	gravity.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;


	if ((abs(a[0]) > MAX_G || abs(a[1]) > MAX_G || abs(a[2]) > MAX_G) && ++max_g_cnt>20) {
		Autopilot.off_throttle(true, e_MAX_ACCELERATION);
		max_g_cnt = 0;
	}
	

	gyroPitch = -n006*(float)g[1];  //in grad
	gyroYaw = -n006*(float)g[2];
	gyroRoll = n006*(float)g[0];

//	GetYawPitchRoll();
//	float gyro_yaw = RAD2GRAD*2.0f*atan2(2.0 * q.x*q.y - 2.0 * q.w*q.z, 2.0 * q.w*q.w + 2.0 * q.x*q.x - 1);
	float head = Hmc.heading*RAD2GRAD;


	yaw += gyroYaw*dt;
	float t = head - yaw;

	if (t > 180)
		yaw += 360;
	else if (t < -180)
		yaw -= 360;
	yaw += (head- yaw)*0.0031f;

	pitch = atan(gravity.x / sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
	roll = atan(gravity.y / sqrt(gravity.x*gravity.x + gravity.z*gravity.z));

	float x = n604*(float)a[0];
	float y = -n604*(float)a[1];  //
	float z = n604*(float)a[2];
	
	sin_cos(pitch, sinPitch, cosPitch);
	sin_cos(roll, sinRoll, cosRoll);

	tiltPower = cosPitch*cosRoll;
	tiltPower = constrain(tiltPower, 0.5f, 1);

	accZ = z*cosPitch + sinPitch*x;
	accZ = 9.8f*(accZ*cosRoll - sinRoll*y - 1)-ac_accZ;

	accX = 9.8f*(x*cosPitch - z*sinPitch)-ac_accX;
	accY = 9.8f*(y*cosRoll + z*sinRoll)-ac_accY;

	if (acc_callibr_time > mputime) {
		ac_accZ += accZ*0.01;
		ac_accY += accY*0.01;
		ac_accX += accX*0.01;
	}
	

	//cosYaw = (float)cos(yaw*GRAD2RAD);
	//sinYaw = (float)sin(yaw*GRAD2RAD);
	sin_cos(yaw*GRAD2RAD, sinYaw, cosYaw);

	//yaw *= RAD2GRAD;
	pitch *= RAD2GRAD;
	roll *= RAD2GRAD;

	set_max_angle(x, y);
	//float pk = pitch / c_pitch;
	//float rk = roll / c_roll;


	
//	Debug.load(0, rdt, dt);
//	Debug.load(1, gyro_yaw / 180, add_2_yaw / 180);
//	Debug.load(2, gyro_yaw / 180, head / 180);
/*	Debug.load(1, pitch / 40, ttPitch / 40);
	Debug.load(2, yaw / 180, ttYaw / 180);


	Debug.load(3, yaw / 180, Hmc.heading/PI);
	Debug.load(4, accX*0.1, accY*0.1);
	Debug.load(5, accZ*0.05, 0);
	//Debug.load(6, pitch / 90, accX/M_PI_2);
	//Debug.load(7, roll / 90, -accY / M_PI_2);
*/	//Debug.load(6, roll / 90, -accY / M_PI_2);
	//Debug.dump();
	return true;
}

#endif


void MpuClass::setDLPFMode_(uint8_t bandwidth){
	gLPF = bandwidth;
	accelgyro.setDLPFMode(bandwidth);
}

bool MpuClass::selfTest(){


	int16_t xa = 0, ya = 0, za = 0;
	int16_t xr = 0, yr = 0, zr = 0;
	uint8_t trys = 0;
	bool ok;
	do {
		int count = 0;
		int errors = 0;

		while (++count < 10){
			int16_t xt, yt, zt;
			accelgyro.getAcceleration(&xt, &yt, &zt);
			errors += (xt == xa || yt == ya || zt == za);
			xa = xt;
			ya = yt;
			za = zt;
			accelgyro.getRotation(&xt, &yt, &zt);

			printf("%i %i %i\n", xt, yt, zt);
	
			errors += (xt == xr || yt == yr || zt == zr || abs(xt) > 10 || abs(yt) > 10 || abs(zt) > 10);
			xr = xt;
			yr = yt;
			zr = zt;
			delay(10);
		}

		ok = errors <= 7;
		if (ok == false){
			printf("ERROR\n");
			accelgyro.setXAccelOffset(1354);
			accelgyro.setYAccelOffset(451);
			accelgyro.setZAccelOffset(1886);
			accelgyro.setXGyroOffset(101);
			accelgyro.setYGyroOffset(-32);
			accelgyro.setZGyroOffset(9);
		}
		else
			return ok;
	} while (++trys < 2);

	return ok;


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MpuClass::new_calibration(const bool onlyGyro){
	LED.off();
	acc_callibr_time = micros()+(uint64_t)10000000;
	gyro_calibratioan = true;
}


MpuClass Mpu;



