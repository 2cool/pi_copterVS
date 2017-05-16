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

//WiFiClass wi_fi;

void MpuClass::init()
{
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

	printf("Initializing MPU6050\n");
#ifndef FALSE_WIRE


	accelgyro.initialize();


	//accelgyro.setI2CMasterModeEnabled(false);
	//accelgyro.setI2CBypassEnabled(true);
	//accelgyro.setSleepEnabled(false);
#endif
#ifndef FALSE_MPU
	//accelgyro.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	//accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	//accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);


	// verify connection
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

	// Set Gyro Low Pass Filter(0..6, 0=fastest, 6=slowest)

	//	accelgyro.setDLPFMode(gLPF);


	accelgyro.setDLPFMode(MPU6050_DLPF_BW_256);

	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_188);///

	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_98);

	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
	//accelgyro.setDLPFMode(gLPF = MPU6050_DLPF_BW_20);
	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);
	//accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);



#ifdef GYRO_CALIBR
	gyro_calibratioan = false;
#else
	gyro_calibratioan = true;
#endif
	int16_t offset_[6];
	mpu_calibrated = Settings.readMpuSettings(offset_);
	gyro_calibratioan &= mpu_calibrated;

	if (mpu_calibrated) {
		accelgyro.setXAccelOffset(offset_[ax_offset]);
		accelgyro.setYAccelOffset(offset_[ay_offset]);
		accelgyro.setZAccelOffset(offset_[az_offset]);
		accelgyro.setXGyroOffset(offset_[gx_offset]);
		accelgyro.setYGyroOffset(offset_[gy_offset]);
		accelgyro.setZGyroOffset(offset_[gz_offset]);
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


	oldmpuTime = micros();

	yaw = pitch = roll = gyroPitch = gyroRoll = gyroYaw = accX = accY = accZ = 0;
	sinPitch = sinRoll = 0;
	tiltPower = cosPitch = cosRoll = 1;
	//COMP_FILTR = 0;// 0.003;
	addStep = 0.001f;
	





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
const float n604 = 0.00006103515625f;

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
	
}

#else


/////////////////////////////////////////////////////////////////////////////////////////////////////

#define ROLL_COMPENSATION_IN_YAW_ROTTATION 0.02
#define PITCH_COMPENSATION_IN_YAW_ROTTATION 0.025


void MpuClass::loop(){//-------------------------------------------------L O O P-------------------------------------------------------------


	int zzz = micros();

//	if (calibrated == false){

		//	new_calibration();
		//re_calibration();
//		calibrated = true;
//	}
	uint32_t mputime = micros();
	dt = (float)(mputime - oldmpuTime)*0.000001f;// *div;
	rdt = 1.0f / dt;
	oldmpuTime = mputime;

	float x, y, z;
	{
		int16_t ax, ay, az, gx, gy, gz;
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifndef SESOR_UPSIDE_DOWN
		ay = -ay;
		az = -az;
		gy = -gy;
		gz = -gz;
#endif
		upsidedown = az>0;


#ifdef ON_MAX_G_MOTORS_OFF
		if ((abs(ax) > MAX_G || abs(ay) > MAX_G || abs(az) > MAX_G) && ++max_g_cnt>2){
			Autopilot.off_throttle(true, e_MAX_ACCELERATION);
			max_g_cnt = 0;
		}
#endif
		//iz na minus


		x = n604*(float)ax;
		y = n604*(float)ay;
		z = -n604*(float)az;



		gyroRoll = ((float)gx)*n003;
		gyroPitch = ((float)gy)*n003;
		gyroYaw = ((float)gz)*n003;

	}

	const float CF = 0.01f;

	yaw += gyroYaw*dt;
	float t = Hmc.headingGrad - yaw;
	if (t > 180)
		yaw += 360;
	else if (t < -180)
		yaw -= 360;




	//yaw = (1 - ACC_F_FREC)*yaw + ACC_F_FREC * Hmc.headingGrad;
	yaw += (Hmc.headingGrad - yaw)*CF;

	//yaw = Hmc.headingGrad;

	cosYaw = (float)cos(yaw*GRAD2RAD);
	sinYaw = (float)sin(yaw*GRAD2RAD);

	roll += gyroRoll*dt;


	pitch += gyroPitch*dt;


/*
	если квадр летит вперед(опуская нос) то sinpitch ортицательный, x - отрицательный.а при ускорении квадра x - уходит в положительную сторону.квадр какбы задирается
	если квадр летит вправо, (наклоняясь по часовой к нам жопой) sinroll положительный, y - отрицательный, а при ускорении квадра н уходит в положит сторону, квадр какбы задирается против часовой
*/

	float cx = x, cy = y;
	if (Autopilot.motors_is_on()){
		//спедд икс и игрик они же недолжны вращатся вместе с const float _ax = cosYaw*GPS.loc.aX + sinYaw*GPS.loc.aY;

		float rspeedX = cosYaw*speedX - sinYaw*speedY;
		float rspeedY = cosYaw*speedY + sinYaw*speedX;


		float break_fx = 0.5f*abs(rspeedX)*rspeedX*(cS+cS*abs(sinPitch));
		float total_ax = sinPitch / cosPitch - break_fx;
		rspeedX = 9.8f*total_ax*dt;
		total_ax *= cosPitch;


		float break_fy = 0.5f*abs(rspeedY)*rspeedY*(cS+cS*abs(sinRoll));
		float total_ay = sinRoll / cosRoll - break_fy;
		rspeedY = 9.8f*total_ay*dt;
		total_ay *= cosRoll;


		speedX += (cosYaw*rspeedX + sinYaw*rspeedY);
		speedY += (cosYaw*rspeedY - sinYaw*rspeedX);

#ifndef MOTORS_OFF
		//cx += total_ax;
		//cy -= total_ay;
#endif

	}
	else{
		speedY = speedX = 0;
	}






#ifdef RESTRICT_PITCH // Eq. 25 and 26
	const float aRoll = (float)atan2(cy, tiltPower) * RAD2GRAD;
	const float aPitch = (float)atan(cx / sqrt(cy * cy + tiltPower * tiltPower)) * RAD2GRAD;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif


	//float aRoll = RAD2GRAD*atan(y / sqrt(x*x + z*z));
	//float aPitch = RAD2GRAD*atan(x / sqrt(y*y + z*z));





	roll -= (aRoll + roll)*CF;
	pitch += (aPitch - pitch)*CF;


	cosPitch = (float)cos(pitch*GRAD2RAD);
	sinPitch = (float)sin(pitch*GRAD2RAD);
	cosRoll = (float)cos(roll*GRAD2RAD);
	sinRoll = (float)sin(roll*GRAD2RAD);

	




	tiltPower = cosPitch*cosRoll;
	tiltPower = constrain(tiltPower, 0.5f, 1);


	//float _accZ = 9.8*(z - tiltPower) * tiltPower;
	//float _accX = 9.8*((x - sinPitch)* tiltPower);
	//float _accY = 9.8*((y + sinRoll)* tiltPower);
	
	//pitch L+ a+
	//Roll  L+ a-

	accZ = z*cosPitch + sinPitch*x;
	accZ = 9.8f*(accZ*cosRoll - sinRoll*y - 1);

	accX = 9.8f*(x*cosPitch - z*sinPitch);
	accY = 9.8f*(y*cosRoll  + z*sinRoll);
	


	

	
	
//	Debug.load(0, dt*100, 0);

//	Debug.load(1, aPitch/40, pitch/40);
//	Debug.load(0, yaw/180, Hmc.headingGrad / 180);
//	Debug.load(3, accX/10, accY/10);
//	Debug.load(4, accZ/10, 0);



		//Debug.load(1, speedY/20, yaw/180);

//	Debug.dump();

	
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
void MpuClass::meansensors(){
	long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

	while (i<(buffersize + 101)){
		// read raw accel/gyro measurements from device
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef SESOR_UPSIDE_DOWN
		az += 32768;///
#else
		//az -= 32768;
#endif
		//wdt_reset();


		if (i>100 && i <= (buffersize + 100)){ //First 100 measures are discarded
			buff_ax = buff_ax + ax;
			buff_ay = buff_ay + ay;
			buff_az = buff_az + az;
			buff_gx = buff_gx + gx;
			buff_gy = buff_gy + gy;
			buff_gz = buff_gz + gz;
		}
		if (i == (buffersize + 100)){
			mean_ax = buff_ax / buffersize;
			mean_ay = buff_ay / buffersize;
			mean_az = buff_az / buffersize;
			float rbuf = 1.0f / (float)buffersize;
			float x = (float)buff_ax*rbuf;
			float y = (float)buff_ay*rbuf;
			float z = (float)buff_az*rbuf;

			roll = RAD2GRAD*(float)atan(y / sqrt(x*x + z*z));
			pitch = RAD2GRAD*(float)atan(x / sqrt(y*y + z*z));

			//float rbufs = n003 / (float)buffersize;
			//const_gyroRoll0= (float)buff_gx * rbufs;
			//const_gyroPitch0 =  (float)buff_gy * rbufs;
			//const_gyroYaw0 =  (float)buff_gz * rbufs;
			gyroTime = millis();
			//Out.println(const_gyroRoll);
			//Out.println(const_gyroPitch);
			mean_gx = buff_gx / buffersize;
			mean_gy = buff_gy / buffersize;
			mean_gz = buff_gz / buffersize;
		}
		i++;
		delay(2); //Needed so we don't get repeated measures
	}
}
void MpuClass::calibrationF0(int16_t ar[]){
	while (1){
		int ready = 0;
		accelgyro.setXAccelOffset(ar[ax_offset]);
		accelgyro.setYAccelOffset(ar[ay_offset]);
		accelgyro.setZAccelOffset(ar[az_offset]);

		accelgyro.setXGyroOffset(ar[gx_offset]);
		accelgyro.setYGyroOffset(ar[gy_offset]);
		accelgyro.setZGyroOffset(ar[gz_offset]);

		meansensors();
		printf("...\n");

		if (abs(mean_ax) <= acel_deadzone) ready++;
		else ar[ax_offset] = ar[ax_offset] - (int16_t)(mean_ax / acel_deadzone);

		if (abs(mean_ay) <= acel_deadzone) ready++;
		else ar[ay_offset] = ar[ay_offset] - (int16_t)(mean_ay / acel_deadzone);

		if (abs(16384 - mean_az) <= acel_deadzone) ready++;
		else ar[az_offset] = ar[az_offset] + (int16_t)((16384 - mean_az) / acel_deadzone);

		if (abs(mean_gx) <= giro_deadzone) ready++;
		else ar[gx_offset] = ar[gx_offset] - (int16_t)(mean_gx / (giro_deadzone + 1));

		if (abs(mean_gy) <= giro_deadzone) ready++;
		else ar[gy_offset] = ar[gy_offset] - (int16_t)(mean_gy / (giro_deadzone + 1));

		if (abs(mean_gz) <= giro_deadzone) ready++;
		else ar[gz_offset] = ar[gz_offset] - (int16_t)(mean_gz / (giro_deadzone + 1));

		if (ready == 6) break;
	}
}
void MpuClass::calibrationF(int16_t ar[]){
	ar[ax_offset] = (int16_t)(-mean_ax / 8);
	ar[ay_offset] = (int16_t)(-mean_ay / 8);
	ar[az_offset] = (int16_t)((16384 - mean_az) / 8);

	ar[gx_offset] = (int16_t)(-mean_gx / 4);
	ar[gy_offset] = (int16_t)(-mean_gy / 4);
	ar[gz_offset] = (int16_t)(-mean_gz / 4);
	calibrationF0(ar);
}

void MpuClass::calibrationPrint(int16_t ar[], const bool onlyGyro){
	meansensors();
	printf("\nFINISHED!\n\nSensor readings with offsets:\t");
	if (onlyGyro == false){
		printf("%i\t%i\t%i\t", mean_ax, mean_ay, mean_az);
	}
	printf("%i\t%i\t%i\n", mean_gx, mean_gy, mean_gz);
	printf("Your offsets:\t");
	if (onlyGyro == false){
		printf("%i\t%i\t%i\t", ar[ax_offset], ar[ay_offset], ar[az_offset]);
	}
	printf("%i\t%i\t%i\n\nData is printed as: ", ar[gx_offset], ar[gy_offset], ar[gz_offset]);

	if (onlyGyro == false)
		printf("acelX acelY acelZ ");
	printf("giroX giroY giroZ\n");
	printf("Check that your sensor readings are close to ");
	if (onlyGyro == false)
		printf("0 0 16384 ");
	printf("0 0 0\n");
	//Out.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
}
void MpuClass::re_calibration_(){
	//Pwm.Buzzer(false);
	//calibrationF0();
	//calibrationPrint();
	//Pwm.Buzzer(true);
}

void MpuClass::new_calibration(const bool onlyGyro){
	LED.off();
	//wdt_disable();
	printf("on begin\n");
	int16_t offset_[6];
	offset_[ax_offset] = accelgyro.getXAccelOffset();
	offset_[ay_offset] = accelgyro.getYAccelOffset();
	offset_[az_offset] = accelgyro.getZAccelOffset();

	offset_[gx_offset] = accelgyro.getXGyroOffset();
	offset_[gy_offset] = accelgyro.getYGyroOffset();
	offset_[gz_offset] = accelgyro.getZGyroOffset();

	calibrationPrint(offset_, onlyGyro);
	gyro_calibratioan = true;
	if (onlyGyro==false || abs(mean_gx) > 2 || abs(mean_gy) > 2 || abs(mean_gz) > 2){


		new_calibration_(offset_);
		delay(1000);
		calibrationPrint(offset_, onlyGyro);
		if (onlyGyro)
			Settings.saveMpuOnlyGyroSettings(offset_);
		else
			Settings.saveMpuSettings(offset_);

		mpu_calibrated = gyro_calibratioan = Settings.readMpuSettings(offset_);
		if (mpu_calibrated){
			if (onlyGyro){
				accelgyro.setXAccelOffset(offset_[ax_offset]);
				accelgyro.setYAccelOffset(offset_[ay_offset]);
				accelgyro.setZAccelOffset(offset_[az_offset]);
			}
#ifndef WORK_WITH_WIFI
			//calibrated = false;
#endif
		}
		else{
			printf("MPU NOT CALIBRATED !!!\n");
		}




	}else

	Pwm.Buzzer(true);
	//wdt_enable(WATCHDOG);
	//Out.println("reseting");
	//delay(10000);
	//calibrated = Settings.readMpuSettings(offset_);
}
void MpuClass::new_calibration_(int16_t ar[]){
	//Pwm.Buzzer(false);
	accelgyro.setXAccelOffset(0);
	accelgyro.setYAccelOffset(0);
	accelgyro.setZAccelOffset(0);
	accelgyro.setXGyroOffset(0);
	accelgyro.setYGyroOffset(0);
	accelgyro.setZGyroOffset(0);
	printf("\nReading sensors for first time...\n");
	meansensors();
	delay(1000);

	printf("\nCalculating offsets...\n");
	calibrationF(ar);


}



MpuClass Mpu;



