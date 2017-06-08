// 
// 
// 

#include "Telemetry.h"
#include "GPS.h"
#include "Hmc.h"
#include "mpu.h"
#include "MS5611.h"
#include "commander.h"
#include "Balance.h"
#include "Stabilization.h"
#include "Wi_Fi.h"
#include "debug.h"

#define BAT_timeout 1000
#define BAT_timeoutRep  2

#define MAX_UPD_COUNTER 100


//#define TEST_4_FULL_VOLTAGE


//#define BAT_ZERO 370 //80 процентов разряд
#define BAT_100P 422
#define BAT_ZERO (360.0f*3)
#define BAT_50P (391.0f*3)
#define MAX_VOLTAGE_AT_START 406
#define MAX_FLY_TIME 120000.0f
//батареи хватает на 
#define FALSE_TIME_TO_BATERY_OFF 120000.0f

void TelemetryClass::addMessage(const string msg){
	printf("%s\n", msg.c_str());
	if (message.length() + msg.length() >= TELEMETRY_BUF_SIZE)
		return;

	if (message.length() < msg.length() || message.compare(msg)==-1)
		message += msg;

}

void TelemetryClass::getSettings(int n){
	printf("up set\n");
	if (n > 4 || n < 0)
		return;

	ostringstream convert;
	convert << "UPS" << n <<",";
	message += convert.str();

	switch (n){
	case 0:
		message += Balance.get_set();
		break;
	case 1:
		message += Stabilization.get_z_set();
		break;
	case 2:
		message += Stabilization.get_xy_set();
		break;
	case 3:
		message += Autopilot.get_set();
		break;
	case 4:
		message += Mpu.get_set();
		break;
	case 5:
		message += Hmc.get_set();
		break;		
	}
	
/*	uint8_t o = (message.length()+1) % 3;  //чтобы длина сообщения была кратная трем.
	if (o == 1){
		message += "  ";
	}
	else if (o==2)
		message += " ";*/
	message += ",";
	//Out.println(message);

}


void TelemetryClass::init_()
{
	uint32_t power_on_time = 0;
	timeAtStart = 0;
	buffer_size = 0;

	powerK = 1;
	minimumTelemetry = false;
	lov_voltage_cnt = 0;
	//inner_clock_old_sec = millis() >> 10;
	low_voltage = voltage50P=false;
	message = "";
	next_battery_test_time = millis()+BAT_timeout;
#ifdef NO_BATTERY
	b[0] = b[1] = b[2] = BAT_100P;
	voltage = BAT_100P * 3;
#else
	b[0] = b[1] = b[2] = BAT_100P;
	update_voltage();
#endif
	newGPSData = false;
	//Out.println("TELEMETRY INIT");
	timeAtStart = 0;
	voltage_at_start = 0;
	
}


void TelemetryClass::loop()
{
	
	if (next_battery_test_time<millis()){
		next_battery_test_time = millis() + BAT_timeout;
		testBatteryVoltage();

		if (Autopilot.progState() && check_time_left_if_go_to_home() < 60 && ++no_time_cnt>3){ // на тестах ошибся на 5 минут.  
			printf("too far from HOME!\n");
			addMessage(e_BATERY_OFF_GO_2_HOME);
			Autopilot.going2HomeStartStop(false);
		}	
	}
	update_buf();
}
#define BALANCE_DELAY 120

int16_t TelemetryClass::check_time_left_if_go_to_home(){
	float max_fly_time=0;
	if (voltage_at_start > 0){
		float work_time = 0.001f*(float)(millis() - timeAtStart);
		if (work_time > BALANCE_DELAY && voltage_at_start > voltage){
			max_fly_time = ((voltage - BAT_ZERO)*work_time / (voltage_at_start - voltage));
		}
		else{
			max_fly_time = MAX_FLY_TIME - work_time;
			no_time_cnt = 0;
		}
		const float dist2home = (float)sqrt(GPS.loc.dist2home_2);
		const float time2home = dist2home *(1.0f / MAX_HOR_SPEED);
		const float time2down = abs((MS5611.altitude())*(1.0f / MAX_VER_SPEED_MINUS));
	//	Debug.dump(max_fly_time, time2home + time2down, voltage, 0);
		return (int16_t)(max_fly_time - time2home - time2down);
	}
	else
		return (int16_t)max_fly_time;

}


#ifdef NO_BATTERY
float false_time = 0;
float false_voltage = BAT_100P;
#endif
void TelemetryClass::update_voltage(){
#ifdef NO_BATTERY
	float voltage_sag = 0;
	if (false_time == 0 && Autopilot.motors_is_on()){
		false_time = millis();
		false_voltage = MAX_VOLTAGE_AT_START;
	}
	if (false_time>0){
		float powerKl = (Balance.get_throttle() * 2);

		powerKl *= powerKl;
		voltage_sag = 16;
		const float drawSpeed = 46.0 * powerKl / FALSE_TIME_TO_BATERY_OFF;
		float dt = 0.001*(float)(millis() - false_time);
		false_time = millis();
		false_voltage -= drawSpeed*dt;
	}
	const float a = false_voltage - voltage_sag;
	b[0] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	b[1] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	b[2] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	voltage = b[0] + b[1] + b[2];
	//Debug.load(0,(a2-(360*3))/138,0);
	//Debug.dump();
#else

	/*
	3.83  2.92
	7.72  2.86
	11.67 2.85
*/
	int16_t buf[3];
	//int t = micros();
	Pwm.get_analog(buf);  //300 microsec

	float a0 = (float)buf[0] / 2.397f;
	float a1 = (float)buf[1] / 1.16f;
	voltage = (float)buf[2] / 0.76f;




	//int t2 = micros() - t;
	//a2 = BAT_K2*(float)buf[2];// analogRead(A2);
	
	//float a0 = BAT_K0*(float)buf[0];// analogRead(A0);
	//float a1 = BAT_K1*(float)buf[1];// analogRead(A1);

	b[0] = a0;
	b[1] = a1 - a0;
	if (buf[2] > 100)
		b[2] = voltage - a1;
	else {
		b[2] = 0;
		voltage = 0;
	}
	
	//Serial.println("bat");
	//Serial.println(a2);
	//Debug.dump(b[0], b[1], b[2], 0);	
	
	

#endif

	if (Telemetry.b[2] == 0) {
		if (power_on_time > 0) {
			power_on_time = 0;
			printf("power off\n");
		}
	}
	else {
		if (power_on_time == 0) {
			power_on_time = millis();
			printf("!!! power on !!!\n");
		}
	}



}


void TelemetryClass::testBatteryVoltage(){

	update_voltage();


	if (timeAtStart == 0){
		if (Autopilot.motors_is_on()){
			timeAtStart = millis();
		}
		else {
			timeAtStart = 0;
			voltage_at_start = 0;
		}
	}

	

	if (timeAtStart > 0 && millis() - timeAtStart < 15000){
		voltage_at_start = voltage;
	}

	if (voltage < BAT_ZERO && voltage>(150.0*3))
		lov_voltage_cnt++;
	else
		lov_voltage_cnt = 0;

	low_voltage = lov_voltage_cnt > 3;
	voltage50P = voltage < BAT_50P;



	if (voltage>0 && voltage < 900){
		voltage = 1110;
		addMessage(e_VOLT_MON_ERROR);
	}
	if (voltage == 0)
		powerK = 1;
	else {
		powerK = (MAX_VOLTAGE_AT_START * 3) / (float)voltage;
		powerK = constrain(powerK, 1, 1.35f);
	}
}

bool newGPSData = false;




void TelemetryClass::loadBUF32(int &i,  int32_t val)
{
	buf[i++] = ((byte*)&val)[0];
	buf[i++] = ((byte*)&val)[1];
	buf[i++] = ((byte*)&val)[2];
	buf[i++] = ((byte*)&val)[3];

}

void TelemetryClass::loadBUF8(int &i,  const float val){
	int8_t t = (int8_t)(val);
	buf[i++] = ((byte*)&t)[0];
}
void TelemetryClass::loadBUF(int &i, const float fval)
{
	int16_t t = (int16_t)(fval);
	buf[i++] = ((byte*)&t)[0];
	buf[i++] = ((byte*)&t)[1];
	 
}


 /////////////////////////////////////////////////////////////////////////////////////////////////
 //uint8_t telemetry_cnt = 0;

bool gps_or_acuracy = false;

enum {MOTORS_ON=1,CONTROL_FALLING=2,Z_STAB=4,XY_STAB=8,GO2HOME=0x10,PROGRAM=0x20,COMPASS_ON=0x40,HORIZONT_ON=0x80,
		MPU_ACC_CALIBR=0x100, MPU_GYRO_CALIBR = 0x200, COMPASS_CALIBR=0x400, COMPASS_MOTOR_CALIBR=0x800, RESETING=0x1000, GIMBAL_PLUS=0x2000,GIMBAL_MINUS=0x4000
	};

int TelemetryClass::read_buf(byte *buffer) {
	if (Autopilot.busy())
		return 4;
	else {
		while (buffer_size == 0) {
			usleep(10000);
		}
		int size = buffer_size;
		memcpy(buffer, buf, size);
		buffer_size = 0;
		return size;
	}


}
uint32_t last_update_time=0;
void TelemetryClass::update_buf() {
	bzero(buf, 32);
	//delay(1000);
	int i = 0;
	uint32_t mod = Autopilot.get_control_bits();
//	printf("out <- %i\n", mod);
	loadBUF32(i, mod);
	//printf("message=", message.c_str());
	const bool err = (GPS.loc.accuracy_hor_pos >= 99);
	loadBUF(i, 1000 + (Balance.get_throttle() * 1000));
	loadBUF32(i, GPS.loc.lat_);
	loadBUF32(i, GPS.loc.lon_);

	buf[i++] = (byte)(err)?99: GPS.loc.accuracy_hor_pos;
	buf[i++] = (byte)(err)?99: GPS.loc.accuracy_ver_pos;

	loadBUF(i, 10.0f*Autopilot.corectedAltitude4tel());// -Autopilot.startAltitude));
	//Out.print(t_old_alt); Out.print(" "); Out.println(MS5611.altitude);// -Autopilot.startAltitude);
	loadBUF8(i, -Mpu.get_pitch());
	loadBUF8(i, Mpu.get_roll());
	loadBUF8(i, Balance.c_pitch);
	loadBUF8(i, Balance.c_roll);
	int16_t t;
	t = (int16_t)b[0];
	buf[i++] = ((byte*)&t)[0];
	t = (int16_t)b[1];
	buf[i++] = ((byte*)&t)[0];
	t = (int16_t)b[2];
	buf[i++] = ((byte*)&t)[0];
	buf[i++] = (int8_t)Autopilot.getGimbalPitch();
	loadBUF8(i, Mpu.yaw * 0.705555555555555f);

	if (message.length() && i + message.length() < TELEMETRY_BUF_SIZE) {
		memcpy(&buf[i], message.c_str(), message.length());
		i += message.length();
		message = "";
	}
	buffer_size = i;
}
//nado echo peredat koordinaty starta i visoti ili luche ih androis socharanaet

TelemetryClass Telemetry;

