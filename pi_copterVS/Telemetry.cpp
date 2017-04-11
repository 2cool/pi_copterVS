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

#define BAT_K0 0.476
#define BAT_K1 0.956
#define BAT_K2 1.394


//#define TEST_4_FULL_VOLTAGE


//#define BAT_ZERO 370 //80 процентов разряд
#define BAT_100P 422
#define BAT_ZERO (360.0*3)
#define BAT_50P (391.0*3)
#define MAX_VOLTAGE_AT_START 406
#define MAX_FLY_TIME 120000.0
//батареи хватает на 
#define FALSE_TIME_TO_BATERY_OFF 120000.0

void TelemetryClass::addMessage(const string msg){

	if (message.length() + msg.length() >= TELEMETRY_BUF_SIZE)
		return;

	if (message.length() < msg.length() || message.compare(msg)==-1)
		message += msg;

}

void TelemetryClass::getSettings(int n){
	Out.println("up set");
	if (n > 4 || n < 0)
		return;

	ostringstream convert;
	convert << "UPS," << n << ",";
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
	update();
	voltage = a2;
#endif
	newGPSData = false;
	//Out.println("TELEMETRY INIT");
	timeAtStart = voltage_at_start = 0;
	
}


void TelemetryClass::loop()
{
	
	if (next_battery_test_time<millis()){
		next_battery_test_time = millis() + BAT_timeout;
		testBatteryVoltage();

		if (Autopilot.progState() && check_time_left_if_go_to_home() < 60 && ++no_time_cnt>3){ // на тестах ошибся на 5 минут.  
			Out.println("too far from HOME!");
			addMessage(e_BATERY_OFF_GO_2_HOME);
			Autopilot.going2HomeStartStop(false);
		}	
	}
	update_buf();
}
#define BALANCE_DELAY 120

int16_t TelemetryClass::check_time_left_if_go_to_home(){
	float max_fly_time;
	if (voltage_at_start > 0){
		float work_time = 0.001*(float)(millis() - timeAtStart);
		if (work_time > BALANCE_DELAY && voltage_at_start > voltage){
			max_fly_time = ((voltage - BAT_ZERO)*work_time / (voltage_at_start - voltage));
		}
		else{
			max_fly_time = MAX_FLY_TIME - work_time;
			no_time_cnt = 0;
		}
		const float dist2home = sqrt(GPS.loc.dist2home_2);
		const float time2home = dist2home *(1.0 / MAX_HOR_SPEED);
		const float time2down = abs((Autopilot.corectedAltitude())*(1.0 / MAX_VER_SPEED_MINUS));
	//	Debug.dump(max_fly_time, time2home + time2down, voltage, 0);
		return max_fly_time - time2home - time2down;
	}
	else
		return max_fly_time;

}



uint8_t bat_cnt = 0;




#ifdef NO_BATTERY
float false_time = 0;
float false_voltage = BAT_100P;
#endif
void TelemetryClass::update(){
#ifdef NO_BATTERY
	float voltage_sag = 0;
	if (false_time == 0 && Autopilot.motors_is_on()){
		false_time = millis();
		false_voltage = MAX_VOLTAGE_AT_START;
	}
	if (false_time>0){
		float powerK = (Balance.get_throttle() * 2);

		powerK *= powerK;
		voltage_sag = 16;
		const float drawSpeed = 46.0 * powerK / FALSE_TIME_TO_BATERY_OFF;
		float dt = 0.001*(float)(millis() - false_time);
		false_time = millis();
		false_voltage -= drawSpeed*dt;
	}
	const float a = false_voltage - voltage_sag;
	b[0] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	b[1] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	b[2] = a + 1-(2*(float)rand() / (float)RAND_MAX);
	a2 = b[0] + b[1] + b[2];
	//Debug.load(0,(a2-(360*3))/138,0);
	//Debug.dump();
#else
	a2 = BAT_K2*(float)analogRead(A2);
	if (bat_cnt == 0){
		float a0 = BAT_K0*(float)analogRead(A0);
		float a1 = BAT_K1*(float)analogRead(A1);

		b[0] += (a0-b[0])*0.1;
		b[2] += (a2 - a1-b[2])*0.1;
		b[1] += (a1 - a0-b[1])*0.1;
		//Serial.println("bat");
		//Serial.println(a2);
		//Debug.dump(b[0], b[1], b[2], 0);	
	}
	bat_cnt++;
	bat_cnt &= 7;


#endif
}


void TelemetryClass::testBatteryVoltage(){

	update();

	voltage += (a2 - voltage)*0.1;

	if (timeAtStart == 0){
		if (Autopilot.motors_is_on()){
			voltage = a2;
			timeAtStart = millis();
		}
		else
			timeAtStart = voltage_at_start = 0;
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



	if (voltage < 900){
		voltage = 1110;
		addMessage(e_VOLT_MON_ERROR);
	}
	powerK = (MAX_VOLTAGE_AT_START*3) / (float)voltage;
	powerK = constrain(powerK, 1, 1.35);
}

boolean newGPSData = false;




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



int TelemetryClass::read_buf(byte *buffer) {
	while (buffer_size == 0) {
		if (Autopilot.get_control_bits()&(Autopilot.MPU_ACC_CALIBR | Autopilot.MPU_GYRO_CALIBR | Autopilot.COMPASS_CALIBR)) {
			return 4;
		}
		usleep(10000);
	}
	int size = buffer_size;
	memccpy(buffer, buf, 1, size);
	buffer_size = 0;
	return size;



}
uint32_t last_update_time=0;
void TelemetryClass::update_buf() {
	int i = 0;
	uint32_t mod = Autopilot.get_control_bits();
	loadBUF32(i, mod);
	//printf("message=", message.c_str());

	loadBUF(i, 1000 + (Balance.get_throttle() * 1000));
	loadBUF32(i, GPS.loc.lat_);
	loadBUF32(i, GPS.loc.lon_);
	buf[i++] = (byte)GPS.loc.accuracy_hor_pos;
	buf[i++] = (byte)GPS.loc.accuracy_ver_pos;
	loadBUF(i, 10.0*Autopilot.corectedAltitude());// -Autopilot.startAltitude));
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
	loadBUF8(i, Mpu.yaw * 0.70555555555555555555555555555556);

	if (message.length() && i + message.length() < TELEMETRY_BUF_SIZE) {
		memccpy(&buf[i], message.c_str(), 1, message.length());
		i += message.length();
		message = "";
	}
	buffer_size = i;
}


TelemetryClass Telemetry;

