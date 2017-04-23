
#include "MS5611.h"
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"
#include "Ultrasound_Radar.h"

unsigned int PROM_read(int DA, char PROM_CMD)
{
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, &PROM_CMD, 1) != 1){
		printf("read set reg Failed to write to the i2c bus.\n");
	}

	if (read(DA, r8b, 2) != 2){
		printf("Failed to read from the i2c bus.\n");
	}

	ret = r8b[0] * 256 + r8b[1];

	return ret;
}

long CONV_read(int DA, char CONV_CMD)
{
	long ret = 0;
	uint8_t D[] = { 0, 0, 0 };

	int  h;
	char zero = 0x0;

	if (write(DA, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(OSR_4096);

	if (write(DA, &zero, 1) != 1) {
		printf("write reset 8 bit Failed to write to the i2c bus.\n");
	}

	h = read(DA, &D, 3);

	if (h != 3) {
		printf("Failed to read from the i2c bus %d.\n", h);

	}

	ret = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];

	return ret;
}
	char RESET = 0x1E;
	
int MS5611Class::init(){

	bar_task = 0;
	bar_zero = 0x0;
	ct=10;
	
	altitude_error = presure_altitude_at_start=0;

	powerK = 1;

#ifndef FALSE_BAROMETR

    Out.println("Initialize High resolution: MS5611");
	

#endif
	pressure = PRESSURE_AT_0;
	altitude_ = speed = 0;
	ms5611_count = 0;
	lastTime = millis();
	
	//----------------------------
	
	
	

#ifndef FALSE_BAROMETR

	curSampled_time = 0;
	prevSampled_time = 0;

	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
   
	if (write(fd, &RESET, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(10000);

	for (i = 0; i < 7; i++){
		usleep(1000);

		C[i] = PROM_read(fd, CMD_PROM_READ + (i * 2));
		//printf("C[%d] = %d\n", i, C[i]);
	}


#endif
	return 0;
	
	
}

#ifndef WORK_WITH_WIFI
int cntssdde = 0;
#endif
float fspeed = 0;
void MS5611Class::copterStarted(){
	if (altitude_error == 0 || presure_altitude_at_start == 0){
		altitude_error = presure_altitude_at_start = altitude_;
	}
}



#define MAX_D_PRESSURE 300.0


double MS5611Class::getAltitude(const float pressure) {
	return (44330.0f * (1.0f - pow((double)pressure / PRESSURE_AT_0, 0.1902949f)));
}




//--------------------------------------------------
#ifdef FALSE_BAROMETR

#include "GPS.h"


#include "Balance.h"






uint32_t timet = millis();
float mthr = 0;


boolean flying = false;
uint8_t MS5611Class::loop(){
	Ultrasound_Radar.loop();
	

	if (millis() - timet < 200)
		return 0;

	pressure = get_pressure(altitude_);
	//powerK = PRESSURE_AT_0 / pressure;
	i_readTemperature = 20;

	const float dt = (millis() - timet)*0.001;
	timet = millis();

	float new_altitude = Mpu.altitude_Z + FALSE_ALTITUDE;
	if (flying == false && Mpu.altitude_Z > 1)
		flying = true;
	speed = Mpu.speed_Z;
	//Serial.println(altitude_)


	ultrasound_radar_corection(new_altitude);



	//Serial.print("alt="); Serial.print(altitude_ + altitude_error); Serial.print(" "); Serial.println(altitude_);
	/*if (altitude_ <= 0){
		altitude_ = speed = 0;
		if (flying){
			flying = false;
			Serial.println("FALL");
			Autopilot.motors_do_on(false, "TST");
		}
	}*/


}

#else


long ms_time = 0;
#define ms_delay 500
uint8_t MS5611Class::loop(){

	update();
	return 0;
	}

#endif

//----------------------------------------------------






float tttalt = 0;

void MS5611Class::phase0() {
	bar_D[0] = bar_D[1] = bar_D[2] = 0;
	bar_zero = 0;
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);
	curSampled_time = round(spec.tv_nsec / 1.0e6);

	prevSampling_time = Sampling_time;
	Sampling_time = (float)curSampled_time - (float)prevSampled_time;

	if (Sampling_time < 0) // to prevent negative sampling time
		Sampling_time = prevSampling_time;
	char CONV_CMD = CONV_D1_4096;
	if (write(fd, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}
	b_timeDelay = millis() + ct;
	bar_task++;
}

void MS5611Class::phase1()
{
	if (millis() > b_timeDelay) {
		if (write(fd, &bar_zero, 1) != 1) {
			printf("write reset 8 bit Failed to write to the i2c bus.\n");
		}

		bar_h = read(fd, &bar_D, 3);

		if (bar_h != 3) {
			printf("Failed to read from the i2c bus %d.\n", bar_h);

		}

		D1 = bar_D[0] * (unsigned long)65536 + bar_D[1] * (unsigned long)256 + bar_D[2];
		bar_task++;
	}
}

void MS5611Class::phase2()
{
	bar_D[0] = bar_D[1] = bar_D[2] = 0;
	bar_zero = 0;
	char CONV_CMD = CONV_D2_4096;
	if (write(fd, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}
	b_timeDelay = millis() + ct;
	bar_task++;
	
}
bool MS5611Class::phase3() {
	if (millis() > b_timeDelay) {
		if (write(fd, &bar_zero, 1) != 1) {
			printf("write reset 8 bit Failed to write to the i2c bus.\n");
		}

		bar_h = read(fd, &bar_D, 3);

		if (bar_h != 3) {
			printf("Failed to read from the i2c bus %d.\n", bar_h);

		}

		D2 = bar_D[0] * (unsigned long)65536 + bar_D[1] * (unsigned long)256 + bar_D[2];
		bar_task = 0;
		return true;
	}
	else
		return false;


}


void MS5611Class::phase4() {
	dT = D2 - (uint32_t)C[5] * pow(2, 8);
	TEMP = (2000 + (dT * (int64_t)C[5] / pow(2, 23)));

	OFF = (int64_t)C[2] * pow(2, 16) + (dT*C[4]) / pow(2, 7);
	SENS = (int32_t)C[1] * pow(2, 15) + dT*C[3] / pow(2, 8);

	/*
	SECOND ORDER TEMPARATURE COMPENSATION
	*/
	if (TEMP < 2000) // if temperature lower than 20 Celsius 
	{
		int32_t T1 = 0;
		int64_t OFF1 = 0;
		int64_t SENS1 = 0;

		T1 = pow((double)dT, 2) / 2147483648;
		OFF1 = 5 * pow(((double)TEMP - 2000), 2) / 2;
		SENS1 = 5 * pow(((double)TEMP - 2000), 2) / 4;

		if (TEMP < -1500) // if temperature lower than -15 Celsius 
		{
			OFF1 = OFF1 + 7 * pow(((double)TEMP + 1500), 2);
			SENS1 = SENS1 + 11 * pow(((double)TEMP + 1500), 2) / 2;
		}

		TEMP -= T1;
		OFF -= OFF1;
		SENS -= SENS1;
	}


	P = ((((int64_t)D1*SENS) / pow(2, 21) - OFF) / pow(2, 15));
	i_readTemperature = ((int8_t)(TEMP * 0.01));

	if (pressure == PRESSURE_AT_0) {
		pressure = P;
	}
	pressure += (P - pressure)*0.3;
	powerK = PRESSURE_AT_0 / pressure;
	if (powerK>1.4)
		powerK = 1.4;

	const float dt = Sampling_time*0.001;// (millis() - lastTime)*0.001;
	lastTime = millis();
	const float new_altitude = getAltitude(pressure);

	speed = (new_altitude - altitude_ - altitude_error) / dt;

	if (tttalt == 0)
		tttalt = new_altitude;
	tttalt += (new_altitude - tttalt)*0.01;
	Debug.load(0, 0, tttalt - new_altitude);
	Debug.dump();

}
void MS5611Class::update(){

	switch (bar_task)
	{
	case 0: 
		phase0();
		break;
	case 1: 
		phase1();
		break;
	case 2:
		phase2();
		break;
	default:
		if (phase3())
			phase4();
	}
	
}
float MS5611Class::get_pressure(float h) {
	return PRESSURE_AT_0 * pow(1 - h*2.25577e-5, 5.25588);
}

MS5611Class MS5611;
