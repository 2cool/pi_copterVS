
#include "MS5611.h"
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"

int fd4S;
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
#define ALT_NOT_SET 0
int MS5611Class::init(){
	old_time = micros();
	bar_task = 0;
	bar_zero = 0x0;
	ct=10000;
	
	speed=altitude_=alt=0;
	altitude_error = ALT_NOT_SET;

	powerK = 1;

#ifndef FALSE_BAROMETR

    Out.println("Initialize High resolution: MS5611");
	

#endif
	pressure = PRESSURE_AT_0;
	ms5611_count = 0;
	
	
	//----------------------------
	
	
	

#ifndef FALSE_BAROMETR

	int fd4S;
	if ((fd4S = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
		close(fd4S);
		return -1;
	}

	if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		close(fd4S);
		return -1;
	}
   
	if (write(fd4S, &RESET, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
		close(fd4S);
		return -1;
	}

	usleep(10000);

	for (i = 0; i < 6; i++){
		usleep(1000);

		C[i] = PROM_read(fd4S, CMD_PROM_READ + (i * 2));
		//printf("C[%d] = %d\n", i, C[i]);

	}
	close(fd4S);

#endif
	return 0;
	
	
}
float MS5611Class::altitude() {
	return alt;// altitude_ - altitude_error;
}
#ifndef WORK_WITH_WIFI
int cntssdde = 0;
#endif
float fspeed = 0;
void MS5611Class::copterStarted(){
//	if (altitude_error == ALT_NOT_SET){
		altitude_error = altitude_;
//	}
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


bool flying = false;
uint8_t MS5611Class::loop(){
	
	

	if (millis() - timet < 200)
		return 0;

	pressure = get_pressure(altitude_);
	//powerK = PRESSURE_AT_0 / pressure;
	i_readTemperature = 20;

	const float dt = (millis() - timet)*0.001;
	timet = millis();

	altitude_ = Mpu.altitude_Z + FALSE_ALTITUDE;
	if (flying == false && Mpu.altitude_Z > 1)
		flying = true;
	speed = Mpu.speed_Z;
	
	alt = altitude_ - altitude_error;
	//Serial.println(altitude_)

#ifdef Z_SAFE_AREA
	if (Autopilot.motors_is_on() && alt > Z_SAFE_AREA) {
		Autopilot.control_falling(i_CONTROL_FALL);
	}
#endif




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

uint8_t MS5611Class::loop(){
	
	switch (bar_task)
	{
	case 0:
		phase0();
		break;
	case 1:
		phase1();
		break;
	default:
		phase2();

	}

	return 0;
}

#endif

//----------------------------------------------------




void MS5611Class::phase0() {
	bar_D[0] = bar_D[1] = bar_D[2] = 0;
	bar_zero = 0;

	
	if ((fd4S = open("/dev/i2c-1", O_RDWR)) < 0) {
		printf("Failed to open the bus.\n");
		close(fd4S);
		return;
	}

	if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		close(fd4S);
		return;
	}
	

	char CONV_CMD = CONV_D1_4096;
	if (write(fd4S, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
		return;
	}
	close(fd4S);
	b_timeDelay = micros() + ct;
	bar_task=1;
}

void MS5611Class::phase1()
{

	if (micros()  > b_timeDelay) {

		int fd4S;
		if ((fd4S = open("/dev/i2c-1", O_RDWR)) < 0) {
			printf("Failed to open the bus.\n");
			close(fd4S);
			bar_task = 0;
			return;
		}

		if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0) {
			printf("Failed to acquire bus access and/or talk to slave.\n");
			close(fd4S);
			bar_task = 0;
			return;
		}





		if (write(fd4S, &bar_zero, 1) != 1) {
			printf("write reset 8 bit Failed to write to the i2c bus.\n");
			bar_task = 0;
			close(fd4S);
			return;
		}

		bar_h = read(fd4S, &bar_D, 3);

		if (bar_h != 3) {
			printf("Failed to read from the i2c bus %d.\n", bar_h);
			bar_task = 0;
			close(fd4S);
			return;
		}

		D1 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];
		if (D1 == 0) {
			bar_task = 0;
			printf("D1 error\n");
			close(fd4S);
			return;
		}
		//-------------------------------------
		bar_D[0] = bar_D[1] = bar_D[2] = 0;
		bar_zero = 0;
		char CONV_CMD = CONV_D2_4096;
		if (write(fd4S, &CONV_CMD, 1) != 1) {
			printf("write reg 8 bit Failed to write to the i2c bus.\n");
			bar_task = 0;
			close(fd4S);
			return;
		}
		b_timeDelay = micros() + ct;
		bar_task=2;
		close(fd4S);
		
	}
}


void MS5611Class::phase2() {
	if (micros()  > b_timeDelay) 
	{
		int fd4S;
		if ((fd4S = open("/dev/i2c-1", O_RDWR)) < 0) {
			printf("Failed to open the bus.\n");
			close(fd4S);
			bar_task = 0;
			return;
		}

		if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0) {
			printf("Failed to acquire bus access and/or talk to slave.\n");
			close(fd4S);
			bar_task = 0;
			return;
		}
		if (write(fd4S, &bar_zero, 1) != 1) {
			printf("write reset 8 bit Failed to write to the i2c bus.\n");
			bar_task = 0;
			close(fd4S);
			return;
		}

		bar_h = read(fd4S, &bar_D, 3);

		if (bar_h != 3) {
			printf("Failed to read from the i2c bus %d.\n", bar_h);
			bar_task = 0;
			close(fd4S);
			return;
		}
		D2 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];
		if (D2 == 0) {
			bar_task = 0;
			close(fd4S);
			printf("D2 error\n");
			return;
		}
		bar_task = 0;
		close(fd4S);
		phase3();
	}
}

void MS5611Class::phase3() {
	dT = D2 - (uint32_t)C[5] * 256;
	TEMP = (2000 + ((int64_t)dT * (int64_t)C[5] / 8388608));
	OFF = (int64_t)C[2] * 65536 + (dT*C[4]) / 128;
	SENS = (int32_t)C[1] * 32768 + dT*C[3] / 256;
	/*
	SECOND ORDER TEMPARATURE COMPENSATION
	*/
	if (TEMP < 2000) // if temperature lower than 20 Celsius 
	{
		int32_t T1 = 0;
		int64_t OFF1 = 0;
		int64_t SENS1 = 0;

		T1 = pow((double)dT, 2) / 2147483648;

		OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;


		if (TEMP < -1500) // if temperature lower than -15 Celsius 
		{
			OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}

		TEMP -= T1;
		OFF -= OFF1;
		SENS -= SENS1;
	}

	//nado proverat na korektnost' vysoty 
	P = ((((int64_t)D1*SENS) / 2097152 - OFF) / 32768);
	if (P < 50000) {
		bar_task = 0;
		close(fd4S);
		printf("H error\n");
		return;
	}


	if (pressure == PRESSURE_AT_0) {
		pressure = (float)P;
	}
	if (P < 0) {
		pressure = P;
		printf("P error\n");
	}
	i_readTemperature = ((int8_t)(TEMP * 0.01));


	pressure += ((float)P - pressure)*0.3;
	powerK = PRESSURE_AT_0 / pressure;

	if (powerK>1.4)
		powerK = 1.4;

	const float dt = (micros() - old_time)*0.000001;
	old_time = micros();
	//printf("%f\n", dt);
	const float new_altitude = getAltitude(pressure);

	speed = (new_altitude - altitude_) / dt;
	altitude_ = new_altitude;
	alt = altitude_ - altitude_error;
#ifdef Z_SAFE_AREA
	if (Autopilot.motors_is_on() && alt > Z_SAFE_AREA) {
		Autopilot.control_falling(e_OUT_OF_PER_V);
	}
#endif
//	if (tttalt == 0)
	//	tttalt = new_altitude;
	//tttalt += (new_altitude - tttalt)*0.01;
	//Debug.load(0, MS_errors, 0);
	//Debug.dump();

}
void MS5611Class::update(){}



float MS5611Class::get_pressure(float h) {
	return PRESSURE_AT_0 * pow(1 - h*2.25577e-5, 5.25588);
}

MS5611Class MS5611;
