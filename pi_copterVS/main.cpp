#include <cstdio>
#include <signal.h>

//#include "KalmanFilterVector.h"
#include "Filter.h"

#include "Filter.h"
#include "define.h"
#include "debug.h"
#include "LED.h"


#include "WProgram.h"
#include "Settings.h"
#include "Prog.h"
#include "Location.h"

#include "Ultrasound_Radar.h"
//#include "Bluetooth.h"
#include "GPS.h"
#include "Telemetry.h"
#include "commander.h"
#include "Wi_Fi.h"
//#include "Interupts.h"
#include "Autopilot.h"

#include "Pwm.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Balance.h"






void loop();



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.


// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

//#include <avr/io.h> //  can be omitted
//#include <avr/interrupt.h> // can be omitted
















uint16_t oldCounter = 1000;


void buzz_until_not_finded() {
	//wdt_disable();
	//pinMode(7, OUTPUT);   // buzzer
	//uint8_t errors = 0;
	//int16_t res = Mpu.getGX();
	
	LED.prog_index = LED.PROG0_P;
	long time_ms = micros();
	int sb = 0;
#ifdef MOTORS_OFF
	Pwm.on(PWM_COUNTER, pwm_OFF_THROTTLE);
	
#else	

	const bool power_on = true;// analogRead(A4) > 500;
	Pwm.on(PWM_COUNTER, power_on ? pwm_OFF_THROTTLE : pwm_MAX_THROTTLE);
	Out.println(power_on ? "	- COPTER IS LOST -" : "wait 4 power");


	/*
	uint32_t temp_time = 0;
	do {
		GPS.loop();

		MS5611.loop();

		WiFi.loop(micros() + (long)1000000);

		if (power_on) {
			if (micros() - time_ms > 1300) {
				time_ms = micros();
				sb++;
				digitalWrite(7, sb & 1);
			}
		}
		LED.loop();


		if (power_on == false) {
			if (WiFi.copterFound) {
				Serial.println("wait4power_on");
			}
			WiFi.copterFound = false;
			if (analogRead(A4) > 500) {
				if (temp_time == 0) {
					temp_time = millis();
					//Serial.println("12 VOLT");
					//Serial.println(analogRead(A4));
				}
				else {
					if (millis() - temp_time > (uint32_t)1000U) {

						//OCR5A = OCR3A = OCR4A = OCR4C = pwm_OFF_THROTTLE;
						WiFi.copterFound = true;
						break;
						//Serial.println("off THR");
					}
				}
			}
		}
		else
			if (WiFi.copterFound) {
				Out.println("COPTER IS FOUND!");
				break;
			}


	} while (true);
	*/
#endif
}

int setup() {////--------------------------------------------- SETUP ------------------------------
	EEPROM.read_set();
	//wdt_disable();
	//wdt_enable(WDTO_4S);
	// pinMode(13, OUTPUT);
	//Out.begin(115200);
	LED.init();
	Ultrasound_Radar.init();
	//Prog.init(",2,,4,5,6,7,8,9,0,");
	//delay(20000);

	{
		for (int i = 0; i < 80; i++)
			Serial.print("-");
		Serial.println();
	}
	Out.println("___setup___");
	//Out.println("Wire.begin");
	delay(100);

	printf("gps init...\n");
	GPS.init();
	//	Bluetooth.init();

	printf("wifi init...\n");
	if (WiFi.init())
		return -1;
	printf("commander init...\n");
	Commander.init();
	printf("Autopilot init...\n");
	Autopilot.init();
	printf("buzz_until not finded...\n");

	buzz_until_not_finded();

	printf("telemetry init...\n");
	Telemetry.init_();
	Telemetry.testBatteryVoltage();

	//wdt_enable(WATCHDOG);
	//wdt_disable();

	return 0;

}


uint8_t teil = 0, maxTeilN = 3;



uint32_t tt, maxTT = 0; \
void print_time() {
	\
		uint32_t d = micros() - tt; \
		if (d > maxTT) {
			\
				maxTT = d; \
				Out.println(maxTT); \
		}\
}
#ifndef WORK_WITH_WIFI
bool foo_flag = false;
uint32_t ttiimer = 0;
#endif


int cccccc_ss = 0;

void loop()
{

	uint32_t time_ms = ten_micros();

	if (Telemetry.voltage50P) {
#ifdef BUZZER_R
		//OCR4B = 100;
#endif
		if (Telemetry.low_voltage) {
			;//pinMode(7, OUTPUT);
#ifdef BUZZER_R
			//OCR4B = 35000;
#endif
		}
	}

	uint32_t t;
	//	tt = micros();
	Balance.loop();			//4800
#ifdef WORK_WITH_WIFI
	Telemetry.loop();
#endif
	Commander.input();
	Autopilot.loop();


	uint32_t tnow = ten_micros();
	if ((tnow - time_ms)<1000) {
		usleep(10*(1000-(tnow - time_ms)));
	}
	
//	if (Autopilot.motors_is_on() || Autopilot.lost_conection_time == 0 || millis() - Autopilot.lost_conection_time<NO_CONNECTION_DELAY_TO_RESET_IF_MOTORS_OFF)
	//	;//wdt_reset();
		 /*
		 cccccc_ss++;
		 if (cccccc_ss > 500){
		 cccccc_ss = 0;
		 if (Autopilot.motors_is_on()){
		 Serial.print("fm="); Serial.println(Mem.freeRam());
		 Serial.println(millis());
		 }
		 }
		 */
		 //else{Out.println("WDT");	Out.println(Autopilot.wdt_mask);	}



}

//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}



int main() {

	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}

	if (setup())
		return -1;

	while (flag==0)
		loop();
	printf("\n Signal caught! and exit\n");
	EEPROM.write_set();

	return 0;

}
