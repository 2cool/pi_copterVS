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

#include "GPS.h"
#include "Telemetry.h"
#include "commander.h"
#include "Wi_Fi.h"

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
	//long time_ms = micros();
	//int sb = 0;
#ifdef MOTORS_OFF
	Pwm.on(0, pwm_OFF_THROTTLE);
	
#else	

	const bool power_on = true;// analogRead(A4) > 500;
	Pwm.on(0, power_on ? pwm_OFF_THROTTLE : pwm_MAX_THROTTLE);
	printf(power_on ? "	- COPTER IS LOST -\n" : "wait 4 power\n");


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
	
	Pwm.on(0,  pwm_MAX_THROTTLE);

	EEPROM.read_set();
	LED.init();
	printf("___setup___\n");
	printf("gps init...\n");
	GPS.init();

#ifdef WORK_WITH_WIFI
	printf("wifi init...\n");
	if (WiFi.init())
		return -1;
#endif
	printf("commander init...\n");
	Commander.init();
	printf("Autopilot init...\n");
	Autopilot.init();
	Telemetry.init_();
	Telemetry.testBatteryVoltage();
	printf("telemetry init OK \n");
	Pwm.beep_code(BEEPS_ON);
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
				printf("%i\n",maxTT); \
		}\
}
#ifndef WORK_WITH_WIFI
bool foo_flag = false;
uint32_t ttiimer = 0;
#endif


int cccccc_ss = 0;

uint64_t old_time4loop;
bool temp4test = true;


int ok_cnt = 0;
int ok_ccc = 0;
int er_cnt = 0;
long dt_sum=0;
int max_dt = 0;
int old_debug = 0;
void loop()
{

#ifndef WORK_WITH_WIFI
	if (temp4test && Autopilot.motors_is_on() == false && millis() > 2000) {
		//Autopilot.motors_do_on(true, "FALSE WIFI");
		//Mpu.new_calibration(false);
		temp4test = false;
	}


#endif


	Balance.loop();
#ifdef WORK_WITH_WIFI
	Telemetry.loop();
#endif
	Commander.input();
	Autopilot.loop();

#ifdef FALSE_WIRE
	usleep(3000);
#endif


/*	const uint64_t tnoww = micros();
	int dtt = (tnoww - old_time4loop);
	if (dtt > 5500)
		er_cnt++;
	//usleep(1000);
	
	
	ok_cnt++;
	ok_ccc++;
	if (ok_ccc > 2000) {
		if (old_debug != Debug.n_debug) {
			old_debug = Debug.n_debug;
			ok_ccc = 0;
			max_dt = 0;
		}
		if (max_dt < dtt)
			max_dt = dtt;
	}
	//if (dtt>0)
	dt_sum += (int)(micros() - old_time4loop);
	if ((ok_cnt & 1023) == 1023)
			printf(" %i %f %i\n", dt_sum / ok_cnt, (er_cnt > 0) ? 100.0*(float)er_cnt / (float)ok_cnt : 0, max_dt);
	old_time4loop = tnoww;*/
	
}

//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}

int printHelp() {
	printf("<-help> for this help\n");
	printf(" <fly at start at hight in sm> <lower hight in sm>\n");
	return -1;
}

int main(int argc, char *argv[]) {
	Debug.n_p1 = 3;
	Debug.n_p2 = 1.6f;
	Debug.n_debug = 0;
	if (argc >= 2) {
		int tt = string(argv[1]).compare("-help");
		if (tt==0) {

		
			return printHelp();

		}
		int t= atoi(argv[1]);
		if (t>=100 && t<=500)
		Debug.n_p1 = 0.01f*(float)t;
		if (argc >= 3) {
			t=atoi(argv[2]);
			if (t >= 50 && t <= 160)
				Debug.n_p2 = 0.01f*(float)t;
		}
		
	}
	else
		return printHelp();


	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}

	if (setup())
		return -1;
	old_time4loop =micros();
	float dfr = 100;
	while (flag == 0) {

		loop();
	//	int ttt = micros();
	//	dfr += ((1000000 / (ttt - old_time4loop)) - dfr)*0.01;
	//	Debug.load(0, dfr, 0);
	//	old_time4loop = ttt;
		int64_t t = micros();
		int32_t time_past = (int32_t)(t - old_time4loop);
		old_time4loop = t;
		usleep(5400);

	}
	printf("\n Signal caught! and exit\n");
	EEPROM.write_set();

	return 0;

}
