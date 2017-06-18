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






bool loop();



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
	fprintf(Debug.out_stream,power_on ? "	- COPTER IS LOST -\n" : "wait 4 power\n");


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

int setup(int cnt) {////--------------------------------------------- SETUP ------------------------------
	
	Pwm.on(0,  pwm_MAX_THROTTLE);

	EEPROM.read_set();
	LED.init();
	fprintf(Debug.out_stream,"___setup___\n");
	

#ifdef WORK_WITH_WIFI
	fprintf(Debug.out_stream,"wifi init...\n");
	if (WiFi.init(cnt))
		return -1;
#endif
	fprintf(Debug.out_stream,"commander init...\n");
	Commander.init();
	fprintf(Debug.out_stream,"Autopilot init...\n");
	Autopilot.init();
	Telemetry.init_();
	Telemetry.testBatteryVoltage();
	fprintf(Debug.out_stream,"telemetry init OK \n");
	Pwm.beep_code(BEEPS_ON);
	GPS.init();
	return 0;

}


uint8_t teil = 0, maxTeilN = 3;



uint64_t tt, maxTT = 0; \
void print_time() {
	\
		uint64_t d = micros() - tt; \
		if (d > maxTT) {
			\
				maxTT = d; \
				fprintf(Debug.out_stream,"%i\n",maxTT); \
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
bool loop()
{

#ifndef WORK_WITH_WIFI
	if (temp4test && Autopilot.motors_is_on() == false && millis() > 2000) {
		//Autopilot.motors_do_on(true, "FALSE WIFI");
		//Mpu.new_calibration(false);
		temp4test = false;
	}


#endif


	if (Balance.loop()) {

#ifdef WORK_WITH_WIFI
		Telemetry.loop();
#endif
		Commander.input();
		Autopilot.loop();

#ifdef FALSE_WIRE
		usleep(3000);
#endif
		return true;
	}
	else
		return false;

	
}

//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}

int printHelp() {
	printf("<-help> for this help\n");
	printf(" <fly at start at hight in sm> <lower hight in sm> <f=write stdout to file> <y=log com and tel>\n");
	printf("example to write in log file : pi_copter 300 100 f y\n");
	printf("example to write in stdout   : pi_copter 300 100 s\n");
	return -1;
}


int main(int argc, char *argv[]) {
	
	
	Debug.n_p1 = 3;
	Debug.n_p2 = 1.6f;
	Debug.n_debug = 0;
	int counter = 0;

	if (argc >= 2) {
		int tt = string(argv[1]).compare("-help");
		if (tt==0) {

		
			return printHelp();

		}
		
		if (argc >= 5) {
			int t = atoi(argv[1]);
			//if (t>=100 && t<=500)
			Debug.n_p1 = 0.01f*(float)t;
			t=atoi(argv[2]);
			
			Debug.n_p2 = 0.01f*(float)t;


			
			FILE *set = fopen("/home/igor/logs/logCounter.txt", "r");
			if (set) {
				fscanf(set, "%i", &counter);

				fclose(set);
				remove("/home/igor/logs/logCounter.txt");
			}

			set = fopen("/home/igor/logs/logCounter.txt", "w+");
			fprintf(set, "%i\n", counter + 1);
			fclose(set);
			if (argv[3][0] == 'f' || argv[3][0] == 'F') {
				

				ostringstream convert;
				convert << "/home/igor/logs/log_out" << counter << ".txt";
				string fname = convert.str();

				Debug.out_stream = fopen(fname.c_str(), "w+");
			}else	
				Debug.out_stream = stdout;

			Debug.writeTelemetry = (argv[4][0] == 'y' || argv[4][0] == 'Y');

		}
		
	}
	else
		return printHelp();

	fprintf(Debug.out_stream, "ver 2.170618 \n");
	fprintf(Debug.out_stream, "picopter par: %s %s %s %s\n", argv[1], argv[2], argv[3], argv[4]);

	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}

	if (setup(counter))
		return -1;
	old_time4loop =micros();
	float dfr = 100;
	Debug.run_main = true;
	while (Debug.run_main && flag == 0) {

		if (loop()) {
			//usleep(5400);
		//	int ttt = micros();
		//	dfr += ((1000000 / (ttt - old_time4loop)) - dfr)*0.01;
		//	Debug.load(0, dfr, 0);
		//	old_time4loop = ttt;
			int64_t t = micros();
			int32_t time_past = (int32_t)(t - old_time4loop);
			old_time4loop = t;
			//if (time_past > 15000)
			//	fprintf(Debug.out_stream,"too long %i\n",time_past);

			//Debug.load(0, time_past, 0);
			//Debug.dump();
		}

	}
	WiFi.stopServer();
	if (Debug.run_main==false)
		fprintf(Debug.out_stream, "\n exit\n");
	if (flag!=0)
		fprintf(Debug.out_stream, "\n main Signal caught!\n");
	EEPROM.write_set();
	fclose(Debug.out_stream);
	

	return 0;

}
