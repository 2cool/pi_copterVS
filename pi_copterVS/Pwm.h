// Pwm.h


/*

timer 	bits 	channel 	Arduino pin 	Mega pin
timer0 	8 		A 			6				13
timer0 	8 		B 			5				4
timer1 	16 		A 			9 				11
timer1 	16 		B 			10				12
timer2 	8 		A 			11			 	10
timer2 	8 		B 			3				9
timer3 	16 		A 			-				5		x
timer3 	16 		B 			-				2
timer3 	16 		C 			-				3		x
timer4 	16 		A 			-				6		x
timer4 	16 		B			- 				7		buzzer
timer4 	16		C			- 				8		x
timer5 	16 		A 			- 				44		ginbal
timer5 	16 		B 			- 				45		ginbal
timer5 	16 		C 			- 				46












    ^
0   |    1
 \  |  /
  \   /
   000
   000
  /   \
 /     \
2       3

*/

#ifndef _PWM_h
#define _PWM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

/*
импульс - сили в граммах
1600 33
1940 97
2280 170
2620 249
2960 341
3300 450
3640 520
3980 620
4320 700
4660 750
*/








/*
2мегагерца

#define MAX_THROTTLE 4000
#define OFF_THROTTLE 2000
#define MAX90_THROTTLE 3800
#define MIN_THROTTLE 2200
*/
//16 мегагерц

#define pwm_MAX_THROTTLE 32000
#define pwm_OFF_THROTTLE 16000
#define MAX90_THROTTLE 30400
#define pwm_MIN_THROTTLE 17600
//#define MAX90_THROTTLE 30400
//#define MIN_THROTTLE 17600

//#define OFF1_THROTTLE 1300 // 

//#define MAX1_THROTHLE 4620 //  750 грамма = 90%
//#define MIN_THROTTLE 1550 //  33 грамма = 0%



 // 
//#define OFF1_THROTTLE_ 2050 // 
// //  максимальная мощность типпа 100 процентов. и дымок )
//#define MAX1_THROTHLE_ 3800 //  750 грамма = 90%
//#define MIN_THROTTLE_ 2150 //  33 грамма = 0%
// 16000000 / 8 / 40000 = 50Hz
  // 20 ms = 40000



//10000
class PwmClass
{
 protected:
	 

 public:
	 static int on(const uint16_t COUNTER, const uint16_t throthle );

	
	 static bool gimagl(float pitch, float roll);
	 static void get_analog(int16_t[]);


	 static void sound(const float);
	 static void beep_code(uint8_t);
	 static void throttle(const float n0, const float n1, const float n2, const float n3);
	static void throttle_0(const float n);
	static void throttle_1(const float n);
	static void throttle_2(const float n);
	static void throttle_3(const float n);


	static void setAll_(const uint16_t n){
		throttle_0(n);
		throttle_1(n);
		throttle_2(n);
		throttle_3(n);
	}

	static void throttle_0(const uint16_t n);
	static void throttle_1(const uint16_t n);
	static void throttle_2(const uint16_t n);
	static void throttle_3(const uint16_t n);
	static void Buzzer(const bool on);



private:

	static uint16_t correct(const float n);

};

extern PwmClass Pwm;

#endif

