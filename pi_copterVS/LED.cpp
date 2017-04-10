// 
// 
// 



// 0 1
// 2 3

#include "LED.h"
#include "debug.h"
#include "Hmc.h"

enum { LED_0 = 30, LED_2 = 32, LED_3 = 36, LED_1 = 38 };


const uint8_t led_indexes[]{ LED_0, LED_1, LED_2, LED_3 };

const uint16_t legMask[][4] = { 
	{ 0, 0, 0, 0 }, 
	{ 0xaaaa, 0xaaaa, 0xff00, 0xff00 }, 
	{ 0xffaa, 0xffaa, 0xffaa, 0xffaa }, 
	{ 0xEEEE, 0xDDDD, 0x7777, 0xBBBB},
	{ 0XFFFA, 0XFFAF, 0XAFFF, 0XFAFF },
	{ 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF }


};
const uint8_t ledMaskI[] = {0,1,1,1,2,3,5,4};
const uint8_t legShift[] = {0,0,1,2,0,1,0,0};



uint8_t LEDClass::prog(const uint8_t shift, const uint16_t mask){
	return ((mask >> ((c>>shift) & 15)) & 1) ;
}


void LEDClass::light(const uint8_t n){
	//digitalWrite(led_indexes[0], HIGH);
	//digitalWrite(led_indexes[1], HIGH);
	//digitalWrite(led_indexes[2], HIGH);
	//digitalWrite(led_indexes[3], HIGH);
	//digitalWrite(led_indexes[n], LOW);
}

void LEDClass::init()
{
	error_time = 0;
	prog_index = 0;
	light_on = true;
	//pinMode(LED_0, OUTPUT);
	//pinMode(LED_1, OUTPUT);
	//pinMode(LED_2, OUTPUT);
//pinMode(LED_3, OUTPUT);
}


uint32_t cnt = 0;
uint32_t step = 31;


void LEDClass::loop(){
#ifdef LED_ON
	uint32_t t = millis();
	if (t > cnt){
		cnt += step;
		c++; 
		if (error_time > 0 && t - error_time < 3000){
			//digitalWrite(LED_0, error_code & 1);
			//digitalWrite(LED_1, error_code & 2);
			//digitalWrite(LED_2, error_code & 4);
			//digitalWrite(LED_3, error_code & 8);
			return;
		}
		else
			error_time = 0;

		if (prog_index < 8 && Hmc.compas_motors_calibr == false){

			//digitalWrite(LED_0, prog(legShift[prog_index], legMask[ledMaskI[prog_index]][0]));
			//digitalWrite(LED_1, prog(legShift[prog_index], legMask[ledMaskI[prog_index]][1]));
			//digitalWrite(LED_2, prog(legShift[prog_index], legMask[ledMaskI[prog_index]][2]));
			//digitalWrite(LED_3, prog(legShift[prog_index], legMask[ledMaskI[prog_index]][3]));
		}
	}		
#else
	//digitalWrite(LED_0, HIGH);
	//digitalWrite(LED_1, HIGH);
	//digitalWrite(LED_2, HIGH);
	//digitalWrite(LED_3, HIGH);
#endif

}

void LEDClass::off(){
	//digitalWrite(LED_0, 1);
	//digitalWrite(LED_1, 1);
	//digitalWrite(LED_2, 1);
	//digitalWrite(LED_3, 1);
}


LEDClass LED;

