// 
// 
// 

#include "Ultrasound_Radar.h"
#define trigPin 49
#define interruptPin 3

#include "debug.h"
#include "Balance.h"

volatile uint32_t duration;
uint32_t time_bug = 0;

void radar() {
	duration = micros() - duration;

}
void Ultrasound_RadarClass::reset(){
	/*digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	dist2ground_ = UNKNOWN_VALUE;
	duration = 0;
	time_bug = millis();
	//Serial.println("reset");
	//Debug.load(0, dist2ground*0.25, 0);
	//Debug.dump();*/
}
void Ultrasound_RadarClass::init()
{
	motors_energized = false;
	time_bug = millis();
	//pinMode(trigPin, OUTPUT);
	//attachInterrupt(digitalPinToInterrupt(interruptPin), radar, CHANGE);
	//reset();
//#ifdef DEBUG_MODE
//	Serial.println("ultrasound init");
//#endif

}

#ifdef FALSE_ULTRASOUND_RADAR


void Ultrasound_RadarClass::loop() {



	const uint32_t t = millis();
	if (t - time_bug > 60){

		
			


			motors_energized = true;


			


			dist2ground_ = Mpu.altitude_Z;
			
			//Serial.println(dist2ground);
			
			

			
			//delayMicroseconds(2);
			duration = 0;
			time_bug = t;

			//Serial.print("dist2ground="); Serial.println(dist2ground_);
			//Serial.println(cnt);
			//	Debug.load(0, speed*0.1, dist2ground*0.5);
			//Debug.dump();
			//Serial.println(dist2ground);

		
		//else if ((t - time_bug) > 200U)
		//reset();
	}

}


#else
void Ultrasound_RadarClass::loop() {


	
	const uint32_t t = millis();
	if (t-time_bug > 60){
		if (duration>=0){
			if (duration < 1000000){
				motors_energized = true;
				dist2ground_ = ((float)duration)*0.00017;
				//Serial.println(dist2ground);
				//Debug.load(0, dist2ground / 2, 0);
				//Debug.dump();

				//digitalWrite(trigPin, LOW);
				//delayMicroseconds(2);
				//digitalWrite(trigPin, HIGH);
				//delayMicroseconds(10);
				//digitalWrite(trigPin, LOW);
				//delayMicroseconds(2);
				duration = 0;
				time_bug = t;
			}
		}
		else
			duration = 0;
		//else if ((t - time_bug) > 200U)
			//reset();
	}

}
#endif



Ultrasound_RadarClass Ultrasound_Radar;

