// 
// 
// 


/*
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
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>


#include "define.h"
#include "Pwm.h"
#include "mpu.h"
//настроить на 16 мигагерц

#define _16MHZ CS10
#define _8MHZ CS11
/*




timer	bits	channel	Arduino pin	Mega pin
timer0	8		A		6			13
timer0	8		B		5			4
timer1	16		A		9			11
timer1	16		B		10			12
timer2	8		A		11			10
timer2	8		B		3			9
timer3	16		A		-			5
timer3	16		B		-			2
timer3	16		C		-			3
timer4	16		A		-			6
timer4	16		B		-			7
timer4	16		C		-			8
timer5	16		A		-			44
timer5	16		B		-			45
timer5	16		C		-			46







5	PE3 ( OC3A/AIN1 )			Digital pin 5 (PWM)
6	PE4 ( OC3B/INT4 )			Digital pin 2 (PWM)
7	PE5 ( OC3C/INT5 )			Digital pin 3 (PWM)
15	PH3 ( OC4A )				Digital pin 6 (PWM)
16	PH4 ( OC4B )				Digital pin 7 (PWM)
17	PH5 ( OC4C )				Digital pin 8 (PWM)
18	PH6 ( OC2B )				Digital pin 9 (PWM)
23	PB4 ( OC2A/PCINT4 )			Digital pin 10 (PWM)
24	PB5 ( OC1A/PCINT5 )			Digital pin 11 (PWM)
25	PB6 ( OC1B/PCINT6 )			Digital pin 12 (PWM)
26	PB7 ( OC0A/OC1C/PCINT7 )	Digital pin 13 (PWM)
PL3 ( OC5A )					Digital pin 46 (PWM)
39	PL4 ( OC5B )				Digital pin 45 (PWM)
40	PL5 ( OC5C )				Digital pin 44 (PWM)


*/

#define  ARDUINO_ADDR 9
int fd;

int PwmClass::on(const uint16_t COUNTER, const uint16_t throthle)
{

	printf("arduino connection test\n");
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		printf("Failed to open the bus.\n");
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, ARDUINO_ADDR) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
/*
	if (write(fd, pwm_out_buffer, 12) != 12) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
		return -1;
	}


	usleep(10000);
	if (read(fd, pwm_in_buffer, 3) != 3) {
		printf("read reg 8 bit Failed to read to the i2c bus.\n");
		return -1;
	}else
		printf("%i,%i,%i\n", (int)pwm_in_buffer[0], (int)pwm_in_buffer[1], (int)pwm_in_buffer[2]);

		*/

	return 0;








	//old_g_pitch = 1000;


	//pinMode(46, OUTPUT);  
	//pinMode(3, OUTPUT);//
	//pinMode(5, OUTPUT);
	//pinMode(6, OUTPUT);
	//pinMode(8, OUTPUT);



	//pinMode(7, OUTPUT);   // buzzer
	//pinMode(44, OUTPUT);   // gimbal roll
	//pinMode(45, OUTPUT);   // bimbal pitch



	//TCCR3A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM13) | _BV(WGM12) | _BV(WGM11);
	//TCCR3B = _BV(_16MHZ) | _BV(WGM12) | _BV(WGM13);

	//TCCR4A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM13) | _BV(WGM12) | _BV(WGM11);
	//TCCR4B = _BV(_16MHZ) | _BV(WGM12) | _BV(WGM13);


	//TCCR5A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM13) | _BV(WGM12) | _BV(WGM11);
	//TCCR5B = _BV(_16MHZ) | _BV(WGM12) | _BV(WGM13);



	//ICR3 = ICR4 = ICR5 = COUNTER;

	//OCR5B = pwm_OFF_THROTTLE +(pwm_OFF_THROTTLE / 2);
	//OCR5C = pwm_OFF_THROTTLE +(pwm_OFF_THROTTLE / 2);

	//OCR5A = OCR3A = OCR4A = OCR4C = throthle;// pwm_OFF_THROTTLE;
	//OCR4B = 0;// 35000;







	//TIMSK3 |= (1 << TOIE3);  //прерывание по переполнению счетчика
	//TCNT1 = 0;	     //счетчик

}
static float old_g_roll = 0;
static float old_g_pitch = 0;
bool PwmClass::gimbal_roll( float angle){

	if (old_g_roll != angle && abs(angle) <= 45){
		old_g_roll = angle;
		angle = angle*(1.0 / 180)*(pwm_OFF_THROTTLE / 2) + ((pwm_OFF_THROTTLE / 2)) + pwm_OFF_THROTTLE;
		//OCR5B = angle;
		return true;
	}
	return false;
}
bool PwmClass::gimagl_pitch( float angle){


	if (old_g_pitch != angle && angle >= -90 && angle <= 10) {
		angle = -angle;
		old_g_pitch = angle;
		//Serial.print("camAng="); Serial.println(angle);
		angle = angle*(1.0 / 180)*(63) + (63) + 127;

		char buf[2];
		((int8_t*)buf)[0] = angle;
		((int8_t*)buf)[1] = 127;
		write(fd, buf,2);
		return true;
	}
	else
		return false;

}

void PwmClass::Buzzer(const bool on){
//	if (on)
//		pinMode(7, OUTPUT);   // buzzer
//	else
	//	pinMode(7, INPUT);
}


uint16_t PwmClass::correct(const float n){    //0-это

		return pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE);
	
}

//#define PWM_PRINT


//m0=0.2   280-449=   170    068		cor 123
//m1=0.2    209-327=  120   =1
//m2=0.2   327 - 484  160	0.75       cor 124
//m3=0.2   327-473 =  145    0.75		cor 123


#ifdef T200HZDDD
float pwm_temp[4] = { 0,0,0,0 };
int pwm_write = 0;
void PwmClass::throttle(const float n0, const float n1, const float n2, const float n3) {
	pwm_temp[0] += (n0 - pwm_temp[0])*0.33;
	pwm_temp[1] += (n1 - pwm_temp[1])*0.33;
	pwm_temp[2] += (n2 - pwm_temp[2])*0.33;
	pwm_temp[3] += (n3 - pwm_temp[3])*0.33;
	if (pwm_write==3) {
		pwm_write = 0;
		char pwm_out_buffer[8];
		((uint16_t*)&pwm_out_buffer)[0] = 1000 + (uint16_t)(pwm_temp[0] * 1000);
		((uint16_t*)&pwm_out_buffer)[1] = 1000 + (uint16_t)(pwm_temp[1] * 1000);
		((uint16_t*)&pwm_out_buffer)[2] = 1000 + (uint16_t)(pwm_temp[2] * 1000);
		((uint16_t*)&pwm_out_buffer)[3] = 1000 + (uint16_t)(pwm_temp[3] * 1000);

		write(fd, pwm_out_buffer, 8);

	}
	pwm_write++;
	

}
#else

void PwmClass::throttle(const float n0, const float n1, const float n2, const float n3) {
	char pwm_out_buffer[4];
	((uint8_t*)&pwm_out_buffer)[0] = 127 + (uint16_t)(n0 * 127);
	((uint8_t*)&pwm_out_buffer)[3] = 127 + (uint16_t)(n1 * 127);
	((uint8_t*)&pwm_out_buffer)[1] = 127 + (uint16_t)(n2 * 127);
	((uint8_t*)&pwm_out_buffer)[2] = 127 + (uint16_t)(n3 * 127);
	write(fd, pwm_out_buffer, 4);

}

#endif

void PwmClass::get_analog(int16_t buffer[]) {
	read(fd, (uint8_t*)buffer, 6);
}



void PwmClass::throttle_0(const float n){ 
	//OCR5A = pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE);//0
	
}//46  
void PwmClass::throttle_2(const float n){ 

	//OCR3A = pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE); //1
}//5
void PwmClass::throttle_3(const float n){ 

	//OCR4A = pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE);//2
}//6
void PwmClass::throttle_1(const float n){ 

	 //OCR4C = pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE);//3
}//8

void PwmClass::sound(const float f){ 
#ifdef BUZZER_R
	//OCR4B = 32000-(int16_t)f; 
#endif

}
void PwmClass::throttle_0(const uint16_t n){ 
	//OCR5A = n; 
	}//46 
void PwmClass::throttle_1(const uint16_t n){ 
	//OCR3A = n; 
	}//5
void PwmClass::throttle_2(const uint16_t n){ 
	//OCR4A = n; 
	}//6
void PwmClass::throttle_3(const uint16_t n){ 
	//OCR4C = n; 
	}//8





	//OCR4B = 38000; //pin7  реверс. 
	//OCR3B = 38000; //pin2  реверс тоесть если 2000 то на выходе будет как 38000. почему незнаю ) 


PwmClass Pwm;

