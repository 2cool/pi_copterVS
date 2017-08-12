#ifndef WPROGRAM_
#define WPROGRAM_

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <thread>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <fstream>


using namespace std;



#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

//#define string std::string
#define bool bool

#define byte uint8_t
#define WDTO_2S 2
#define WDTO_1S 1
//void wdt_enable(int a){};
//void digitalWrite(int i1, int i2){}
//void pinMode(int i1, int i2){}
#define INPUT 0
#define OUTPUT 0
#define LOW 0
#define HIGH 1
#define Serial2 Serial
#define Serial1 Serial

uint32_t millis(void);
int64_t micros(void);
void delay(unsigned long);


/*
const char * dtostre(float f, string ch[], int n, int fl=0){
		
		//int Number = 123;
		//string Result;
		ostringstream convert;
		convert << f;
		return convert.str();//.c_str();
		
	//cout<<Result<<endl;
	}
*/

/*
class Serial_class{
	public:
	void setTimeout(const int i){}
	void begin(int i){}
	void write(char* i, int ii){}
	bool find(char *c){return false;}
	bool available(){return false;}
	byte read(){return 0;}
	unsigned int fprintf(Debug.out_stream,char s[]);
	unsigned int fprintf(Debug.out_stream,char c);
	unsigned int fprintf(Debug.out_stream,string &s);
	unsigned int fprintf(Debug.out_stream,string s);
	unsigned int println(char s[]);
	unsigned int println(char c);
	//unsigned int println(string &s);
	unsigned int println(string s);
	unsigned int println(void);
	
};
*/





inline void sin_cos(const float a, float &s, float &c) {
	s = (float)sin(a);
	c = (float)cos(a);
	/*
	const double ss = s*s;
	c = (float)sqrt(1 - min(1.0f, ss));
	//30.7.2017 corected
	if (abs(a) > 90)
		c = -c;
		*/
}



#endif


