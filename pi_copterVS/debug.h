// debug.h

#ifndef _DEBUG_h
#define _DEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "mpu.h"

class DebugClass
{
 protected:

float readSeralFloat(){
	uint8_t buf[20];
	float n = 0;
	float d = 0.1;
	bool dot = false;
	int len=0;//Out.readBytes(buf,10);
	for (int i = 0; i < len; i++){
		if (buf[i] == '.')
			dot = true;
		else
			if ((buf[i] >= '0') && (buf[i] <= '9'))
				if (dot==false){
					n *= 10;
					n += buf[i] - '0';
				}
				else{
					n+=(buf[i] - '0')*d;
					d *= 0.1;
				}
	}
	return n;
}



#define MAXARR 10
	 int i;
	 float ar[MAXARR][2];
	void graphic(const uint8_t n,const float x,const float y) {
		Out.print(n);
		Out.print(",");
		Out.print(x);
		Out.print(",");
		Out.println(y);
	}
 public:
	 void init(){ i = 0; }
	 void dump(const long f1, long f2, long f3, long f4){
		 Out.println();
		 Out.print(f1);
		 Out.print(",");
		 Out.print(f2);
		 Out.print(",");
		 Out.print(f3);
		 Out.print(",");
		 Out.print(f4);
		 Out.println();

	 }
	 void dump(const float f1, float f2, float f3, float f4){
		 Out.println();
		 Out.print(f1);
		 Out.print(",");
		 Out.print(f2);
		 Out.print(",");
		 Out.print(f3);
		 Out.print(",");
		 Out.print(f4);
		 Out.println();

	 }
	 void dump(const uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4){
		 Out.println();
		 Out.print(f1);
		 Out.print(",");
		 Out.print(f2);
		 Out.print(",");
		 Out.print(f3);
		 Out.print(",");
		 Out.print(f4);
		 Out.println();

	 }
	 void load(const uint8_t i, const float x, const float y){
		 ar[i][0] = x;
		 ar[i][1] = y;
	 }
	 int cnt = 0;
	 void dump(){
		 int n = 0;//Out.available();
		 if (n > 3){
			 float f = readSeralFloat();
			 Out.println(f);
			 Mpu.temp_deb = f;
			// Mpu.faccX.setF(f);
		//	 Mpu.faccY.setF(f);
		//	 Mpu.faccZ.setF(f);
		 }
		 else{
			 int b = 0;//Out.read();
			 if (b >= '0' && b <= '9')
				 i = b - '0';
		 }
		 //if (((++cnt) & 7) == 0){
			 graphic(i, ar[i][0], ar[i][1]);
		// }
			// Out.println(Mpu.temp_deb);
	 }




};

extern DebugClass Debug;

#endif

