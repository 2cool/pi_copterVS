// 
// 
// 

#include "debug.h"








float DebugClass::readSeralFloat() {  
	uint8_t buf[20];
	float n = 0;
	float d = 0.1;
	bool dot = false;
	int len = 0;//Out.readBytes(buf,10);
	for (int i = 0; i < len; i++) {
		if (buf[i] == '.')
			dot = true;
		else
			if ((buf[i] >= '0') && (buf[i] <= '9'))
				if (dot == false) {
					n *= 10;
					n += buf[i] - '0';
				}
				else {
					n += (buf[i] - '0')*d;
					d *= 0.1;
				}
	}
	return n;
}

void DebugClass::graphic(const uint8_t n, const float x, const float y) {
	printf("%i,%f,%f\n", (unsigned int)n, x, y);

}
 
	 void DebugClass::init() { i = 0; }
	 void DebugClass::dump(const long f1, long f2, long f3, long f4) {
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
	 void DebugClass::dump(const float f1, float f2, float f3, float f4) {
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
	 void DebugClass::dump(const uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4) {



		 printf("%i,%i,%i,%i\n", (unsigned int)f1, (unsigned int)f2, (unsigned int)f3, (unsigned int)f4);
		/* Out.println();
		 Out.print(f1);
		 Out.print(",");
		 Out.print(f2);
		 Out.print(",");
		 Out.print(f3);
		 Out.print(",");
		 Out.print(f4);
		 Out.println();
		 */
	 }
	 void DebugClass::load(const uint8_t i, const float x, const float y) {
		 ar[i][0] = x;
		 ar[i][1] = y;
	 }
	// int cnt = 0;
	 uint32_t old_time = 0;
	 void DebugClass::dump() {//--------------------------------------------------------------
		 if (n_debug > 9)
			 return;
		 uint32_t t = millis();
		 if (t - old_time < 100)//20)
			 return;
		 old_time = t;

		 /*
		 int n = 0;//Out.available();
		 if (n > 3) {
			 float f = readSeralFloat();
			 Out.println(f);
			 Mpu.temp_deb = f;
			 // Mpu.faccX.setF(f);
			 //	 Mpu.faccY.setF(f);
			 //	 Mpu.faccZ.setF(f);
		 }
		 else {
			 int b = 0;//Out.read();
			 if (b >= '0' && b <= '9')
				 i = b - '0';
		 }
		 //if (((++cnt) & 7) == 0){
		 */
		 i = n_debug;

		 graphic(i, ar[i][0], ar[i][1]);
		 // }
		 // Out.println(Mpu.temp_deb);
	 }
DebugClass Debug;

