// Telemetry.h

#ifndef _TELEMETRY_h
#define _TELEMETRY_h


#include "WProgram.h"
#include "define.h"
#include "debug.h"


enum {	NO_MESSAGE = 0, START_ROTTATION = 1, STOP_ROTTATION};

enum { T_TEMP = 0, T_PRES = 1, T_LAT = 3, T_LON = 7, T_GPS_HEIGHT = 11, T_GPS_HEADING = 13, T_HEADING = 14, T_ROL_X = 15, T_PITCH_Y = 16, T_BATARY = 17, T_RECIVED_COUNTER = 20,T_MES_CODE = 21,T_SPEED };

class TelemetryClass
{
 protected:
	 uint32_t power_on_time;
	 uint8_t buf[TELEMETRY_BUF_SIZE];
	 volatile int buffer_size;
	 void loadBUF32(int &i, int32_t val);
	 void loadBUF16(int &i, int16_t val);
	 void loadBUF(int &i,  const float fval);
	 void loadBUF8(int &i, const float val);

	 bool newGPSData;
	
	 
	 
	// uint8_t inner_clock_old_sec;
	
	 unsigned long next_battery_test_time;
	  uint32_t pressure;
	 float voltage,voltage_at_start;
	 uint32_t  time_at_start;
	 long timeAtStart;
	 
	 string message;
	 void update_buf();
 public:
	float b[3];
	uint32_t get_power_on_time() { return power_on_time; }
	bool power_is_on();

	 uint8_t no_time_cnt = 0;
	 void update_voltage();
	 float powerK;

	 void clearMessage(){ message = ""; }
	 void addMessage(const string msg);
	 string getMessage(){ return message; }
	 void getSettings(int n);
	 bool minimumTelemetry;
	 bool low_voltage,voltage50P;
	 
	 uint8_t lov_voltage_cnt;



	 int check_time_left_if_go_to_home();
	 void testBatteryVoltage();

	 void init_();

	 void loop();

	 float get_voltage(){ return voltage; }

	
	 int read_buf(byte *buf);
	// string getSport();
	 
	

};

extern TelemetryClass Telemetry;

#endif

/*


Charge voltage 	  					3.3V 	3.5V 	3.6V 	3.7V 	3.8V 	3.9V 	4.0V 	4.05V 	4.1V 	4.15V 	4.2V 	4.25V* 	4.3V*
Percentage of 4.2V capacity 	  	0% 		2.9% 	5.0% 	8.6% 	36% 	62% 	73% 	83% 	89% 	94% 	100% 	105% 	106%



*/