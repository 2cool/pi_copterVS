// WiFi.h

#ifndef _WIFI_h
#define _WIFI_h


#include "WProgram.h"



#include "Telemetry.h"
#include "commander.h"

class WiFiClass
{
 protected:

	int connected;
	int blinkState;

	bool commander_done;
	bool command_resived;
	//thread t;
 public:
	 bool stopServer();
	~WiFiClass();
	bool connectedF();
	 bool newConnection_;
	bool is_connected(void){return connected>0;}
	int init(int cnt);

};

extern WiFiClass WiFi;

#endif

