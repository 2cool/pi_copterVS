// 
// 
// 

#include "commander.h"
#include "Pwm.h"
#include "Autopilot.h"
#include "Telemetry.h"
#include "Balance.h"
//#include "Hmc.h"
//#include "mpu.h"
#include "Prog.h"
#include "define.h"
#include "Hmc.h"
#include "Wi_Fi.h"
#include "Stabilization.h"
#include "debug.h"
#include "Wi_Fi.h"


void CommanderClass::init()
{
	//Out.println("COMMANDER INIT");
	data_size = 0;
	yaw = yaw_offset = pitch = roll = throttle = 0;


}

//static private final int s_COUNTER = 0;
//static private final int s_BUTTONS_S = 2;
//static private final int s_HEADING = 3;
//static private final int s_HEIGHT = 5;
//static private final int s_ACCELEROMETER = 7;



#define r_COUNTER  0
#define r_BUTTONS_S  2
#define r_HEADING  4
#define r_HEIGHT 6
#define r_ACCELEROMETER  8




#define  MAX_bit_on 219
#define  OFF_bit_on  36
#define  START_STOP 170












short old_height_c = 0;


/*
M,COUNTER,HEIGHT,HEADING,X,Y,
A,COUNTER,LAT,LON,HEIGHT,HEADING, SPEED,TIME,


*/
//max angel = 20 градусов
//




#define ANGK 0.1
uint8_t data_errors = 0;



static float ar_t333[SETTINGS_ARRAY_SIZE+1];
float * load(const string  buf, const uint8_t  * filds){
	ar_t333[SETTINGS_ARRAY_SIZE] = SETTINGS_IS_OK;
	for (int i = 0; i < SETTINGS_ARRAY_SIZE; i++){
		float val= stod(buf.substr(filds[i], filds[i + 1]- filds[i] - 1));
		if (val != 0 || buf.substr(filds[i], filds[i + 1]- filds[i] - 1).find("0.0")==0)
			ar_t333[i] = val;
		else{
			Out.println(buf.substr(filds[i], filds[i + 1]- filds[i] - 1));
			ar_t333[10] = SETTINGS_ERROR;
			Out.println(i);
			return ar_t333;
		}
	}

	return ar_t333;
}



uint8_t CommanderClass::_set(const float  val, float &set, bool secure){
	if (val == 0)
		return 1;
	if (secure == false){
		set = val;
	}else
		if (Autopilot.motors_is_on()){
			if (set*0.8 > val)
				set *= 0.8;
			else
			if (set*1.2 < val)
				set *= 1.2;
			else
				set = val;
		}else
			set = val;
	return 0;
}

bool CommanderClass::Settings(string buf){

	Out.println("settings");
	uint8_t filds[11];
	uint8_t fi = 0;
	uint8_t i = 0;

	i++;

	while (fi<10 && i<buf.length()){
		while (buf[i++] != ',');
		filds[fi++] = i++;

	}
	filds[10] = buf.length();

	uint8_t n = stoi(buf.substr(0, filds[0] - 1));
	switch (n){
	case 0:
		Balance.set(load(buf, filds));
		break;
	case 1:
		Stabilization.setZ(load(buf, filds));
		break;
	case 2:
		Stabilization.setXY(load(buf, filds));
		break;
	case 3:
		Autopilot.set(load(buf, filds));//secure
		break;
	case 4:
		Mpu.set(load(buf,filds));
		break;
	case 5:
		Hmc.set(load(buf, filds));
		break;
	default:
		return false;
	}
	return true;
	
}





/*

B,COUNTER,MAX,   full throttle
B,COUNTER,OFF,   off throttle
B,COUNTER,S_S,		start stop


*/






bool CommanderClass::ButtonMessage(string msg){

#ifdef DEBUG_MODE
	Out.print("<- "); Out.println(msg);
#endif

	bool command_correct = false;
/*
	
	if (msg.find(BEGIN_CONVERSATION) == 0){
		if (Autopilot.lost_conection_time > 0){
				
			Autopilot.lost_conection_time = 0;
			Telemetry.beginConversation();
			reset_old();
			res = true;
		}
		else{
			//Autopilot.lost_conection_time = millis();
		//	Autopilot.connectionLost_();
			return false;
		}

			
	}
	else */
	
	

	if (msg.find("UP") == 0 && msg[2] >= '0' && msg[2] <= '9'){
		int_fast8_t n = msg[2] - '0';
		//Out.println(msg);
		//Out.println(n);
		/////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		Telemetry.getSettings(n);
		command_correct = true;
	}

	if (command_correct == false)
		printf("WORNG MESSAGE\n");

	return command_correct;

}

float CommanderClass::getPitch(){ return Autopilot.horizont_onState() ? pitch : 0; }
float CommanderClass::getRoll(){ return Autopilot.horizont_onState() ? roll : 0; }


void CommanderClass::new_data(byte *buffer, int n) {
	//control_bits |= (MPU_ACC_CALIBR | MPU_GYRO_CALIBR)
	if (n > 4 && !Autopilot.busy()) {
		while (data_size)
			usleep(10000);

		memcpy(buf, buffer, n);
		//printf("in-> %i\n", buffer[0]);
		Autopilot.last_time_data_recived = millis();
		data_size = n;
	}
}


int get32to8bMask(int v) {
	int mask = v & 255;
	mask ^= ((v >> 8) & 255);
	mask ^= ((v >> 16) & 255);
	mask ^= ((v >> 24) & 255);
	return mask;
}

int get16to8bMask(int v) {
	int mask = v & 255;
	mask ^= ((v >> 8) & 255);

	return mask;
}

bool CommanderClass::input(){
	if (data_size) {

		//Autopilot.last_time_data_recived = millis();
		if (data_size >= 12) {
			uint32_t mode = *(uint32_t*)buf;
			int sec_mask = mode >> 24;
			
			mode &= 0x00ffffff;
			int mask = get32to8bMask(mode);

			int i = 4;
			int16_t i_throttle = *(int16_t*)(buf + i);
			mask ^= get16to8bMask(i_throttle);
			i += 2;
			

			int16_t i_yaw = *(int16_t*)(buf + i);
			i += 2;
			mask ^= get16to8bMask(i_yaw);
			

			int i_yaw_offset = *(int16_t*)(buf + i);
			i += 2;
			mask ^= get16to8bMask(i_yaw_offset);
			

			int i_pitch = *(int16_t*)(buf + i);
			i += 2;
			mask ^= get16to8bMask(i_pitch);
			
			int i_roll = *(int16_t*)(buf + i);
			i += 2;
			mask ^= get16to8bMask(i_roll);


			if (mask == sec_mask) {
				Autopilot.set_control_bits(mode);
				throttle = 0.00003125f*(float)i_throttle;
				yaw = -ANGK*(float)i_yaw;
				yaw_offset = ANGK*(float)i_yaw_offset;
				pitch = ANGK*(float)i_pitch;
				roll = ANGK*(float)i_roll;
				if ((i + 3) < data_size) {
					string msg = "";
					msg += *(buf + i++);
					msg += *(buf + i++);
					msg += *(buf + i++);
					if (msg.find(m_PROGRAM) == 0 && Autopilot.progState()==false) {
						Prog.add(buf + i);
					}
					else if (msg.find(m_SETTINGS) == 0) {
							Settings(string((char*)(buf+i)));
					}
					else if (msg.find("UPS") == 0) {
						Telemetry.getSettings(buf[i++]);
					}
				}
			}
			else {
				printf("COMMANDER ERROR\n");
			}
			data_size = 0;
			return true;
		}
	}

	return false;
}

CommanderClass Commander;

