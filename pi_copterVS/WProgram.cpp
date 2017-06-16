#include <sys/types.h>
#include "WProgram.h"



uint32_t start_seconds = 0;

uint32_t millis(){
	timespec t;
	clock_gettime(CLOCK_REALTIME,&t);
	uint32_t ret;
	if (start_seconds == 0)
		start_seconds = t.tv_sec;
	ret=((t.tv_sec-start_seconds)*1000)+(t.tv_nsec/1000000);
	return ret;
}







int64_t micros(void){
	timespec t;
	clock_gettime(CLOCK_REALTIME,&t);
	int64_t ret;
	if (start_seconds == 0)
		start_seconds = t.tv_sec;
	ret=((int64_t)(t.tv_sec-start_seconds)*1000000)+(t.tv_nsec/1000);
	return ret;
}




void delay(unsigned long t){
	usleep(t*1000);
}




	#define EEPROM_SIZE 4096
	char EEPROM_MEM[EEPROM_SIZE];

	bool setings_file_exist = false;
	int EEPROM_Class::read_set() {
		ifstream f;
		f.open("/home/igor/eeprom.set");
		if (f.is_open()){
			f.read(EEPROM_MEM, EEPROM_SIZE);
			f.close();
			return 0;
		}
		return -1;
	}

	int EEPROM_Class::write_set() {


		
		ofstream f;
		f.open("/home/igor/eeprom.set", fstream::in | fstream::out | fstream::trunc);
		if (f.is_open()) {
			f.write(EEPROM_MEM, EEPROM_SIZE);
		//	f << EEPROM_MEM;
			f.close();
			return 0;
		}
		return -1;
	}



	
	void EEPROM_Class::write (int i, char c){
		if (EEPROM_SIZE > i) {
			EEPROM_MEM[i] = c;
		}
	};
	char EEPROM_Class::read(int i){
		if (EEPROM_SIZE > i) {
			return EEPROM_MEM[i];
		}else
		return 0;
	
	}
	
	
	
	
	
	
EEPROM_Class EEPROM;	
	

