// 
// 
// 

#include "Settings.h"




#define EEPROM_SIZE 256
char EEPROM_MEM[EEPROM_SIZE];

template <class T> int writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		Settings.write(ee++, *p++);
	return i;
}

template <class T> int readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = Settings.read(ee++);
	return i;
}



uint32_t get_hash(const char *mem, const int len) {

	// Hashing
	uint32_t magic = 5381;
	for (int i = 4; i < len; i++)
		magic = ((magic << 5) + magic) + mem[i]; // magic * 33 + c		
	return magic;
}


void SettingsClass::init()
{


}


uint32_t SettingsClass::writeBuf(uint8_t adr, float buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		writeAnything(i * 4 + adr, buf[i]);
	}
}
uint32_t SettingsClass::readBuf(uint8_t adr, float buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		readAnything(i * 4 + adr, buf[i]);
	}
}
uint32_t SettingsClass::writeBuf(uint8_t adr, int16_t buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		writeAnything(i * 2 + adr, buf[i]);
	}
}
uint32_t SettingsClass::readBuf(uint8_t adr, int16_t buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		readAnything(i * 2 + adr, buf[i]);
	}
}

bool  SettingsClass::saveCompasMotorSettings(float base[]) {

	writeBuf(MOTOR_COMPAS, base, 12);
	uint32_t hash = get_hash((char*)base, 12);
	*(uint32_t*)(EEPROM_MEM + MOTOR_COMPAS_HASH) = hash;
	return write()!=-1;
	
}

bool  SettingsClass::saveCompssSettings(int16_t sh[]) {

	writeBuf(HMC_CALIBR, sh, 6);
	uint32_t hash = get_hash((char*)sh, 6);
	*(uint32_t*)(EEPROM_MEM + HCM_HASH) = hash;
	write();
	return true;
}

bool  SettingsClass::readCompassSettings(int16_t sh[]) {
	readBuf(HMC_CALIBR, sh, 6);
	uint32_t hash=get_hash((char*)sh, 6);
	bool ret = *(uint32_t*)(EEPROM_MEM + HCM_HASH) == hash;
	return ret;
}

bool  SettingsClass::readCompasMotorSettings(float base[]) {
	readBuf(MOTOR_COMPAS, base, 12);
	uint32_t hash = get_hash((char*)base, 12);
	bool ret = *(uint32_t*)(EEPROM_MEM + MOTOR_COMPAS_HASH) == hash;
	return ret;

}








int SettingsClass::read() {

	ifstream f;
	f.open("/home/igor/eeprom.set");
	if (f.is_open()) {
		f.read(EEPROM_MEM, EEPROM_SIZE);
		f.close();
		return 0;
	}
	fprintf(Debug.out_stream, "\n can't read settings \n");
	return -1;
}





int SettingsClass::write() {

	ofstream f;
	f.open("/home/igor/eeprom.set", fstream::in | fstream::out | fstream::trunc);
	if (f.is_open()) {
		f.write(EEPROM_MEM, EEPROM_SIZE);
		f.close();
		return 0;
	}
	fprintf(Debug.out_stream, "\n cant write settings \n");
	return -1;
}




void SettingsClass::write(int i, char c) {
	if (EEPROM_SIZE > i) {
		EEPROM_MEM[i] = c;
	}
};

char SettingsClass::read(int i) {
	if (EEPROM_SIZE > i) {
		return EEPROM_MEM[i];
	}
	else
		return 0;

}


SettingsClass Settings;

