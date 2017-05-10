// Settings.h

#include "mpu.h"
#ifndef _SETTINGS_h
#define _SETTINGS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define HMC_SAVED 0
#define HMC_CALIBR 1
#define MPU_SAVED 13
#define MPU_CALIBR 14
#define MOTOR_COMPAS  28
//37



#define COMPAS_S 'm'
#define MPU_S 'm'


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		EEPROM.write(ee++, *p++);
	return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = EEPROM.read(ee++);
	return i;
}


class SettingsClass
{
 protected:

	 void writeBuf(uint8_t adr, float buf[], uint8_t len){
		 for (uint8_t i = 0; i < len; i++){
			 EEPROM_writeAnything(i * 4 + adr, buf[i]);
		 }
	 }
	 void readBuf(uint8_t adr, float buf[], uint8_t len){
		 for (uint8_t i = 0; i < len; i++){
			 EEPROM_readAnything(i * 4 + adr, buf[i]);
		 }
	 }
	 void writeBuf(uint8_t adr, int16_t buf[], uint8_t len){
		 for (uint8_t i = 0; i < len; i++){
			 EEPROM_writeAnything(i*2 + adr, buf[i]);
		 }
	 }
	 void readBuf(uint8_t adr, int16_t buf[], uint8_t len){
		 for (uint8_t i = 0; i < len; i++){
			 EEPROM_readAnything(i * 2 + adr, buf[i]);
		 }
	 }


 public: 
	 bool loadCompssSettings2(float base[]){
		 readBuf(MOTOR_COMPAS, base, 12);
		 return true;
	 }
	 bool saveCompssSettings2(float base[]){

		 writeBuf(MOTOR_COMPAS, base, 12);
		 return true;
	 }
	
	 bool saveCompssSettings(int16_t sh[]){
		 EEPROM.write(HMC_SAVED, COMPAS_S);
		 writeBuf(HMC_CALIBR, sh, 6);
		 EEPROM.write_set();
		 return true;
	 }


	 bool readCompassSettings(int16_t sh[]){
		 bool ret = EEPROM.read(HMC_SAVED) == COMPAS_S;
		 if (ret){
			 readBuf(HMC_CALIBR, sh, 6);
		 }
		 return ret;
	 }

	 bool saveMpuSettings(int16_t sh[]){
		 EEPROM.write(MPU_SAVED, MPU_S);
		 writeBuf(MPU_CALIBR, sh, 6);
		 EEPROM.write_set();
		 return true;
	 }

	 bool saveMpuOnlyGyroSettings(int16_t sh[]){
		 EEPROM.write(MPU_SAVED, MPU_S);
		 writeBuf(MPU_CALIBR+6, sh+3, 3);
		 EEPROM.write_set();
		 return true;
	 }

	 bool readMpuSettings(int16_t sh[]){
		 bool ret = EEPROM.read(MPU_SAVED) == MPU_S;
		 if (ret){
			 readBuf(MPU_CALIBR, sh, 6);
		 }
		 return ret;
	 }

	

	void init();









};


	
extern SettingsClass Settings;

#endif

