



#include "WProgram.h"

#ifndef DEFINE_H

#define DEFINE_H


#define WATCHDOG WDTO_1S



//#define WIFIPRINT1
//#define WIFIPRINT2
//#define GPSPRINT
//#define MPU_PRINTTIME_CICLE


#define XY_SAFE_AREA 200
#define Z_SAFE_AREA 60

#define RAD2GRAD 57.29578f
#define GRAD2RAD 0.0174533f

//כמדט פכארטעü ןונוה נוסועמל ןמ גאקהמד


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




//#define DEBUG_MODE

// millis take less time then micros
//#define OFF_MOTOR_IF_LOST_CONNECTION  
#define _cos(_sin_) (sqrt(1 - min(1, _sin_*_sin_)))

//#define PLUS_CAMERA
#define WORK_WITH_WIFI
#define LED_ON


#define BEEPS_ON 0


#define ULTRASOUND_MAX_DETECT_HEIGHT 4


//#define FALSE_WIRE




//#define LOST_BEEP

//#define FALSE_GPS
//#define FASLE_GPS_STILL
//#define MOTORS_OFF
//#define NO_BATTERY

#ifdef FALSE_WIRE
#define FALSE_GPS
//#define FASLE_GPS_STILL
#define FALSE_COMPAS
#define FALSE_BAROMETR
#define FALSE_MPU
#define COMPAS_MOTORS_OFF  
#define NO_BATTERY
#define FALSE_ULTRASOUND_RADAR
//#define MOTORS_OFF
#define FALSE_ALTITUDE 30

#else
#define BUZZER_R
//#define GYRO_CALIBR
//#define ON_MAX_G_MOTORS_OFF
#endif



#define NEED_ANGLE_4_SPEED_10_MS 15.1f

#define FALL_IF_STRONG_WIND


#ifdef FALL_IF_STRONG_WIND
#define MAX_DIST_ERROR_TO_FALL 200.0f
#define e_TOO_STRONG_WIND "TSW"
#endif


#define NO_CONNECTION_DELAY_TO_RESET_IF_MOTORS_OFF 60000
#define NO_GPS_TIME_TO_FALL 5000
#define MIN_ACUR_HOR_POS_TO_FLY 7
#define MIN_ACUR_HOR_POS_2_START 5


#define PRESSURE_AT_0 101325

#define Out Serial
//#define ErrorLog Serial


#define MAX_ACC_HOR 3
#define MAX_ACC_VER 1.5f

#define MAX_ANGLE 30
#define MIN_ANGLE 15
#define COS_MIN_ANGLE 0.966f

#define HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME 30
#define FAST_DESENDING_TO_HIGH 15

#define MAX_HOR_SPEED 10
#define MAX_VER_SPEED_PLUS 5
#define MAX_VER_SPEED_MINUS -3

#define ACCURACY_XY 3
#define ACCURACY_Z 3
#define HOWER_TIME 60

#define MAX_DELTA 0.2f
#define MAX_YAW_DELTA 0.1f
#define FULL_THROTTLE_ 1.0f
#define MAX_THROTTLE_ (FULL_THROTTLE_-MAX_DELTA)


#ifdef PLUS_CAMERA
#define HOVER_THROTHLE 0.6
#define MIN_THROTTLE_ 0.5
#define FALLING_THROTTLE 0.55
#else
#define HOVER_THROTHLE 0.5f
#define MIN_THROTTLE_ 0.4f
#define FALLING_THROTTLE 0.45f
#endif



#define STOP_THROTTLE_ 0.2f



//#define BEGIN_ROTTATION_THROTTLE 1060;


#define MAX_G 32760  

#define TELEMETRY_BUF_SIZE 1024
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_PI(x) (x < -PI ? x+TWO_PI : (x > PI ? x - TWO_PI: x))


#define SETTINGS_ARRAY_SIZE 10
#define SETTINGS_IS_OK 1
#define SETTINGS_ERROR 0
#define BEGIN_CONVERSATION "GET"
#define e_OUT_OF_PER_H   "TFR"
#define e_OUT_OF_PER_V   "THG"
#define e_NO_WIFI_2_LONG "NWF"
#define e_LOW_VOLTAGE    "LWV"
#define e_GPS_ERROR      "GPE"
#define e_NO_GPS_2_LONG  "NGP"
#define e_MAX_ACCELERATION    "MXG"
#define e_GPS_ERRORS_M_50 "GER"
#define e_GPS_NO_UPDATE  "GRR"
#define e_VOLTAGE_ERROR "TEE"
#define e_VOLT_MON_ERROR "VME"
#define e_PRESURE_DEV_ZER "PRE"
#define e_BATERY_OFF_GO_2_HOME "BOH"
#define e_CONNECTION_TIME_LAG "LAG"
#define e_LOST_CONNECTION "LST"
#define e_SYSTEM_MALFUNCTION "WDT"
#define i_OFF_MOTORS     "MD0"
#define i_CONTROL_FALL   "CNF"
#define i_MAX_THR        "MXT"

#define m_START_STOP  "S_S"
#define m_HOLD_HIGHT  "AHD"
#define m_SMART_CNTR  "SCT"
#define m_GO_2_HOME   "THM"
#define m_MPU_GYRO_CAL "STS"
#define m_COMPAS_CAL  "CMC"
#define m_MAX_THR     "MAX"
#define m_OFF_THR     "OFF"
#define m_MPU_NEW_CAL "HOR"
#define m_RESET       "RST"
#define m_GIMBAL_PA   "CDN"
#define m_GIMBAL_PS   "CUP"
#define m_DIRECTION_C "CMP"
#define m_XY_CONTROL  "HRT"
#define m_MOTOR_COMP_C "MCC"
#define m_START_PROG   "SRP"
#define m_SETTINGS	   "SET"
#define m_PROGRAM	   "PRG"

#endif
