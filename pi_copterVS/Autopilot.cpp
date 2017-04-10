// 
// 
// 


/*
message for LOG

MXT     max throttle
OFT		off_throttle
MD1		motors_do_on
MD0		motors do off
LWV		low voltage
GPE		gps error
CNF		controled fall
MXG		перегрузка
MODXXX  резим работі автопилота,XXX - число - control_bits

TFR	out of Perimetr gps
THG out of Perimetr high



*/






//добавить проверку на возможность вернутся домой.


#include "define.h"
#include "WProgram.h"

#include "Pwm.h"
#include "MS5611.h"
#include "Autopilot.h"
#include "Balance.h"
#include "Ultrasound_Radar.h"
#include "commander.h"

#include "GPS.h"

#include "LED.h"
#include "Telemetry.h"
#include "Stabilization.h"
#include "debug.h"
#include "Prog.h"
#include "Wi_Fi.h"
//каждий новий режим работі добовляется в месадж

#define MIDDLE_POSITION 0.5


void AutopilotClass::init(){/////////////////////////////////////////////////////////////////////////////////////////////////
	lowest_height = LOWEST_HEIGHT_;
	last_time_data_recived = 0;
	gimBalPitch = 0;
	Balance.init();
	MS5611.init();


	sens_z = 6;
	sens_xy = 0.2;

	newData = false;

	//holdLocationStartStop(false);
	//holdAltitudeStartStop(false);

	lost_conection_time = millis();
	height_to_lift_to_fly_to_home = HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME;
	aPitch = aRoll = aYaw_=0;

	//was_connected_to_wifi = NO_WIFI_WATCH_DOG_IN_SECONS < 30;
	control_bits = COMPASS_ON|HORIZONT_ON;
	aPitch = aRoll = 0;

//	for (int i = 0; i < 10; i++)
//	while (MS5611.loop() != 0)
//		delay(10);

	controlDeltaTime = millis();

}


void AutopilotClass::add_2_need_yaw(float speed, const float dt){
	aYaw_ += speed*dt;
	aYaw_ = wrap_180(aYaw_);
}



void AutopilotClass::add_2_need_altitude(float speed, const float dt){


	if (speed > 0){
		if (speed > MAX_VER_SPEED_PLUS)
			speed = MAX_VER_SPEED_PLUS;
	}else
		if (speed < MAX_VER_SPEED_MINUS)
			speed = MAX_VER_SPEED_MINUS;

	tflyAtAltitude += speed * dt;
	if (tflyAtAltitude < lowest_height)
		tflyAtAltitude = lowest_height;

	flyAtAltitude = tflyAtAltitude + Stabilization.getDist_Z(speed);
	if (flyAtAltitude < lowest_height)
		flyAtAltitude = lowest_height;

}
//-------------------------------------------------------------------------
void AutopilotClass::smart_commander(const float dt){
	if (Commander.getPitch() != 0 || Commander.getRoll() != 0){
		const float addX = sens_xy*(Commander.getPitch());
		const float addY = -sens_xy*(Commander.getRoll());
		const float yaw = Commander.get_contr_yaw()*GRAD2RAD;
		const float cosL = cos(yaw);
		const float sinL = sin(yaw);
		float speedX = addX * cosL + addY *sinL;
		float speedY = -(addX * sinL - addY *cosL);
		const float speed2 = (speedX*speedX + speedY*speedY);
		const float maxSpeed2 = Stabilization.max_speed_xy*Stabilization.max_speed_xy;
		if (speed2>maxSpeed2){
			float k = sqrt(maxSpeed2 / speed2);
			speedY *= k;
			speedX *= k;

		}
		GPS.loc.add2NeedLoc(speedX, speedY, dt);
	}
	else{
		GPS.loc.setSpeedZero();
	}
}

void AutopilotClass::loop(){/////////////////////////////////////////////////////////////////////////////////////////////////
	
	const uint32_t t = millis();
	const float dt = 0.001*(float)(t - controlDeltaTime); 
	if (dt < 0.05)
		return;
	controlDeltaTime = t;
	uint8_t smart = 0;

	if (control_bits&CONTROL_FALLING){
		aYaw_ = Mpu.yaw;
		off_throttle(false,"cntr_fall");
	}
	else{
		if (control_bits & PROGRAM){
			
			Prog.loop();
		}
		else{
			if (control_bits & GO2HOME){
				LED.prog_index = LED.GO_TO_HOME_P;
				go2HomeProc(dt);
			}
			else{

				const bool timeLag = (millis() - last_time_data_recived > 100);

				if (compass_onState())
					aYaw_ = Commander.getYaw();
				if (control_bits & Z_STAB){
					smart++;
					const float thr = timeLag ? MIDDLE_POSITION : Commander.getThrottle();
					const float speed = (thr - MIDDLE_POSITION) * sens_z;
					add_2_need_altitude(speed, dt);
				}
				else{
					throttle = Commander.getThrottle();
				}
				if (control_bits & XY_STAB){
					smart++;
					smart_commander(dt);
				}
				else{
					if (timeLag){
						aPitch = aRoll = 0;
					}
					else{
						aPitch = Commander.getPitch();
						aRoll = Commander.getRoll();
					}
				}
				LED.prog_index = (smart == 0) ? LED.MANUAL_P : ((smart == 2) ? LED.FULL_SMART_P : LED.SEMI_SMART_P);
			}
			
		}
			
	}

}


string AutopilotClass::get_set(){
	
	ostringstream convert;
	convert<<\
	height_to_lift_to_fly_to_home<<","<<\
	MAX_THROTTLE_<<","<<\
	MIN_THROTTLE_<<","<<\
	sens_xy<<","<<\
	sens_z<<","<<\
	lowest_height;
	string ret = convert.str();
	return string(ret);
}

void AutopilotClass::set(const float ar[]){
	Out.println("Autopilot set");
	uint8_t error = 1;

	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		uint8_t i = 0;
		error = 0;
		
		
		error += Commander._set(ar[i++], height_to_lift_to_fly_to_home);

		//Balance.set_min_max_throttle(ar[i++], ar[i++]);
		i += 2;
		error += Commander._set(ar[i++], sens_xy);
		error += Commander._set(ar[i++], sens_z);
		error += Commander._set(ar[i], lowest_height,false);

		if (error == 0){
			int ii = 0;
			Out.println("Safe set:");

			for (ii = 0; ii < i; ii++){
				Out.print(ar[ii]); Out.print(",");
			}
			Out.println(ar[ii]);
		}
	}
	if (error>0){
		Out.println("ERROR");
	}
}







void AutopilotClass::set_new_altitude(float alt){
	tflyAtAltitude = flyAtAltitude = alt;
}

boolean AutopilotClass::holdAltitude(float alt){

	tflyAtAltitude = flyAtAltitude = alt;
	if ((control_bits & Z_STAB) == 0){
		control_bits |= Z_STAB;
		Stabilization.init_Z();
	}
	Out.print("FlyAt:");
	Out.println(flyAtAltitude);

	return true;
}

boolean AutopilotClass::holdAltitudeStartStop(){

	if (!motors_onState() || go2homeState() || progState())
		return false;
	bool h = (control_bits & Z_STAB)==0;
	if (h){
		return holdAltitude(corectedAltitude());
	}
	else{
		control_bits ^= Z_STAB;
		throttle = HOVER_THROTHLE;
		return true;
		
	}
	return false;
}
//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
enum{ JUMP = 0, HOWER = 1, GO_UP_OR_NOT = 2, TEST_ALT1 = 3, GO2HOME_LOC = 4, TEST4HOME_LOC = 5, START_FAST_DESENDING = 6, TEST_ALT2 = 7,SLOW_DESENDING=8 };
boolean AutopilotClass::go2HomeProc(const float dt){
	
		
 switch (go2homeIndex){

	case JUMP:{	
#ifdef FALL_IF_STRONG_WIND
				  dist2home_at_begin2 = GPS.loc.dist2home_2;
#endif
				  if (corectedAltitude() < 3)
					  holdAltitude(3);
				  go2homeIndex=HOWER;
				  break;
	}

	case HOWER:{	//висеть 20 секунд
			   f_go2homeTimer += dt;
			   if (f_go2homeTimer > ((howeAt2HOME)?HOWER_TIME:4)){ //for stabilization and connection
				   go2homeIndex = GO_UP_OR_NOT;
			   }
			   break;

	}
	case GO_UP_OR_NOT:{
			   const float accuracy = ACCURACY_XY + GPS.loc.accuracy_hor_pos;
			   if (fabs(GPS.loc.x2home) <= accuracy && fabs(GPS.loc.y2home) <= accuracy){
				   f_go2homeTimer = 6; //min time for stab
				   go2homeIndex = (corectedAltitude() <= (FAST_DESENDING_TO_HIGH)) ? SLOW_DESENDING : START_FAST_DESENDING;
				   GPS.loc.setNeedLoc2HomeLoc();
				   break;
			   }	
			   //поднятся  на высоту  X м от стартовой высоты или опуститься
			   tflyAtAltitude=flyAtAltitude = height_to_lift_to_fly_to_home;
			   go2homeIndex = TEST_ALT1;
			   break;
	}
	case TEST_ALT1:{
			   if (fabs(corectedAltitude() - flyAtAltitude) <= (ACCURACY_Z)){
				   go2homeIndex = GO2HOME_LOC;
			   }
			   break;
	}
	case GO2HOME_LOC:{//перелететь на место старта
			   // led_prog = 4;
			   GPS.loc.setNeedLoc2HomeLoc();
			   go2homeIndex = TEST4HOME_LOC;
			   break;
	}
	case TEST4HOME_LOC:{//прилет на место старта
			   const float accuracy = ACCURACY_XY + GPS.loc.accuracy_hor_pos;
			   if (fabs(GPS.loc.x2home) <= accuracy && fabs(GPS.loc.y2home) <= accuracy){
				   go2homeIndex = START_FAST_DESENDING;
				   f_go2homeTimer = 0;
			   }
			   break;
	}
	case START_FAST_DESENDING:
		f_go2homeTimer += dt;
		if (f_go2homeTimer > 5){
			go2homeIndex = TEST_ALT2;
			tflyAtAltitude = flyAtAltitude = (FAST_DESENDING_TO_HIGH);
		}
		break;


	case TEST_ALT2:{//спуск до FAST_DESENDING_TO_HIGH метров
			   if (fabs(corectedAltitude() - flyAtAltitude) < (ACCURACY_Z)){
				   go2homeIndex = SLOW_DESENDING;
				}
			   
			   break;
	}
	case SLOW_DESENDING:
	{ 
		//плавній спуск
				if (corectedAltitude()>lowest_height){
					float k = corectedAltitude()*0.05;
					if (k < 0.1)
						k = 0.1;
					flyAtAltitude -= (dt*k);
					tflyAtAltitude = flyAtAltitude;
				}
				else{
					tflyAtAltitude = flyAtAltitude = lowest_height;
				}
						   
			   break;
	}
 }
#ifdef FALL_IF_STRONG_WIND
	if (GPS.loc.dist2home_2 - dist2home_at_begin2 > (MAX_DIST_ERROR_TO_FALL*MAX_DIST_ERROR_TO_FALL)){
		Autopilot.off_throttle(false, e_TOO_STRONG_WIND);
	}
#endif
	return true;
}
boolean AutopilotClass::going2HomeON(const bool hower){

	Stabilization.setDefaultMaxSpeeds();

	howeAt2HOME = hower;//зависнуть на месте или нет

	bool res = holdAltitude(corectedAltitude());
	res &= holdLocation(GPS.loc.lat_, GPS.loc.lon_);
	if (res){
		control_bits |= GO2HOME;
		f_go2homeTimer = 0;
		Out.println("Hanging on the site!");
		go2homeIndex=JUMP;
	}
	return res;
}

boolean AutopilotClass::going2HomeStartStop(const bool hower){
	
	if (!motors_onState())
		return false;
	bool f = (control_bits & GO2HOME);
	if (f){
		tflyAtAltitude = flyAtAltitude;
		control_bits ^=GO2HOME;
		holdLocation(GPS.loc.lat_, GPS.loc.lon_);
		return true;
	}
	else{
		if (progState())
			start_stop_program(true);
		return going2HomeON(hower);
	}
}


boolean AutopilotClass::holdLocation(const long lat, const long lon){
	control_bits &= (0xffffffff ^ (COMPASS_ON | HORIZONT_ON));
	aPitch = aRoll = 0;
	//if (holdAltitude()){

		GPS.loc.setNeedLoc(lat,lon);
		Out.print("Hower at:");
		Out.print(GPS.loc.lat_);
		Out.print(",");
		Out.println(GPS.loc.lon_);
		oldtime = millis();
		
		//float cosBearing = cos(GPS.bearing);
		//float sinBearing = sin(GPS.bearing);
		Stabilization.init_XY(
			0,//GPS.loc.x2home,
			0,// GPS.loc.speedX,
			0,//GPS.loc.y2home,
			0);// GPS.loc.speedY);

		control_bits |= XY_STAB;
		return true;
	//}
	//else
	//	return false;
}

boolean AutopilotClass::holdLocationStartStop(){/////////////////////////////////////////////////////////////////////////////////////////////////
	if (!motors_onState() || go2homeState() || progState())
		return false;
	bool h = (control_bits &XY_STAB)==0;
	if (h){
		return holdLocation(GPS.loc.lat_, GPS.loc.lon_);
	}
	else{
		control_bits |= (COMPASS_ON | HORIZONT_ON);
			control_bits ^=  XY_STAB;
			//holdAltitude();
			return true;
	}
	return false;
}

boolean AutopilotClass::motors_do_on(const boolean start, const string msg){////////////////////////  M O T O R S  D O  ON  /////////////////////////////////////////////////////////////////////////
	//Out.print(msg);
	if (start){
		LED.error_code = 0;
		LED.error_code = 255;
		if (Mpu.gyro_calibratioan && Hmc.calibrated){

			if (Telemetry.low_voltage){
				Telemetry.addMessage(e_LOW_VOLTAGE);
				Out.println(" LOW VOLTAGE");
				return false;
			}

			if (GPS.loc.accuracy_hor_pos > MIN_ACUR_HOR_POS_2_START ){
				Out.println(" GPS error");
				Telemetry.addMessage(e_GPS_ERROR);
				LED.error_time = millis();
				LED.error_code &= 255^1;
				return false;
			}
				
			Telemetry.update();
			
			control_bits = MOTORS_ON;

			//Out.println(" - OK");

			GPS.loc.setHomeLoc();

			MS5611.copterStarted();
			tflyAtAltitude = flyAtAltitude = corectedAltitude();
			
			Mpu.max_g_cnt = 0;
			holdAltitude(3);
			holdLocation(GPS.loc.lat_, GPS.loc.lon_);
			aYaw_ = Mpu.yaw;

#ifdef DEBUG_MODE
			Out.print("\nhome loc:");
			Out.print(GPS.loc.lat_);
			Out.print(" ");
			Out.print(GPS.loc.lon_);
			Out.print(" ");
			Out.print(GPS.loc.altitude);
			Out.println("M");
			Out.println("home alt set");
			Out.println(flyAtAltitude);
#endif







		}
		else{
			if (Hmc.calibrated == false){
				Out.print("compas, ");
				LED.error_time = millis();
				LED.error_code &= 255 ^ 2;
			}
			if (Mpu.gyro_calibratioan == false){
				Out.print("gyro");
				LED.error_time = millis();
				LED.error_code &= 255 ^ 4;
			}
			Out.println(" calibr FALSE");
		}
	}//------------------------------OFF----------------
	else {
		Telemetry.addMessage(i_OFF_MOTORS);
		off_throttle(true, msg);

		Out.println(" - OK");
	}
	return true;
}

void AutopilotClass::control_falling(const string msg){
	if (motors_is_on() && (control_bits & CONTROL_FALLING) == 0){
		throttle = FALLING_THROTTLE*Balance.powerK();
		aPitch = aRoll = 0;
#ifdef DEBUG_MODE
		Out.println("CNTROLL FALLING");
#endif
		Telemetry.addMessage(msg);
		Telemetry.addMessage(i_CONTROL_FALL);
		control_bits = CONTROL_FALLING | MOTORS_ON;
	}
}

boolean AutopilotClass::off_throttle(const boolean force, const string msg){/////////////////////////////////////////////////////////////////////////////////////////////////
	
	if ( force)
	{

		Balance.set_off_th_();
		Telemetry.addMessage(msg);
		control_bits = 0;
		return true;
	}
	else{
	//	if (control_bits_ & (255 ^ (COMPASS_ON | HORIZONT_ON)))
	//		return true;

		if (corectedAltitude()  < 2){
			motors_do_on(false,msg);
		}
		else{
			control_falling(msg);
			
			//Out.println(throttle);
		}
	}
	return false;

}

void AutopilotClass::connectionLost_(){ ///////////////// LOST


	//Out.println("CONNECTION LOST");
	
if (lost_conection_time == 0) {
	lost_conection_time = millis();
	Telemetry.addMessage(e_LOST_CONNECTION);
	Out.println("con lost!");
	//Out.println(millis());

	Commander.init();
}
#ifdef OFF_MOTOR_IF_LOST_CONNECTION
if (motors_is_on())
off_throttle(true, "lost connection");
return;
#endif
if (motors_is_on())
if (go2homeState() == false && progState() == false) {
	aPitch = aRoll = 0;

	if (going2HomeON(true) == false && (millis() - lost_conection_time) > NO_GPS_TIME_TO_FALL) {
		off_throttle(false, e_NO_GPS_2_LONG);
	}
}

}
void AutopilotClass::calibration() {/////////////////////////////////////////////////////////////////////////////////////////////////

	Out.println("Set Calibr NOT USET.");
	/*
	if (abs(cPitch + Commander.pitch) > 0.1 || abs(cRoll + Commander.roll) > 0.1)
		return;

	cPitch += Commander.pitch;
	Commander.pitch = 0;
	cRoll += Commander.roll;
	Commander.roll = 0;
	*/
}





/*




if (ctrl_flag == MANUAL){//---------------manual
	if (flyAtPressure >= SET_HEIGHT_MANUAL){
		throttle = Commander.throttle;
		if (throttle > max_throtthle){
			throttle = max_throtthle;
		}
		startThrottle = throttle;
	}
	else{
		addZ = (startThrottle - Commander.throttle) * 12;
		holdHeight();
		if (throttle > max_throtthle)
			throttle = max_throtthle;
	}
	if (smart_ctrl == false){


		aPitch = Commander.pitch;
		aRoll = Commander.roll;
	}
	else{
		addX -= (Commander.pitch * 10 + addX)*0.1;
		addY += (Commander.roll * 10 - addY)*0.1;
		go2location();
	}
	aPitch = correctingAngle(aPitch);
	aRoll = correctingAngle(aRoll);
	wdt_mask |= CONTROL;

	*/

void AutopilotClass::compass_tr() {
	if (!progState())
		control_bits ^= COMPASS_ON;
}

void AutopilotClass::horizont_tr() {
	if ((control_bits & GO2HOME) == 0 && (control_bits & PROGRAM) == 0)
		control_bits ^= HORIZONT_ON;
}





boolean AutopilotClass::selfTest(){/////////////////////////////////////////////////////////////////////////////////////////////////
	//wdt_enable(WDTO_2S);
	Out.println("Self Test running");
	uint8_t ok = 0;
	if (Mpu.selfTest())
		ok += 1;
	if (Hmc.selfTest())
		ok += 2;
#ifdef BUZZER_R
	OCR4B = 35000;
#endif
	if (ok == 3){
		delay(100);
#ifdef BUZZER_R
		OCR4B = 0;
#endif
	}
	else{
		delay(10000);
	}
	//wdt_enable(WATCHDOG);
	return false;
}

void AutopilotClass::gimBalPitchADD(const float add){
	if (!progState())
		if (Pwm.gimagl_pitch(gimBalPitch + add))
			gimBalPitch += add;

}


bool AutopilotClass::start_stop_program(const bool stopHere){
	if (progState()){
		control_bits ^= PROGRAM;
		LED.prog_index = LED.PROG1_P;
		Prog.clear();
		Stabilization.setDefaultMaxSpeeds();
		if (stopHere){
			float alt = corectedAltitude();
			if (alt  < 10)
				alt = 10;
			holdAltitude(alt);
			holdLocation(GPS.loc.lat_, GPS.loc.lon_);
		}
		return true;
	}
	else{
		if (Prog.start()){
			if (go2homeState())
				going2HomeStartStop(false);
			bool res = holdAltitude(corectedAltitude());
			res &= holdLocation(GPS.loc.lat_, GPS.loc.lon_);
			if (res){
				control_bits |= PROGRAM;
				Out.println("prog started");
				return true;
			}
		}
	return false;
}
	
	
}

bool AutopilotClass::set_control_bits(uint32_t bits) {
	if (control_bits == bits)
		return true;
	//	uint8_t mask = control_bits_^bits;
	printf("comm=%i\n", bits);
	if ((control_bits^bits) & MOTORS_ON) {
		Hmc.compas_motors_calibr = false;
		bool on = motors_is_on() == false;
		on = motors_do_on(on, m_START_STOP);
		if (on == false) {
			Out.println("motors on denied!");
		}
	}

	if ((control_bits^bits) & GO2HOME)
		going2HomeStartStop(false);

	if ((control_bits^bits) & PROGRAM)
		start_stop_program(true);

	if ((control_bits^bits) & Z_STAB)
		holdAltitudeStartStop();

	if ((control_bits^bits) & XY_STAB)
		holdLocationStartStop();


	if ((control_bits^bits) & COMPASS_ON)
		compass_tr();

	if ((control_bits^bits) & HORIZONT_ON)
		horizont_tr();
	//-----------------------------------------------
	if (bits & (MPU_ACC_CALIBR | MPU_GYRO_CALIBR)) {
		control_bits |= (MPU_ACC_CALIBR | MPU_GYRO_CALIBR);
		Mpu.new_calibration(!(bits&MPU_ACC_CALIBR));
		control_bits &= (0xffffffff ^ (MPU_ACC_CALIBR | MPU_GYRO_CALIBR));
	}
	if ((control_bits^bits) & COMPASS_MOTOR_CALIBR) {
		Hmc.start_motor_compas_calibr();
		if (Hmc.compas_motors_calibr)
			control_bits |= COMPASS_MOTOR_CALIBR;
	}
	if ((control_bits^bits) & COMPASS_CALIBR) {
		control_bits |= COMPASS_CALIBR;
		Hmc.calibration(true);
		control_bits &= ~COMPASS_CALIBR;
	}
	if ((control_bits^bits) & RESETING) 
	{}
	if (bits & GIMBAL_PLUS)
		gimBalPitchADD(5);
	if (bits & GIMBAL_MINUS)
		gimBalPitchADD(-5);
}




AutopilotClass Autopilot;

