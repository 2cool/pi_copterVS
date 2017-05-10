// 
// 
// 

//#define SERIAL_PRINT

#include "Hmc.h"
#include "Telemetry.h"
#include "mpu.h"
#include "define.h"
#include "Autopilot.h"
#include "Settings.h"
#include "Balance.h"

#include "MS5611.h"
#include "debug.h"
#include "LED.h"
#define DEFAULT_DEV "/dev/i2c-1"

void HmcClass::init()
{
	compas_motors_calibr = false;
	motor_index = 0;
	startTime = 10000;

#ifdef COMPAS_MOTORS_OFF
	motors_power_on = false;
#else
	motors_power_on = true;
#endif

	baseX = baseY = baseZ = 0;
	dx = dy = dz = 0;
	headingGrad = 0;
	ok = true;
	calibrated = true;
#ifndef FALSE_COMPAS


	Out.println("Initializing I2C devices...");
	initialize();
	Out.println("Testing device connections...");
	ok = testConnection();
	Out.println(ok ? "HMC5883L connection successful" : "HMC5883L connection failed");
	if (ok) {
		calibration(false);
	}

#endif	
}


string HmcClass::get_set(){
	string s = motors_power_on ? "1" : "0";
	//Out.print("hmc set:"); Out.println(s);
	return s;
}

void HmcClass::set(const float buf[]){
	motors_power_on = (buf[0] > 0);
	Out.println("compas");
	Out.println(buf[0]);
}
//---------------------------------------------------------



float baseX = 0, baseY = 0, baseZ = 0;
int baseI = 0;
bool motors_is_on_ = false;

void HmcClass::start_motor_compas_calibr(){
	if (compas_motors_calibr == false && Autopilot.motors_is_on()==false){
		Out.println("START MOTOR COMPAS CAL");
		compas_motors_calibr = true;
		motors_is_on_ = false;
		baseX = baseY = baseZ = 0;
		startTime = millis() + 5000;
		motor_index = 0;
		LED.prog_index = LED.MOT_OFF_P;
	}
}



#define MAX_M_WORK_VOLTAGE 1250.0


float _base[3];
void HmcClass::motTest(const float fmx, const float fmy, const float fmz){
	if (millis() > startTime){


		if (baseI < 1000){
			_base[0] += fmx;
			_base[1] += fmy;
			_base[2] += fmz;
			baseI++;
			
		}
		else{
			if (motors_is_on_){

				float fv = MAX_M_WORK_VOLTAGE / Telemetry.get_voltage();
				fv*=(PRESSURE_AT_0/MS5611.pressure);
				Autopilot.motors_do_on(false, "CMT");
				motors_is_on_ = false;
				int index = motor_index * 3;

				base[index]   = fv*(_base[0] - base[index])*0.001;
				base[index+1] = fv*(_base[1] - base[index+1])*0.001;
				base[index+2] = fv*(_base[2] - base[index+2])*0.001;

				Out.print(base[index] * 1000); Out.print("\t"); Out.print(base[index + 1] * 1000); Out.print("\t"); Out.println(base[index + 2] * 1000);

				Out.println(" MOTOR ON");
				Out.print("compas test: 4 m"); Out.println(motor_index);
				Out.println(Telemetry.get_voltage());
				


				if (motor_index < 3){
					motor_index++;
					startTime = millis() + (unsigned long)3000;
					
				}
				else{
					compas_motors_calibr = false;
					Autopilot.reset_compas_motors_calibr_bit();
					Settings.saveCompssSettings2(base);
					Out.println("set saved");
				}
				

			}
			else{
				
				int index = motor_index * 3;
				base[index] =   _base[0];
				base[index+1] = _base[1];
				base[index+2] = _base[2];

				Out.println(" MOTOR OFF");
				Out.print("compas test: 4 m"); Out.println(motor_index);
				
				Out.print(base[index]); Out.print("\t"); Out.print(base[index+1]); Out.print("\t"); Out.println(base[index+2]);

				startTime = millis() + (unsigned long)3000;
				Autopilot.motors_do_on(true, "CMT");
				motors_is_on_ = true;

			}
			
			
			
			baseI = 0;
			_base[0] = _base[1] = _base[2] = 0;
		}
	}
	
}




#ifdef FALSE_COMPAS

uint32_t comTime = 0;
void HmcClass::loop(){
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
	if (millis() - comTime < 50)
		return;
	comTime = millis();
	float dt = 0.05;
	if (Autopilot.corectedAltitude()>0){
		float f = Autopilot.get_yaw();


		//(-Autopilot.get_Yaw() - Mpu.yaw)
		headingGrad = -f;
	}
//	headingGrad = 0;


#ifndef WORK_WITH_WIFI
	headingGrad = 0;
#endif
}

#else

bool hmc_cnt = false;

void HmcClass::loop(){
	//бельіе ноги
#ifdef T200HZ
	if (hmc_cnt ^= true)
		return;
#endif
	int16_t mx,my,mz;
	float fmz, fmx, fmy;
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mx = (((int16_t)buffer[0]) << 8) | buffer[1];
	my = (((int16_t)buffer[4]) << 8) | buffer[5];
	mz = (((int16_t)buffer[2]) << 8) | buffer[3];





	fmx=(float)(mx - baseX)*dx;	
	fmy=(float)(my - baseY)*dy;
	fmz=(float)(mz - baseZ)*dz;




#ifndef SESOR_UPSIDE_DOWN
	my = -my;
	mz = -mz;
#endif

	if (compas_motors_calibr)
		motTest(fmx, fmy, fmz);

	if (Autopilot.motors_is_on() && motors_power_on){
		float kx, ky, kz;
		//m0;
		//float pk = MS5611.pressure / 101663.0;


	
		const float ek = MS5611.pressure * (float)(1.0 / PRESSURE_AT_0) * (Telemetry.get_voltage() * (float)(1.0 / MAX_M_WORK_VOLTAGE));

		float k = Balance.gf0() * 2;
			k *= k;
			k *= ek;
			kx = base[0] * k;
			ky = base[1] * k;
			kz = base[2] * k;

		//m1;
			k = Balance.gf1() * 2;
			k *= k;
			k *= ek;
			kx += base[3] * k;
			ky += base[4] * k;
			kz += base[5] * k;

		//m2
			k = Balance.gf2() * 2;
			k *= k;
			k *= ek;
			kx += base[6] * k;
			ky += base[7] * k;
			kz += base[8] * k;

		//m3
			k = Balance.gf3() * 2;
			k *= k;
			k *= ek;
			kx += base[9]  * k;
			ky += base[10] * k;
			kz += base[11] * k;
	
			fmx -= kx;
			fmy -= ky;
			fmz -= kz;
	}

		//---------------------------------
	
	
	//testtt(fmx, fmy, fmz, Autopilot.getThrottle());

	
	//--------------------------------


	// Tilt compensation
	float Xh = fmx * Mpu.cosPitch - fmz * Mpu.sinPitch;
	float Yh = fmx * Mpu.sinRoll * Mpu.sinPitch + fmy * Mpu.cosRoll - fmz * Mpu.sinRoll * Mpu.cosPitch;
	

	headingGrad = RAD2GRAD*atan2(Yh, Xh);
	//Out.println(fmx);
	//Out.print(roll); Out.print(" "); Out.print(pitch); Out.print(" ");  Out.println(Hmc.heading / PI * 180);
	
//	Out.println(Mpu.yaw); Out.print(" "); Out.println(heading*RAD2GRAD);
//	if (heading<0)
//		heading += 2 * PI;



	//if (heading > PI)
	///	heading -= 2 * PI;
	//if (heading <= -PI)
	//	heading += 2 * PI;



#ifdef SERIAL_PRINT
	Out.print("heading:\t");
	Out.println(heading);
#endif
}

#endif


#include "LED.h"

void HmcClass::newCalibration(int16_t sh[]){
	//wdt_enable(WDTO_4S);
	sh[0] = sh[1] = sh[2] = sh[3] = sh[4] = sh[5] = 0;
	Out.println("START ROTATION");


	delay(2000);
	int16_t mx, my, mz;
	long cnt = 0;
	LED.off();
	while (true){
		
		bool new_val = false;
		readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
		if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
		mx = (((int16_t)buffer[0]) << 8) | buffer[1];
		my = (((int16_t)buffer[4]) << 8) | buffer[5];
		mz = (((int16_t)buffer[2]) << 8) | buffer[3];

		if (mx > sh[0]){
			sh[0] = mx;
			Out.print("max_X: "); Out.println(mx);
			new_val = true;
		}
		if (mx < sh[1]){
			sh[1] = mx;
			Out.print("min_X: "); Out.println(mx);
			new_val = true;
		}

		if (my > sh[2]){
			sh[2] = my;
			Out.print("max_Y: "); Out.println(my);
			new_val = true;
		}
		if (my < sh[3]){
			sh[3] = my;
			Out.print("min_Y: "); Out.println(my);
			new_val = true;
		}
		if (mz > sh[4]){
			sh[4] = mz;
			Out.print("max_Z: "); Out.println(mz);
			new_val = true;
		}
		if (mz < sh[5]){
			sh[5] = mz;
			Out.print("min_Z: "); Out.println(mz);
			new_val = true;
		}
		delay(10);
		if (new_val)
			cnt = 0;
		else
			cnt++;
		if (cnt>6000)
			break;
		//wdt_reset();
	}
	Out.println("Stop Rottation");
	Out.print("x "); Out.print(sh[0]); Out.print(","); Out.println(sh[1]);
	Out.print("y "); Out.print(sh[2]); Out.print(","); Out.println(sh[3]);
	Out.print("z "); Out.print(sh[4]); Out.print(","); Out.println(sh[5]);
	
}

bool HmcClass::calibration(const bool newc){

	if (newc && Autopilot.motors_is_on())
		return false;



	

	int16_t sh[6];

	if (newc){
		newCalibration(sh);
		Settings.saveCompssSettings(sh);
		//wdt_enable(WDTO_120MS);//reset
		Serial.println("RESET HMC");
	}


	calibrated = Settings.readCompassSettings(sh);
	if (calibrated)
		Settings.loadCompssSettings2(base);
	if (calibrated==false){
		Out.println("! ! ! ! Comppas not Calibrated ! ! ! !");
		return false;
	}
	
	dx = (float)(sh[0] - sh[1])*0.5;
	baseX = dx + sh[1];
	Out.print(sh[0]); Out.print("\t"); Out.println(sh[1]);
	dx = 1.0 / dx;

	dy = (float)(sh[2] - sh[3])*0.5;
	baseY = dy + sh[3];
	Out.print(sh[2]); Out.print("\t"); Out.println(sh[3]);
	dy = 1.0 / dy;

	dz = (float)(sh[4] - sh[5])*0.5;
	baseZ = dz + sh[5];
	Out.print(sh[4]); Out.print("\t"); Out.println(sh[5]);
	dz = 1.0 / dz;
	return true;
}



bool HmcClass::selfTest(){
	Out.println("COMPAS TEST");
	if (ok){
		int16_t mx, my, mz;
		int16_t tx = 0, ty = 0, tz = 0;
		int8_t error = 0;
		for (int8_t i = 0; i < 10; i++){
			readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
			if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
			mx = (((int16_t)buffer[0]) << 8) | buffer[1];
			my = (((int16_t)buffer[4]) << 8) | buffer[5];
			mz = (((int16_t)buffer[2]) << 8) | buffer[3];
			Out.print(mx); Out.print(" "); Out.print(my); Out.print(" "); Out.println(mz);
			error += (tx == mx || ty == my || tz == mz);
			tx = mx;
			ty = my;
			tz = mz;
			delay(10);
		}
		ok &= error < 10;

	}
	if (!ok)
		Out.println("ERROR");
	return ok;
}

/** Default constructor, uses default I2C address.
* @see HMC5883L_DEFAULT_ADDRESS
*/
HmcClass::HmcClass() {
	devAddr = HMC5883L_DEFAULT_ADDRESS;
}

/** Specific address constructor.
* @param address I2C address
* @see HMC5883L_DEFAULT_ADDRESS
* @see HMC5883L_ADDRESS
*/
HmcClass::HmcClass(uint8_t address) {
	devAddr = address;
}

/** Power on and prepare for general usage.
* This will prepare the magnetometer with default settings, ready for single-
* use mode (very low power requirements). Default settings include 8-sample
* averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
* terms of LSB/Gauss). Be sure to adjust any settings you need specifically
* after initialization, especially the gain settings if you happen to be seeing
* a lot of -4096 values (see the datasheet for mor information).
*/
void HmcClass::initialize() {
	// write CONFIG_A register
	writeByte(devAddr, HMC5883L_RA_CONFIG_A,
		(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
		(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
		(HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

	// write CONFIG_B register
	setGain(HMC5883L_GAIN_1090);

	// write MODE register
	setMode(HMC5883L_MODE_SINGLE);
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool HmcClass::testConnection() {
	if (readBytes(devAddr, HMC5883L_RA_ID_A, 3, buffer) == 3) {
		return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
	}
	return false;
}

// CONFIG_A register

/** Get number of samples averaged per measurement.
* @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
* @see HMC5883L_AVERAGING_8
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_AVERAGE_BIT
* @see HMC5883L_CRA_AVERAGE_LENGTH
*/
uint8_t HmcClass::getSampleAveraging() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, buffer);
	return buffer[0];
}
/** Set number of samples averaged per measurement.
* @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_AVERAGE_BIT
* @see HMC5883L_CRA_AVERAGE_LENGTH
*/
void HmcClass::setSampleAveraging(uint8_t averaging) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Get data output rate value.
* The Table below shows all selectable output rates in continuous measurement
* mode. All three channels shall be measured within a given output rate. Other
* output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
* interrupt pin in single measurement mode.
*
* Value | Typical Data Output Rate (Hz)
* ------+------------------------------
* 0     | 0.75
* 1     | 1.5
* 2     | 3
* 3     | 7.5
* 4     | 15 (Default)
* 5     | 30
* 6     | 75
* 7     | Not used
*
* @return Current rate of data output to registers
* @see HMC5883L_RATE_15
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_RATE_BIT
* @see HMC5883L_CRA_RATE_LENGTH
*/
uint8_t HmcClass::getDataRate() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, buffer);
	return buffer[0];
}
/** Set data output rate value.
* @param rate Rate of data output to registers
* @see getDataRate()
* @see HMC5883L_RATE_15
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_RATE_BIT
* @see HMC5883L_CRA_RATE_LENGTH
*/
void HmcClass::setDataRate(uint8_t rate) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
/** Get measurement bias value.
* @return Current bias value (0-2 for normal/positive/negative respectively)
* @see HMC5883L_BIAS_NORMAL
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_BIAS_BIT
* @see HMC5883L_CRA_BIAS_LENGTH
*/
uint8_t HmcClass::getMeasurementBias() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement bias value.
* @param bias New bias value (0-2 for normal/positive/negative respectively)
* @see HMC5883L_BIAS_NORMAL
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_BIAS_BIT
* @see HMC5883L_CRA_BIAS_LENGTH
*/
void HmcClass::setMeasurementBias(uint8_t bias) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** Get magnetic field gain value.
* The table below shows nominal gain settings. Use the "Gain" column to convert
* counts to Gauss. Choose a lower gain value (higher GN#) when total field
* strength causes overflow in one of the data output registers (saturation).
* The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
*
* Value | Field Range | Gain (LSB/Gauss)
* ------+-------------+-----------------
* 0     | +/- 0.88 Ga | 1370
* 1     | +/- 1.3 Ga  | 1090 (Default)
* 2     | +/- 1.9 Ga  | 820
* 3     | +/- 2.5 Ga  | 660
* 4     | +/- 4.0 Ga  | 440
* 5     | +/- 4.7 Ga  | 390
* 6     | +/- 5.6 Ga  | 330
* 7     | +/- 8.1 Ga  | 230
*
* @return Current magnetic field gain value
* @see HMC5883L_GAIN_1090
* @see HMC5883L_RA_CONFIG_B
* @see HMC5883L_CRB_GAIN_BIT
* @see HMC5883L_CRB_GAIN_LENGTH
*/
uint8_t HmcClass::getGain() {
	readBits(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH, buffer);
	return buffer[0];
}
/** Set magnetic field gain value.
* @param gain New magnetic field gain value
* @see getGain()
* @see HMC5883L_RA_CONFIG_B
* @see HMC5883L_CRB_GAIN_BIT
* @see HMC5883L_CRB_GAIN_LENGTH
*/
void HmcClass::setGain(uint8_t gain) {
	// use this method to guarantee that bits 4-0 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	writeByte(devAddr, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
}

// MODE register

/** Get measurement mode.
* In continuous-measurement mode, the device continuously performs measurements
* and places the result in the data register. RDY goes high when new data is
* placed in all three registers. After a power-on or a write to the mode or
* configuration register, the first measurement set is available from all three
* data output registers after a period of 2/fDO and subsequent measurements are
* available at a frequency of fDO, where fDO is the frequency of data output.
*
* When single-measurement mode (default) is selected, device performs a single
* measurement, sets RDY high and returned to idle mode. Mode register returns
* to idle mode bit values. The measurement remains in the data output register
* and RDY remains high until the data output register is read or another
* measurement is performed.
*
* @return Current measurement mode
* @see HMC5883L_MODE_CONTINUOUS
* @see HMC5883L_MODE_SINGLE
* @see HMC5883L_MODE_IDLE
* @see HMC5883L_RA_MODE
* @see HMC5883L_MODEREG_BIT
* @see HMC5883L_MODEREG_LENGTH
*/
uint8_t HmcClass::getMode() {
	readBits(devAddr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement mode.
* @param newMode New measurement mode
* @see getMode()
* @see HMC5883L_MODE_CONTINUOUS
* @see HMC5883L_MODE_SINGLE
* @see HMC5883L_MODE_IDLE
* @see HMC5883L_RA_MODE
* @see HMC5883L_MODEREG_BIT
* @see HMC5883L_MODEREG_LENGTH
*/
void HmcClass::setMode(uint8_t newMode) {
	// use this method to guarantee that bits 7-2 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	writeByte(devAddr, HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
* In the event the ADC reading overflows or underflows for the given channel,
* or if there is a math overflow during the bias measurement, this data
* register will contain the value -4096. This register value will clear when
* after the next valid measurement is made. Note that this method automatically
* clears the appropriate bit in the MODE register if Single mode is active.
* @param x 16-bit signed integer container for X-axis heading
* @param y 16-bit signed integer container for Y-axis heading
* @param z 16-bit signed integer container for Z-axis heading
* @see HMC5883L_RA_DATAX_H
*/
void HmcClass::getHeading(int16_t *x, int16_t *y, int16_t *z) {
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (((int16_t)buffer[0]) << 8) | buffer[1];
	*y = (((int16_t)buffer[4]) << 8) | buffer[5];
	*z = (((int16_t)buffer[2]) << 8) | buffer[3];
}
/** Get X-axis heading measurement.
* @return 16-bit signed integer with X-axis heading
* @see HMC5883L_RA_DATAX_H
*/
int16_t HmcClass::getHeadingX() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis heading measurement.
* @return 16-bit signed integer with Y-axis heading
* @see HMC5883L_RA_DATAY_H
*/
int16_t HmcClass::getHeadingY() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get Z-axis heading measurement.
* @return 16-bit signed integer with Z-axis heading
* @see HMC5883L_RA_DATAZ_H
*/
int16_t HmcClass::getHeadingZ() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t)buffer[2]) << 8) | buffer[3];
}

// STATUS register

/** Get data output register lock status.
* This bit is set when this some but not all for of the six data output
* registers have been read. When this bit is set, the six data output registers
* are locked and any new data will not be placed in these register until one of
* three conditions are met: one, all six bytes have been read or the mode
* changed, two, the mode is changed, or three, the measurement configuration is
* changed.
* @return Data output register lock status
* @see HMC5883L_RA_STATUS
* @see HMC5883L_STATUS_LOCK_BIT
*/
bool HmcClass::getLockStatus() {
	readBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer);
	return buffer[0];
}
/** Get data ready status.
* This bit is set when data is written to all six data registers, and cleared
* when the device initiates a write to the data output registers and after one
* or more of the data output registers are written to. When RDY bit is clear it
* shall remain cleared for 250 us. DRDY pin can be used as an alternative to
* the status register for monitoring the device for measurement data.
* @return Data ready status
* @see HMC5883L_RA_STATUS
* @see HMC5883L_STATUS_READY_BIT
*/
bool HmcClass::getReadyStatus() {
	readBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer);
	return buffer[0];
}

// ID_* registers

/** Get identification byte A
* @return ID_A byte (should be 01001000, ASCII value 'H')
*/
uint8_t HmcClass::getIDA() {
	readByte(devAddr, HMC5883L_RA_ID_A, buffer);
	return buffer[0];
}
/** Get identification byte B
* @return ID_A byte (should be 00110100, ASCII value '4')
*/
uint8_t HmcClass::getIDB() {
	readByte(devAddr, HMC5883L_RA_ID_B, buffer);
	return buffer[0];
}
/** Get identification byte C
* @return ID_A byte (should be 00110011, ASCII value '3')
*/
uint8_t HmcClass::getIDC() {
	readByte(devAddr, HMC5883L_RA_ID_C, buffer);
	return buffer[0];
}


HmcClass Hmc;

