
#include "compass.h"

HMC6352compass::HMC6352compass()
{
	// define the read address (data sheet specifies 0x43) TWI does a << 1 so 0x21 = 0x43
	 _address = 0x21;
	// use the LED onboard as an indicator that we are doing something
	pinMode(ONBOARD_LED_PIN, OUTPUT);
	digitalWrite(ONBOARD_LED_PIN, HIGH); 
	// start the I2C peripheral as a master
	_wire.begin();
	// Turn the LED off when this routine is complete
	digitalWrite(ONBOARD_LED_PIN, LOW); 
}

uint16_t HMC6352compass::getHeading()
{
	uint16_t return_value;
	// Turn the LED on
	digitalWrite(ONBOARD_LED_PIN, HIGH);
	// Start the read process
	_wire.beginTransmission(_address);
	// The "Get Data" command
	_wire.send('A');
	// A bus stop state
  	_wire.endTransmission();
	// wait for the device to calculate (should be at least 6000uS)
	delay(6);
	// get the data from the compass
	_wire.requestFrom(_address, 2);
	// wait until something is available
	while(!_wire.available());
	// justify the result
	return_value = (_wire.receive() << 8);
	// wait for the next byte
	while(!_wire.available());
	// mask the new read with the last read
	return_value |= _wire.receive();
	// store the last heading
	_last_heading = return_value;
	// Turn the LED off
	digitalWrite(ONBOARD_LED_PIN, LOW);
	// return the heading
	return return_value;
}

uint16_t HMC6352compass::getLastHeading()
{
	return _last_heading;
}

void HMC6352compass::enterCalibration()
{
	// send the calibration command (write)
	_wire.beginTransmission(_address & 0xfe);
	// The "Calibrate" command
	_wire.send('C');
	// A bus stop state
  	_wire.endTransmission();

	// start the calibrate loop - toggle the status LED
	for (int i=0; i<20; i++){
		digitalWrite(ONBOARD_LED_PIN, HIGH);
		delay(500);
		digitalWrite(ONBOARD_LED_PIN, LOW);
		delay(500);
	}

	// send the calibration command (write)
	_wire.beginTransmission(_address & 0xfe);
	// The "Exit" command
	_wire.send('E');
	// A bus stop state
  	_wire.endTransmission();
	// make sure the LED is off
	digitalWrite(ONBOARD_LED_PIN, LOW);
}
