
#ifndef COMPASS_H
#define COMPASS_H

#include "ArduinoRoverLib.h"
#include "Wire.h"

/** Honeywell HMC6352compass software driver (polled)
 * @file	compass.h
 * @brief	An object to communicate with a HMC6352 magno-resistive compass
 * @version	1.01
 * @see 	__
 * @image 	__ "Caption for the image"
 *
 * @example	compass
 *	An example of how to build and use the HMC6352compass object
 * @code
 * #include "ArduinoRoverLib.h"
 *
 * HMC6352compass compass; 
 *
 * int main() {
 *	// get the current heading
 *	uint16_t heading = compass.getHeading();
 *	// get the last heading
 *	uint16_t last_heading = compass.getLastHeading();
 *	// find the difference (if its needed)
 *	uint16_t difference = last_heading - heading;
 *	// Calibrate if needed
 *	compass.enterCalibration();     
 * }
 * @endcode
 */

 
/**
 *	@class HMC6352compass
 * 	@brief A class for the HMC6352compass digital compass
 */
class HMC6352compass
{
private:
	//! Build the I2C object and keep it away from users
	TwoWire _wire;
	//! Address for the compass set in the constructor
	uint8_t _address;
	//! temporay storage for the last heading - used to calculate delta
	uint16_t _last_heading;

public:
	
	//! HMC6352compass constructor
    /**
     * This is the default constructor for the HMC6352compass object
	 * The compass should be connected to the TWI peripheral pins
     */
	HMC6352compass();

	//! HMC6352compass read method for the current heading
	/**  
     * A method to return the heading of the compass in degrees * 10.
     *	
	 * @return 	uint16_t containing the heading of the sensor
  	 */ 
	uint16_t getHeading();

	//! HMC6352compass read method for the last know heading
	/**  
     * A method to return the last known heading.  Useful if delta
	 * heading needs to be calculated
     *
	 * @return 	uint16_t containing the last known heading of the sensor
  	 */ 
	uint16_t getLastHeading();
	
	//! HMC6352compass method for calibration mode
	/**
	 * Enter user calibration mode.  2 complete rotations should occur (720 degrees) in 
	 * 20 seconds.  The LED will flash a rate of 1hz during this process.  The rotation 
	 * should be smooth and uniform
	 */
	 void enterCalibration();

};

#endif

