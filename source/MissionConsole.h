// $Id$  

/**  

  * @file MissionConsole.h  
  * The header file for class Cmissionconsole.  This is a class to handle communication
  * with the host PC running the MissionConsole software
  *  
  * @brief This file contains the class resources for Cmissionconsole - communication software 
  *  
  * @author sgrove   
  *
  * @version 1.01   
  *
  */  

// $Log$ 

#ifndef MISSIONCONSOLE_H
#define MISSIONCONSOLE_H

#include "ArduinoRoverLib.h"

/** Honeywell HMC6352compass software driver (polled)
 * @file	compass.h
 * @brief	An object to communicate with a HMC6352 magno-resistive compass
 * @version	1.01
 * @see 	__
 * @image 	__ "Caption for the image"
 *
 * @example	HMC6352compass
 *	An example of how to build and use the HMC6352compass object
 * @code
 * #include "ArduinoRoverLib.h"
 *
 * HMC6352compass compass; 
 *
 * int main() {
 *	// get the current heading
 *	uint16_t heading = compass.getHeading();
 */

 	//! HMC6352compass read method for the last know heading
	/**  
     * A method to return the last known heading.  Useful if delta
	 * heading needs to be calculated
     *
	 * @return 	uint16_t containing the last known heading of the sensor
  	 */ 

#ifndef FOSC
#define FOSC 16000000 // Clock Speed
#endif
#define UART_BAUD		57600
#define REG_BAUD_VAL 	FOSC/16/UART_BAUD-1

//! scope for peripheral library driver software
//namespace ArduinoLibrary{

//! Mission Console communication class definition
class Cmissionconsole{

private:
	// functions that are used for transmitting
	void write(char*);
	void write(uint8_t);
	
public:
 	
	/**  
    	This is the default constructor for the Cmissionconsole class.
  	*/ 
	Cmissionconsole();

	/**  
    	This is the default deconstructor for the Cmissionconsole class.
  	*/ 
	~Cmissionconsole();	

	/**  
    	Used to enable communication.  This must be called before any transmitting
		members are called
  	*/ 
	void enable();

	/**  
    	Used to disable communication.  If a partial transmission in taking place it
		will be dropped or partially sent
  	*/ 
	void disable();
	
	/**  
    	Send an analog to digital result value to the debugging console.  This is
		stored in the a_d_log.csv file if it is capturing

		@param[in]	in - A 10-bit analog to digital result for any conversion
  	*/ 
	void adValue(uint16_t in);

	/**  
    	Send the Long Range IR result to to the debugging console
		
		@param[in]	in - A 10-bit analog to digital result for the long range IR sensor
  	*/ 
	void longRangeIR(uint16_t in);

	/**  
    	Send the Medium Range IR result to to the debugging console
		
		@param[in]	in - A 10-bit analog to digital result for the meduim range IR sensor
  	*/ 
	void mediumRangeIR(uint16_t in);
		
	
};

#endif
