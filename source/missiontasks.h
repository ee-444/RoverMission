
#ifndef MISSIONTASKS_H
#define MISSIONTASKS_H

//! contains the pin assignments for the hardware as well as
//! other c-library files that are used throught the program
#include "ArduinoRoverLib.h"

//! Constants for the methods below making code easier to change
//! while poking at things
const uint8_t ROOM_DIFFERENCE_AMOUNT = 100;
const uint8_t ROOM_CONSECUTIVE_CNT	= 3;
const uint8_t MAX_PLAQUE_CNT = 5;

/**
 *	@class Mission
 * 	@brief A class with methods to complete the mission
 */
class Mission{
private:
	// Objects for the left and right motors.  PWM frequency is 64kHz
	//AF_DCMotor motor_l;//(3, MOTOR12_64KHZ);
	//AF_DCMotor motor_r;//(4, MOTOR12_64KHZ);
	// 1.8 degrees/step stepper motor (200 steps = 360 degrees)
	//AF_Stepper motor_s;//(200, 1);
	int16_t current_step_cnt;
	//uint16_t* map1;
	//uint16_t* map2;

public:	
	//! Default constructor
	/**
	 *	Make sure there is room for the map memory
	 *
	 */
	 Mission()/*	:	motor_l(3, MOTOR12_64KHZ),
	 				motor_r(4, MOTOR12_64KHZ),
					motor_s(200, 1)*/
	 {
	 	current_step_cnt = 0;
		//map1 = (uint16_t*)calloc(200, 0);
		//map2 = (uint16_t*)calloc(200, 0);
		//if ((map1== NULL ) || (map2 == NULL)){
		//	exit(1);
		//}
	 }

	//! Make the rover move forward
	/**
	 *	This is a wrapper for the AF_DCMotor
	 *	 objects that this program uses.  This method
	 *	 does not return until the time in the parameter has elapsed
	 *
	 *	@param[in] time - the amount of time to turn the motors on.  If 0
	 *						they are on until stopRobot() is called.
	 */
	void goStraight(uint16_t time=0);

	//! Make the rover stop moving
	/**
	 *	This is a wrapper for the AF_DCMotor
	 *	 objects that this program uses.  This will 
	 *	 stop the rover if it is moving and has no affect if it is 
	 *	 standing still
	 */
	void stopRobot();

	//! create a map of the enviroment
	/**
	 *	this method will create a map of the room.  The IR distances are 
	 *	 taken every step.  The step angle depends on the AF_Servo object settings
	 *
	 *	param[out] map - an array to store the distances in
	 *	param[in] map_size - the amount of readings that the map can hold.  If a number larger than
	 *					the map size is passed or nothing is passed - RAM corruction may occur
	 */
	void scanEnvironment(uint16_t* map, uint16_t map_size = 200);

	//! create a plan of attack
	/**
	 *	Create a distance to plaque and angle to plaque array for use in the map seek and destroy 
	 *	 method.  Based on constants such as ROOM_DIFFERENCE_AMOUNT, ROOM_CONSECUTIVE_CNT and MAX_PLAQUE_CNT
	 *	 find what is thought to be the left most edge of the plaque.  REturns the distance in one array and 
	 *	 angle from the last plaque in the other.  elements are 0 if nothing is found
	 *
	 *	@param[in] map1 - A map with data from the first scan (with or without plaques)
	 *	@param[in] map2 - A map with data from the second scan (with or without plaques)
	 *	@param[out] heading_map - An array with the distance from home to each found plaque
	 *	@param[out] angle_map - An array with angles from home (and then the last plaque) to the next plaqe
	 */
	void analyzeRoom(uint16_t* map1, uint16_t* map2, uint16_t* dist_map, uint16_t* heading_map);

	//! Once at a plaque - find out how to get to the next
	/**
	 *	Based on the distance and angle arrays - from any plaque the next one can be found.  The next plaque will
	 *	 be the next plaque to the right.  As did Zoolander - we can only turn right.  This methods result make
	 *	 one assumption - that you are facing the plaque head on.
	 *
	 *	@param[in] dist - The array containing the distances to each plaque
	 *	@param[in] angle - The array containing the angles from one plaque to the next
	 *	@param[in] plaque_num - what plaque is next.  Must be from 2 -> n.
	 *	@param[out] plaque_dist - the distance to the next plaque
	 *	@param[out] plaque_angle - If facing the plaque headon - this is the anle to the next plaque
	 */
	void findPlaqueDistanceAngle(uint16_t* dist, uint16_t* angle, uint8_t plaque_num, uint16_t& plaque_dist, uint16_t& plaque_angle);

	//! Rotate in place to any given heading
	/**
	 *	Rotate to any heading from 0-360.  Will not move forward - just rotate.  If the rover is 
	 *	 in a infinate moving loop this will stop it once the heading is found.  
	 *
	 *	@param[in] angle - heading you wish to face.  Input is in degrees * 10 so 10 degrees is 100
	 *						 and 300 degrees is 3000.  this gives the resulution of 0.1 degrees
	 */
	void adjustHeading(uint16_t angle);

	//! A method to read and deciminate the ADC result
	/**
	 *	This method returns the result of a IR sample.  This is used by irDistance to 
	 *
	 *	@param[in] pin - the analog pin that the sensor is connected to
	 */
	int16_t readADC(uint16_t pin);

	//! read the distance from a truncated and memory friendly FLASH table
	/**
	 *	Differences from the std 1024 distance array are that:
	 *	 1. only values that are within an adjusted range are stored - logic returns otherwise
	 *	 2. value is stored as 8 bits and multiplied by 2 when found.  This uses 1/4 the FLASH as before
	 *
	 *	@param[in] pin - the analog pin that the sensor is connected to. - Makes a call to readADC
	 */
	uint16_t irDistance(uint8_t pin);

	//! change the heading of the scan platform
	/**
	 *	A wrapper for the AF_Stepper object.  Based on steps of 1.8 degrees per step
	 *	 can return the current heading if the second parameter is NULL.
	 *	 When returning the current value -  the first paramter is ignored
	 *
	 *	@param[in] steps - the amount of steps to take
	 *	@param[in] update - change the position to the steps parameter if > than 0
	 *
	 *	@return the current steps offset from home that the platform is facing
	 */
	uint8_t adjustScanPlatform(uint8_t steps, uint8_t update=0);

};

#endif

