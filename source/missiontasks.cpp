
#include "missiontasks.h"

//! declare object that are used in this file
extern PID <uint16_t>pid;
extern HMC6352compass compass;
extern AF_DCMotor motor_l;
extern AF_DCMotor motor_r;
extern AF_Stepper motor_s;
extern Cmissionconsole debug;

void Mission::goStraight(uint16_t time)
{
	motor_l.setSpeed(STRAIGHT_DUTY_CYCLE);
  	motor_r.setSpeed(STRAIGHT_DUTY_CYCLE);
  	motor_l.run(FORWARD);
 	motor_r.run(FORWARD);
	if (time > 0){
		delay(time);
		motor_l.run(RELEASE);
		motor_r.run(RELEASE);
	}
}

void Mission::stopRobot()
{
	motor_l.run(RELEASE);
	motor_r.run(RELEASE);
}

void Mission::scanEnvironment(uint16_t* map, uint16_t map_size)
{	
	// now we can move back and start storing sensor data
	for(uint8_t cnt=0; cnt<map_size; cnt++){
		// create the map element
		map[cnt] = irDistance(LONG_RANGE_IR_PIN);
		// move the motor and do it again
		adjustScanPlatform((adjustScanPlatform(0)+1), 1);
		// allow the platform movement to settle
		delay(100);
	}
	
	// Return the scan platform to the home position
	adjustScanPlatform(0, 1);
}

void Mission::analyzeRoom(uint16_t* map1, uint16_t* map2, uint16_t* dist_map, uint16_t* heading_map)
{
	uint16_t result = 0;
	uint8_t consecutive_cnt = 0;
	uint16_t result_cnt = 0;

	// clear the result map
	for (uint8_t index = 0; index < MAX_PLAQUE_CNT; index++){
		heading_map[index] = 0;
		dist_map[index] = 0;
	}
	// look for the difference in the room
	for (uint8_t index = 0; index < 200; index++){
		result = map1[index] - map2[index];
		result = abs(result);
		//Serial.println(result);
		// look for large differences
		if (result > ROOM_DIFFERENCE_AMOUNT){
			// the amount of consecutive large differences indicated a plaque
			if (consecutive_cnt > ROOM_CONSECUTIVE_CNT){
				// store the heading to this plaque (degrees * 10)
				dist_map[result_cnt] = map2[index];
				heading_map[result_cnt] = index*18;
				// look for the max amount of plaques
				result_cnt++;
				// exit if all the plaques were found
				if (result_cnt == MAX_PLAQUE_CNT){
					for (uint8_t loop = 0; loop < 5; loop++){
						// convert to absolute angle from home to the angle between the last plaque can current
						if (heading_map[loop+1] > heading_map[loop]){
							heading_map[loop] = heading_map[loop+1] - heading_map[loop];
						}
						else{
							loop = 5;
						}
					}					
					return;
				}
			}
			consecutive_cnt++;
		}
		else{
			consecutive_cnt = 0;
		}
	}
}

void Mission::findPlaqueDistanceAngle(uint16_t* dist, uint16_t* angle, uint8_t plaque_num, uint16_t& plaque_dist, uint16_t& plaque_angle)
{
	if (plaque_num < 2){
		plaque_num = 2;
	}

	double dist1 = static_cast<double>(dist[plaque_num-1]);
	double dist2 = static_cast<double>(dist[plaque_num-2]);
	double angle1 = static_cast<double>(angle[plaque_num-1]);
	// find the distance to the next plaque 
	double tmp_plaque_dist = sqrt( ( pow(dist1,2) + 
						  			pow(dist2,2)) - (2 * dist2 * dist1 * (cos((angle1*PI)/180.0)) )
					  			 );
	plaque_dist = static_cast<uint16_t>(tmp_plaque_dist);
	// find the angle to the next plaque
	double tmp_plaque_angle = acos( ( pow(dist2,2) + pow(plaque_dist,2) - (dist1 * dist1) ) / 
						   			(2 * dist2 * plaque_dist)
					   			  );
	// convert back to degrees
	tmp_plaque_angle *= 180.00000 / PI; 
	plaque_angle = static_cast<uint16_t>(tmp_plaque_angle);
}

void Mission::adjustHeading(uint16_t angle)
{
 	// make sure the heading is valid
	while (angle > 3599){
		angle -= 3600;
	}
	// set the motors to the same speed
	motor_r.setSpeed(150);
  	motor_l.setSpeed(150);
  	// one should go forward and another backwards
  	motor_r.run(FORWARD);
  	motor_l.run(BACKWARD);
  
 	//exits only once the offset angle is reached
  	int targetOffset = compass.getHeading();
  
  	// keep looping until we are less than 5 degrees off
	// from desired setting 
	while(abs(angle - targetOffset) > 50){
    	targetOffset = compass.getHeading();
    	//Serial.println(targetOffset);
  	}
  
  	//once desired heading is found turn motors off
  	motor_l.run(RELEASE);
  	motor_r.run(RELEASE);
}

// ISSUES WITH SOME OF THE CODE HERE - commented out
int16_t Mission::readADC(uint16_t pin)
{
	const uint8_t sample_amout = 100;		// number of real ADC samples
	int16_t x_i = 0;      					// ADC input for sample i
	int16_t A_1 = 0;   						// current  i   running average
	int16_t A_0 = 0;       					// previous i-1 running average
	
	// deciminate the IR sensor to get a better idea of the distance
	for (uint8_t i=1; i<=sample_amout; i++){
    	// read from the ADC
		x_i = analogRead(pin);
		// rapid calculation method - http://en.wikipedia.org/wiki/Standard_deviation
		// accumulate the result
		A_1 = A_0 + (( x_i - A_0 ) / i);
		// stored for the next use
		A_0 = A_1;
	}
	// cast and return the result
	return A_1;
}

uint16_t Mission::irDistance(uint8_t pin)
{
	// store the value from the ADC converter
	uint16_t tmp_adc_result1 = readADC(LONG_RANGE_IR_PIN);
	uint16_t tmp_adc_result2 = readADC(MEDIUM_RANGE_IR_PIN);
	uint16_t dist_from_flash1;
	uint16_t dist_from_flash2;

	// Get the medium range result
	if (tmp_adc_result2 > (MEDIUM_RANGE_OFFSET+MEDIUM_RANGE_TABLE_SIZE)){
		dist_from_flash2 = MIN_MEDIUM_RANGE_IR_DISTANCE;
	}
	else if (tmp_adc_result2 < MEDIUM_RANGE_OFFSET){
		dist_from_flash2 = MAX_MEDIUM_RANGE_IR_DISTANCE;
	}
	else{
		// get the value from flash (offset from all the same values)
		dist_from_flash2 = pgm_read_byte(&medium_range_data[tmp_adc_result2-MEDIUM_RANGE_OFFSET]);
		dist_from_flash2 *= 2;
	}	
	
	// get the long range result
	// reduce the size of the lookup table
	if (tmp_adc_result1 > (LONG_RANGE_OFFSET+LONG_RANGE_TABLE_SIZE)){
		dist_from_flash1 = MIN_LONG_RANGE_IR_DISTANCE;
	}
	else if (tmp_adc_result1 < LONG_RANGE_OFFSET){
		dist_from_flash1 = MAX_LONG_RANGE_IR_DISTANCE;
	}
	else{
		// get the value from flash (offset from all the same values)
		dist_from_flash1 = pgm_read_byte(&long_range_data[tmp_adc_result1-LONG_RANGE_OFFSET]);
		dist_from_flash1 *= 2;
	}
	
	float long_result = dist_from_flash1 / (MAX_LONG_RANGE_IR_DISTANCE + MIN_LONG_RANGE_IR_DISTANCE);
	float medium_result = dist_from_flash2 / (MAX_MEDIUM_RANGE_IR_DISTANCE + MIN_MEDIUM_RANGE_IR_DISTANCE);

	long_result *= 100;
	medium_result *= 100;

	long_result = abs(long_result);
	medium_result = abs(medium_result);

	if (long_result < medium_result){
		// medium range validation
		if (dist_from_flash2 == MIN_LONG_RANGE_IR_DISTANCE){
			return MIN_LONG_RANGE_IR_DISTANCE;
		}
		return dist_from_flash1;
	}
	else{
		// long range validation
		if (dist_from_flash2 == MAX_MEDIUM_RANGE_IR_DISTANCE){
			return dist_from_flash1;
		}
		return dist_from_flash2;
	}
	
	return 0;
}

uint8_t Mission::adjustScanPlatform(uint8_t steps, uint8_t update)
{
	//debug.enable();
	//static uint16_t current_step_cnt = 0;
	// Only accepts steps from 0 -> 200
	if ( (steps >= 200) && (update == 1) ){
		return current_step_cnt;
	}
	
	// update = 1 means adjust the position
	if (update > 0){
		// move the stepper to the angle requested
		if (current_step_cnt < steps){
			while(current_step_cnt != steps){
				motor_s.onestep(FORWARD, SINGLE);
				current_step_cnt++;
				debug.mediumRangeIR(current_step_cnt);
				delay(10);
			}
		}
		else if (current_step_cnt > steps){
			while(current_step_cnt != steps){
				motor_s.onestep(BACKWARD, SINGLE);
				current_step_cnt--;
				delay(10);
				debug.mediumRangeIR(current_step_cnt);
			}
		}
	}
	// just return the angle
	return current_step_cnt;
}

