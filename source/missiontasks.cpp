
#include "missiontasks.h"

//! declare object that are used in this file
extern PID <uint16_t>pid;
extern HMC6352compass compass;
extern AF_DCMotor motor_l;
extern AF_DCMotor motor_r;
extern AF_Stepper motor_s;
extern Cmissionconsole debug;

void goStraight(uint16_t time)
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

void stopRobot()
{
	motor_l.run(RELEASE);
	motor_r.run(RELEASE);
}


//void goStraight(uint16_t time)
//{
//	// find out the millis() start time
//	uint16_t last_millis = millis();
//	// set the pid setpoint with the direction of the compass
//	pid.setTarget(compass.getHeading());
//	time /= PID_UPDATE_INTERVAL;
//	
//	// set up the motors and get them moving straight
//	motor_l.setSpeed(STRAIGHT_DUTY_CYCLE);
//  	motor_r.setSpeed(STRAIGHT_DUTY_CYCLE);
//  	motor_l.run(FORWARD);
// 	motor_r.run(FORWARD);
//
//	// time per itteration set by caller
//	while( time != 0){	
//		// hold the program while waiting for an update interval
//		// allow serial communication and other processing to occur
//		while ((millis() - last_millis) <= PID_UPDATE_INTERVAL){
//			// use the free running millis() as a timer
//			if (millis() < last_millis){
//				last_millis = 0;	// rollover occured
//			}
//			// processing can occur here - better than a wait statement
//			//-->  <--
//		}
//
//		// update the motors based on the callers request(time)
//		if ((millis() - last_millis) > PID_UPDATE_INTERVAL){
//			// update the last_millis count
//			last_millis += PID_UPDATE_INTERVAL;
//			// get the error from the PID module - 180 degrees the the max desired error
//			uint16_t error = pid.getError(compass.getHeading(), 1800);
//			// update the motors based on the error - converting degree_of_error
//			// to duty_cycle for correction
//			motor_r.setSpeed( constrain((STRAIGHT_DUTY_CYCLE) + error, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE) );
//			motor_l.setSpeed( constrain((STRAIGHT_DUTY_CYCLE) - error, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE) );
//		}
//	
//		// decrement the amount of time that the caller asked for per itteration
//		time--;
//	}
//
//	// Stop the motors - we should be either near the home position or at a distance x far from the starting point
//	motor_l.run(RELEASE);
//	motor_r.run(RELEASE);
//	// set the duty cycle staight for the next use
//	motor_l.setSpeed(STRAIGHT_DUTY_CYCLE);
//  	motor_r.setSpeed(STRAIGHT_DUTY_CYCLE);
//}

void scanEnvironment(uint16_t* map, uint16_t map_size)
{	
	// now we can move back and start storing sensor data
	for(uint8_t cnt=0; cnt<map_size; cnt++){
	/* NEW */
		map[cnt] = irDistance(LONG_RANGE_IR_PIN);
		
	/* OLD */
		// Take a measurement and store the result
		//map[cnt-1] = pgm_read_word(&long_range_data[readADC(LONG_RANGE_IR_PIN)]);
		//uint16_t cnt = pgm_read_word(&long_range_data[readADC(LONG_RANGE_IR_PIN)]);
		//debug.longRangeIR(cnt);
		//debug.adValue(map[cnt-1]);
		//debug.longRangeIR(map[cnt-1]);
		// move the motor and do it again
		adjustScanPlatform((adjustScanPlatform(0)+1), 1);
		// allow the platform movement to settle
		delay(100);
	}
	
	// Return the scan platform to the home position
	adjustScanPlatform(0, 1);

}

void analyzeRoom(uint16_t* map1, uint16_t* map2, uint16_t* heading_map)
{
	uint16_t result = 0;
	uint8_t consecutive_cnt = 0;
	uint16_t result_cnt = 0;

	// clear the result map
	for (uint8_t index = 0; index < 5; index++){
		heading_map[index] = 0;
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
				heading_map[result_cnt] = index*18;
				// look for the max amount of plaques
				result_cnt++;
				// exit if all the plaques were found
				if (result_cnt == MAX_PLAQUE_CNT){
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

void findPlaqueDistanceAngle(uint16_t* dist, uint16_t* angle, uint8_t plaque_num, uint16_t& plaque_dist, uint16_t& plaque_angle)
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


//	int16_t return_value = -1;	// initialize the return as a failure and change if something is found
//	// The amount of difference needed to detect an edge
//	uint8_t valid_edge = 500;
//	uint8_t consecutive_readings = 0;
//	uint16_t on_target = 0;
//	uint16_t tmp_difference = 0;
//	double object_angle = 0.0;
//	bool pass_fail = true;
//
//	for( ; valid_edge > 10; valid_edge -= 20){
//		// find an edge	
//		for (uint8_t cnt=0; cnt<199; cnt++){
//  			// store the difference between the elements and test the difference
//			tmp_difference = abs( map[cnt] - map[cnt+1] );
//			// is this an edge??
//			if(tmp_difference > valid_edge){
//  			// get the angle that will tell us how many consecutive values should be read
//				//object_angle = atan(static_cast<double>(30.4/map[cnt+1]));  	//once edge is found give us angle
//				//object_angle = (object_angle * 180) / PI;
//				//consecutive_readings = (object_angle / 1.8);  	//how many readings we should have based on distance
//				// determines if the target is found - used below
//				pass_fail = true;
//				consecutive_readings = 4;
//				// start at the edge and look for consecutive valid distances
//    			for(uint8_t w = 1; w < (consecutive_readings); w++){
//      				on_target = abs(map[cnt+w] - map[cnt+w]);
//      				// not the plaque if the difference is geater that xx
//					if(on_target > 10){
//        				// force an exit
//						w = consecutive_readings;
//        				pass_fail = false;
//      				}
//				}
//				// found object, send the angle back to the caller
//      			if(pass_fail == true){
//        			return ((consecutive_readings/2) + cnt);
//      			}
//    		}
//  		}
//	}
//
//	// return the heading in degrees to the most detectable object
//	return return_value;
//}

void adjustHeading(uint16_t angle, uint8_t dance)
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
int16_t readADC(uint16_t pin)
{
	const uint8_t sample_amout = 100;		// number of real ADC samples
	const uint8_t threshold_amount = 25;	// number of threshold samples
	uint16_t threshold = 0;					// temporary accumulator
	int16_t x_i = 0;      					// ADC input for sample i
	int16_t A_1 = 0;   						// current  i   running average
	int16_t A_0 = 0;       					// previous i-1 running average

	// Take an initial reading to calculate a threshold
	//for (uint8_t i=1; i<=threshold_amount; i++){
	//	// accumulate the readings
	//	threshold += analogRead(pin);
	//}
  	
	// now make it the average
	//threshold /= threshold_amount;
	
	// and for the real calculation
	for (uint8_t i=1; i<=sample_amout; i++){
    	x_i = analogRead(pin);
		// Check if the reading is within the accepted bounds
		//if ( (x_i < (threshold+25)) && (x_i > (threshold-25)) ){
			// rapid calculation method - http://en.wikipedia.org/wiki/Standard_deviation
			// accumulate the result
			A_1 = A_0 + (( x_i - A_0 ) / i);
			// stored for the next use
			A_0 = A_1;
		//}
	}

	// cast and return the result
	return A_1;
}

uint16_t irDistance(uint8_t pin)
{
	// store the value from the ADC converter
	uint16_t tmp_adc_result = readADC(pin);
	uint16_t dist_from_flash;
	// return a result from the short range lookup table
	if (pin == SHORT_RANGE_IR_PIN){
		
		// not using - another group can add
	
	}
	// return a result from the medium range lookup table
	else if (pin == MEDIUM_RANGE_IR_PIN){
		// reduce the size of the lookup table
		if (tmp_adc_result > (MEDIUM_RANGE_OFFSET+MEDIUM_RANGE_TABLE_SIZE)){
			return MAX_MEDIUM_RANGE_IR_DISTANCE;
		}
		else if (tmp_adc_result < MEDIUM_RANGE_OFFSET){
			return MIN_MEDIUM_RANGE_IR_DISTANCE;
		}
		else{
			// get the value from flash (offset from all the same values)
			dist_from_flash = pgm_read_byte(&medium_range_data[tmp_adc_result-MEDIUM_RANGE_OFFSET]);
			return (dist_from_flash*2);
		}	
	}
	// return a result from the long range lookup table
	else if (pin == LONG_RANGE_IR_PIN){
		// reduce the size of the lookup table
		if (tmp_adc_result > (LONG_RANGE_OFFSET+LONG_RANGE_TABLE_SIZE)){
			return MAX_LONG_RANGE_IR_DISTANCE;
		}
		else if (tmp_adc_result < LONG_RANGE_OFFSET){
			return MIN_LONG_RANGE_IR_DISTANCE;
		}
		else{
			// get the value from flash (offset from all the same values)
			dist_from_flash = pgm_read_byte(&long_range_data[tmp_adc_result-LONG_RANGE_OFFSET]);
			return (dist_from_flash*2);
		}	
	}
	return 0;
}

uint8_t adjustScanPlatform(uint8_t steps, uint8_t update)
{
	static int16_t current_step_cnt = 0;
	//debug.enable();
	
	// Only accepts steps from 0 -> 200
	if ( (steps >= 200) && (update == 1) ){
		return current_step_cnt;
	}
	
	// update = 1 means adjust the position
	if (update == 1){
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

