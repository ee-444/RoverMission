
#include "missiontasks.h"

//! declare object that are used in this file
extern PID <uint16_t>pid;
extern HMC6352compass compass;
extern AF_DCMotor motor_l;
extern AF_DCMotor motor_r;
extern AF_Stepper motor_s;

Cmissionconsole debug;

void goStraight(uint16_t time, uint16_t itterations)
{
	// find out the millis() start time
	uint16_t last_millis = millis();
	// set the pid setpoint with the direction of the compass
	pid.setTarget(compass.getHeading());
	
	// set up the motors and get them moving straight
	motor_l.setSpeed(STRAIGHT_DUTY_CYCLE);
  	motor_r.setSpeed(STRAIGHT_DUTY_CYCLE);
  	motor_l.run(FORWARD);
  	motor_r.run(FORWARD);

	// loop the amount of passes requested by the caller (itterations)
	for (uint16_t cnt=0 ; cnt <= itterations; cnt++){
		// time per itteration set by caller
		while( time != 0){	
			// hold the program while waiting for an update interval
			// allow serial communication and other processing to occur
			while ((millis() - last_millis) <= PID_UPDATE_INTERVAL){
				// use the free running millis() as a timer
				if (millis() < last_millis){
					last_millis = 0;	// rollover occured
				}
				// processing can occur here - better than a wait statement
				//-->  <--
			}

			// update the motors based on the callers request(time)
			if ((millis() -last_millis) > PID_UPDATE_INTERVAL){
				// update the last_millis count
				last_millis += PID_UPDATE_INTERVAL;
				// get the error from the PID module - 180 degrees the the max desired error
				uint16_t error = pid.getError(compass.getHeading(), 1800);
				// update the motors based on the error - converting degree_of_error
				// to duty_cycle for correction
				motor_r.setSpeed( constrain((STRAIGHT_DUTY_CYCLE) + error, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE) );
				motor_l.setSpeed( constrain((STRAIGHT_DUTY_CYCLE) - error, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE) );
			}
		
			// decrement the amount of time that the caller asked for per itteration
			time--;
		}
		// change the direction of the rover by 180 degrees (must be signed number for logic to work)
		int16_t update_target = pid.getTarget() - 1800;
		//heading is limited from 0 to 3599
  		if(update_target >= 3600){
			//example: if 'update_target' is 3659 degrees, the program corrects it to 59 degrees
			update_target -= 3600;
		}
  		else if(update_target < 0){
			//example: if 'update_target' is -20 degress, the program corrects it to 3580 degrees
			update_target += 3599;
		}
		// Update the PID module setpoint
		pid.setTarget(update_target);
	}

	// Stop the motors - we should be either near the home position or at a distance x far from the starting point
	motor_l.run(RELEASE);
	motor_r.run(RELEASE);
	// set the duty cycle staight for the next use
	motor_l.setSpeed(STRAIGHT_DUTY_CYCLE);
  	motor_r.setSpeed(STRAIGHT_DUTY_CYCLE);
}

void scanEnvironment(uint16_t* map)
{	
	// setup the a/d peripheral
	analogReference(EXTERNAL);
	debug.enable();

	// From center - adjust to 180 degrees in a direction
	for(uint8_t cnt=1; cnt<=30; cnt++){
		// move the scan platform
		motor_s.onestep(FORWARD, SINGLE);
		delay(10);
	}
	
	// now we can move back and start storing sensor data
	for(uint8_t cnt=1; cnt<=60; cnt++){
		// Take a measurement and store the result
		map[cnt-1] = pgm_read_word(&long_range_data[readADC(LONG_RANGE_IR_PIN)]);
		debug.adValue(map[cnt-1]);
		debug.longRangeIR(map[cnt-1]);
		// move the motor and do it again
		motor_s.onestep(BACKWARD, SINGLE);
		delay(100);
	}

	// Return the scan platform to the home position
	for(uint8_t cnt=1; cnt<=30; cnt++){
		// move the scan platform
		motor_s.onestep(FORWARD, SINGLE);
		delay(10);
	}

}

int16_t analyzeRoom(uint16_t* map)
{
	uint8_t match_cnt = 0;
	int16_t return_value = -1;	// initialize the return as a failure and change if something is found

	// scan the data and look for something
	for (uint8_t cnt=1; cnt<200; cnt++){
		// look for an edge - greater than the length of a plaque
		if ( abs(map[cnt-1] - map[cnt]) > 30){
			// If there are x amount of matches then try to validate the 
			// plaque based on the distance and angles, we know the plaque size - ~30cm


		}
	}

	// return the heading in degrees to the most detectable object
	return return_value;
}

// ISSUES WITH SOME OF THE CODE HERE
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

//motor_s.setSpeed(25);  // 10 rpm ??
//motor_s.step(100, FORWARD, SINGLE); // 100 steps is 180 degrees movement, since the motor goes 360 deg. in 200 steps
//motor_s.release();
// move back home
//motor_s.step(100, BACKWARD, SINGLE); // same as FORWARD, but BACKWARD :)

// Here are different types of movement for the Stepper motor.  Will consider trying this if the torque needs
// to be increased due to the heavy weight of the scan platform.  
//  motor.step(100, FORWARD, SINGLE); 
//  motor.step(100, BACKWARD, SINGLE); 
//
//  motor.step(100, FORWARD, DOUBLE); 
//  motor.step(100, BACKWARD, DOUBLE);
//
//  motor.step(100, FORWARD, INTERLEAVE); 
//  motor.step(100, BACKWARD, INTERLEAVE); 
//
//  motor.step(100, FORWARD, MICROSTEP); 
//  motor.step(100, BACKWARD, MICROSTEP); 
