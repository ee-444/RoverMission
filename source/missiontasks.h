
#ifndef MISSIONTASKS_H
#define MISSIONTASKS_H

#include "ArduinoRoverLib.h"

//! Motor Duty Cycle constraints
const uint8_t STRAIGHT_DUTY_CYCLE	= 125;
const uint8_t MIN_DUTY_CYCLE 	 	= 50;
const uint8_t MAX_DUTY_CYCLE		= 200;

//! PID error update constraint
const uint8_t PID_UPDATE_INTERVAL	= 10;

//! mission task #1
void goStraight(uint16_t time, uint16_t itterations);
//! mission task #2a
void scanEnvironment(uint16_t* map);
//! mission task #2b
int16_t analyzeRoom(uint16_t* map);
//! mission task #2c
void findPlaque();



//! Contributed code from Kevin taken from lab4 - altered to work
int16_t readADC(uint16_t pin);

#endif

