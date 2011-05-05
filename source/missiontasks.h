
#ifndef MISSIONTASKS_H
#define MISSIONTASKS_H

#include "ArduinoRoverLib.h"

//! Motor Duty Cycle constraints
const uint8_t STRAIGHT_DUTY_CYCLE	= 125;
const uint8_t MIN_DUTY_CYCLE 	 	= 50;
const uint8_t MAX_DUTY_CYCLE		= 200;

//! PID error update constraint
const uint8_t PID_UPDATE_INTERVAL	= 10;

const uint8_t ROOM_DIFFERENCE_AMOUNT = 100;
const uint8_t ROOM_CONSECUTIVE_CNT	= 3;

//! mission task #1
void goStraight(uint16_t time);
//! mission task #2a
void scanEnvironment(uint16_t* map);
//! mission task #2b
int16_t analyzeRoom(uint16_t* map);
//! mission task #2c
void findPlaques();

void analyzeRoom(uint16_t* map1, uint16_t* map2, uint16_t* heading_map);

//! turn to a plaque
void turnToFace(uint16_t angle);



//! Contributed code from Kevin taken from lab4 - altered to work
int16_t readADC(uint16_t pin);

uint16_t irDistance(uint8_t pin);

//! helper functions
uint8_t adjustScanPlatform(uint8_t angle, uint8_t update = false);

#endif

