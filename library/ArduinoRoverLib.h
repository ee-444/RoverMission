
/**  

  * @file ArduinoRoverLib.h  
  * This is the main header file for EE444 Rover Library.  This Library includes all code
  * collections from Arduino.cc and other shield suppliers that has been previously used
  * by EE444 classes.  For documentation on how to use the Arduino architecture visit
  * http://www.arduino.cc/
  *  
  * @brief This file contains the namespace definition and header file collection for the
  * Arduino source
  *  
  * @author sgrove and other listed in the source as necessary
  *
  * @version 1.01   
  *
  */ 

#ifndef ARDUINOROVERLIB_H
#define ARDUINOROVERLIB_H

//! standard include files for memory allocation and string helpers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//! AVR files
#include <avr/pgmspace.h>

//! global project defines
#define ONBOARD_LED_PIN		13
#define LONG_RANGE_IR_PIN	3
#define MEDIUM_RANGE_IR_PIN	2
#define SHORT_RANGE_IR_PIN	1

//! Motor Duty Cycle constraints
const uint8_t STRAIGHT_DUTY_CYCLE	= 125;
const uint8_t MIN_DUTY_CYCLE 	 	= 50;
const uint8_t MAX_DUTY_CYCLE		= 200;

//! PID error update constraint
const uint8_t PID_UPDATE_INTERVAL	= 10;

//! Resolves linking errors when using arduino core in AVR Studio
#include "cplusplushelper.h"

//! main include for standard Arduino library
#include "WProgram.h"

//! additional assist code for Rover Mission
#include "EEPROM.h"
#include "AFMotor.h"

#include "rangedata3.h"
#include "missiontasks.h"

#include "MissionConsole.h"

//! sensors and drivers
#include "compass.h"
#include "pid.h"

#endif
