#ifndef MAIN_H
#define MAIN_H

//! make available externally defined objects
#ifdef EEPROM
extern EEPORMClass EEPROM
#endif
#ifdef Wire
extern TwoWire Wire;
#endif
#ifdef motor1
extern AF_DCMotor motor1;
#endif
#ifdef motor2
extern AF_DCMotor motor2;
#endif
#ifdef motor
extern AF_Stepper motor;
#endif


//////////////////////////////////////////////////////////
// Code from IRON RAD
//int ledPin = 13;                 // LED connected to digital pin 13
#define map_length 400
#define plaque_array_size 20

int compass();
void locateLongTarget();
void locateShortTarget();
void driveForward();
void driveBackward();
void rangeFinder();
void makeMap();
void tweak();
void turnToFace();
void verify(int numDetected);
void findObjects(int mapEnd);
void findObjects(){findObjects(map_length);}
void driveToObject(int plaqueNum);
int autoRange();

#endif
