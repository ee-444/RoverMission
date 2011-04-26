#ifndef MAIN_H
#define MAIN_H

#include "ArduinoRoverLib.h"


//////////////////////////////////////////////////////////
// Code from IRON RAD
//int ledPin = 13;                 // LED connected to digital pin 13
//#define map_length 400
//#define plaque_array_size 20

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
//void findObjects(){findObjects(map_length);}
void driveToObject(int plaqueNum);
int autoRange();

#endif
