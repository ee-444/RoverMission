
#include "ArduinoRoverLib.h"
#include "main.h"

int HMC6352Address = 0x42;

// This is calculated in the setup() function
int slaveAddress;
int ledPin = 13;
int sp=0;
int pv = 0;
int err = 0;
int p = 0;
int rm = 0;
int lm = 0;
int kp=3;
int rm_ref = 97;
int lm_ref = 97;
int lostTest = 0;
int foundIt = 0;
boolean ledState = false;
byte headingData[2];
int i, headingValue,timer;
//IR Variables
int plaqueDist;
int startingLong = 999;
int startingShort = 999;
byte sensorPin0=0;
byte sensorPin1=1;
volatile uint16_t dn;
word shortRange; // same as unsigned int
word longRange;
//word lostTest;
const int longMinRange = 152; // 101.6 is the actual min, but to reduce possible errors we raised it to 60 because the short Range is more accurate.
const int longMaxRange = 508;
const int shortMaxRange = 152;
const int shortMinRange = 23;
//STEPPER MOTOR VARIABLES
int map_data[400];  // 400 entries x 2 bytes/entry = 800 bytes, ATmega328P has 2,048 bytes of SRAM
// verify constants and variables
int p_angle[plaque_array_size];  // 12 *2 = 24 bytes
int p_dist[plaque_array_size];
int p_length = 0;  // number of verified plaques, may be less than
// actual array size if some plaques were not approved
int lostMap;
int turnLeft = 0;
int turnRight = 0;
int tolerance;
// findObjects variables
int numPlaques;
int addDistPROM = 100;
int sensitivity=15; // difference in distance (cm) of mapping data for a
                    // plaque to be suspected             
int minWidth=3;

int main(void)
{
    // Must be called to configure delay code 
	// within Arduino IDE architecture
	init();
	
	
	
	
	
	
	
	// CODE FROM IRONRAD
	setup();
	// Setup the IO test led
	pinMode(ledPin, OUTPUT);      // sets the digital pin as output
	//Serial.begin(57600);

	// enter the main processing loop
	while(1){
		digitalWrite(ledPin, HIGH);   // sets the LED on
  		delay(1000);                  // waits for a second
  		digitalWrite(ledPin, LOW);    // sets the LED off
  		delay(1000);
		Serial.println("We are here again!");
	
		// call the program
		//loop();
	}
        
	return 0;
}


// CODE FROM IRONRAD
void loop(){
	 // from other project
	 boolean turnNow = digitalRead(0);
	 if (turnNow == true){
	        makeMap();
	        findObjects();
	        delay(8000);
	 }
	 delay(4000);
	 turnToFace();
	 delay(10000);
}

void setup()
{
 // Shift the device's documented slave address (0x42) 1 bit right
 // This compensates for how the TWI library only wants the
 // 7 most significant bits (with the high bit padded with 0)
 slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
 //Serial.begin(9600);
 Serial.begin(57600);
 Serial.println("in setup");
 analogReference(EXTERNAL); // where Vref is 3.3V
 pinMode(ledPin, OUTPUT);          // Set the LED pin as output
 Wire.begin();
 sp=compass();
 motor1.setSpeed(lm_ref);
 motor2.setSpeed(rm_ref);
 motor.setSpeed(10);
 motor.release();
 delay(1000);
}

int compass()
{
 // Flash the LED on pin 13 just to show that something is happening
 // Also serves as an indication that we're not "stuck" waiting for TWI data
 ledState = !ledState;
 int myHeading = 0;
 if (ledState) {
        digitalWrite(ledPin,HIGH);
 }
 else
 {
        digitalWrite(ledPin,LOW);
 }
 // Send a "A" command to the HMC6352
 // This requests the current heading data
 Wire.beginTransmission(slaveAddress);
 Wire.send("A");                  // The "Get Data" command
 Wire.endTransmission();
 delay(10);
 Wire.requestFrom(slaveAddress, 2);            // Request the 2 byte heading (MSB comes first)
 i = 0;
 while(Wire.available() && i < 2)
 {
        headingData[i] = Wire.receive();
        i++;
 }
 headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
 myHeading = (headingValue / 10);
 return myHeading;
 delay(20);
}


void driveForward(){
 int newPlaqueDistL;
 int newPlaqueDistR;
 motor1.run(FORWARD);
 motor2.run(FORWARD);
 if(lostTest > 45) motor.step(8, FORWARD, INTERLEAVE);
 while((lostTest > 45)){
        turnRight = 0;
        turnLeft = 0;
        newPlaqueDistL = 0;
        newPlaqueDistR = 0;
        for(i=0; i < 8; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnRight++;
           lostTest = lostMap;
         }
        }
        for(i=0; i < 8; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnLeft++;
           lostTest = lostMap;
         }
        }
   
        //lostTest = (newPlaqueDistL + newPlaqueDistR)/(turnLeft + turnRight);
        err = turnLeft - turnRight;
        p=2*kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(lm);
        motor2.setSpeed(rm);
        if (lostTest < 45){
         motor.step(8, FORWARD, INTERLEAVE);
         delay(400);
         break;
        }
        turnRight = 0;
        turnLeft = 0;
        newPlaqueDistL = 0;
        newPlaqueDistR = 0;
        for(i=0; i < 8; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnLeft++;
           lostTest=lostMap;
           //newPlaqueDistR = (lostMap + newPlaqueDistL);
           }
         }
         for(i=0; i < 8; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnRight++;
           lostTest=lostMap;
           //newPlaqueDistR = (lostMap + newPlaqueDistR);
           }
         }
        //lostTest = (newPlaqueDistL + newPlaqueDistR)/(turnLeft + turnRight);
        if (lostTest < 45)   motor.step(8, BACKWARD, INTERLEAVE);
        err = turnLeft - turnRight;
        p=2*kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(lm);
        motor2.setSpeed(rm);
        delay(400);
 }
 motor1.run(FORWARD);
 motor2.run(FORWARD);
 motor1.setSpeed(lm_ref);
 motor2.setSpeed(rm_ref);
 while(timer<=3){
        timer += 1;
        pv=compass();
        err=pv-sp;
        if (err > 180) err -= 360;
        else if (err < -180) err += 360;
        p=kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(lm);
        motor2.setSpeed(rm);
 }
}


void driveBackward(){
 timer=0;
 motor1.run(BACKWARD);
 motor2.run(BACKWARD);
 if(lostTest < (plaqueDist - 45)) motor.step(8, FORWARD, INTERLEAVE);
 while((lostTest < (plaqueDist - 45))){
        turnRight = 0;
        turnLeft = 0;
        for(i=0; i < 8; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnRight++;
           lostTest = lostMap;
         }
        }
        for(i=0; i < 8; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnLeft++;
           lostTest = lostMap;
         }
        }
   
        //lostTest = (newPlaqueDistL + newPlaqueDistR)/(turnLeft + turnRight);
        err = turnLeft - turnRight;
        p=2*kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(lm);
        motor2.setSpeed(rm);
        if (lostTest > (plaqueDist - 45)){
         motor.step(8, FORWARD, INTERLEAVE);
         delay(400);
         break;
        }
        turnRight = 0;
        turnLeft = 0;
        for(i=0; i < 8; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnLeft++;
           lostTest=lostMap;
           //newPlaqueDistR = (lostMap + newPlaqueDistL);
           }
         }
         for(i=0; i < 8; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnRight++;
           lostTest=lostMap;
           //newPlaqueDistR = (lostMap + newPlaqueDistR);
           }
         }
        //lostTest = (newPlaqueDistL + newPlaqueDistR)/(turnLeft + turnRight);
        if (lostTest > (plaqueDist - 45))   motor.step(8, BACKWARD, INTERLEAVE);
        err = turnLeft - turnRight;
        p=2*kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(lm);
        motor2.setSpeed(rm);
        delay(400);
 }
 while(timer<=3){
        timer += 1;
        pv=compass();
        err=pv-sp;
        if (err > 180) err -= 360;
        else if (err < -180) err += 360;
        p=kp*err;
        lm=constrain(lm_ref-p,50,100);
        rm=constrain(rm_ref+p,50,100);
        motor1.setSpeed(rm);  //Values of rm and lm are sent to opposite motors and inverted to track in reverse
        motor2.setSpeed(lm);
 }
}



void rangeFinder(){
int xminVal;
int xmaxVal;
int yminVal;
int ymaxVal;
int n = 100;
 int x_i;
 int y_i;
 float A_0;
 float A_1;
 float Q_0;
 float Q_1;
 float A_2;
 float A_3;
 float Q_2;
 float Q_3;
// initialization
xminVal = 1024;
xmaxVal = 0;
yminVal = 1024;
ymaxVal = 0;
A_0 = 0;
Q_0 = 0;
for (int i=1; i <= n; i++){
        x_i = analogRead(sensorPin0);
        y_i = analogRead(sensorPin1);
        if (x_i < xminVal){
            xminVal = x_i;
        }
        if (x_i > xmaxVal){
            xmaxVal = x_i;
        }
        if (y_i < yminVal){
            yminVal = y_i;
        }
        if (y_i > ymaxVal){
            ymaxVal = y_i;
        }
        // statistic package
        // rapid calculation method
        // http://en.wikipedia.org/wiki/Standard_deviation
        A_1 = A_0 + (x_i - A_0)/i;
        Q_1 = Q_0 + (x_i - A_0)*(x_i - A_1);
        A_0 = A_1;
        Q_0 = Q_1;
         
        A_3 = A_2 + (y_i - A_2)/i;
        Q_3 = Q_2 + (y_i - A_2)*(y_i - A_3);
        A_2 = A_3;
        Q_2 = Q_3;
}
 x_i=A_3;
 y_i=A_1;
  //shortRange = pgm_read_word(&range_data[x_i]);
  //longRange = pgm_read_word(&range_data[y_i + 1024]);
}



void locateLongTarget(){
 int lostMap;
 int turnLeft = 0;
 int turnRight = 0;
 int tolerance = lostTest + 15;
 rangeFinder();
 if(longRange > tolerance){
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        for(i=0; i < 10; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity) turnLeft++;
        }
        motor.step(10, FORWARD, INTERLEAVE);
        for(i=0; i < 10; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity) turnRight++;
        }
        motor.step(10, BACKWARD, INTERLEAVE);
        delay(500);
}
        motor1.setSpeed(70);
        motor2.setSpeed(70);
        delay(10);
        while (turnRight > turnLeft){
         motor1.run(FORWARD);
         rangeFinder();
         delay(20);
         if(longRange <= tolerance){
           motor1.run(RELEASE);
           motor2.run(RELEASE);
           break;
         }
        }
        delay(200);
        while (turnLeft > turnRight){
         motor2.run(FORWARD);
         rangeFinder();
         delay(20);
         if(longRange <= tolerance){
         motor1.run(RELEASE);
         motor2.run(RELEASE);
         break;
         }
        }
        delay(400);
        sp=compass();
        motor1.setSpeed(lm_ref);
        motor2.setSpeed(rm_ref);
 }



void locateShortTarget(){
 int lostMap;
 int turnLeft = 0;
 int turnRight = 0;
 tolerance = lostTest + 15;
 rangeFinder();
 if(shortRange > tolerance){
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        for(i=0; i < 15; i++){
         motor.step(1, BACKWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnLeft++;
         }
        }
        motor.step(15, FORWARD, INTERLEAVE);
        for(i=0; i < 15; i++){
         motor.step(1, FORWARD, INTERLEAVE);
         lostMap = autoRange();
         if ((lostMap - lostTest) < sensitivity){
           turnRight++;
         }
        }
        motor.step(15, BACKWARD, INTERLEAVE);
}
        motor1.setSpeed(75);
        motor2.setSpeed(75);
        delay(50);
        while (turnRight > turnLeft){
         motor1.run(FORWARD);
         rangeFinder();
         delay(100);
         if(shortRange <= tolerance){
           motor1.run(RELEASE);
           motor2.run(RELEASE);
           break;
         }
        }
        delay(200);
        while (turnLeft > turnRight){
         motor2.run(FORWARD);
         rangeFinder();
         delay(50);
         if(shortRange <= tolerance){
         motor1.run(RELEASE);
         motor2.run(RELEASE);
         break;
         }
        }
        delay(400);
        sp=compass();
        motor1.setSpeed(lm_ref);
        motor2.setSpeed(rm_ref);
 }



void makeMap()
{
tweak();                  // adjust IR Ranger zero angle
// take 200 measurements and 199 steps (stops at 179.1 degrees clock-wise)   
for (int i = 0; i < 200; i = i++) {
        map_data[i] = autoRange();
        Serial.println(map_data[i]);
        motor.step(1, FORWARD, INTERLEAVE);  // 0.9 degrees/step
        delay(40);                       // @ 10 rpm, this should take 15 ms + 45 ms margin
}
// rotate counter clock-wise 399 steps (stops at 180 degrees clock-wise)
motor.step(399, BACKWARD, INTERLEAVE);   // 0.9 degrees/step x 399 steps = - 359.1 degrees
delay(6000);                        // @ 10 rpm, this should take 6 seconds + 4 second margin
// take 200 measurements and 199 steps (stops at 359.1 degrees clock-wise)
for (int i = 0; i < 200; i = i++) {
        map_data[i + 200] = autoRange();
        Serial.println(map_data[i + 200]);
        motor.step(1, FORWARD, INTERLEAVE);  // 0.9 degrees/step
        delay(40);                       // @ 10 rpm, this should take 15 ms + 45 ms margin
}
// cancel extra step
motor.step(1, BACKWARD, INTERLEAVE);
Serial.println("initial mapping complete");
Serial.println("add targets and enter go");
tweak();                  // adjust IR Ranger zero angle
// take 200 measurements and 199 steps (stops at 179.1 degrees clock-wise)   
for (int i = 0; i < 200; i = i++) {
        map_data[i] = map_data[i] - autoRange();
        Serial.println(map_data[i]);
        motor.step(1, FORWARD, INTERLEAVE);  // 0.9 degrees/step
        delay(40);                       // @ 10 rpm, this should take 15 ms + 45 ms margin
}
// rotate counter clock-wise 399 steps (stops at 180 degrees clock-wise)
motor.step(399, BACKWARD, INTERLEAVE);   // 0.9 degrees/step x 399 steps = - 359.1 degrees
delay(8000);                        // @ 10 rpm, this should take 6 seconds + 4 second margin
// take 200 measurements and 199 steps (stops at 359.1 degrees clock-wise)
for (int i = 0; i < 200; i = i++) {
        map_data[i + 200] = map_data[i + 200] - autoRange();
        Serial.println(map_data[i + 200]);
        motor.step(1, FORWARD, INTERLEAVE);  // 0.9 degrees/step
        delay(40);                       // @ 10 rpm, this should take 15 ms + 45 ms margin
}
// cancel extra step
motor.step(1, BACKWARD, INTERLEAVE);
Serial.println("difference mapping complete");
}



/*
*  Terminal adjust of starting angle
*  Enter the plus or minus character (+/-) and the number of steps (n = 1 to 9).  
*/
void tweak(){
        char ch;                       // used by tweak
        Serial.println("enter +n, -n, or go where n = 1-9 steps and go = start scan");
        while(1)
        {
            while (Serial.available() <= 1) {
              // do nothing
            }
          
        // send data only when you receive data:
        if (Serial.available() > 1) {
          // read the incoming byte:
          ch = Serial.read();
              // Serial.println(ch, HEX);
            if ( ch == 0x2d ) {
               Serial.print("minus ");
               ch = Serial.read();
               ch = ch - 0x30;
               Serial.println(ch, HEX);
          // 0.9 degrees/step x 399 steps = - 359.1 degrees
          motor.step(ch, BACKWARD, INTERLEAVE);   
            }
            
            else if ( ch == 0x2b ) {
                Serial.print("plus ");
               ch = Serial.read();
               ch = ch - 0x30;
               Serial.println(ch, HEX);
               motor.step(ch, FORWARD, INTERLEAVE);  // 0.9 degrees/step
            }
            
              else if (ch == 0x67 ) {
                 ch = Serial.read();
                 delay(250);
                 Serial.println("scanning...");
                 return;
            }
            
            else {
                ch = Serial.read();
                Serial.println("unknown entry");  
              }
        }
 }
}



int autoRange()
{
// replace with experimentally determined value
          
int range;
rangeFinder();
range = shortRange;
if (range >= shortMaxRange){
        range = longRange;
}  
// Serial.println(range);
return (range);
}



void findObjects(int mapEnd){
Serial.println("finding objects");
boolean plaqueDetected=false; // flag determining recording status
// false means that there is currently no plaque detected,
// true means a plaque is being evaluated
int gapCounter=0;
int gapTolerance=1;
int beginning=0;           // starting index of a suspected plaque (see sensitivity)
int ending=0;              // ending index of a suspected plaque
int p_Idx=0;               // index of a detected plaque
//  makeMap(mapEnd);
for (int i = 0; i < map_length; i++){                   // don't stop detecting plaques
        // if the difference is greater than (15), and we're not already recording,
  // then start recording plaque length
        if((map_data[i]>=sensitivity)&!plaqueDetected){
          beginning = i;
          // The previous gap count could be moved in code for different effects.
        // It is placed here so that it counts the total length of gaps in a
        // signal as opposed to any individual gap in a signal ie: for a
        // tolerance of 2 10101011 won't pass.
          gapCounter = 0;                                     
          plaqueDetected=true;                                
        }
        if((map_data[i]<sensitivity)&plaqueDetected){         
          gapCounter++;                     // if the value falls below (15) don�t count
          ending = i - gapCounter;          // the extra gap in the ending of the plaque
          if(gapCounter > gapTolerance) {        // only stop after the gap has exceeded
              plaqueDetected=false;                 // the tolerance  
              if(ending-beginning >= minWidth){ // if plaque was wide enough count it
                  p_angle[p_Idx]= (ending + beginning)/2; // save the angle as the
                  //Serial.print(p_Idx);              // midpoint of the beginning and end
                  //Serial.print(": ");
                  //Serial.println(p_angle[p_Idx]);
                  p_Idx++;                     // move to the next array location
          }
          }
        }
}
verify(p_Idx);             // manual override y/n of detected plaques
}
// else gapCounter=0; // place the reset to gapCounter here if you'd like
                        // to clear it after the end of every gap




void verify(int numDetected){
 char userIn;
 int angle;         // current angle to detected but not verified plaque
 int dist;          // current angle to detected but not verified plaque
 boolean valid; // used to see if user input is valid
 Serial.println("Please manually verify with a y/n if this is a plaque");
 Serial.print("This is out of: ");
 Serial.println(numDetected);
 Serial.println("Angle:Distance");
 p_length = -1;  // initialize number of verified plaques
 for (int p_Idx = 0; p_Idx < numDetected; p_Idx++){
        valid=false;
        // angle is a interleave number of steps NOT degrees or radians
        angle = p_angle[p_Idx];
        // This assumes FORWARD is CW and BACKWARD is CCW
        // This may not be a good assumption for your rover.
        motor.step(angle,FORWARD,INTERLEAVE);
        delay(40);
        dist = autoRange();
        Serial.print(angle);
        Serial.print(": ");
        Serial.println(dist);
        Serial.flush();
        while(Serial.available()==0){}
        valid = false;
        while(!valid){
         userIn=Serial.read();
         if (userIn=='y'||userIn=='Y'){
           p_length++;
           p_angle[p_length] = angle;
           if (angle > 255){
             EEPROM.write((3 * p_length), 255);
             EEPROM.write(((3 * p_length) + 1), (angle - 255));
           }
           if (angle < 255){
             EEPROM.write((3 * p_length), 0);
             EEPROM.write((3 * p_length + 1), angle);
           }
           p_dist[p_length] = dist;
           EEPROM.write(((3 * p_length) + 2), dist);
           valid=true;
         } else if(userIn=='n'||userIn=='N'){
           valid=true;
         }
         else {
          Serial.println("Invalid input");
          Serial.flush();
        }
        // now that the user has verified or disallowed this plaque
        // return to home position.
        motor.step(angle,BACKWARD,INTERLEAVE);
  }
 }
 Serial.println("Done");
 numPlaques = p_length;
}




void turnToFace(){
 int angle;
 int j;
 int compassAngle;
 int err;
 for(j = 0 ; j <= numPlaques ; j++){
        angle = EEPROM.read(3*j) + EEPROM.read((3*j)+1);
        Serial.print("These should be Step Numbers up to 399: ");
        Serial.println(angle);
        compassAngle = compass();
        Serial.print("CompassAngle : ");
        Serial.println(compassAngle);
        angle = (angle * 0.9) + compassAngle;
        if (angle > 360) angle -=360;        // These 3 lines of code determine the true bearing of plaques
        Serial.print("Angle of the Plaque (TRUE) : ");
        Serial.println(angle);
        if (angle > 255){
             EEPROM.write((3 * j), 255);
             EEPROM.write(((3 * j) + 1), (angle - 255));
           }
           if (angle < 255){
             EEPROM.write((3 * j), 0);
             EEPROM.write(((3 * j) + 1), angle);
           }
 }
 for(j = 0 ; j <= numPlaques ; j++){  // numPlaques is determined from numDetected in verify function
        angle = EEPROM.read(3*j) + EEPROM.read((3*j) + 1);
        Serial.print("True Bearing of Plaque");
        Serial.println(angle);
        Serial.print("Comopass Reading ");
        Serial.println(compassAngle);
   
         err = compassAngle - angle;
         if (err > 180) err -= 360;
         else if (err < -180) err += 360;
         
         if (err < 0){
           motor1.setSpeed(70);
           motor2.setSpeed(70);
           motor1.run(FORWARD);
           motor2.run(BACKWARD);
         }
         if (err > 0){
           motor1.setSpeed(70);
           motor2.setSpeed(70);
           motor1.run(BACKWARD);
           motor2.run(FORWARD);
         }
           while((err > 2) || (err < -2)){
             Serial.println("CounterClockwise Rotation");
             Serial.print("Angle of Plaque ");
             Serial.println(angle);
             Serial.print("Compass Reading");
             Serial.println(compassAngle);
             compassAngle = compass();
             err = compassAngle - angle;
             if (err > 180) err -= 360;
             else if (err < -180) err += 360;
           }
         motor1.run(RELEASE);
         motor2.run(RELEASE);
         motor1.setSpeed(lm_ref);
         motor2.setSpeed(rm_ref);
         driveToObject(j);
         delay(5000);
 }
}




void driveToObject(int plaqueNum){
 plaqueDist = EEPROM.read((3*plaqueNum)+2);
 Serial.print("EEPROM Distance: ");
 Serial.println(plaqueDist);
 int objective = 1;
 lostTest = plaqueDist;
 if(plaqueDist <= shortMaxRange){
        objective = 2;
        locateShortTarget();
 }
 else locateLongTarget();
 while(objective == 1){
        Serial.print("Long Range Distance = ");
        Serial.println(longRange);
        driveForward();
        rangeFinder();
        if(longRange >= (lostTest + 15)){
         Serial.print("You are lost!!!");
         locateLongTarget();
        }
        if(shortRange <= shortMaxRange){
         objective=2;
         break;
        }
        lostTest=longRange;
 }
lostTest=shortRange;
 while(objective == 2){
        Serial.print("Short Range Distance = ");
        Serial.println(shortRange);
        driveForward();
        rangeFinder();
        if(shortRange >= (lostTest + 20)){
         Serial.print("You are lost!!!");
         locateShortTarget();
        }
        if(shortRange <= shortMinRange){
         objective = 3;
         motor1.run(RELEASE);
         motor2.run(RELEASE);
         delay(400);
         motor1.setSpeed(lm_ref);
         motor2.setSpeed(rm_ref);
         break;
        }
        lostTest=shortRange;
 }
 while(objective == 3){
   
        if(lostTest >= plaqueDist){  
         motor1.run(RELEASE);
         motor2.run(RELEASE);
         Serial.print("You made it home shortly!!!");
         delay(10000);
        }
        Serial.print("Short Range Distance = ");
        Serial.println(shortRange);
        driveBackward();
        rangeFinder();
        if(shortRange >= lostTest + 15){
         Serial.print("You are lost!!!");
         locateShortTarget();
        }
        if(shortRange >= plaqueDist){
        motor1.run(RELEASE);
        motor2.run(RELEASE);
        break;
        }
        if(shortRange >= shortMaxRange){
        objective = 4;
        break;
        }
        lostTest=shortRange;
 }
 lostTest=longRange;
 while(objective == 4){
         Serial.print("Long Range Distance = ");
         Serial.println(longRange);
         driveBackward();
         rangeFinder();
         if(longRange >= lostTest + 25){
           Serial.print("You are lost!!!");
           locateLongTarget();
         }
         lostTest=longRange;
         if(lostTest >= plaqueDist){
           motor1.run(RELEASE);
           motor2.run(RELEASE);
           Serial.print("you made it home!!!");
         }
        }
}

