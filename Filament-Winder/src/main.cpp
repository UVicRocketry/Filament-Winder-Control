#include "Arduino.h"
#include "Axis.h"

//x axis
#define StpX_S A0
#define StpX_D A1
#define StpX_EN 38

#define StpX_HomeLim 3 //INT pins (18,19,20,21 on ArduinoMega) are used for optimization  
#define StpX_HardLim 2

//z axis
#define StpZ_S 46 
#define StpZ_D 48
#define StpZ_EN A8

#define StpZ_HomeLim 18
#define StpZ_HardLim 19

//B AND C MIXED? - yes

//c axis
#define StpC_S 26
#define StpC_D 28
#define StpC_EN 24
//b axis
#define StpB_S 36
#define StpB_D 34 
#define StpB_EN 30

//joystick (not currently using button)
#define vrx A10
#define vry A11

//relay
#define relayPin 3

//button
#define buttonPin 13

//TODO: if current position and target position are too close, motor will grind because stepper needs to accelerate and deccelerate, paruse library to figure out how to fix. lib
//probably covers this issue and im implementing it wrong. FIX THIS AFTER HOLE DRILLING

//most definitely an issue with giving a speed that is too small for the distance or something, check how the library sets acceleration


//TODO: all motors have different step per rotation ratios. the ones filled in below were just eyeballed 
Axis zAxis(StpX_EN, StpZ_D, StpZ_S, StpZ_HomeLim, StpZ_HardLim, 1600.0, 678.9821813 , 14000.0, 20000.0); //TODO: calibrate axis better!!
Axis xAxis(StpZ_EN, StpX_D, StpX_S, StpX_HomeLim, StpX_HardLim, 1600.0, 679.9548143, 14000.0, 20000.0); //TODO: calibrate axis
Axis cAxis(StpC_EN, StpC_D, StpC_S,0xFF,0xFF, 6800.0, 7000.0, 140000.0, 14000.0); //add in to step conversion, could be used for circumference!! this motor has a gearbox!!
Axis bAxis(StpB_EN, StpB_D, StpB_S,0xFF,0xFF, 6500.0, 6500.0, 14000.0, 20000.0); //add in to step conversion, could be used for circumference!!

struct point{
  float xPos;
  float zPos;
}point;

struct holePatternData{
  float hole_diam;
  float axial_location;
  int num_holes;
  float offset_angle; 
}holePatternData;

void spindleOn();
void spindleOff();
bool checkLimits();
void jog(bool xEnable, bool zEnable);
void spinAndMove(float stockZero, float stockEnd, float RPM, float linearSpeed);
void sanding(int numberOfPasses, float RPM, float linearSpeed);
void toolchange(float zWorkHome);
void holePattern(float previousHoleDiam, float fuselageDiameter, float hole_diam, float axial_location, int num_holes, float offset_angle);

void setup() {
  Serial.println("here");
  Serial.begin(115200);
  while(!Serial); //hold the device in loop until Serial is started
  Serial.println("Init");

  spindleOff();
  xAxis.init();
  zAxis.init();
  bAxis.init();
  cAxis.init();

//initialize joystick
  pinMode(buttonPin, INPUT);
  pinMode(vrx, INPUT);
  pinMode(vry, INPUT);

//initialize relay
  pinMode(relayPin, OUTPUT);

  Serial.println("Setup complete, Position the axis away from home.");

  //enable all steppers
  zAxis.enableAxis();
  xAxis.enableAxis();
  bAxis.enableAxis();
  cAxis.enableAxis();

  Serial.println("Stepper enabled, Homing carriage");
  Serial.println("Current Action: Homing X then Z");
  
  Serial.println("here");

  //Home X
  xAxis.homing();

  Serial.println("here");

  //Home Z
  zAxis.homing();
  /*
  cAxis.moveIncremental(20000);
  cAxis.setSpeed(500);
  while(cAxis.distanceToGo() != 0.0){
    cAxis.run();
  }
  */

  //jog w both axis
  //jog(true, true);
  /*
  Serial.println(zAxis.currentPosition());
  zAxis.setSpeed(-750);
  zAxis.moveIncremental(-678.9821813*50);
  while((zAxis.distanceToGo() != 0.0)){
    zAxis.run();
  }
  Serial.println(zAxis.currentPosition());
  */ 

  //Serial.print("here");
  //I LOVE SANDING
  //void sanding(int numberOfPasses, float RPM, float linearSpeed)
  //sanding(20000004, -420690000, 10000);


  
  //tim's recovery piston hole pattern side A
  
  //jog(true, true);
/*
//6-32 med clr 
  holePattern(0.204, -2.65625, 0.204, 60, 3, 0.0);


  zAxis.moveIncremental(float(zAxis.inToStep*2.65625));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }

  //stop and switch drill bit.
  while(digitalRead(buttonPin) != HIGH){}
//8-32 med clr
  holePattern(0.204, -2.65625, 0.204, 60, 8, 0.0);

  //end program fr

*/
  //tim's recovery piston hole pattern side B

//6-32 med clr
    //holePattern(0.204, -2.65625, 0.204, 60, 8, 0.0);

  /*
  zAxis.moveIncremental(float(zAxis.inToStep*0.4));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }

  holePattern(0.204, -0.4, 0.204, 60, 2, 0.0);
  */

  //jog(true, true);
  //holePattern(0.204, xAxis.currentPosition(), 0.204, 0, 6, 0.0);


  //test program vvv
  
/*
  xAxis.moveIncremental(-1600);
  xAxis.setSpeed(-5);
  while((xAxis.distanceToGo() != 0.0) && (digitalRead(StpX_HomeLim) == HIGH)){
    xAxis.run();
  }

  //move to position of holes (relative to fuselage zero)
  zAxis.moveIncremental(float(zAxis.inToStep*2.5));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }
*/

//XENIA MOTHERFUCKING TWO LOWER FUSELAGE HOLE PATTERN 2023-08-17 FIBERGLASS

//START BY ZERO TO BOT FUSELAGE
jog(true, true);



//
//NEXT ZERO TO TOP FUSELAGE

//

//

//XENIA MOTHERFUCKING TWO LOWER FUSELAGE HOLE PATTERN 2023-08-17 FIBERGLASS plan --> bottom to top --> OFFSETS ALL CW INC

//START TOOL #18

//MOVE 0.231" LEFT
zAxis.moveIncremental(float(zAxis.inToStep*0.231));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }

//THRUST RING 8X8-32 RADIAL HOLES OFFSET 15 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 8, -15.0); //change back to 8 holes

//TOOLCHANGE LAUNCH SLUG 1/4-20 CLR OFFSET 45
jog(true, false);

delay(2000);
//LAUNCH SLUG 1/4-20 CLR
holePattern(0.204, -2.65625, 0.204, 60, 1, -45.0);

//TOOLCHANGE 8-32 CLR #
jog(true, false);

//MOVE 1.151" LEFT
zAxis.moveIncremental(float(zAxis.inToStep*1.151));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }

//FCR 1
//FIN CAGE FIRST OFFSET 12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -12.241);

//FIN CAGE SECOND OFFSET 90-12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -77.759);

//MOVE 2.833 LEFT
zAxis.moveIncremental(float(zAxis.inToStep*2.833));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  } 

//FCR 2
//FIN CAGE FIRST OFFSET 12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -12.241);

//FIN CAGE SECOND OFFSET 90-12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -77.759);

//MOVE 2.833 LEFT
zAxis.moveIncremental(float(zAxis.inToStep*2.833));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }  

//FCR 3
//FIN CAGE FIRST OFFSET 12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -12.241);

//FIN CAGE SECOND OFFSET 90-12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -77.759);


//MOVE 2.833 LEFT
zAxis.moveIncremental(float(zAxis.inToStep*2.833));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }  

//FCR 4
//FIN CAGE FIRST OFFSET 12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -12.241);

//FIN CAGE SECOND OFFSET 90-12.241 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 4, -77.759);


//MOVE 14.081 LEFT 
zAxis.moveIncremental(float(zAxis.inToStep*14.081+0.0625));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  } 

//*** ADJUST FOR TRUE FUSELAGE DIMENSIONS HERE

//TOOLCHANGE LAUNCH SLUG 1/4-20 CLR OFFSET 45
jog(true, false);

//LAUNCH SLUG 1/4-20 CLR
holePattern(0.204, -2.65625, 0.204, 60, 1, -45.0);

//TOOLCHANGE PROP BULKHEAD #10 clr
jog(true, false);

//PROP BULKHEAD 10-32 RADIAL HOLES OFFSET 15 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 8, -15.0);


//TOOLCHANGE 6-32 #28
jog(true, false);

//MOVE 2.389 LEFT
zAxis.moveIncremental(float(zAxis.inToStep*2.389));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  } 

//DRILL SCREW SWITCH HOLE OFFSET 107.5 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -107.5);

//DRILL SCREW SWITCH HOLE OFFSET 133 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -133.0);

//MOVE 0.236
zAxis.moveIncremental(float(zAxis.inToStep*0.236));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  } 


//DRILL SCREW SWITCH HOLE OFFSET 99.6 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -99.6);

//DRILL SCREW SWITCH HOLE OFFSET 127 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -127.0);

//DRILL SCREW SWITCH HOLE OFFSET 140 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -140.0);

//DRILL ANTENNA HOLE OFFSET 272 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -272.0);

//DRILL SCREW SWITCH HOLE OFFSET 285 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 1, -285.0);

//MOVE 7.163
zAxis.moveIncremental(float(zAxis.inToStep*7.163));
  zAxis.setSpeed(200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  } 

//TOOLCHANGE 10-32 #10
jog(true, false);

// COUPLER 10-32 RADIAL HOLES OFFSET 15 DEGREES
holePattern(0.204, -2.65625, 0.204, 60, 8, -15.0);
}







void loop() {}

void spindleOn(){
  digitalWrite(relayPin, LOW);
}

void spindleOff(){
  digitalWrite(relayPin, HIGH);
}

bool checkLimits(){
//make this for all movement conditions w limit switches
return true;
}


//TODO: ADD TO AXIS.CPP, PASS IN 
// input: axis enable (true = on, false = off) in order x,z
void jog(bool xEnable, bool zEnable){
  while(digitalRead(buttonPin) != HIGH){
    int axisMaxSpeed = 2000;
    int x = analogRead(vrx);
    int z = analogRead(vry);
    int deadband = 20;
    //move x
    if( ( (xEnable == true) && (abs(512 - x) > deadband) && (digitalRead(StpX_HomeLim) == HIGH) && (digitalRead(StpX_HardLim) == HIGH) )
      ||
      ( (xEnable == true) &&  (x > (512 + deadband)) && (digitalRead(StpX_HomeLim) == LOW) )
      ||
      ( (xEnable == true) &&  (x < (512 - deadband)) && (digitalRead(StpX_HardLim) == LOW) ) )
      {
      xAxis.setSpeed(map(x, 0, 1023, -axisMaxSpeed, axisMaxSpeed)); //TODO: CHANGE FROM STEPS TO INCH
      xAxis.run();
      Serial.println(checkLimits());
    }
    //move z
    if( ( (zEnable == true) && (abs(512 - z) > deadband) && (digitalRead(StpZ_HomeLim) == HIGH) && (digitalRead(StpZ_HardLim) == HIGH) )
      ||
      ( (zEnable == true) && (z > (512 + deadband)) && (digitalRead(StpZ_HardLim) == LOW) )
      ||
      ( (zEnable == true) && (z < (512 - deadband)) && (digitalRead(StpZ_HomeLim) == LOW) ) )
      {
      zAxis.setSpeed(map(z, 0, 1023, axisMaxSpeed, -axisMaxSpeed)); //TODO: CHANGE FROM STEPS TO INCH
      zAxis.run();
    }
  }
  zAxis.setSpeed(0.0);
  xAxis.setSpeed(0.0);
}

inline void spinAndMove(float stockZero, float stockEnd, float RPM, float linearSpeed){
  cAxis.setSpeed(RPM);
  zAxis.moveAbsolute(stockEnd);
  while((zAxis.distanceToGo() != 0) && (checkLimits() == true)){
    cAxis.runSpeed();
    zAxis.run();
  }
}

//TODO: CHANGE STEPS TO IN
//TODO: make sure same rpm is being passed in each time c axis moves frfr
//TODO: SEEMS TO JERK THE C AXIS EVERY TIME Z SWITCH DIRECTION, FIX LATER
void sanding(int numberOfPasses, float RPM, float linearSpeed){
  //find stock zero
  jog(true, true);
  float stockZeroX = xAxis.currentPosition(); //relative to home coordinates
  float stockZeroZ = zAxis.currentPosition(); //relative to home coordinates
  delay(1500);
  //find stock end
  jog(false, true);
  float stockEndZ = zAxis.currentPosition(); //relative home coordinates
  float back = -3000;

  //back up 
  xAxis.setSpeed(-2000);
  xAxis.moveIncremental(back);
  while((xAxis.distanceToGo() != 0.0)){
    xAxis.run();
  }

  //move to stock zero z
  zAxis.setSpeed(-3000);
  zAxis.moveAbsolute(stockZeroZ);
  while((zAxis.distanceToGo() != 0.0)){
    zAxis.run();
  }

  //start spinning c
  //wait until full speed
  //move to stock zero x
  //spin c and move x at the same time
  xAxis.moveAbsolute(stockZeroX);
  cAxis.setSpeed(RPM);
  while((xAxis.distanceToGo() != 0.0)){
    cAxis.runSpeed();
    xAxis.run();
  }

  bool dir = 0; // 0 --> positive z direction, 1 --> negative z direction
  while(numberOfPasses >= 0){
    if(dir == 1){
    Serial.print("here");
      spinAndMove(stockZeroZ, stockEndZ, RPM, linearSpeed);
    }else if(dir == 0){
      spinAndMove(stockEndZ, stockZeroZ, RPM, linearSpeed);
    }
    dir = !dir;
    numberOfPasses--;
  }
  //back x
  //stop motor
  xAxis.moveIncremental(back);
  cAxis.setSpeed(RPM);
  while((xAxis.distanceToGo() != 0.0)){
    cAxis.runSpeed();
    xAxis.run();
  }
  //done!
}

//TODO arbitrary axis speed, change later
void toolchange(float zWorkHome){ //pass in z zero
//move x axis back for toolchange
  xAxis.moveAbsolute(0.0);
  xAxis.setSpeed(-1000);
  while((xAxis.distanceToGo() != 0.0) || (digitalRead(StpX_HomeLim) == HIGH)){
    xAxis.run();
  }
  //change tool then hit button
  spindleOff();
  Serial.println("Spindle Off: Change tool and Press Button To Resume Program");
  while (digitalRead(buttonPin) != HIGH);
  delay(2000);

  //machine moves back to original work z
  zAxis.moveAbsolute(zWorkHome);
  zAxis.setSpeed(-2000);
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }
  //user jogs to new x zero
  jog(true,false);
  //done!
}

//TODO: WHEN THE LIMIT SWITCH CONTACTS THE C AXIS KEEPS GOING, FIX THIS LOL --> this issue can be fixed with jog code!!
//already zeroed when this function is called
void holePattern(float previousHoleDiam, float fuselageDiameter, float hole_diam, float axial_location, int num_holes, float offset_angle){

  
  //check if we need to preform a toolchange
  /*
  if(previousHoleDiam != hole_diam){
    toolchange(zAxis.currentPosition());
  }
  */

  xAxis.setAcceleration(30);

  /*back up 
  xAxis.moveIncremental(-1600);
  xAxis.setSpeed(-5);
  while((xAxis.distanceToGo() != 0.0) && (digitalRead(StpX_HomeLim) == HIGH)){
    xAxis.run();
  }

  move to position of holes (relative to fuselage zero)
  zAxis.moveIncremental(float(zAxis.inToStep*axial_location));
  zAxis.setSpeed(-200); //arbitrary speed
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }
  */

  //if more than one hole, need to figure out spacing for the pattern to work
  if(num_holes > 1){
    float holeSpacing = cAxis.stepPerRevolution/num_holes;
    offset_angle = (cAxis.stepPerRevolution/360)*offset_angle;
    if(offset_angle != 0.0){
      cAxis.moveIncremental(offset_angle);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      } 
    }

    spindleOn();

    for(int x = num_holes; x>0; x--){
      //drill
      xAxis.moveIncremental(800);
      xAxis.setSpeed(5);
      while((xAxis.distanceToGo() != 0.0) && (digitalRead(StpX_HardLim) == HIGH)){
        xAxis.run();
      }

      delay(2000);
      xAxis.moveIncremental(-800);
      xAxis.setSpeed(-5);
      while((xAxis.distanceToGo() != 0.0) /*&& (digitalRead(StpX_HomeLim) == HIGH)*/){
        xAxis.run();
      }
      delay(2000);

      cAxis.moveIncremental(holeSpacing);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      }


      
    }


    spindleOff();
    if( offset_angle != 0.0){   //reset the zero
      cAxis.moveIncremental(- offset_angle);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      }      
    }
    

  }else{
    //just drill the hole

    spindleOn();
    offset_angle = (cAxis.stepPerRevolution/360)*offset_angle;

    if( offset_angle != 0.0){
      cAxis.moveIncremental(offset_angle);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      }      
    }
    //drill
      xAxis.moveIncremental(800);
      xAxis.setSpeed(5);
      while((xAxis.distanceToGo() != 0.0) && (digitalRead(StpX_HardLim) == HIGH)){
        xAxis.run();
      }

      delay(2000);
      xAxis.moveIncremental(-800);
      xAxis.setSpeed(-5);
      while((xAxis.distanceToGo() != 0.0) /*&& (digitalRead(StpX_HomeLim) == HIGH)*/){
        xAxis.run();
      }
      delay(2000);

    spindleOff();

    if( offset_angle != 0.0){   //reset the zero
      cAxis.moveIncremental(- offset_angle);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      }      
    }

  }
  
}

