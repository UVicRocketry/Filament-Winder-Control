#include "Arduino.h"
#include "Axis.h"

//stepper enable pin
#define Stp_EN 22

//x axis
#define StpX_S 4
#define StpX_D 5
#define StpX_HomeLim 18 //INT pins (18,19,20,21 on ArduinoMega) are used for optimization  
#define StpX_HardLim 19

//z axis
#define StpZ_S 6 
#define StpZ_D 7
#define StpZ_HomeLim 20
#define StpZ_HardLim 21

//c axis
#define StpC_S 8
#define StpC_D 9
//b axis
#define StpB_S 10
#define StpB_D 11 

//joystick (not currently using button)
#define vrx A10
#define vry A11

//relay
#define relayPin 3

//button
#define buttonPin 13

//TODO: all motors have different step per rotation ratios. the ones filled in below were just eyeballed
Axis zAxis(Stp_EN, StpZ_D, StpZ_S, StpZ_HomeLim, StpZ_HardLim, 1600.0, 679.9548143 , 14000.0, 20000.0); //TODO: calibrate axis better!!!
Axis xAxis(Stp_EN, StpX_D, StpX_S, StpX_HomeLim, StpX_HardLim, 1600.0, 679.9548143, 14000.0, 20000.0); //TODO: calibrate axis
Axis cAxis(Stp_EN, StpC_D, StpC_S,0xFF,0xFF, 7000.0, 7000.0, 140000.0, 14000.0); //add in to step conversion, could be used for circumference!!! this motor has a gearbox!!!
Axis bAxis(Stp_EN, StpB_D, StpB_S,0xFF,0xFF, 6500.0, 6500.0, 14000.0, 20000.0); //add in to step conversion, could be used for circumference!!!

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

void jog(bool xEnable, bool zEnable);
void spindleOn();
void spindleOff();
void spinAndMove(float stockZero, float stockEnd, float RPM, float linearSpeed);
void sanding(int numberOfPasses, float RPM, float linearSpeed);
void toolchange();
void holePattern(float fuselageDiameter, struct operation);

void setup() {
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
  
  //set home coordinates

  //Home X
  xAxis.homing();
  //Home Z
  zAxis.homing();

  //jog  
  //jog(true, true);
  cAxis.setSpeed(200);
  zAxis.setSpeed(220);
    cAxis.moveAbsolute(8000);
    zAxis.moveAbsolute(2000);
  while(cAxis.distanceToGo() != 0.0){
    zAxis.run();
    cAxis.run();
  }
  
  delay(5000);
  zAxis.setSpeed(220);
      zAxis.moveAbsolute(2000);
  while(1){
    zAxis.run();
  }
}

void loop() {}

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

void spindleOn(){
  digitalWrite(relayPin, LOW);
}

void spindleOff(){
  digitalWrite(relayPin, HIGH);
}

void spinAndMove(float stockZero, float stockEnd, float RPM, float linearSpeed){
  cAxis.setSpeed(RPM);
  zAxis.moveAbsolute(stockEnd);
  while(zAxis.distanceToGo() != 0){
    cAxis.runSpeed();
    zAxis.run();
  }
}

void sanding(int numberOfPasses, float RPM, float linearSpeed){
  //find stock zero
  jog(true, true);
  float stockZeroX = xAxis.currentPosition();
  float stockZeroZ = zAxis.currentPosition();
  delay(1000);
  jog(true, true);
  float stockEndZ = zAxis.currentPosition();
  float back = stockZeroX - 800;

  //back up
  while(xAxis.currentPosition() != back){
    xAxis.moveAbsolute(back);
    xAxis.run();
  }

  //move to stock zero z
  while(zAxis.currentPosition() != stockZeroZ){
    zAxis.moveAbsolute(stockZeroZ);
    zAxis.run();
  }
  //start spinning c
  //wait until full speed
  //move to stock zero x
  //spin c and move x at the same time
  while(xAxis.currentPosition() != stockZeroX){
    cAxis.run();
    xAxis.moveAbsolute(stockZeroX);
    xAxis.run();
  }

  bool dir = 0; // 0 --> positive z direction, 1 --> negative z direction
  while(numberOfPasses >= 0){
    if(dir == 1){
      spinAndMove(stockZeroZ, stockEndZ, RPM, linearSpeed);
    }else if(dir == 0){
      spinAndMove(stockEndZ, stockZeroZ, RPM, linearSpeed);
    }
    dir = !dir;
    numberOfPasses--;
  }
  //back x
  //stop motor
  while(xAxis.currentPosition() != back){
    cAxis.runSpeed();
    xAxis.moveAbsolute(back);
  }
}

//TODO arbitrary axis speed, change later
void toolchange(float zWorkHome){ //pass in z zero
  //user jogs the tool back //TODO:CHANGE TO MOVE TOOL BACK TO HOME LIMIT
  xAxis.moveAbsolute(200);
  xAxis.setSpeed(200); //do i even need to set speed everytime? just set it after jogging?
  while(digitalRead(StpX_HomeLim) == HIGH){
    xAxis.run();
  }
  jog(false,true);
  //change tool then hit button
  spindleOff();
  Serial.println("Spindle Off: Change tool and Press Button To Resume Program");
  while (digitalRead(buttonPin) != HIGH);
  //machine moves back to original work z
  zAxis.moveAbsolute(zWorkHome);
  zAxis.setSpeed(200);
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }
  //user jogs to new x zero
  jog(false,true);
    //done!

}

//already zeroed when this function is called
void holePattern(float previousHoleDiam, float fuselageDiameter){
  //check if we need to preform a toolchange
  if(previousHoleDiam != holePatternData.hole_diam){
    toolchange();
  }

  //move to position of holes (relative to fuselage zero)
  zAxis.setSpeed(200); //arbitrary speed
  zAxis.moveIncremental(holePatternData.axial_location);
  while(zAxis.distanceToGo() != 0.0){
    zAxis.run();
  }

  //if more than one hole, need to figure out spacing for the pattern to work
  if(holePatternData.num_holes > 1){
    float holeSpacing = cAxis.stepPerRevolution/holePatternData.num_holes;

    if( holePatternData.offset_angle != 0.0){
      cAxis.moveAbsolute(holePatternData.offset_angle);
      while(cAxis.distanceToGo() != 0.0){
        cAxis.run();
      }
    }

    spindleOn();
    for(int x = holePatternData.num_holes; x>0; x--){
      //drill
      xAxis.moveIncremental(fuselageDiameter - 400);
      delay(1000);
      xAxis.moveIncremental(fuselageDiameter + 800);

      x--;
      cAxis.moveIncremental(holeSpacing);
    }
    

  }else{
    //just drill the hole
    if( holePatternData.offset_angle != 0.0){
      cAxis.moveIncremental(holePatternData.offset_angle);
    }
    //drill
      xAxis.moveIncremental(fuselageDiameter - 400);
      delay(1000);
      xAxis.moveIncremental(fuselageDiameter + 800);
  }
  
}