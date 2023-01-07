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
//#define StpB_Stp 10 //Not used currently
//#define StpB_Dir 11 //Not used currently

//joystick (not currently using button)
#define vrx A10
#define vry A11

//relay
#define relayPin 3

//button
#define buttonPin 13

//tuned
Axis zAxis(Stp_EN, StpZ_D, StpZ_S, StpZ_HomeLim, StpZ_HardLim, 14000.0, 20000.0); //needs adjustment
Axis xAxis(Stp_EN, StpX_D, StpX_S, StpX_HomeLim, StpX_HardLim, 14000.0, 20000.0); //tuned
//Axis CAxis(Stp_EN, StpC_D, StpC_S,0xFF,0xFF, 140000.0, 14000.0);
Axis bAxis(Stp_EN, StpB_D, StpB_S,0xFF,0xFF, 14000.0, 20000.0);

void jog();
void spindleOn();
void spindleOff();

void setup() {
  Serial.begin(115200);
  while(!Serial); //hold the device in loop until Serial is started
  Serial.println("Init");

  spindleOff();
  xAxis.init();
  zAxis.init();
  bAxis.init();

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

  Serial.println("Stepper enabled, Homing carriage");
  Serial.println("Current Action: Homing X then Z");
  
  //Home X
  xAxis.homing();
  //Home Z
  zAxis.homing();

  //jog  
  Serial.println("Jog()");
  jog();
  delay(2000);
  //zAxis.moveToPos(1600);
  //delay(1000);
  /*zAxis.moveToPos(400);
  //spindleOn();
  xAxis.moveToPos(400);
  delay(2000);
  xAxis.moveToPos(-400);
  spindleOff();*/
  Serial.println("loop()");

}

void loop() {
  Serial.println("setting temp");
  float temp = 2000.0;
  Serial.println("1st chunk");
  zAxis.moveToPos(temp);
  bAxis.moveToPos(1800.0);
  xAxis.moveToPos(temp+3000.0);
  Serial.println("1st chunk end");
  delay(5000);
  Serial.println("2nd chunk");
  xAxis.moveToPos(0.0);
  zAxis.moveToPos(0.0);
  
}


void jog(){
  while(digitalRead(buttonPin) != HIGH){
    int axisMaxSpeed = 2000;
    int x = analogRead(vrx);
    int z = analogRead(vry);
    int deadband = 20;
    //move x
    if( ( (abs(512 - x) > deadband) && (digitalRead(StpX_HomeLim) == HIGH) && (digitalRead(StpX_HardLim) == HIGH) )
      ||
      ( (x > (512 + deadband)) && (digitalRead(StpX_HomeLim) == LOW) )
      ||
      ( (x < (512 - deadband)) && (digitalRead(StpX_HardLim) == LOW) ) )
      {
      xAxis.moveSpeed(map(x, 0, 1023, -axisMaxSpeed, axisMaxSpeed));
    }
    //move z
    if( ( (abs(512 - z) > deadband) && (digitalRead(StpZ_HomeLim) == HIGH) && (digitalRead(StpZ_HardLim) == HIGH) )
      ||
      ( (z > (512 + deadband)) && (digitalRead(StpZ_HardLim) == LOW) )
      ||
      ( (z < (512 - deadband)) && (digitalRead(StpZ_HomeLim) == LOW) ) )
      {
      zAxis.moveSpeed(map(z, 0, 1023, axisMaxSpeed, -axisMaxSpeed));
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