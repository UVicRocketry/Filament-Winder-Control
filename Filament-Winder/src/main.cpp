#include "Arduino.h"
#include "Axis.h"

#define Stp_EN 22

#define StpX_S 4
#define StpX_D 5
#define StpX_HomeLim 18 //INT pins (18,19,20,21 on ArduinoMega) are used for optimization  
#define StpX_HardLim 19

#define StpZ_S 6 
#define StpZ_D 7
#define StpZ_HomeLim 20
#define StpZ_HardLim 21

#define StpC_S 8
#define StpC_D 9

//#define StpB_Stp 10 //Not used currently
//#define StpB_Dir 11 //Not used currently
Axis zAxis(Stp_EN, StpZ_D, StpZ_S, StpZ_HomeLim, StpZ_HardLim, 300000.0, 500000.0);
Axis xAxis(Stp_EN, StpX_D, StpX_S, StpX_HomeLim, StpX_HardLim, 14000.0, 5000000.0);

void setup() {
  Serial.begin(115200);
  while(!Serial); //hold the device in loop until Serial is started
  Serial.println("Init");
  
  xAxis.init();
  zAxis.init();

  Serial.println("Setup complete, Position the axis away from home. Press any button when finished");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  zAxis.enableAxis();
  xAxis.enableAxis();
  //enable all steppers
  Serial.println("Stepper enabled, press any button to start Homing");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println("Current Action: Homing X then Z");
  
  //Home X
  xAxis.homing();
  //Home Z
  zAxis.homing();
}

void loop() {
  //Home X
  //StpC.runSpeed();
  //homing(StpX_Lim,StpX);


  //Home Z

}