#include "Arduino.h"
#include "AccelStepper.h"

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

AccelStepper StpX(AccelStepper::DRIVER, StpX_S, StpX_D);
AccelStepper StpZ(AccelStepper::DRIVER, StpZ_S, StpZ_D);
AccelStepper StpC(AccelStepper::DRIVER, StpC_S, StpC_D);
//AccelStepper StpB(AccelStepper::DRIVER, StpB_S, StpB_D);

inline void homing(int limitSW, AccelStepper Stepper){
  Stepper.setSpeed(5000);
    
  while(digitalRead(18) == HIGH){ //while the limit SW is not touched
    Stepper.runSpeed();
    //Stepper.run();
  }
  Stepper.setSpeed(0);
  Stepper.setCurrentPosition(0);
}


void setup() {

  pinMode(StpX_HomeLim, INPUT_PULLUP);
  pinMode(StpX_HardLim, INPUT_PULLUP);
  pinMode(StpZ_HomeLim, INPUT_PULLUP);
  pinMode(StpZ_HardLim, INPUT_PULLUP);
  
  Serial.begin(115200);
  while(!Serial); //hold the device in loop until Serial is started
  Serial.println("Init");

  StpX.setEnablePin(Stp_EN);
  StpX.setSpeed(140000);
  StpX.setAcceleration(14000);

  StpZ.setEnablePin(Stp_EN);
  StpZ.setSpeed(140000);
  StpZ.setAcceleration(14000);

  StpC.setEnablePin(Stp_EN);
  StpC.setMaxSpeed(300000);
  StpC.setAcceleration(500000);   

  //disable all the output to let user to position freely
  StpX.disableOutputs();
  StpZ.disableOutputs();
  StpC.disableOutputs();
  Serial.println("Setup complete, Position the axis away from home. Press any button when finished");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  //enable all steppers
  StpX.enableOutputs();
  StpZ.enableOutputs();
  StpC.enableOutputs();

  Serial.println("Stepper enabled, press any button to start Homing");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println("Current Action: Homing X then Z");
  
  //Home X
  digitalWrite(Stp_EN, LOW);
  //Home Z
}

void loop() {
  //Home X
  //StpC.runSpeed();
  //homing(StpX_Lim,StpX);


  //Home Z

}