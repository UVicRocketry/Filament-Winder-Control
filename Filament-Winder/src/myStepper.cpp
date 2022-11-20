#include "myStepper.h"
//constructor
Axis::Axis(uint8_t uEnablePin, uint8_t uDirPin, uint8_t uPulsePin, uint8_t uHomeLimPin, uint8_t uHardLimPin, float uMaxSpeed, float uMaxAccel ){
    _enablePin = uEnablePin;
    _dirPin = uDirPin;
    _pulsePin = uPulsePin;
    _homeLimPin = uHomeLimPin;
    _hardLimPin = uHardLimPin;
    _maxSpeed = uMaxSpeed;
    _maxAccel = uMaxAccel;
}

void Axis::init(){ //add all pin definition and pin set up
// stepper pins
// limit SW
}

void Axis::enableAxis(){
    digitalWrite(Axis::_enablePin, LOW); //Pull pin LOW to enable
    Axis::_state = true;
}

void Axis::disableAxis(){ //TODO check how to use the .disableOutput properly
    digitalWrite(Axis::_enablePin, HIGH);
    Axis::_state = false;
}

void Axis::calibrate(){

}

/*Homing function will run the follow
1) enable steper
2) move to home limit switch (fast speed)
3) stop
4) set current posistion zero
5) move away from home for xx steps
6) move to home limit switch (slow speed)
7) set current position zero
*/  

void Axis::homing(){ 
    enableAxis();

    Axis::stepper.setSpeed(100000);
    while(digitalRead(_homeLimPin) == HIGH){
        Axis::stepper.runSpeed();
    } 
    Axis::stepper.setCurrentPosition(0);

    Axis::stepper.move(1000);

    Axis::stepper.setSpeed(5000);
    while(digitalRead(_homeLimPin) == HIGH){
        Axis::stepper.runSpeed();
    } 
    Axis::stepper.setCurrentPosition(0);

}

void Axis::moveToPos(float pos){

}

void Axis::moveWithSpeed(float speed){

}
