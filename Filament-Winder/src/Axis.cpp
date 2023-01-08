#include "Axis.h"
//constructor
Axis::Axis(uint8_t uEnablePin, uint8_t uDirPin, uint8_t uPulsePin, uint8_t uHomeLimPin, uint8_t uHardLimPin, float uMaxSpeed, float uMaxAccel ): _stepper(AccelStepper::DRIVER, uPulsePin, uDirPin){
    _enablePin = uEnablePin;
    _dirPin = uDirPin;
    _pulsePin = uPulsePin;
    _homeLimPin = uHomeLimPin;
    _hardLimPin = uHardLimPin;
    _maxSpeed = uMaxSpeed;
    _maxAccel = uMaxAccel;
};


void Axis::enableAxis(){
    _stepper.enableOutputs();
    _state = true;
}

void Axis::disableAxis(){ //TODO check how to use the .disableOutput properly
    _stepper.disableOutputs();
    _state = false;
}

void Axis::stop(){
    _stepper.stop();
}

void Axis::eStop(){
    disableAxis();
}

void Axis::init(){ //add all pin definition and pin set up
// stepper pins
    _stepper.setEnablePin(_enablePin);
    _stepper.setMaxSpeed(_maxSpeed);
    _stepper.setAcceleration(_maxAccel);
    _stepper.setPinsInverted(false, false, true);
    //_stepper.setCurrentPosition(0);
// limit SW
    pinMode(_homeLimPin, INPUT_PULLUP);
    pinMode(_hardLimPin, INPUT_PULLUP);

    //attachInterrupt(digitalPinToInterrupt(_hardLimPin),digitalWrite(_enablePin, HIGH),FALLING);


// disable all stepper
    disableAxis();
}


void Axis::calibrate(){

}

void Axis::setSpeed(float speed){ //TODO Update this function for the big Machine class
    _stepper.setSpeed(speed);
}

bool Axis::runSpeed(){
    return _stepper.runSpeed();
}

long Axis::distanceToGo(){
    return _stepper.distanceToGo();
}

void Axis::run(){
    _stepper.run();
}

long Axis::currentPosition(){
    return _stepper.currentPosition();
    //TODO later convert steps to inch!
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
    Serial.println("Axis enabled");

    _stepper.setSpeed(-1000.0); // move towards home
    while(digitalRead(_homeLimPin) == HIGH){
        _stepper.runSpeed();
    } 
    _stepper.setSpeed(0.0);

    _stepper.setCurrentPosition(0);

    _stepper.move(2000);
    while(_stepper.distanceToGo() > 0){
        _stepper.run();
    }

    _stepper.setSpeed(-500.0);
    
    while(digitalRead(_homeLimPin) == HIGH){
        _stepper.runSpeed();
    }
    _stepper.setSpeed(0.0);
    _stepper.setCurrentPosition(0);

}

//pos relative to the home coordinates //fix so takes in inches and converts
void Axis::moveToPos(float pos){
    _stepper.moveTo(long(pos));
    _stepper.run();
}

//used in jogging where speed changes with input
void Axis::moveSpeed(float speed){
    //Serial.println();
    _stepper.setSpeed(speed);
    _stepper.runSpeed();
}