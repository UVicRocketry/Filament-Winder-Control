#include "Axis.h"

//constructor
Axis::Axis(uint8_t uEnablePin, uint8_t uDirPin, uint8_t uPulsePin, uint8_t uHomeLimPin, uint8_t uHardLimPin, float uStepPerRevolution, float uInToStep, float uMaxSpeed, float uMaxAccel): _stepper(AccelStepper::DRIVER, uPulsePin, uDirPin){
    _enablePin = uEnablePin;
    _dirPin = uDirPin;
    _pulsePin = uPulsePin;
    _homeLimPin = uHomeLimPin;
    _hardLimPin = uHardLimPin;
    stepPerRevolution = uStepPerRevolution;
    inToStep = uInToStep; //CURRENTLY UNUSED, THIS WILL CONVERT QTY TO STEP RENAME 
    _maxSpeed = uMaxSpeed;
    _maxAccel = uMaxAccel;

};

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

//return true if axis is enabled
void Axis::enableAxis(){
    _stepper.enableOutputs();
    _state = true;
}

//return false if axis disabled
void Axis::disableAxis(){ //TODO check how to use the .disableOutput properly
    _stepper.disableOutputs();
    _state = false;
}

//the current motor position relative to home zero
long Axis::currentPosition(){
    return (_stepper.currentPosition() );
    //TODO later convert steps to inch!
}

long Axis::distanceToGo(){
    return _stepper.distanceToGo();
}

void Axis::setSpeed(float speed){ 
    //TODO: CONVERT INCH/S // DEGREE/S TO STEP/S
    _stepper.setSpeed(speed);
}

void Axis:: setAcceleration(float acceleration){
    _stepper.setAcceleration(acceleration);
}

//TODO: MAKE NOT BLOCKING OR JUST SET SPEED AND RUN?
bool Axis::runSpeed(){
    //TODO: CONVERT INCH/S // DEGREE/S TO STEP/S
    return _stepper.runSpeed();
}

//set absolute position
void Axis::moveAbsolute(float pos){
    //TODO later convert steps//degrees to inch!
    _stepper.moveTo((long(pos)));
}

//set incremental distance
void Axis::moveIncremental(float steps){
    //TODO Change input to degrees and add conversion here
    _stepper.move(long(steps));
}

void Axis::run(){
    _stepper.run();
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
