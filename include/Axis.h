#include "Arduino.h"
#ifndef MYSTEPPER_H
    #define MYSTEPPER_H
#endif
#include "AccelStepper.h"


class AccelStepper;

class Axis;

class Axis {
    public:
        float pos = 0.0;
        float speed = 0.0;
        float accel = 0.0;
        float stepPerRevolution = 200.0;
        float inToStep = 0.0;

        bool _state = false;

        Axis(uint8_t uEnablePin, uint8_t uDirPin, uint8_t uPulsePin, uint8_t uHomeLimPin, uint8_t uHardLimPin, float uStepPerRevolution, float uInToStep, float uMaxSpeed, float uMaxAccel);

        void init();
        void enableAxis();
        void disableAxis();

        long currentPosition();
        long distanceToGo();

        void setSpeed(float speed);
        void setAcceleration(float acceleration);
        bool runSpeed();

        void moveAbsolute(float pos);
        void moveIncremental(float steps);
        void run();

        void homing();
        void jog();

//add limit switch
    private:
        uint8_t _enablePin;
        uint8_t _dirPin;
        uint8_t _pulsePin;
        uint8_t _homeLimPin = 0xFF;
        uint8_t _hardLimPin = 0xFF;
        float _maxSpeed;
        float _maxAccel;
        float _stepToPosRatio = 0.0;
        AccelStepper _stepper;//(uint8_t uInterface,uint8_t uPulsePin,uint8_t uDirPin);
};
/*
Stepper properties (should I use Axis?)
- pos
- speed
- accel

- home zero (origin)
- run state
- max speed
- max accel
- step to pos ratio

Features/Functions
- move to certain location (run())
- move with certain speed (runSpeed())
- move to vertain location with certain speed (runSpeedtoPos())

- enable
- disable

*/
