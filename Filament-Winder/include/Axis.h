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

        Axis(uint8_t uEnablePin, uint8_t uDirPin, uint8_t uPulsePin, uint8_t uHomeLimPin, uint8_t uHardLimPin, float uMaxSpeed, float uMaxAccel );

        void init();
        void calibrate();
        void homing();
        void moveToPos(float pos);
        void moveSpeed(float speed);
        void enableAxis();
        void disableAxis();
        void stop();
        long currentPosition();
        void eStop();
        void run();
        long distanceToGo();
        void setSpeed(float speed);
        bool runSpeed();

//add limit switch
    private:
        bool _state = false;
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
