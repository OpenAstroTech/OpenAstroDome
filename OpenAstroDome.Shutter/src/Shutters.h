#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include "OpenAstroDome.h"
#include "MotorSettings.h"

typedef void (*RightStopHandler) ();
typedef void (*LeftStopHandler) ();

class Shutters
{
    public:
        Shutters(MotorSettings& settings);
        void init();
        void loop();

        void energizeMotor() const;
        void releaseMotor();
        void registerRightStopHandler(RightStopHandler handler);
        void registerLeftStopHandler(LeftStopHandler handler);

        int32_t limitOfTravel() const;
        void SetLimitOfTravel(uint32_t limit);
        void setMaximumSpeed(uint16_t speed);
        uint16_t getMaximumSpeed();
        uint16_t getMinimumSpeed();
        void setRampTime(uint16_t milliseconds);

        void hardStop();
        void stopOpenRight();
        void stopClosedRight();
        void stopOpenLeft();
        void stopClosedLeft();

        void open();
        void close();
        void stop();

        AccelStepper* leftStepper;
        AccelStepper* rightStepper;
    
    protected:
		MotorSettings* configuration;

    private:
        long rightNewPosition;
        long leftNewPosition;
        float minSpeed;
        bool leftCanRun = false;
        bool rightCanRun = false;
        RightStopHandler rightStopHandler;
        LeftStopHandler leftStopHandler;
};