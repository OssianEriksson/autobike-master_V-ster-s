// balancing.h
#pragma once

#ifdef BALANCING_EXPORTS
#define BALANCING_API __declspec(dllexport)
#else
#define BALANCING_API __declspec(dllimport)
#endif



extern "C" BALANCING_API extern double stabilizeBike(double rollRate, double Kp, double Ki, double Kd);

extern "C" BALANCING_API double calculateSteeringPWM(double angularVelocity);

extern "C" BALANCING_API double pid(double reference, double currentValue, double Kp, double Ki, double Kd);


