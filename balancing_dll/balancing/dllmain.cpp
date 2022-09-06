// dllmain.cpp : Définit le point d'entrée de l'application DLL.
#include "pch.h"
#include "balancing.h"

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

#define gearRatio 111.0
#define pi 3.141592

#define windupGuard 6

double integral = 0;
double derivative = 0;

double previousError = 0;




double pid(double reference, double currentValue, double Kp, double Ki, double Kd) {
    double error = reference - currentValue;

    integral += error;
    derivative = error - previousError;

    if (integral < -windupGuard) {
        integral = -windupGuard;
    }
    else if (integral > windupGuard) {
        integral = windupGuard;
    }

    previousError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}


// Take in the wanted angular velocity of the handlebar,
// And return the required duty cycle for that velocity.
double calculateSteeringPWM(double angularVelocity) {
    // Convert from angular velocity (rad/s) of the handlebar,
    // to rpm of the motor.
    double rpm = -angularVelocity * 30 / pi * gearRatio;

    // Convert from rpm to duty cycle,
    // 4000 is the maximum speed of the motor (configured in Escon Studio).
    return 50 + rpm * 40.0 / 4000.0;
}

extern double stabilizeBike(double rollRate, double Kp, double Ki, double Kd) {
    double steeringRate = pid(0, rollRate, Kp, Ki, Kd);

    // Send Steering Rate Reference value to steering motor controller
    double steeringPWM = calculateSteeringPWM(steeringRate);

    return steeringPWM;
}





