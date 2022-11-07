#ifndef _BALANCING_H_
#define _BALANCING_H_

extern double stabilizeBike(double Ts, double referenceRoll, double accelerationY, double accelerationZ, double rollRate, double speed, double steeringAngle, double wheelbase, double gearRatio, double Kp, double Ki, double Kd);

double calculateSteeringPWM(double angularVelocity, double gearRatio);

double pid(double Ts, double reference, double referenceRate, double value, double valueRate, double Kp, double Ki, double Kd);

double rollComplementaryFilter(double Ts, double accelerationY, double accelerationZ, double rollRate, double speed, double steeringAngle, double wheelbase);

#endif