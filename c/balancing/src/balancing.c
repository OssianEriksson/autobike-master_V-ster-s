#include <math.h>

#include "../../configuration/configuration.h"

/**
 * Takes in the wanted angular velocity of the handlebar and return the required duty cycle for that velocity.
 * 
 * @param angularVelocity Angular velocity [rad/s]
 * @param gearRatio Gear ratio between motor and steering axis [1]
 * @return PWM (ranging ffrom 0 to 100) corresponding to the supplied angular velocity [%]
 * 
 * @author Hannes Hultergård, Elliot Andersson
 */
double calculateSteeringPWM(double angularVelocity)
{
    // Convert from angular velocity (rad/s) of the handlebar,
    // to rpm of the motor.
    double rpm = -angularVelocity * 30 / M_PI * STEERING_GEAR_RATIO;

    // Convert from rpm to duty cycle,
    // 4000 is the maximum speed of the motor (configured in Escon Studio).
    return 50 + rpm * 40.0 / 4000.0;
}

/**
 * PID control block 
 *
 * @param Ts Time step [s]
 * @param reference Reference value [X]
 * @param referenceRate Time derivative of reference value [X/s]
 * @param value Current value [X]
 * @param valueRate Time derivative of current value [X/s]
 * @param Kp PID proportionality constant [Y/X]
 * @param Ki PID integration constant [Y/s/X]
 * @param Kd PID derivative constant [Y*s/X]
 * @return PID control signal [Y]
 * 
 * @author Hannes Hultergård, Elliot Andersson
 */
double pid(double Ts, double reference, double referenceRate, double value, double valueRate, double Kp, double Ki, double Kd)
{
    static double errorIntegral = 0;

    const double windupGuard = 6;

    double error = reference - value;
    errorIntegral += Ts * error;
    double errorRate = referenceRate - valueRate;

    if (errorIntegral < -windupGuard * Ts)
    {
        errorIntegral = -windupGuard * Ts;
    }
    else if (errorIntegral > windupGuard * Ts)
    {
        errorIntegral = windupGuard * Ts;
    }

    return Kp * error + Ki * errorIntegral + Kd * errorRate;
}

/**
 * @param Ts Time step [s]
 * @param referenceRoll Reference roll [rad]
 * @param roll Current roll [rad]
 * @param rollRate Curernt roll rate [rad/s]
 * @param Kp PID proportionality constant [s⁻¹]
 * @param Ki PID integration constant [s⁻²]
 * @param Kd PID derivative constant [1]
 * 
 * @author Hannes Hultergård, Elliot Andersson
 */
extern double stabilizeBike(double Ts, double referenceRoll, double roll, double rollRate, double Kp, double Ki, double Kd)
{
    // Apply PID control to roll to find steering angle rate
    double steeringRate = pid(Ts, referenceRoll, 0, roll, rollRate, Kp, Ki, Kd);

    // Send Steering Rate Reference value to steering motor controller
    double steeringPWM = calculateSteeringPWM(steeringRate);

    return steeringPWM;
}
