#include <math.h>

#include "balancing.h"

/**
 * @param Ts Time step [s]
 * @param referenceRoll Reference roll [rad]
 * @param accelerationY Accelerometer Y value [m/s²]
 * @param accelerationZ Accelerometer Z value [m/s²]
 * @param rollRate Accelerometer roll rate (around X axis) [rad/s]
 * @param speed Approximative current speed of bike, e.g. reference speed [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param wheelBase Distance between front and rear wheel contact points [m]
 * @param gearRatio Gear ratio between motor and steering axis [1]
 * @param Kp PID proportionality constant [s⁻¹]
 * @param Ki PID integration constant [s⁻²]
 * @param Kd PID derivative constant [1]
 * 
 * @author Hannes Hultergård, Elliot Andersson
 */
extern double stabilizeBike(double Ts, double referenceRoll, double accelerationY, double accelerationZ, double rollRate, double speed, double steeringAngle, double wheelbase, double gearRatio, double Kp, double Ki, double Kd)
{
    // Estimate current roll
    double estimatedRoll = rollComplementaryFilter(Ts, accelerationY, accelerationZ, rollRate, speed, steeringAngle, wheelbase);

    // Apply PID control to roll to find steering angle rate
    double steeringRate = pid(Ts, referenceRoll, 0, estimatedRoll, rollRate, Kp, Ki, Kd);

    // Send Steering Rate Reference value to steering motor controller
    double steeringPWM = calculateSteeringPWM(steeringRate, gearRatio);

    return steeringPWM;
}

/**
 * Takes in the wanted angular velocity of the handlebar and return the required duty cycle for that velocity.
 * 
 * @param angularVelocity Angular velocity [rad/s]
 * @param gearRatio Gear ratio between motor and steering axis [1]
 * @return PWM (ranging ffrom 0 to 100) corresponding to the supplied angular velocity [%]
 * 
 * @author Hannes Hultergård, Elliot Andersson
 */
double calculateSteeringPWM(double angularVelocity, double gearRatio)
{
    // Convert from angular velocity (rad/s) of the handlebar,
    // to rpm of the motor.
    double rpm = -angularVelocity * 30 / M_PI * gearRatio;

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
 * Estimates roll using a complementary filter
 *
 * @param Ts Time step [s]
 * @param accelerationY Accelerometer Y value [m/s²]
 * @param accelerationZ Accelerometer Z value [m/s²]
 * @param rollRate Accelerometer roll rate (around X axis) [rad/s]
 * @param speed Approximative current speed of bike, e.g. reference speed [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param wheelBase Distance between front and rear wheel contact points [m]
 * @return Approximated roll angle (around X axis) [rad]
 * 
 * @author Ossian Eriksson
 */
double rollComplementaryFilter(double Ts, double accelerationY, double accelerationZ, double rollRate, double speed, double steeringAngle, double wheelbase)
{
    static double lastEstimatedRoll = 0;

    // From Umer's report. He used a time step of 0.04 seconds
    const double C_ref = 0.985;
    const double Ts_ref = 0.04;

    // Update Umer's C value to match our time step
    double timeConstant = C_ref * Ts_ref / (1 - C_ref);
    double C = timeConstant / (timeConstant + Ts);

    // This formula is taken from a draft of Yixiao's paper. It is an approximation
    double ac = speed * speed / wheelbase * tan(steeringAngle);
    double accelerationRoll = atan2(accelerationY - ac * cos(lastEstimatedRoll), accelerationZ + ac * sin(lastEstimatedRoll));

    // Estimated roll is LP filter applied to acceleration roll approximation + HP filter applied to roll rate roll approximation
    double estimatedRoll = (1 - C) * accelerationRoll + C * (lastEstimatedRoll + Ts * rollRate);
    lastEstimatedRoll = estimatedRoll;
    return estimatedRoll;
}
