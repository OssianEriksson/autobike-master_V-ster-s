#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "../../configuration/configuration.h"

#define RADIUS_OF_THE_EARTH 6371000.0

/**
 * Estimates roll using a complementary filter
 *
 * @param Ts Time step [s]
 * @param speed Approximative current speed of bike, e.g. reference speed [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param gyroX Accelerometer roll rate (around X axis) [rad/s]
 * @param accY Accelerometer Y value [m/s²]
 * @param accZ Accelerometer Z value [m/s²]
 * @return Approximated roll angle (around X axis) [rad]
 *
 * @author Ossian Eriksson
 */
double rollComplementaryFilter(double Ts, double speed, double steeringAngle, double gyroX, double accY, double accZ)
{
    static double lastEstimatedRoll = 0;

    // From Umer's report. He used a time step of 0.04 seconds
    const double C_ref = 0.985;
    const double Ts_ref = 0.04;

    // Update Umer's C value to match our time step
    double timeConstant = C_ref * Ts_ref / (1 - C_ref);
    double C = timeConstant / (timeConstant + Ts);

    // This formula is taken from a draft of Yixiao's paper. It is an approximation
    double ac = speed * speed / WHEELBASE * tan(steeringAngle);
    double accelerationRoll = atan2(accY - ac * cos(lastEstimatedRoll), accZ + ac * sin(lastEstimatedRoll));

    // Estimated roll is LP filter applied to acceleration roll approximation + HP filter applied to roll rate roll approximation
    double estimatedRoll = (1 - C) * accelerationRoll + C * (lastEstimatedRoll + Ts * gyroX);
    lastEstimatedRoll = estimatedRoll;
    return estimatedRoll;
}

void useLastValueIfNaN(double *value, double *lastValue)
{
    if (isnan(*value))
    {
        *value = *lastValue;
    }
    else
    {
        *lastValue = *value;
    }
}

/**
 * Estimates roll using a complementary filter
 *
 * @param ret String to hold the estimated state encoded as "X,Y,Psi,roll,rollRate"
 * @param Ts Time step [s]
 * @param latitude GPS latitude [rad]
 * @param longitude GPS longitude [rad]
 * @param speed GPS speed [m/s]
 * @param headingAngle GPS headingAngle [m/s]
 * @param steeringAngle Current sterring angle of bike [rad]
 * @param gyroX IMU roll rate (around X axis) [rad/s]
 * @param gyroY IMU pitch rate (around Y axis) [rad/s]
 * @param gyroZ IMU yaw rate (around Z axis) [rad/s]
 * @param accY Accelerometer X value [m/s²]
 * @param accY Accelerometer Y value [m/s²]
 * @param accZ Accelerometer Z value [m/s²]
 *
 * @author Ossian Eriksson
 */
extern void *stateEstimator(char *ret, double Ts, double latitude, double longitude, double speed, double headingAngle, double steeringAngle, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ)
{
    static bool initialized = false;
    static double latitude0, longitude0;

    static double lastLatitude = NAN;
    static double lastLongitude = NAN;
    static double lastSpeed = NAN;
    static double lastHeadingAngle = NAN;

    useLastValueIfNaN(&latitude, &lastLatitude);
    useLastValueIfNaN(&longitude, &lastLongitude);
    useLastValueIfNaN(&speed, &lastSpeed);
    useLastValueIfNaN(&headingAngle, &lastHeadingAngle);

    if (!initialized && !isnan(latitude) && !isnan(longitude))
    {
        latitude0 = latitude;
        longitude0 = longitude;

        initialized = true;
    }

    if (initialized)
    {
        double X = RADIUS_OF_THE_EARTH * (longitude - longitude0) * cos(latitude0);
        double Y = RADIUS_OF_THE_EARTH * (latitude - latitude0);
        double Psi = headingAngle;
        double roll = rollComplementaryFilter(Ts, speed, steeringAngle, gyroX, accY, accZ);
        double rollRate = gyroX;

        sprintf(ret, "%ld,%ld,%ld,%ld,%ld", X, Y, Psi, roll, rollRate);
    }
}
