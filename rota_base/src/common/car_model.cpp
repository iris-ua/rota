
#include "rota_base/common/car_model.h"

double rota::CarModel::toDistanceFromPulses(uint16_t npulses)
{
    return npulses * kDistancePerTick / 1000.0;
}

uint16_t rota::CarModel::toPulsesFromMotorVelocity(double vl)
{
    return vl / kVelocityPerTick;
}

double rota::CarModel::toSteeringAngleFromSetpoint(uint16_t setpoint)
{
    // linear conversion between set point and steer angle.
    return 0.0033 * setpoint - 0.0067;
}

double rota::CarModel::toCurvatureFromSetpoint(uint16_t setpoint)
{
    // linear conversion between set point and steer angle.
    const double angle = toSteeringAngleFromSetpoint(setpoint);
    return std::tan(angle) / kAxisDistanceInMeters;
}

std::pair<double, double>
rota::CarModel::toPanTiltAnglesFromSetpoints(uint16_t pan, uint16_t tilt)
{
    const double invM = (300.0 * M_PI / 180.0) / 1023.0;
    return { pan * invM, -tilt * invM };
}

std::pair<uint16_t, uint16_t>
rota::CarModel::toPanTiltSetpointsFromAngles(double pan, double tilt)
{
    const double M = 1023.0 / (300 * M_PI / 180.0);
    return {pan * M, tilt * M};
}

//==============================================================================

rota::CarModel::CarModel()
{
    // the dead reckoning pose always starts at the origin
    x = y = theta = 0.0;
}
