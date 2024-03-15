
#include "rota_base/common/car_model.h"

double rota::CarModel::toDistanceFromPulses(int16_t npulses)
{
    return npulses * kDistancePerTick / 1000.0;
}

int16_t rota::CarModel::toPulsesFromMotorVelocity(double vl)
{
    return vl / kVelocityPerTick;
}

double rota::CarModel::toSteeringAngleFromSetpoint(int16_t setpoint)
{
    // linear conversion between set point and steer angle.
    return 0.0033 * setpoint - 0.0067;
}

double rota::CarModel::toCurvatureFromSetpoint(int16_t setpoint)
{
    // linear conversion between set point and steer angle.
    const double angle = toSteeringAngleFromSetpoint(setpoint);
    return std::tan(angle) / kAxisDistanceInMeters;
}

std::pair<double, double>
rota::CarModel::toPanTiltAnglesFromSetpoints(int16_t pan, int16_t tilt)
{
    const double invM = (300.0 * M_PI / 180.0) / 1023.0;
    return { pan * invM, -tilt * invM };
}

std::pair<int16_t, int16_t>
rota::CarModel::toPanTiltSetpointsFromAngles(double pan, double tilt)
{
    const double M = 1023.0 / (300 * M_PI / 180.0);
    return {pan * M, tilt * M};
}

//==============================================================================

void rota::CarModel::updateDeadReckoning()
{
    if (std::fabs(c) < 0.001) {
        // assume the car is going ahead
        x = x + dl * std::cos(theta);
        y = y + dl * std::sin(theta);
    } else {
        double dtheta = dl * c;
        x = x + (std::sin(theta + dtheta) - std::sin(theta)) / c;
        y = y - (std::cos(theta + dtheta) - std::cos(theta)) / c;

        // keep angle between [-pi, pi)
        theta = std::fmod(theta + dtheta + M_PI, 2*M_PI);
        if (theta < 0.0)
            theta += 2*M_PI;
        theta -= M_PI;
    }

    dl = 0;
    c = 0;
}

rota::CarModel::CarModel()
{
    // the dead reckoning pose always starts at the origin
    x = y = theta = 0.0;
}
