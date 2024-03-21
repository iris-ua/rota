
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
    const double invM = (300.0 * M_PI / 180.0) / 1023.0;
    return setpoint * invM;
}

double rota::CarModel::toCurvatureFromSetpoint(int16_t setpoint)
{
    constexpr double a = kSteeringA, aa = a*a;
    constexpr double b = kSteeringB, bb = b*b;
    constexpr double c = kSteeringC, cc = c*c;
    constexpr double d = kSteeringD, dd = d*d;
    constexpr double e = kSteeringE, ee = e*e;
    constexpr double f = kSteeringRiser, ff = e*e;

    double alpha = toSteeringAngleFromSetpoint(setpoint);
    double s = alpha > 0.0 ? 1.0 : -1.0;

    alpha = std::fabs(alpha);

    double x = std::sqrt(dd + cc - 2*d*c*std::cos(alpha + kSteeringAlphaPrime));
    double xx = x*x;

    double phi = std::acos((xx + cc - dd) / (2*x*c));
    double gamma = std::acos((xx + ff - ee) / (2*x*f));
    double delta = std::acos((ee + ff - xx) / (2*e*f));
    double theta = 2*M_PI - alpha - kSteeringAlphaPrime - phi - gamma - delta;

    double angle = s*(M_PI*0.5 - theta);

    return std::tan(angle) / kAxisDistanceInMeters;
}

int16_t rota::CarModel::toSetpointFromCurvature(double curv)
{
    constexpr double a = kSteeringA, aa = a*a;
    constexpr double b = kSteeringB, bb = b*b;
    constexpr double c = kSteeringC, cc = c*c;
    constexpr double d = kSteeringD, dd = d*d;
    constexpr double e = kSteeringE, ee = e*e;
    constexpr double f = kSteeringRiser, ff = e*e;

    double beta = std::atan(curv * kAxisDistanceInMeters);
    double s = beta > 0.0 ? 1.0 : -1.0;

    beta = std::fabs(beta);

    double theta = M_PI*0.5 - beta;
    double x = std::sqrt(dd + ee - 2*d*e*std::cos(theta));
    double xx = x*x;

    double phi = std::acos((xx + ee - d*d) / (2*x*e));
    double gamma = std::acos((xx + ff - cc) / (2*x*f));
    double delta = std::acos((cc + ff - xx) / (2*c*f));
    double alpha = 2*M_PI - theta - phi - gamma - delta - kSteeringAlphaPrime;

    const double M = 1023.0 / (300 * M_PI / 180.0);
    return s*alpha * M;
}

int16_t rota::CarModel::toSteeringSetpointFromVelocities(double v, double w)
{
    if (std::fabs(v) < 0.1)
        return toSetpointFromCurvature(w);
    else
        return toSetpointFromCurvature(w/v);
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
    if (std::fabs(curv) < 0.001) {
        // assume the car is going ahead
        x = x + dl * std::cos(theta);
        y = y + dl * std::sin(theta);
    } else {
        double dtheta = dl * curv;
        x = x + (std::sin(theta + dtheta) - std::sin(theta)) / curv;
        y = y - (std::cos(theta + dtheta) - std::cos(theta)) / curv;

        // keep angle between [-pi, pi)
        theta = std::fmod(theta + dtheta + M_PI, 2*M_PI);
        if (theta < 0.0)
            theta += 2*M_PI;
        theta -= M_PI;
    }

    dl = 0;
    curv = 0;
}

rota::CarModel::CarModel()
{
    // the dead reckoning pose always starts at the origin
    x = y = theta = 0.0;
}
