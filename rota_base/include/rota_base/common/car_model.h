
#pragma once

#include <cstdint>
#include <utility>
#include <cmath>

namespace rota {

struct CarModel {

    static constexpr double kWheelDiameter = 144;
    static constexpr double kPinionTeeth   = 30;
    static constexpr double kRackGearTeeth = 10;
    static constexpr double kGearRelation  = 26;

    static constexpr double kEncoderResolution = 500;
    static constexpr double kEncoderMultFactor = 4;
    static constexpr double kEncoderSampleTime = 5;

    static constexpr double kDistancePerTick = -M_PI * (kPinionTeeth / kRackGearTeeth) /
        (kGearRelation * kEncoderResolution * kEncoderMultFactor);
    static constexpr double kVelocityPerTick = kDistancePerTick / kEncoderSampleTime;

    static constexpr double kAxisDistance = 400;
    static constexpr double kAxisDistanceInMeters = kAxisDistance / 1000.0;

    // encoding/decoding of data sent to, and received from the hardware gateway.
    // driving wheel
    static double toDistanceFromPulses(int16_t npulses);
    static int16_t toPulsesFromMotorVelocity(double vl);
    // steering
    static double toSteeringAngleFromSetpoint(int16_t setpoint);
    static double toCurvatureFromSetpoint(int16_t setpoint);
    // pan & tilt
    static std::pair<double, double> toPanTiltAnglesFromSetpoints(int16_t pan, int16_t tilt);
    static std::pair<int16_t, int16_t> toPanTiltSetpointsFromAngles(double pan, double tilt);

    // dead reckoning pose
    double x, y, theta;

    CarModel();

};

}// namespace rota
