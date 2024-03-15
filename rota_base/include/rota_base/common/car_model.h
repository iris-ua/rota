
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
    static double toDistanceFromPulses(uint16_t npulses);
    static uint16_t toPulsesFromMotorVelocity(double vl);
    // steering
    static double toSteeringAngleFromSetpoint(uint16_t setpoint);
    static double toCurvatureFromSetpoint(uint16_t setpoint);
    // pan & tilt
    static std::pair<double, double> toPanTiltAnglesFromSetpoints(uint16_t pan, uint16_t tilt);
    static std::pair<uint16_t, uint16_t> toPanTiltSetpointsFromAngles(double pan, double tilt);

    // dead reckoning pose
    double x, y, theta;

    CarModel();

};

}// namespace rota
