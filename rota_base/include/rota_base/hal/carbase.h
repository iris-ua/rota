
#pragma once

// c++ stl
#include <thread>
#include <mutex>

#include <ros/ros.h>

#include "rota_base/common/car_model.h"
#include "rota_base/hal/can.h"

namespace rota {

// TODO
struct CarBase {

    static constexpr int kTxRate = 40;

    CAN can;        // communication channels
    CarModel model; // robot's model

    // Recurring can messages.
    CanMessage motor_msg;
    CanMessage steer_msg;
    CanMessage pntlt_msg;
    // Driving commands can be sent by an agent or by a teleop device.
    // When the priority_token is _not_ zero, commands from an agent are ignored.
    // When a command is sent by a teleop device, the priority token is set to a
    // fixed valued. The priority token is decreased at the end of the send
    // commands loop.
    int priority_token;
    // The write/read of the messages are async, therefore,
    // we need to ensure proper concurrency.
    std::mutex msg_mutex;

    // ROS stuff
    ros::NodeHandle node;

    bool ignition();
    void drive();
};

}// namespace rota
