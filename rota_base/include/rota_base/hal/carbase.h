
#pragma once

// c++ stl
#include <thread>

#include <ros/ros.h>

#include "rota_base/hal/can.h"

namespace rota {

// TODO
struct CarBase {
    // communication with the base
    CAN can;

    // ROS stuff
    ros::NodeHandle node;

    bool ignition();
    void drive();
};

}// namespace rota
