
#pragma once

// c++ stl
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

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
    ros::Publisher odom_pub;

    tf2_ros::TransformBroadcaster tfbr;

    void publishOdometry();

    // This is the initialization function.
    // It will open the CAN port and setup the necessary ROS pubs/subs.
    bool ignition();

    // CarBase main loop.
    // NOTE: This function should only be called after `ignition()`.
    void drive();

    // Called by the data rx loop.
    void handleFeedbackMessage(const CanMessage& msg);
};

}// namespace rota
