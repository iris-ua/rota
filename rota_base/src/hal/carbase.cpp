
// ROS tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
// ROS msgs
#include <nav_msgs/Odometry.h>

#include "rota_base/hal/carbase.h"

void rota::CarBase::publishOdometry()
{
    auto stamp = ros::Time::now();

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = model.x;
    odom_msg.pose.pose.position.y = model.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q; q.setRPY(0, 0, model.theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    odom_pub.publish(odom_msg);
    tfbr.sendTransform(tf_msg);
}

bool rota::CarBase::ignition()
{
    // 1. Communications with the base
    // It opens the device the has the provided serial number.
    // NOTE: If the gateway hardware changes, so will the serial number.
    bool ok = can.open("FTAK73XQ");
    if (not ok) return false;

    // 2. Setup the recurring can messages
    motor_msg = {CANID_MOTOR,    6, {0,0,0,0,0,0,0,0}, true};
    steer_msg = {CANID_STEERING, 3, {0,0,0,0,0,0,0,0}, true};
    pntlt_msg = {CANID_PAN_TILT, 6, {0,0,0,0,0,0,0,0}, true};

    priority_token = 0;

    // 3. Setup ROS publisher/subscribers/services
    node = ros::NodeHandle("~");
    odom_pub = node.advertise<nav_msgs::Odometry>("odometry", 10);

    return true;
}

void rota::CarBase::drive()
{
    // tx and rx are handled asynchronously
    // tx is handled in the main thread, rx has its own thread.
    auto rx_thr = std::thread([this](){
        while ( ros::ok() ){
            CanMessage msg;
            bool ok = this->can.read(msg);
            if (!ok) continue;

            handleFeedbackMessage(msg);
        }// end while
    });

    ros::Rate rate(kTxRate);
    while ( ros::ok() ) {

        msg_mutex.lock();

        // Send the recurring messages to the base.
        // The messages are only valid for one write.
        //
        // The order of write matters.
        // The goal is to receive the encoders and the steer feedback in this very same order.
        can.writeAndInvalidateMsg(motor_msg);
        can.writeAndInvalidateMsg(steer_msg);
        can.writeAndInvalidateMsg(pntlt_msg);

        // Decrease the priority_token so the commands sent by an agent
        // can eventually be used when there is no teleoperation.
        if (priority_token)
            priority_token -= 1;

        msg_mutex.unlock();

        rate.sleep();
    }// end while

    rx_thr.join();
}

void rota::CarBase::handleFeedbackMessage(const CanMessage& msg)
{
    // only handle valid messages.
    if (msg.valid == false) return;

    switch(msg.id) {

        case CANID_ENC: {
            int16_t npulses = (int16_t)(msg.data[1] << 8 | msg.data[0]);
            model.dl = CarModel::toDistanceFromPulses(npulses);
            break;
        }// end case

        case CANID_RD_STEERING: {
            int16_t setpoint = (int16_t)(msg.data[1] << 8 | msg.data[0]);
            double steer_ang = CarModel::toSteeringAngleFromSetpoint(setpoint);
            model.c = CarModel::toCurvatureFromSetpoint(setpoint);

            // a curvature higher than 1.1 is an error
            // we can silently ignore the error, hoping that the next
            // values are correct.
            if (std::fabs(model.c) > 1.1) {
                ROS_WARN("CarBase: invalid curvature: %f", model.c);
                break;
            }

            // NOTE: updating the dead reckoning will
            // set model.dl and model.c to zero (0).
            model.updateDeadReckoning();

            publishOdometry();
            break;
        }// end case

        case CANID_RD_PANTILT: {
            int16_t pan_st = (int16_t)(msg.data[1] << 8 | msg.data[0]);
            int16_t tlt_st = (int16_t)(msg.data[3] << 8 | msg.data[2]);
            auto pt_angles = CarModel::toPanTiltAnglesFromSetpoints(pan_st, tlt_st);

            // TODO publish
            break;
        }// end case

        default: break; // just ignore the message
    }// end switch

}
