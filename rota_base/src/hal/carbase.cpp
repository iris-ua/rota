
#include "rota_base/hal/carbase.h"

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
    // node = ros::NodeHandle("~");

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

            // TODO: handle msg

        }// end while
    });

    ros::Rate rate(kTxRate);
    while ( ros::ok() ) {

        msg_mutex.lock();
        // The order of write matters.
        // The goal is to receive the encoders and the steer feedback
        // in this very same order.
        can.write(motor_msg);
        can.write(steer_msg);
        can.write(pntlt_msg);

        // Decrease the priority_token so the commands sent by an agent
        // can eventually be used when there is no teleoperation.
        if (priority_token)
            priority_token -= 1;

        msg_mutex.unlock();

        rate.sleep();
    }// end while

    rx_thr.join();
}
