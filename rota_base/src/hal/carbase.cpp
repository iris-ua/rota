
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
            double distance = CarModel::toDistanceFromPulses(npulses);
            //TODO:
            break;
        }// end case

        case CANID_RD_STEERING: {
            int16_t setpoint = (int16_t)(msg.data[1] << 8 | msg.data[0]);
            double curvature = CarModel::toCurvatureFromSetpoint(setpoint);
            double steer_ang = CarModel::toSteeringAngleFromSetpoint(setpoint);
            //TODO:
            break;
        }// end case

        case CANID_RD_PANTILT: {
            int16_t pan_st = (int16_t)(msg.data[1] << 8 | msg.data[0]);
            int16_t tlt_st = (int16_t)(msg.data[3] << 8 | msg.data[2]);
            auto pt_angles = CarModel::toPanTiltAnglesFromSetpoints(pan_st, tlt_st);
            //TODO:
            break;
        }// end case

        default: break; // just ignore the message
    }// end switch

}
