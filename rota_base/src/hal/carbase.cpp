
#include "rota_base/hal/carbase.h"

bool rota::CarBase::ignition()
{
    // 1. Communications with the base
    // It opens the device the has the provided serial number.
    // NOTE: If the gateway hardware changes, so will the serial number.
    bool ok = can.open("FTAK73XQ");
    if (not ok) return false;

    // 2. Setup ROS publisher/subscribers/services
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

    ros::Rate rate(10);
    while ( ros::ok() ) {

        // simple test to make sure it is working..
        // set motor velocity to 0
        CanMessage msg = { CANID_MOTOR, 6, {0,0,0,0,0,0,0,0}, true};
        auto ok = can.write(msg);

        rate.sleep();
    }// end while

    rx_thr.join();
}
