
#include <ros/ros.h>
#include <rota_base/hal/carbase.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "carbase");

    rota::CarBase carbase;
    bool ok = carbase.ignition();
    if ( ok ) carbase.drive();
}
