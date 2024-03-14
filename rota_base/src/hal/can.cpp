// stl
#include <iostream>

#include <vector>
#include <string>

#include <ros/ros.h>

#include "rota_base/hal/can.h"

bool rota::CAN::open(const std::string& id)
{
    // 1. list all available ports
    std::vector<serial::PortInfo> port_list = serial::list_ports();

    // 2. select the device with the correct serial number
    std::string port;
    for (auto& p : port_list){
        auto pos = p.hardware_id.find(id);
        if ( pos != std::string::npos ) {
            port = p.port;
            break;
        }// end if
    }// end for

    if (port.empty()) {
        ROS_ERROR("CAN: device not found");
        exit(1);
    }

    ROS_INFO("CAN: opening port: %s", port.c_str());
    comm.setPort(port);
    comm.setBaudrate(115200);
    auto timeout = serial::Timeout::simpleTimeout(1000); // in miliseconds
    comm.setTimeout(timeout);

    try {
        comm.open();
    } catch (std::exception& e) {
        ROS_ERROR("CAN: failed to open port with error: %s", e.what());
        return false;
    }

    return true;
}

bool rota::CAN::read(CanMessage& msg)
{
    int n, ctn;
    uint8_t buf[MSG_MAX_SZ];

    msg.valid = false;

    // make sure we are at the start of a frame
    while (true){
        n = comm.read(&buf[0], 1);
        if (n == 0) {
            ROS_WARN("CAN: read: timeout on start");
            return false;
        }// end if
        if (buf[0] == '%') break; // start of a frame found !!
    }

    ctn = 1;
    while(ctn < (MSG_MAX_SZ)){
        n = comm.read(&buf[ctn], 1);
        if (n == 0) {
            ROS_WARN("CAN: read: timeout");
            return false;
        }// end if

        ctn += n;
        if (buf[ctn-1] == '#') break; // end of frame
    }// end while

    if (ctn == (MSG_MAX_SZ)){
        ROS_ERROR("CAN: read: frame size exceds maximum size, skipping frame");
        return false;
    }

    // Now, convert the frame to a can message
    int checksum = 0;

    // extract id
    ctn = 1;
    sscanf((const char*)&buf[ctn], "%4x%n", &msg.id, &n);
    if (n != 4) {
        ROS_ERROR("CAN: read: error reading id, skipping frame");
        return false;
    }// end if
    checksum += msg.id;

    // extract data
    ctn += 4;
    while (buf[ctn] != '&') {

        sscanf((const char*)&buf[ctn], "%02hhx%n", &msg.data[(ctn-5)/2], &n);
        if (n != 2) {
            ROS_ERROR("CAN: read: error reading frame data, skipping frame");
            return false;
        }// end if

        checksum += msg.data[(ctn-5)/2];
        ctn += 2;
    }// end while
    msg.dlc = (ctn - 5) / 2;

    // extract checksum
    ctn += 1; // discard '&'
    int chk = 0;
    sscanf((const char*)&buf[ctn], "%02x%n", &chk, &n);
    if (n != 2) {
        ROS_ERROR("CAN: read: error reading checksum, skipping frame");
        return false;
    }// end if

    checksum += chk;
    if ((checksum & 0xff) != 0) {
        ROS_ERROR("CAN: read: invalid checksum, skipping frame");
        return false;
    }

    msg.valid = true;
    return true;
}

bool rota::CAN::write(const CanMessage& msg)
{
    int n;
    uint8_t buf[MSG_MAX_SZ];

    // init checksum
    int checksum = msg.id;

    // frame start indicator
    buf[0] = '%'; n = 1;

    // set id, a 4-hex char sequence
    sprintf((char*)&buf[n], "%04X", msg.id); n+= 4;

    // set data, a 2-hex char sequence for each byte
    const int dlc = std::min(msg.dlc, 8); // clamp to max number of bytes
    for (int i = 0; i < msg.dlc; ++i) {
        checksum += msg.data[i];
        sprintf((char*)&buf[n], "%02X", msg.data[i]);
        n += 2;
    }

    // terminate data sequence with '&'
    buf[n] = '&'; n += 1;

    // write the checksum, a 2-hex char sequence
    sprintf((char*)&buf[n], "%02X", (~checksum + 1) & 0xff); n+= 2;

    // terminate the frame
    buf[n] = '#'; n += 1;

    auto ret = comm.write(buf, n);
    if (ret != n) {
        ROS_ERROR("CAN: write: failed to write frame , skipping frame");
        return false;
    }

    return true;
}


rota::CAN::~CAN()
{
    comm.close();
}
