
#pragma once

#include <serial/serial.h>

#include "can_ids.h"

namespace rota {

struct CanMessage {
    int id;
    int dlc;
    uint8_t data[8];
    bool valid;
};

struct CAN {
    // Format and size of a message
    //
    // '%' :: id :: data :: '&' :: checksum :: '#'
    // ----::----::------::-----::----------::-----
    //  1B :: 4B :: 8x2B ::  1B ::    2B    :: 1B
    // ----::----::------::-----::----------::-----
    //
    // NOTE: 8x2B is the maximum data bytes, hence the '&' separator
    constexpr static int MSG_MAX_SZ = 1 + 4 + 8*2 + 1 + 2 + 1;
    // communication channel
    serial::Serial comm;

    // Open the communication channel with a provided id.
    // The id is the serial number of the FTDI device.
    bool open(const std::string& id);

    bool read(CanMessage& msg);
    bool write(const CanMessage& msg);

    ~CAN();
};

}// namespace rota
