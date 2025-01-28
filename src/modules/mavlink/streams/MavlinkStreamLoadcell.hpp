#pragma once

#include "mavlink_stream.h"
#include <uORB/topics/loadcell_data.h>
#include <uORB/uORB.h>

class MavlinkStreamLoadcell : public MavlinkStream {
public:
    MavlinkStreamLoadcell(Mavlink *mavlink);

    // Static methods to provide stream name and ID
    static const char *get_name_static();
    static uint16_t get_id_static();
    static MavlinkStream *new_instance(Mavlink *mavlink);

    // Method to get the message size
    unsigned get_size();

    // Method to send the MAVLink message
    void send();

private:
    // uORB subscription for loadcell data
    int _loadcell_sub;
};
