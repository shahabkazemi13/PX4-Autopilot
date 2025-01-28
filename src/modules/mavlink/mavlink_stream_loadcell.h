#pragma once

#include "mavlink_stream.h"
#include <uORB/topics/loadcell_data.h>
#include <uORB/uORB.h>

class MavlinkStreamLoadcell : public MavlinkStream {
public:
    MavlinkStreamLoadcell(Mavlink *mavlink);

    static const char *get_name_static();
    static uint16_t get_id_static();
    static MavlinkStream *new_instance(Mavlink *mavlink);

    unsigned get_size();
    void send();

private:
    int _loadcell_sub;
};
