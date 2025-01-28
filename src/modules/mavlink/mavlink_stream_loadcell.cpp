#include "mavlink_stream_loadcell.h"
#include <mavlink.h>
#include <uORB/topics/loadcell_data.h>
#include <uORB/uORB.h>

MavlinkStreamLoadcell::MavlinkStreamLoadcell(Mavlink *mavlink) : MavlinkStream(mavlink) {}

const char *MavlinkStreamLoadcell::get_name_static() {
    return "LOADCELL_DATA";
}

uint16_t MavlinkStreamLoadcell::get_id_static() {
    return MAVLINK_MSG_ID_LOADCELL_DATA;
}

MavlinkStream *MavlinkStreamLoadcell::new_instance(Mavlink *mavlink) {
    return new MavlinkStreamLoadcell(mavlink);
}

unsigned MavlinkStreamLoadcell::get_size() {
    return sizeof(mavlink_loadcell_data_t);
}

void MavlinkStreamLoadcell::send() {
    struct loadcell_data_s loadcell_data;

    // Pull the data from uORB (Make sure your driver is publishing to this topic)
    if (_loadcell_sub.copy(&loadcell_data)) {
        // Pack the data into a MAVLink message
        mavlink_message_t msg;
        mavlink_msg_loadcell_data_pack(_mavlink->get_system_id(),
                                       _mavlink->get_component_id(),
                                       &msg,
                                       loadcell_data.timestamp,
                                       loadcell_data.axis_data);

        // Send the message
        _mavlink->send_message(&msg);
    }
}

MavlinkStreamLoadcell::MavlinkStreamLoadcell(Mavlink *mavlink) : MavlinkStream(mavlink) {
    _loadcell_sub = orb_subscribe(ORB_ID(loadcell_data));
}

MavlinkStreamLoadcell::~MavlinkStreamLoadcell() {
    orb_unsubscribe(_loadcell_sub);
}
