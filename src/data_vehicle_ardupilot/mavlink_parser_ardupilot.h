#ifndef MAVLINK_PARSER_ARDUPILOT_H
#define MAVLINK_PARSER_ARDUPILOT_H

#include "data_vehicle_MAVLINK/mavlink_parser.h"

namespace DataVehicleArduPilot
{


class MAVLINKParserArduPilot
{
public:

    template<typename T>
    MaceCore::TopicDatagram Parse(const mavlink_message_t* message, const T* topic) const{

    }
};

}

#endif // MAVLINK_PARSER_ARDUPILOT_H
