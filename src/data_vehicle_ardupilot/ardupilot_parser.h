#ifndef ARDUPILOT_PARSER_H
#define ARDUPILOT_PARSER_H

#include "data_vehicle_mavlink/mavlink_parser.h"

#include "data_container_ardupilot.h"

namespace DataARDUPILOT{

class ARDUPILOTParser : public DataMAVLINK::MAVLINKParser
{
public:
    ARDUPILOTParser(DataContainer_ARDUPILOT* dataContainer);
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message);

private:
    DataContainer_ARDUPILOT* data;
};

} //end of namespace DataArdupilot

#endif // ARDUPILOT_PARSER_H
