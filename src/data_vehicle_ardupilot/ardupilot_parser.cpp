#include "ardupilot_parser.h"

namespace DataArdupilot {

ARDUPILOTParser::ARDUPILOTParser(const DataContainer_ARDUPILOT &dataContainer) :
    DataMAVLINK::MAVLINKParser(dataContainer), data(dataContainer)
{

}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ARDUPILOTParser::ParseForVehicleData(const mavlink_message_t *message)
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    rtnVector = DataMAVLINK::MAVLINKParser::ParseForVehicleData(message);
    return rtnVector;
}

} //end of namespace DataArdupilot
