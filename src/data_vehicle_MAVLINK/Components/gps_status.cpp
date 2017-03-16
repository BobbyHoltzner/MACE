#include "gps_status.h"

namespace DataMAVLINK
{

const char GPSStatus_name[] = "global_position";
const MaceCore::TopicComponentStructure GPSStatus_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("fixStatus");
    structure.AddTerminal<int>("numberOfSats");
    structure.AddTerminal<int>("horizontalDOP");
    structure.AddTerminal<int>("verticalDOP");

    return structure;
}();


MaceCore::TopicDatagram GPSStatus::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("fixStatus", fixStatus);
    datagram.AddTerminal<int>("numberOfSats", numberOfSats);
    datagram.AddTerminal<int>("horizontalDOP", horizontalDOP);
    datagram.AddTerminal<int>("verticalDOP", verticalDOP);

    return datagram;
}


void GPSStatus::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    fixStatus = datagram.GetTerminal<int>("fixStatus");
    numberOfSats = datagram.GetTerminal<int>("numberOfSats");
    horizontalDOP = datagram.GetTerminal<int>("horizontalDOP");
    verticalDOP = datagram.GetTerminal<int>("verticalDOP");
}

}
