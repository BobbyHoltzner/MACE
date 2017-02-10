#include "data_vehicle_generic_topic_GPS.h"

namespace DataVehicleGenericTopic {

const char GenericVehicleTopicGPS_name[] = "gpsStatus";
const MaceCore::TopicComponentStructure GenericVehicleTopicGPS_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<DataVehicleGenericTopic_GPS::GPSFIX>("fix");
    structure.AddTerminal<uint16_t>("satellitesVisible");
    structure.AddTerminal<uint16_t>("vdop");
    structure.AddTerminal<uint16_t>("hdop");
    return structure;
}();


MaceCore::TopicDatagram DataVehicleGenericTopic_GPS::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GPSFIX>("fix", fixtype);
    datagram.AddTerminal<uint16_t>("satellitesVisible", satellitesVisible);
    datagram.AddTerminal<uint16_t>("vdop", VDOP);
    datagram.AddTerminal<uint16_t>("hdop", HDOP);
    return datagram;
}

void DataVehicleGenericTopic_GPS::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    fixtype = datagram.GetTerminal<GPSFIX>("fix");
    satellitesVisible = datagram.GetTerminal<uint16_t>("satellitesVisible");
    VDOP = datagram.GetTerminal<uint16_t>("vdop");
    HDOP = datagram.GetTerminal<uint16_t>("hdop");
}

} //end of namespace DataVehicleGenericTopic
