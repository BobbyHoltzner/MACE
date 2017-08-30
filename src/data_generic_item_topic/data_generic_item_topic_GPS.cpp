#include "data_generic_item_topic_GPS.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicGPS_name[] = "gpsStatus";
const MaceCore::TopicComponentStructure DataGenericItemTopicGPS_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<DataGenericItemTopic_GPS::GPSFixType>("fix");
    structure.AddTerminal<uint16_t>("satellitesVisible");
    structure.AddTerminal<uint16_t>("vdop");
    structure.AddTerminal<uint16_t>("hdop");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_GPS::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GPSFixType>("fix", fixtype);
    datagram.AddTerminal<uint16_t>("satellitesVisible", satellitesVisible);
    datagram.AddTerminal<uint16_t>("vdop", VDOP);
    datagram.AddTerminal<uint16_t>("hdop", HDOP);
    return datagram;
}

void DataGenericItemTopic_GPS::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    fixtype = datagram.GetTerminal<GPSFixType>("fix");
    satellitesVisible = datagram.GetTerminal<uint16_t>("satellitesVisible");
    VDOP = datagram.GetTerminal<uint16_t>("vdop");
    HDOP = datagram.GetTerminal<uint16_t>("hdop");
}

DataGenericItemTopic_GPS::DataGenericItemTopic_GPS()
    :DataGenericItem::DataGenericItem_GPS()
{

}

DataGenericItemTopic_GPS::DataGenericItemTopic_GPS(const DataGenericItem::DataGenericItem_GPS &copyObj):
    DataGenericItem::DataGenericItem_GPS(copyObj)
{

}

} //end of namespace DataGenericItemTopic
