#ifndef VEHICLE_OBJECT_MAVLINKS_H
#define VEHICLE_OBJECT_MAVLINKS_H

#include "data_vehicle_MAVLINK/mavlink_parser.h"
#include "data_vehicle_MAVLINK/data_container_mavlink.h"

#include "data_vehicle_MAVLINK/MACE_to_MAVLINK/container_mace_to_mavlink.h"
#include "data_vehicle_MAVLINK/MAVLINK_to_MACE/container_mavlink_to_mace.h"

namespace DataMAVLINK{

class VehicleObject_MAVLINK : public Container_MAVLINKTOMACE, public Container_MACETOMAVLINK
{
public:
    VehicleObject_MAVLINK(const int &vehicleID, const int &systemID, const int &systemComp);

    bool generateBasicGuidedMessage(const std::shared_ptr<CommandItem::AbstractCommandItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg);
    virtual std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetAllTopicData();

public:
    MAVLINKParser *parser;
    DataContainer_MAVLINK *data;
private:
    int m_VehicleID;
    int m_SystemID;
    int m_SystemComp;

};

} //end of namespace DataMAVLINK

#endif // VEHICLE_OBJECT_MAVLINKS_H
