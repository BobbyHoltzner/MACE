#include "vehicle_object_mavlink.h"

namespace DataMAVLINK{

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &systemID, const int &systemComp) :
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_MAVLINKTOMACE(vehicleID),Container_MACETOMAVLINK(systemID,systemComp)
{
    data = new DataContainer_MAVLINK();
    parser = new MAVLINKParser(data);
}

bool VehicleObject_MAVLINK::generateBasicGuidedMessage(const std::shared_ptr<MissionItem::AbstractMissionItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg)
{
    switch(missionItem->getMissionType())
    {
    case(Data::MissionItemType::MI_NAV_WAYPOINT):
    {
        if(missionItem->getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> baseItem = *castItem.get();
            mavlink_mission_item_t newItem = Waypoint_MACETOMAVLINK(baseItem,0,0);
            newItem.current = 2;
            msg = packMissionItem(newItem,chan);
            return true;
        }else{
            return false;
        }
        break;
    }
    default:
        return false;
    } //end of switch statement
}

} //end of namespace DataMAVLINK
