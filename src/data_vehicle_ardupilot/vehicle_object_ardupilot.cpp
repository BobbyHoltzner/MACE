#include "vehicle_object_ardupilot.h"

namespace DataARDUPILOT {

VehicleObject_ARDUPILOT::VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp):
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_ARDUPILOTTOMACE(vehicleID),Container_MACETOARDUPILOT(systemID,systemComp)
{
    data = new DataContainer_ARDUPILOT();
    parser = new ARDUPILOTParser(data);
}

bool VehicleObject_ARDUPILOT::generateBasicGuidedMessage(const std::shared_ptr<MissionItem::AbstractMissionItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg)
{
    switch(missionItem->getMissionType())
    {
    case(MissionItem::MissionItemType::WAYPOINT):
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
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

}//end of namespace DataARDUPILOT
