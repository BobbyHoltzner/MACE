#include "vehicle_object_ardupilot.h"

namespace DataARDUPILOT {

VehicleObject_ARDUPILOT::VehicleObject_ARDUPILOT(const int &vehicleID, const int &systemID, const int &systemComp):
    m_VehicleID(vehicleID), m_SystemID(systemID), m_SystemComp(systemComp),
    Container_ARDUPILOTTOMACE(vehicleID),Container_MACETOARDUPILOT(systemID,systemComp)
{
    data = new DataContainer_ARDUPILOT();
    parser = new ARDUPILOTParser(data);
}

bool VehicleObject_ARDUPILOT::generateBasicGuidedMessage(const std::shared_ptr<CommandItem::AbstractCommandItem> &missionItem, const uint8_t &chan, mavlink_message_t &msg)
{
    switch(missionItem->getCommandType())
    {
    case(Data::CommandItemType::CI_NAV_WAYPOINT):
    {
        if(missionItem->getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            std::shared_ptr<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> baseItem = *castItem.get();
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
