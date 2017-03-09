#include "mace_to_ardupilot.h"

namespace DataArdupilot {

mavlink_message_t MACEMissionToMAVLINKMission(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    UNUSED(chan);
    UNUSED(itemIndex);

    mavlink_message_t msg;
    mavlink_mission_item_t item;
    item.autocontinue = 1;
    item.command = 0;
    item.current = 0;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    item.param1 = 0.0;
    item.param2 = 0.0;
    item.param3 = 0.0;
    item.param4 = 0.0;
    item.seq = 0;
    item.target_system = missionItem->getVehicleID();
    item.target_component = compID;
    item.x = 0.0;
    item.y = 0.0;
    item.z = 0.0;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::WAYPOINT:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            item.command = MAV_CMD_NAV_WAYPOINT;
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;
        }else if(missionItem->getPositionalFrame() == Data::PositionalFrame::LOCAL){
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateLocalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateLocalPosition>>(missionItem);
            item.frame = MAV_FRAME_LOCAL_ENU;
            item.command = MAV_CMD_NAV_WAYPOINT;
            item.x = castItem->position.x;
            item.y = castItem->position.y;
            item.z = castItem->position.z;
        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_UNLIMITED:
    {
        item.command = MAV_CMD_NAV_LOITER_UNLIM;
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                item.param3 = castItem->radius;
            }else{
                item.param3 = 0-castItem->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_TURNS:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
            item.command = MAV_CMD_NAV_LOITER_TURNS;
            item.param1 = castItem->turns;
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;
            if(castItem->direction == Data::LoiterDirection::CW)
            {
                item.param3 = castItem->radius;
            }else{
                item.param3 = 0-castItem->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_TIME:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
            item.command = MAV_CMD_NAV_LOITER_TIME;
            item.param1 = castItem->duration;
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;

            if(castItem->direction == Data::LoiterDirection::CW)
            {
                item.param3 = castItem->radius;
            }else{
                item.param3 = 0-castItem->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::RTL:
    {
        item.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            item.command = MAV_CMD_NAV_LAND;
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;
        }
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            item.command = MAV_CMD_NAV_TAKEOFF;
            item.x = castItem->position.latitude;
            item.y = castItem->position.longitude;
            item.z = castItem->position.altitude;
        }
        break;
    }
    default:
        break;
    }
    mavlink_msg_mission_item_encode(255,190,&msg,&item);
    return msg;
}

} //end of namespace
