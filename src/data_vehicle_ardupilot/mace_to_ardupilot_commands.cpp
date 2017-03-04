#include "mace_to_ardupilot.h"

namespace DataArdupilot {
mavlink_message_t generateArdupilotCommandMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmdLong;

    cmdLong.command = 0;
    cmdLong.confirmation = 0;
    cmdLong.param1 = 0.0;
    cmdLong.param2 = 0.0;
    cmdLong.param3 = 0.0;
    cmdLong.param4 = 0.0;
    cmdLong.param5 = 0.0;
    cmdLong.param6 = 0.0;
    cmdLong.param7 = 0.0;
    cmdLong.target_system = missionItem->getVehicleID();
    cmdLong.target_component = compID;

    switch (missionItem->getMissionType()) {

    case MissionItem::MissionItemType::WAYPOINT:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_WAYPOINT;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;
        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_UNLIMITED:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_LOITER_UNLIM;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;

            if(item->direction == Data::LoiterDirection::CW)
            {
                cmdLong.param3 = item->radius;
            }else{
                cmdLong.param3 = 0-item->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_TURNS:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_LOITER_TURNS;
            cmdLong.param1 = item->turns;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;
            if(item->direction == Data::LoiterDirection::CW)
            {
                cmdLong.param3 = item->radius;
            }else{
                cmdLong.param3 = 0-item->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::LOITER_TIME:
    {
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_LOITER_TIME;
            cmdLong.param1 = item->duration;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;
            float radius = 0.0;

            if(item->direction == Data::LoiterDirection::CW)
            {
                cmdLong.param3 = item->radius;
            }else{
                cmdLong.param3 = 0-item->radius;
            }
        }else{

        }
        break;
    }
    case MissionItem::MissionItemType::RTL:
    {
        cmdLong.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_LAND;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;
        }
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            cmdLong.command = MAV_CMD_NAV_TAKEOFF;
            cmdLong.param5 = item->position.latitude;
            cmdLong.param6 = item->position.longitude;
            cmdLong.param7 = item->position.altitude;
        }
        break;
    }
    default:
        break;
    }

    mavlink_msg_command_long_encode_chan(255,190,chan,&msg,&cmdLong);
    return msg;
}

}
