#include "mission_mavlink_to_mace.h"

namespace DataMAVLINK{

Mission_MAVLINKTOMACE::Mission_MAVLINKTOMACE(const int &systemID):
    mSystemID(systemID)
{

}

void Mission_MAVLINKTOMACE::Home_MAVLINKTOMACE(const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setGeneratingSystem(mSystemID);
    missionItem.position.latitude = mavlinkItem.latitude / pow(10,7);
    missionItem.position.longitude = mavlinkItem.longitude / pow(10,7);
    missionItem.position.altitude = mavlinkItem.altitude / pow(10,3);
}

std::shared_ptr<CommandItem::AbstractCommandItem> Mission_MAVLINKTOMACE::Covert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem;

    switch(mavlinkItem.command)
    {
    case MAV_CMD_DO_CHANGE_SPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        ChangeSpeed_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TIME:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition> missionItem;
            LoiterTime_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition> missionItem;
            LoiterTime_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> missionItem;
            LoiterTurns_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition> missionItem;
            LoiterTurns_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> missionItem;
            LoiterUnlimited_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> missionItem;
            LoiterUnlimited_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        RTL_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case MAV_CMD_NAV_TAKEOFF:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem;
            Takeoff_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialTakeoff<DataState::StateLocalPosition> missionItem;
            Takeoff_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialTakeoff<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_WAYPOINT:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> missionItem;
            Waypoint_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialWaypoint<DataState::StateLocalPosition> missionItem;
            Waypoint_MAVLINKTOMACE(mavlinkItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    default:
    {
        break;
    }
    } //end of switch statement

    if(newMissionItem)
    {
        newMissionItem->setGeneratingSystem(mSystemID);
    }
    return newMissionItem;
}

void Mission_MAVLINKTOMACE::ChangeSpeed_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem)
{
    if(mavlinkItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setDesiredSpeed(mavlinkItem.param2);
        if(mavlinkItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Mission_MAVLINKTOMACE::LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::LoiterTurns_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterTurns_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::LoiterUnlimited_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_MAVLINKTOMACE::LoiterUnlimited_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_MAVLINKTOMACE::RTL_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem)
{
    UNUSED(missionItem);
    if(mavlinkItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){

    }
}

void Mission_MAVLINKTOMACE::Takeoff_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_MAVLINKTOMACE::Takeoff_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

void Mission_MAVLINKTOMACE::Waypoint_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_MAVLINKTOMACE::Waypoint_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

} //end of namespace DataMAVLINK
