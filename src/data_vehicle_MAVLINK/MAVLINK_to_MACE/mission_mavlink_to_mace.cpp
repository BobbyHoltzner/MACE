#include "mission_mavlink_to_mace.h"

namespace DataMAVLINK{

Mission_MAVLINKTOMACE::Mission_MAVLINKTOMACE(const int &systemID):
    mSystemID(systemID)
{

}

void Mission_MAVLINKTOMACE::Home_MAVLINKTOMACE(const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setOriginatingSystem(mSystemID);
    missionItem.position->setX(mavlinkItem.latitude / pow(10,7));
    missionItem.position->setY(mavlinkItem.longitude / pow(10,7));
    missionItem.position->setZ(mavlinkItem.altitude / pow(10,3));
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
    case MAV_CMD_NAV_LAND:
    {
        CommandItem::SpatialLand missionItem;
        Land_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLand>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        LoiterTime_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        LoiterTurns_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        LoiterUnlimited_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
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
        CommandItem::SpatialTakeoff missionItem;
        Takeoff_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case MAV_CMD_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        Waypoint_MAVLINKTOMACE(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialWaypoint>(missionItem);
        break;
    }
    default:
    {
        break;
    }
    } //end of switch statement

    if(newMissionItem)
    {
        newMissionItem->setOriginatingSystem(mSystemID);
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

void Mission_MAVLINKTOMACE::Land_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand &missionItem)
{
    updatePosition(mavlinkItem,missionItem.position);
}

CommandItem::SpatialLoiter_Time Mission_MAVLINKTOMACE::LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem)
{
    CommandItem::SpatialLoiter_Time rtnItem;
    if(mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME)
    {
        rtnItem.duration = mavlinkItem.param1;
        rtnItem.radius = fabs(mavlinkItem.param3);
        rtnItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
    updatePosition(mavlinkItem,rtnItem.position);
}

void Mission_MAVLINKTOMACE::LoiterTime_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time &missionItem)
{
    updatePosition(mavlinkItem,missionItem.position);
    missionItem.duration = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Mission_MAVLINKTOMACE::LoiterTurns_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns &missionItem)
{
    updatePosition(mavlinkItem,missionItem.position);
    missionItem.turns = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Mission_MAVLINKTOMACE::LoiterUnlimited_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.position.setX(mavlinkItem.x);
        missionItem.position.setY(mavlinkItem.y);
        missionItem.position.setZ(mavlinkItem.z);
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

void Mission_MAVLINKTOMACE::Takeoff_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff &missionItem)
{
    updatePosition(mavlinkItem,missionItem.position);
}

void Mission_MAVLINKTOMACE::Waypoint_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint &missionItem)
{
    updatePosition(mavlinkItem,missionItem.position);
}

void Mission_MAVLINKTOMACE::updatePosition(const mavlink_mission_item_t &mavlinkItem, DataState::Base3DPosition &pos)
{
    Data::CoordinateFrameType frame = static_cast<Data::CoordinateFrameType>(mavlinkItem.frame);
    pos.setCoordinateFrame(frame);
    pos.setPosition3D(mavlinkItem.x,mavlinkItem.y,mavlinkItem.z);
}

} //end of namespace DataMAVLINK
