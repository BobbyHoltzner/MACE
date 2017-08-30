#include "helper_mission_mavlink_to_mace.h"

namespace DataMAVLINK {

Helper_MissionMAVLINKtoMACE::Helper_MissionMAVLINKtoMACE(const int &originatingID):
    systemID(originatingID)
{

}

Helper_MissionMAVLINKtoMACE::~Helper_MissionMAVLINKtoMACE()
{

}

std::shared_ptr<CommandItem::AbstractCommandItem> Helper_MissionMAVLINKtoMACE::Convert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = NULL;

    switch(mavlinkItem.command)
    {
    case MAV_CMD_DO_CHANGE_SPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        convertChangespeed(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LAND:
    {
        CommandItem::SpatialLand missionItem;
        convertLand(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLand>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        convertLoiterTime(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        convertLoiterTurns(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        convertLoiterUnlimted(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        break;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        convertRTL(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case MAV_CMD_NAV_TAKEOFF:
    {
        CommandItem::SpatialTakeoff missionItem;
        convertTakeoff(mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case MAV_CMD_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        convertWaypoint(mavlinkItem,missionItem);
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
        newMissionItem->setOriginatingSystem(systemID);
        newMissionItem->setTargetSystem(systemID);
    }
    return newMissionItem;
}


void Helper_MissionMAVLINKtoMACE::convertHome(const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.position->setX(mavlinkItem.latitude / pow(10,7));
    missionItem.position->setY(mavlinkItem.longitude / pow(10,7));
    missionItem.position->setZ(mavlinkItem.altitude / pow(10,3));
}

void Helper_MissionMAVLINKtoMACE::convertChangespeed(const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
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

void Helper_MissionMAVLINKtoMACE::convertLand(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

void Helper_MissionMAVLINKtoMACE::convertLoiterTime(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.duration = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertLoiterTurns(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.turns = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertLoiterUnlimted(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertRTL(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    UNUSED(missionItem);
    if(mavlinkItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){

    }
}

void Helper_MissionMAVLINKtoMACE::convertTakeoff(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

void Helper_MissionMAVLINKtoMACE::convertWaypoint(const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

DataState::Base3DPosition Helper_MissionMAVLINKtoMACE::getBasePosition(const mavlink_mission_item_t &mavlinkItem)
{
    DataState::Base3DPosition pos;
    Data::CoordinateFrameType frame = static_cast<Data::CoordinateFrameType>(mavlinkItem.frame);
    pos.setCoordinateFrame(frame);
    pos.setPosition3D(mavlinkItem.x,mavlinkItem.y,mavlinkItem.z);
    return pos;
}

} //end of namespace DataMAVLINK
