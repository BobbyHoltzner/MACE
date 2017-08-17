#include "helper_mission_comms_to_mace.h"

namespace DataInterface_MACE {

Helper_MissionCOMMStoMACE::Helper_MissionCOMMStoMACE()

{

}

Helper_MissionCOMMStoMACE::~Helper_MissionCOMMStoMACE()
{

}

void Helper_MissionCOMMStoMACE::updateIDS(const int &originatingID)
{
    this->systemID = originatingID;
}

std::shared_ptr<CommandItem::AbstractCommandItem> Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(const mace_mission_item_t &maceItem)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = NULL;

    switch(static_cast<COMMANDITEM>(maceItem.command))
    {
    case COMMANDITEM::CI_ACT_CHANGESPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        convertChangespeed(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LAND:
    {
        CommandItem::SpatialLand missionItem;
        convertLand(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLand>(missionItem);
        break;
    }        
    case COMMANDITEM::CI_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        convertLoiterTime(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        convertLoiterTurns(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        convertLoiterUnlimted(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        convertRTL(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_TAKEOFF:
    {
        CommandItem::SpatialTakeoff missionItem;
        convertTakeoff(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        convertWaypoint(maceItem,missionItem);
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


void Helper_MissionCOMMStoMACE::convertHome(const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.position->setX(maceItem.latitude / pow(10,7));
    missionItem.position->setY(maceItem.longitude / pow(10,7));
    missionItem.position->setZ(maceItem.altitude / pow(10,3));
}

void Helper_MissionCOMMStoMACE::convertChangespeed(const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    if(maceItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setDesiredSpeed(maceItem.param2);
        if(maceItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Helper_MissionCOMMStoMACE::convertLand(const mace_mission_item_t &maceItem, CommandItem::SpatialLand &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(maceItem));
}

void Helper_MissionCOMMStoMACE::convertLoiterTime(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(maceItem));
    missionItem.duration = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionCOMMStoMACE::convertLoiterTurns(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(maceItem));
    missionItem.turns = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionCOMMStoMACE::convertLoiterUnlimted(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    if((maceItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (maceItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setPosition(getBasePosition(maceItem));
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Helper_MissionCOMMStoMACE::convertRTL(const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    UNUSED(missionItem);
    if(maceItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){

    }
}

void Helper_MissionCOMMStoMACE::convertTakeoff(const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(maceItem));
}

void Helper_MissionCOMMStoMACE::convertWaypoint(const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint &missionItem)
{
    missionItem.setTargetSystem(systemID);
    missionItem.setOriginatingSystem(systemID);
    missionItem.setPosition(getBasePosition(maceItem));
}

DataState::Base3DPosition Helper_MissionCOMMStoMACE::getBasePosition(const mace_mission_item_t &maceItem)
{
    DataState::Base3DPosition pos;
    Data::CoordinateFrameType frame = static_cast<Data::CoordinateFrameType>(maceItem.frame);
    pos.setCoordinateFrame(frame);
    pos.setPosition3D(maceItem.x,maceItem.y,maceItem.z);
    return pos;
}

} //end of namespace DataMAVLINK
