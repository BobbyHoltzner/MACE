#include "mission_comms_to_mace.h"

namespace DataCOMMS{

Mission_COMMSTOMACE::Mission_COMMSTOMACE()
{

}

std::shared_ptr<CommandItem::AbstractCommandItem> Mission_COMMSTOMACE::Covert_COMMSTOMACE(const mace_mission_item_t &maceItem)
{
    int systemID = maceItem.mission_system;
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem;
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    switch(missionType)
    {
    case Data::CommandItemType::CI_ACT_CHANGESPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        ChangeSpeed_COMMSTOMACE(systemID,maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_TIME:
    {
        if(coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_TURNS:
    {
        if(coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_UNLIM:
    {
        if(coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        RTL_COMMSTOMACE(systemID,maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_TAKEOFF:
    {
        if(coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialTakeoff<DataState::StateLocalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialTakeoff<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case Data::CommandItemType::CI_NAV_WAYPOINT:
    {
        if(coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
        }else{
            CommandItem::SpatialWaypoint<DataState::StateLocalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<CommandItem::SpatialWaypoint<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    default:
    {

    }
    } //end of switch statement
    return newMissionItem;
}


void Mission_COMMSTOMACE::Home_COMMSTOMACE(const int &vehicleID, const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setGeneratingSystem(vehicleID);
    missionItem.position.latitude = maceItem.latitude / pow(10,7);
    missionItem.position.longitude = maceItem.longitude / pow(10,7);
    missionItem.position.altitude = maceItem.altitude / pow(10,3);
}

void Mission_COMMSTOMACE::ChangeSpeed_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    if(missionType == Data::CommandItemType::CI_ACT_CHANGESPEED){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.setDesiredSpeed(maceItem.param2);
        if(maceItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_TIME) && (coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.duration = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_TIME) && (coordinateFrame == Data::CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.duration = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_TURNS) && (coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.turns = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_TURNS) && (coordinateFrame == Data::CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.turns = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_UNLIM) && (coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_LOITER_UNLIM) && (coordinateFrame == Data::CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::RTL_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);

    if(missionType == Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
    }
}

void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_TAKEOFF) && (coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
    }
}
void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_TAKEOFF) && (coordinateFrame == Data::CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
    }
}

void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_WAYPOINT) && (coordinateFrame == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
    }
}
void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    if((missionType == Data::CommandItemType::CI_NAV_WAYPOINT) && (coordinateFrame == Data::CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setGeneratingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
    }
}

} //end of namespace DataCOMMS
