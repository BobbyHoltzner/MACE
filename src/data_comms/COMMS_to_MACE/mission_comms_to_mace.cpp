#include "mission_comms_to_mace.h"

namespace DataCOMMS{

Mission_COMMSTOMACE::Mission_COMMSTOMACE()
{

}

std::shared_ptr<CommandItem::AbstractCommandItem> Mission_COMMSTOMACE::Covert_COMMSTOMACE(const mace_mission_item_t &maceItem)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem;
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);

    switch(missionType)
    {
    case Data::CommandItemType::CI_ACT_CHANGESPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        ChangeSpeed_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        LoiterTime_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        LoiterTurns_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        LoiterUnlimited_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        RTL_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_TAKEOFF:
    {
        CommandItem::SpatialTakeoff missionItem;
        Takeoff_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case Data::CommandItemType::CI_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        Waypoint_COMMSTOMACE(maceItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialWaypoint>(missionItem);
        break;
    }
    default:
    {

    }
    } //end of switch statement
    return newMissionItem;
}


void Mission_COMMSTOMACE::Home_COMMSTOMACE(const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.position.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
    missionItem.position.setX(maceItem.x);
    missionItem.position.setY(maceItem.y);
    missionItem.position.setZ(maceItem.z);
}

void Mission_COMMSTOMACE::ChangeSpeed_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    if(missionType == Data::CommandItemType::CI_ACT_CHANGESPEED){
        missionItem.setOriginatingSystem(maceItem.mission_creator);
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

void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time &missionItem)
{
    updatePosition(maceItem,missionItem.position);

    missionItem.setOriginatingSystem(maceItem.mission_creator);
    missionItem.setTargetSystem(maceItem.mission_system);
    missionItem.duration = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;

}

void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns &missionItem)
{
    updatePosition(maceItem,missionItem.position);

    missionItem.setOriginatingSystem(maceItem.mission_creator);
    missionItem.setTargetSystem(maceItem.mission_system);
    missionItem.turns = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited &missionItem)
{
    updatePosition(maceItem,missionItem.position);

    missionItem.setOriginatingSystem(maceItem.mission_creator);
    missionItem.setTargetSystem(maceItem.mission_system);
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Mission_COMMSTOMACE::RTL_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);

    if(missionType == Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH){
        missionItem.setOriginatingSystem(maceItem.mission_creator);
        missionItem.setTargetSystem(maceItem.mission_system);
    }
}

void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    updatePosition(maceItem,missionItem.position);

}

void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint &missionItem)
{
    Data::CommandItemType missionType = static_cast<Data::CommandItemType>(maceItem.command);
    updatePosition(maceItem,missionItem.position);

}

void Mission_COMMSTOMACE::updatePosition(const mace_mission_item_t &maceItem, DataState::Base3DPosition &pos)
{
    Data::CoordinateFrameType coordinateFrame = static_cast<Data::CoordinateFrameType>(maceItem.frame);

    pos.setCoordinateFrame(coordinateFrame);
    pos.setX(maceItem.x);
    pos.setY(maceItem.y);
    pos.setZ(maceItem.z);
}
} //end of namespace DataCOMMS
