#include "mission_comms_to_mace.h"

namespace DataCOMMS{

Mission_COMMSTOMACE::Mission_COMMSTOMACE()
{

}

std::shared_ptr<MissionItem::AbstractMissionItem> Mission_COMMSTOMACE::Covert_COMMSTOMACE(const mace_mission_item_t &maceItem)
{
    int systemID = maceItem.mission_system;
    std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem;
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    switch(missionType)
    {
    case MissionItemType::MI_ACT_CHANGESPEED:
    {
        MissionItem::ActionChangeSpeed missionItem;
        ChangeSpeed_COMMSTOMACE(systemID,maceItem,missionItem);
        newMissionItem = std::make_shared<MissionItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case MissionItemType::MI_NAV_LOITER_TIME:
    {
        if(coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MissionItemType::MI_NAV_LOITER_TURNS:
    {
        if(coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MissionItemType::MI_NAV_LOITER_UNLIM:
    {
        if(coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MissionItemType::MI_NAV_RETURN_TO_LAUNCH:
    {
        MissionItem::SpatialRTL missionItem;
        RTL_COMMSTOMACE(systemID,maceItem,missionItem);
        newMissionItem = std::make_shared<MissionItem::SpatialRTL>(missionItem);
        break;
    }
    case MissionItemType::MI_NAV_TAKEOFF:
    {
        if(coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialTakeoff<DataState::StateLocalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialTakeoff<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MissionItemType::MI_NAV_WAYPOINT:
    {
        if(coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,maceItem,missionItem);
            missionItem.print();
            newMissionItem = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialWaypoint<DataState::StateLocalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,maceItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    default:
    {

    }
    } //end of switch statement
    return newMissionItem;
}


void Mission_COMMSTOMACE::Home_COMMSTOMACE(const int &vehicleID, const mace_set_home_position_t &maceItem, MissionItem::SpatialHome &missionItem)
{
    missionItem.setVehicleID(vehicleID);
    missionItem.position.latitude = maceItem.latitude / pow(10,7);
    missionItem.position.longitude = maceItem.longitude / pow(10,7);
    missionItem.position.altitude = maceItem.altitude / pow(10,3);
}

void Mission_COMMSTOMACE::ChangeSpeed_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::ActionChangeSpeed &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);

    if(missionType == MissionItemType::MI_ACT_CHANGESPEED){
        missionItem.setVehicleID(vehicleID);
        missionItem.setDesiredSpeed(maceItem.param2);
        if(maceItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_TIME) && (coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.duration = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_TIME) && (coordinateFrame == CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.duration = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_TURNS) && (coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.turns = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_TURNS) && (coordinateFrame == CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.turns = maceItem.param1;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_UNLIM) && (coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_LOITER_UNLIM) && (coordinateFrame == CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::RTL_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialRTL &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);

    if(missionType == MissionItemType::MI_NAV_RETURN_TO_LAUNCH){
        missionItem.setVehicleID(vehicleID);
    }
}

void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_TAKEOFF) && (coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
    }
}
void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_TAKEOFF) && (coordinateFrame == CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
    }
}

void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_WAYPOINT) && (coordinateFrame == CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = maceItem.x;
        missionItem.position.longitude = maceItem.y;
        missionItem.position.altitude = maceItem.z;
    }
}
void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mace_mission_item_t &maceItem, MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem)
{
    MissionItemType missionType = static_cast<MissionItemType>(maceItem.command);
    CoordinateFrameType coordinateFrame = static_cast<CoordinateFrameType>(maceItem.frame);

    if((missionType == MissionItemType::MI_NAV_WAYPOINT) && (coordinateFrame == CoordinateFrameType::CF_LOCAL_ENU)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = maceItem.x;
        missionItem.position.y = maceItem.y;
        missionItem.position.z = maceItem.z;
    }
}

} //end of namespace DataCOMMS
