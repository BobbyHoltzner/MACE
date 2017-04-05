#include "mission_comms_to_mace.h"

namespace DataCOMMS{

Mission_COMMSTOMACE::Mission_COMMSTOMACE()
{

}

std::shared_ptr<MissionItem::AbstractMissionItem> Mission_COMMSTOMACE::Covert_COMMSTOMACE(const mavlink_mission_item_t &mavlinkItem)
{
    int systemID = mavlinkItem.target_system;
    std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem;

    switch(mavlinkItem.command)
    {
    case MAV_CMD_DO_CHANGE_SPEED:
    {
        MissionItem::ActionChangeSpeed missionItem;
        ChangeSpeed_COMMSTOMACE(systemID,mavlinkItem,missionItem);
        newMissionItem = std::make_shared<MissionItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TIME:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> missionItem;
            LoiterTime_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> missionItem;
            LoiterTurns_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> missionItem;
            LoiterUnlimited_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        MissionItem::SpatialRTL missionItem;
        RTL_COMMSTOMACE(systemID,mavlinkItem,missionItem);
        newMissionItem = std::make_shared<MissionItem::SpatialRTL>(missionItem);
        break;
    }
    case MAV_CMD_NAV_TAKEOFF:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialTakeoff<DataState::StateLocalPosition> missionItem;
            Takeoff_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            newMissionItem = std::make_shared<MissionItem::SpatialTakeoff<DataState::StateLocalPosition>>(missionItem);
        }
        break;
    }
    case MAV_CMD_NAV_WAYPOINT:
    {
        if(mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,mavlinkItem,missionItem);
            missionItem.print();
            newMissionItem = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
        }else{
            MissionItem::SpatialWaypoint<DataState::StateLocalPosition> missionItem;
            Waypoint_COMMSTOMACE(systemID,mavlinkItem,missionItem);
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


void Mission_COMMSTOMACE::Home_COMMSTOMACE(const int &vehicleID, const mavlink_set_home_position_t &mavlinkItem, MissionItem::SpatialHome &missionItem)
{
    missionItem.setVehicleID(vehicleID);
    missionItem.position.latitude = mavlinkItem.latitude / pow(10,7);
    missionItem.position.longitude = mavlinkItem.longitude / pow(10,7);
    missionItem.position.altitude = mavlinkItem.altitude / pow(10,3);
}

void Mission_COMMSTOMACE::ChangeSpeed_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::ActionChangeSpeed &missionItem)
{
    if(mavlinkItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setVehicleID(vehicleID);
        missionItem.setDesiredSpeed(mavlinkItem.param2);
        if(mavlinkItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTime_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TIME) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.duration = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterTurns_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_TURNS) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.turns = mavlinkItem.param1;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}
void Mission_COMMSTOMACE::LoiterUnlimited_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
        missionItem.radius = fabs(mavlinkItem.param3);
        missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Mission_COMMSTOMACE::RTL_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialRTL &missionItem)
{
    if(mavlinkItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){
        missionItem.setVehicleID(vehicleID);
    }
}

void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_COMMSTOMACE::Takeoff_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialTakeoff<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_TAKEOFF) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateGlobalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.latitude = mavlinkItem.x;
        missionItem.position.longitude = mavlinkItem.y;
        missionItem.position.altitude = mavlinkItem.z;
    }
}
void Mission_COMMSTOMACE::Waypoint_COMMSTOMACE(const int &vehicleID, const mavlink_mission_item_t &mavlinkItem, MissionItem::SpatialWaypoint<DataState::StateLocalPosition> &missionItem)
{
    if((mavlinkItem.command == MAV_CMD_NAV_WAYPOINT) && (mavlinkItem.frame == MAV_FRAME_LOCAL_NED)){
        missionItem.setVehicleID(vehicleID);
        missionItem.position.x = mavlinkItem.x;
        missionItem.position.y = mavlinkItem.y;
        missionItem.position.z = mavlinkItem.z;
    }
}

} //end of namespace DataCOMMS
