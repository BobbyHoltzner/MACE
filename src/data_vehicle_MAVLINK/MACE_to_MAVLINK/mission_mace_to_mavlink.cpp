#include "mission_mace_to_mavlink.h"
namespace DataMAVLINK{

Mission_MACETOMAVLINK::Mission_MACETOMAVLINK(const int &systemID, const int &compID):
    mSystemID(systemID),mCompID(compID)
{

}

mavlink_message_t Mission_MACETOMAVLINK::Home_MACETOMAVLINK(const CommandItem::SpatialHome &missionItem, const uint8_t &chan, const uint8_t &compID)
{
    mavlink_message_t msg;
    mavlink_home_position_t homePosition;
    homePosition.latitude = missionItem.position.getX() * pow(10,7);
    homePosition.longitude = missionItem.position.getY() * pow(10,7);
    homePosition.altitude = missionItem.position.getZ() * pow(10,3);
    mavlink_msg_home_position_encode_chan(missionItem.getOriginatingSystem(),0,chan,&msg,&homePosition);
    return msg;
}

void Mission_MACETOMAVLINK::initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission)
{
    mavMission.autocontinue = 1;
    mavMission.command = 0;
    mavMission.current = 0;
    mavMission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mavMission.param1 = 0.0;
    mavMission.param2 = 0.0;
    mavMission.param3 = 0.0;
    mavMission.param4 = 0.0;
    mavMission.seq = 0;
    mavMission.target_system = 0;
    mavMission.target_component = 0;
    mavMission.x = 0.0;
    mavMission.y = 0.0;
    mavMission.z = 0.0;
    //mavMission.mission_type = MAV_MISSION_TYPE_MISSION;
}

mavlink_message_t Mission_MACETOMAVLINK::packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_mission_item_t tmpItem = mavMission;
    mavlink_msg_mission_item_encode_chan(mSystemID,mCompID,chan,&msg,&tmpItem);
    return msg;
}


bool Mission_MACETOMAVLINK::MACEMissionToMAVLINKMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex, mavlink_message_t &msg)
{
    switch(missionItem->getCommandType())
    {
    case(Data::CommandItemType::CI_ACT_CHANGESPEED):
    {
        std::shared_ptr<CommandItem::ActionChangeSpeed> castItem = std::dynamic_pointer_cast<CommandItem::ActionChangeSpeed>(missionItem);
        CommandItem::ActionChangeSpeed baseItem = *castItem.get();
        msg = ChangeSpeed_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LAND):
    {
        std::shared_ptr<CommandItem::SpatialLand> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLand>(missionItem);
        CommandItem::SpatialLand baseItem = *castItem.get();
        msg = Land_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_TIME):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Time>(missionItem);
        CommandItem::SpatialLoiter_Time baseItem = *castItem.get();
        msg = LoiterTime_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_TURNS):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Turns>(missionItem);
        CommandItem::SpatialLoiter_Turns baseItem = *castItem.get();
        msg = LoiterTurns_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_UNLIM):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        CommandItem::SpatialLoiter_Unlimited baseItem = *castItem.get();
        msg = LoiterUnlimited_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH):
    {
        std::shared_ptr<CommandItem::SpatialRTL> castItem = std::dynamic_pointer_cast<CommandItem::SpatialRTL>(missionItem);
        CommandItem::SpatialRTL baseItem = *castItem.get();
        msg = RTL_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_TAKEOFF):
    {
        std::shared_ptr<CommandItem::SpatialTakeoff> castItem = std::dynamic_pointer_cast<CommandItem::SpatialTakeoff>(missionItem);
        CommandItem::SpatialTakeoff baseItem = *castItem.get();
        msg = Takeoff_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_WAYPOINT):
    {
        std::shared_ptr<CommandItem::SpatialWaypoint> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint>(missionItem);
        CommandItem::SpatialWaypoint baseItem = *castItem.get();
        msg = Waypoint_MACETOMAVLINK(baseItem,chan,compID,itemIndex);
        return true;
        break;
    }
    default:
        return false;
    } //end of switch statement
}

mavlink_message_t Mission_MACETOMAVLINK::ChangeSpeed_MACETOMAVLINK(const CommandItem::ActionChangeSpeed &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_DO_CHANGE_SPEED;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    item.param1 = 0.0; //assume the default required is AIRSPEED
    item.param2 = missionItem.getDesiredSpeed();
    if(missionItem.getSpeedFrame() == Data::SpeedFrame::GROUNDSPEED)
    {
        item.param1 = 1.0;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::Land_MACETOMAVLINK(const CommandItem::SpatialLand &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LAND;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    updateMissionPosition(missionItem.position,item);

    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::LoiterTime_MACETOMAVLINK(const CommandItem::SpatialLoiter_Time &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    item.param1 = missionItem.duration;
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::LoiterTurns_MACETOMAVLINK(const CommandItem::SpatialLoiter_Turns &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    item.param1 = missionItem.turns;
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::LoiterUnlimited_MACETOMAVLINK(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::RTL_MACETOMAVLINK(const CommandItem::SpatialRTL &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_message_t Mission_MACETOMAVLINK::Takeoff_MACETOMAVLINK(const CommandItem::SpatialTakeoff &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_TAKEOFF;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    updateMissionPosition(missionItem.position,item);
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

mavlink_mission_item_t Mission_MACETOMAVLINK::Waypoint_MACETOMAVLINK(const CommandItem::SpatialWaypoint &missionItem, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    this->initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.target_component = compID;
    updateMissionPosition(missionItem.position,item);
    return item;
}

mavlink_message_t Mission_MACETOMAVLINK::Waypoint_MACETOMAVLINK(const CommandItem::SpatialWaypoint &missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item = Waypoint_MACETOMAVLINK(missionItem,compID,itemIndex);
    mavlink_message_t msg = this->packMissionItem(item,chan);
    return msg;
}

void Mission_MACETOMAVLINK::updateMissionPosition(const DataState::Base3DPosition &pos, mavlink_mission_item_t &item)
{
    if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT){
        item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    }
    else if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_LOCAL_ENU)
    {
        item.frame = MAV_FRAME_LOCAL_ENU;
    }
    else{
        //KEN FIX THIS
        item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    }

    item.x = pos.getX();
    item.y = pos.getY();
    item.z = pos.getZ();
}


} //end of namespace DataMAVLINK
