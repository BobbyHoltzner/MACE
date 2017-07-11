#include "mission_mace_to_comms.h"
namespace DataCOMMS{

Mission_MACETOCOMMS::Mission_MACETOCOMMS(const int &systemFrom, const int &systemTo, const Data::MissionKey &missionKey, const uint8_t &chan):
    fromID(systemFrom),toID(systemTo), key(missionKey), commsChan(chan)
{

}

Mission_MACETOCOMMS::Mission_MACETOCOMMS(const int &systemFrom, const uint8_t &chan):
    fromID(systemFrom),toID(0),key(Data::MissionKey(systemFrom,systemFrom,0,Data::MissionType::ALL)),commsChan(chan)
{

}

mace_message_t Mission_MACETOCOMMS::Home_MACETOCOMMS(const CommandItem::SpatialHome &missionItem)
{
    mace_message_t msg;
    mace_home_position_t homePosition;
    homePosition.latitude = missionItem.position.getX() * pow(10,7);
    homePosition.longitude = missionItem.position.getY() * pow(10,7);
    homePosition.altitude = missionItem.position.getZ() * pow(10,3);
    mace_msg_home_position_encode_chan(fromID,0,commsChan,&msg,&homePosition);
    return msg;
}

void Mission_MACETOCOMMS::initializeMACECOMMSMissionItem(mace_mission_item_t &mavMission)
{
    mavMission.seq = 0;
    mavMission.command = 0;
    mavMission.current = 0;
    mavMission.frame = (uint8_t)Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
    mavMission.autocontinue = 1;
    mavMission.param1 = 0.0;
    mavMission.param2 = 0.0;
    mavMission.param3 = 0.0;
    mavMission.param4 = 0.0;
    mavMission.x = 0.0;
    mavMission.y = 0.0;
    mavMission.z = 0.0;

    //these items help the where is it going and who is it about MissionKey
    mavMission.mission_system = key.m_systemID;
    mavMission.mission_creator = key.m_creatorID;
    mavMission.mission_id = key.m_missionID;
    mavMission.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
    //this item helps identify where the mission item needs to go if it is outbound
    mavMission.target_system = toID;
}

mace_message_t Mission_MACETOCOMMS::packMissionItem(const mace_mission_item_t &mavMission)
{
    mace_message_t msg;
    mace_mission_item_t tmpItem = mavMission;
    mace_msg_mission_item_encode_chan(fromID,0,commsChan,&msg,&tmpItem);
    return msg;
}

bool Mission_MACETOCOMMS::MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mace_message_t &msg)
{
    switch(missionItem->getCommandType())
    {
    case(Data::CommandItemType::CI_ACT_CHANGESPEED):
    {
        std::shared_ptr<CommandItem::ActionChangeSpeed> castItem = std::dynamic_pointer_cast<CommandItem::ActionChangeSpeed>(missionItem);
        CommandItem::ActionChangeSpeed baseItem = *castItem.get();
        msg = ChangeSpeed_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LAND):
    {
        std::shared_ptr<CommandItem::SpatialLand> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLand>(missionItem);
        CommandItem::SpatialLand baseItem = *castItem.get();
        msg = Land_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_TIME):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Time>(missionItem);
        CommandItem::SpatialLoiter_Time baseItem = *castItem.get();
        msg = LoiterTime_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_TURNS):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Turns>(missionItem);
        CommandItem::SpatialLoiter_Turns baseItem = *castItem.get();
        msg = LoiterTurns_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_LOITER_UNLIM):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        CommandItem::SpatialLoiter_Unlimited baseItem = *castItem.get();
        msg = LoiterUnlimited_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH):
    {
        std::shared_ptr<CommandItem::SpatialRTL> castItem = std::dynamic_pointer_cast<CommandItem::SpatialRTL>(missionItem);
        CommandItem::SpatialRTL baseItem = *castItem.get();
        msg = RTL_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_TAKEOFF):
    {
        std::shared_ptr<CommandItem::SpatialTakeoff> castItem = std::dynamic_pointer_cast<CommandItem::SpatialTakeoff>(missionItem);
        CommandItem::SpatialTakeoff baseItem = *castItem.get();
        msg = Takeoff_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    case(Data::CommandItemType::CI_NAV_WAYPOINT):
    {
        std::shared_ptr<CommandItem::SpatialWaypoint> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint>(missionItem);
        CommandItem::SpatialWaypoint baseItem = *castItem.get();
        msg = Waypoint_MACETOCOMMS(baseItem,itemIndex);
        return true;
        break;
    }
    default:
        return false;
    } //end of switch statement
}

mace_message_t Mission_MACETOCOMMS::ChangeSpeed_MACETOCOMMS(const CommandItem::ActionChangeSpeed &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_ACT_CHANGESPEED;
    item.seq = itemIndex;
    item.param1 = 0.0; //assume the default required is AIRSPEED
    item.param2 = missionItem.getDesiredSpeed();
    if(missionItem.getSpeedFrame() == Data::SpeedFrame::GROUNDSPEED)
    {
        item.param1 = 1.0;
    }
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::Land_MACETOCOMMS(const CommandItem::SpatialLand &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_LAND;
    item.seq = itemIndex;
    if(!missionItem.position.has2DPositionSet())
    {
        updatePosition(missionItem.position,item);
    }
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::LoiterTime_MACETOCOMMS(const CommandItem::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.param1 = missionItem.duration;
    updatePosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::LoiterTurns_MACETOCOMMS(const CommandItem::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.param1 = missionItem.turns;
    updatePosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::LoiterUnlimited_MACETOCOMMS(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    updatePosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::RTL_MACETOCOMMS(const CommandItem::SpatialRTL &missionItem, const uint16_t &itemIndex)
{
    UNUSED(missionItem);
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH;
    item.seq = itemIndex;
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::Takeoff_MACETOCOMMS(const CommandItem::SpatialTakeoff &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_TAKEOFF;
    item.seq = itemIndex;
    updatePosition(missionItem.position,item);
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

mace_message_t Mission_MACETOCOMMS::Waypoint_MACETOCOMMS(const CommandItem::SpatialWaypoint &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    this->initializeMACECOMMSMissionItem(item);
    item.command = (uint16_t)Data::CommandItemType::CI_NAV_WAYPOINT;
    item.seq = itemIndex;
    updatePosition(missionItem.position,item);
    mace_message_t msg = this->packMissionItem(item);
    return msg;
}

void Mission_MACETOCOMMS::updatePosition(const DataState::Base3DPosition &pos, mace_mission_item_t &maceItem)
{
    maceItem.frame = (uint8_t)pos.getCoordinateFrame();
    maceItem.x = pos.getX();
    maceItem.y = pos.getY();
    maceItem.z = pos.getZ();
}
} //end of namespace DataCOMMS
