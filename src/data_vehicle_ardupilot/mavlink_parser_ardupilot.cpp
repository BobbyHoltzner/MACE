#include "mavlink_parser_ardupilot.h"

namespace DataVehicleArdupilot
{

mavlink_message_t MAVLINKParserArduPilot::generateArdupilotMessage(MissionItem::AbstractMissionItem &missionItem, const uint8_t &chan)
{

    mavlink_message_t msg;

    switch (missionItem.getMissionType()) {
    case MissionItem::MissionItemType::ARM:
    {
        MissionItem::ActionArm& item = dynamic_cast<MissionItem::ActionArm &>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,1,0,400,0,item.getRequestArm(),0,0,0,0,0,0);
        break;
    }
    case MissionItem::MissionItemType::CHANGE_MODE:
    {
        MissionItem::ActionChangeMode& item = dynamic_cast<MissionItem::ActionChangeMode &>(missionItem);
        int newMode = m_CurrentArduVehicleState->getFlightMode(item.getRequestMode());
        mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,1,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
        break;
    }
    default:
        break;
    }

    return msg;
}

}
