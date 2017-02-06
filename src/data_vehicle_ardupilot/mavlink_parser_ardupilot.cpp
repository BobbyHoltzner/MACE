#include "mavlink_parser_ardupilot.h"

namespace DataVehicleArdupilot
{

mavlink_message_t MAVLINKParserArduPilot::generateArdupilotMessage(MissionItem::AbstractMissionItem* missionItem, const uint8_t &chan)
{

    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::ARM:
    {
        MissionItem::ActionArm* item = dynamic_cast<MissionItem::ActionArm*>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,400,0,item->getRequestArm(),0,0,0,0,0,0);
        break;
    }
    case MissionItem::MissionItemType::CHANGE_MODE:
    {
        MissionItem::ActionChangeMode* item = dynamic_cast<MissionItem::ActionChangeMode*>(missionItem);
        if(heartbeatSeen == true){
            int newMode = m_CurrentArduVehicleState->getFlightMode(item->getRequestMode());
            mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,item->getVehicleID(),MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
        }
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            MissionItem::SpatialLand<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>*>(missionItem);
            if(item->getLandFlag() == true)
            {
                mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,21,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
            }else{
                mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,21,0,0.0,0.0,0.0,0.0,item->position.latitude,item->position.longitude,item->position.altitude);
            }
        }else{
            MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
        }

        break;
    }
    case MissionItem::MissionItemType::RTL:
    {
        //This is command number 20
        MissionItem::SpatialRTL* item = dynamic_cast<MissionItem::SpatialRTL*>(missionItem);
        mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,20,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>*>(missionItem);
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,22,0,0.0,2.0,0.0,0.0,item->position.latitude,item->position.longitude,item->position.altitude);
        }else{
            MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
        }
        break;
    }
    case MissionItem::MissionItemType::WAYPOINT:
    {
        //This is command number 16
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* item = dynamic_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>*>(missionItem);
            mavlink_msg_command_long_pack_chan(255,190,chan,&msg,item->getVehicleID(),0,16,0,0.0,0.0,0.0,0.0,item->position.latitude,item->position.longitude,item->position.altitude);
        }else{
            MissionItem::SpatialLand<DataState::StateLocalPosition>* item = dynamic_cast<MissionItem::SpatialLand<DataState::StateLocalPosition>*>(missionItem);
        }
        break;
    }
    default:
        break;
    }

    return msg;
}

}
