#include "ardupilot_to_mace_mission.h"

namespace DataVehicleArdupilot
{

mavlink_message_t ArdupilotToMACEMission::generateArdupilotMissionMessage(std::shared_ptr<MissionItem::AbstractMissionItem> missionItem, const uint8_t &chan, const uint8_t &compID, const uint16_t &itemIndex)
{

    mavlink_message_t msg;

    switch (missionItem->getMissionType()) {
    case MissionItem::MissionItemType::RTL:
    {
        //This is command number 20
        uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        std::shared_ptr<MissionItem::SpatialRTL> item = std::dynamic_pointer_cast<MissionItem::SpatialRTL>(missionItem);
        mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,20,0,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        break;
    }
    case MissionItem::MissionItemType::LAND:
    {
        //This is command number 21
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialLand<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialLand<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,21,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::TAKEOFF:
    {
        //This is command number 22
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,22,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);
        }
        break;
    }
    case MissionItem::MissionItemType::WAYPOINT:
    {
        //This is command number 16
        if(missionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> item = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(missionItem);
            float latitude = item->position.latitude;
            float longitude = item->position.longitude;
            float altitude = item->position.altitude;
            mavlink_msg_mission_item_pack_chan(255,190,chan,&msg,item->getVehicleID(),compID,itemIndex,frame,16,0,1,0.0,0.0,0.0,0.0,latitude,longitude,altitude);

        }
        break;
    }
    default:
        break;
    }

    return msg;
}

}
