#include "module_vehicle_ardupilot.h"

bool ModuleVehicleArdupilot::ParseMAVLINKMissionMessage(DataARDUPILOT::VehicleObject_ARDUPILOT* vehicleData, const std::string &linkName, const mavlink_message_t* message)
{
    bool parsedMissionMSG = true;
    int sysID = message->sysid;
    int compID = message->compid;
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel(linkName);

    switch ((int)message->msgid) {
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mavlink_mission_item_t decodedMSG;
        mavlink_msg_mission_item_decode(message,&decodedMSG);

        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, sysID, MissionItem::MissionList::INCOMPLETE, Data::MissionType::AUTO_CURRENT);

        if(decodedMSG.seq == 0)
        {
            //This is the home position item associated with the vehicle
            MissionItem::SpatialHome newHome;
            newHome.position.latitude = decodedMSG.x;
            newHome.position.longitude = decodedMSG.y;
            newHome.position.altitude = decodedMSG.z;
            std::shared_ptr<MissionTopic::MissionHomeTopic> homeTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
            homeTopic->setVehicleID(sysID);
            homeTopic->setHome(newHome);

            MaceCore::TopicDatagram topicDatagram;
            m_VehicleMission.SetComponent(homeTopic, topicDatagram);

            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
            });
        }else{
            int currentIndex = decodedMSG.seq - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
            //04/03/2017 Ken Fix This
            std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem = vehicleData->Covert_MAVLINKTOMACE(decodedMSG);
            missionList.replaceMissionItemAtIndex(newMissionItem,currentIndex);
        }

        //If there was no mission list we do not want to do anything further
        if(!validity)
            return true;

        //If there was a mission list in the core, this means that we have previously
        //received a count and therefore should expect a full new mission
        MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            mavlink_message_t msg;
            int indexRequest = status.remainingItems.at(0)+1;
            mavlink_msg_mission_request_pack_chan(255,190,m_LinkChan,&msg,sysID,0,indexRequest); //we have to index this +1 because ardupilot indexes 0 as home
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                ptr->UpdateVehicleMission(this, status, missionList);
            });
        }else{
            //We should update the core
            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                ptr->UpdateVehicleMission(this, status, missionList);
            });
            //We should update all listeners
            std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>();
            missionTopic->setVehicleID(sysID);
            missionTopic->setMissionList(missionList);

            MaceCore::TopicDatagram topicDatagram;
            m_VehicleMission.SetComponent(missionTopic, topicDatagram);
            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
            });
        }
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        //This is message definition 40
        //Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
        mavlink_mission_request_t decodedMSG;
        mavlink_msg_mission_request_decode(message,&decodedMSG);
        std::cout<<"The requested message seq"<<decodedMSG.seq<<std::endl;
        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, sysID, MissionItem::MissionList::COMPLETE, Data::MissionType::AUTO_PROPOSED);
        if(validity)
        {
            std::shared_ptr<MissionItem::AbstractMissionItem> missionItem = missionList.getMissionItem(decodedMSG.seq);
            //04/03/2017 KEN FIX
            mavlink_message_t msg;
            vehicleData->MACEMissionToMAVLINKMission(missionItem,chan,compID,decodedMSG.seq,msg);
            m_LinkMarshaler->SendMessage<mavlink_message_t>(linkName, msg);
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        //This is message definition 41
        //Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
        mavlink_mission_set_current_t decodedMSG;
        mavlink_msg_mission_set_current_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        //This is message definition 42
        //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::MissionItemCurrentTopic> missionTopic = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
        missionTopic->setVehicleID(sysID);
        missionTopic->setMissionItemIndex(decodedMSG.seq);

        if(m_CurrentMissionItem == NULL || *missionTopic != *m_CurrentMissionItem)
        {
            MaceCore::TopicDatagram topicDatagram;
            m_VehicleMission.SetComponent(missionTopic, topicDatagram);
            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
            });
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        //This is message definition 43
        //Request the overall list of mission items from the system/component.
        mavlink_mission_request_list_t decodedMSG;
        mavlink_msg_mission_request_list_decode(message,&decodedMSG);

        //Now we have to respond with the mission count
        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, sysID, MissionItem::MissionList::COMPLETE, Data::MissionType::AUTO_PROPOSED);
        if(!validity)
            return true;

        mavlink_mission_count_t missionCount;
        missionCount.target_system = sysID;
        missionCount.target_component = decodedMSG.target_component;
        missionCount.count = missionList.getQueueSize();

        mavlink_message_t msg;
        mavlink_msg_mission_count_encode_chan(255,190,m_LinkChan,&msg,&missionCount);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        //This is message definition 44
        //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
        //The GCS can then request the individual mission item based on the knowledge of the total number of MISSION
        mavlink_mission_count_t decodedMSG;
        mavlink_msg_mission_count_decode(message,&decodedMSG);
        MissionItem::MissionList newMissionList;
        newMissionList.setMissionType(Data::MissionType::AUTO_CURRENT);
        newMissionList.setVehicleID(sysID);

        int queuesize = decodedMSG.count - 1; //we have to decrement 1 here because in actuality ardupilot references home as 0
        newMissionList.initializeQueue(queuesize);
        MissionItem::MissionList::MissionListStatus status = newMissionList.getMissionListStatus();

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->UpdateVehicleMission(this, status, newMissionList);
        });

        mavlink_message_t msg;
        mavlink_msg_mission_request_pack_chan(255,190,m_LinkChan,&msg,sysID,0,0);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        //This is message definition 45
        //Delete all mission items at once.
        mavlink_mission_clear_all_t decodedMSG;
        mavlink_msg_mission_clear_all_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
    {
        //This is message definition 46
        //A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or
        //(if the autocontinue on the WP was set) continue to the next MISSION.
        mavlink_mission_item_reached_t decodedMSG;
        mavlink_msg_mission_item_reached_decode(message,&decodedMSG);

        std::shared_ptr<MissionTopic::MissionItemReachedTopic> missionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>();
        missionTopic->setVehicleID(sysID);
        missionTopic->setMissionItemIndex(decodedMSG.seq);

        MaceCore::TopicDatagram topicDatagram;
        m_VehicleMission.SetComponent(missionTopic, topicDatagram);
        //notify listneres of topic
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        //This is message definition 47
        //Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
        mavlink_mission_ack_t decodedMSG;
        mavlink_msg_mission_ack_decode(message,&decodedMSG);
        std::cout<<"The mission command was acknowledged"<<std::endl;
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        //This is message definition 242
        //This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on.
        //The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after.
        //The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames,
        //while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can
        //be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight
        //mode and then perform a landing sequence along the vector.
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(message,&decodedMSG);

        MissionItem::SpatialHome spatialHome;
        spatialHome.position.latitude = decodedMSG.latitude / pow(10,7);
        spatialHome.position.longitude = decodedMSG.longitude / pow(10,7);
        spatialHome.position.altitude = decodedMSG.altitude / 1000;
        spatialHome.setVehicleID(sysID);

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->NewVehicleHomePosition(this,spatialHome);
        });

        std::shared_ptr<MissionTopic::MissionHomeTopic> missionTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
        missionTopic->setVehicleID(sysID);
        missionTopic->setHome(spatialHome);

        MaceCore::TopicDatagram topicDatagram;
        m_VehicleMission.SetComponent(missionTopic, topicDatagram);
        //notify listneres of topic
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleMission.Name(), sysID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }

    default:
    {
        parsedMissionMSG = false;
    }
    }
    return parsedMissionMSG;
}

void ModuleVehicleArdupilot::MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult)
{
    UNUSED(publishResult);
    switch(missionResult) {
    case MAV_MISSION_ACCEPTED:
    {
        break;
    }
    case MAV_MISSION_ERROR:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED_FRAME:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED:
    {
        break;
    }
    case MAV_MISSION_NO_SPACE:
    {
        break;
    }
    case MAV_MISSION_INVALID:
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7:
    {
        std::cout<<"One of the parameters has an invalid value"<<std::endl;
        break;
    }
    case MAV_MISSION_DENIED:
    {
        break;
    }
    default:
    {
        break;
    }
    }
}
