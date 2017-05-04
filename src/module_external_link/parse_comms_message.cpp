#include "module_external_link.h"

void ModuleExternalLink::ParseForData(const mavlink_message_t* message){
    MaceCore::TopicDatagram topicDatagram;
    int systemID = message->sysid;
    int compID = message->compid;

    switch ((int)message->msgid) {
    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL STATE EVENTS: These are events that may have a direct
    ////////////////////////////////////////////////////////////////////////////
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t decodedMSG;
        mavlink_msg_heartbeat_decode(message,&decodedMSG);
        if(systemIDMap.find(systemID) == systemIDMap.end())
        {
            //The system has yet to have communicated through this module
            //We therefore have to notify the core that there is a new vehicle
            systemIDMap.insert({systemID,0});
            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                ptr->NewConstructedVehicle(this, systemID);
            });
        }
//        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrParameters = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
//        m_VehicleDataTopic.SetComponent(ptrParameters, topicDatagram);
//        //notify listneres of topic
//        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//        });
        //ptrParameters->setVehicleType(decodedMSG.type);
        //ptrParameters->setVehicleArmed(decodedMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        mavlink_sys_status_t decodedMSG;
        mavlink_msg_sys_status_decode(message,&decodedMSG);

        DataGenericItem::DataGenericItem_Fuel newFuel = DataCOMMS::Generic_COMMSTOMACE::Fuel_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> ptrFuel = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Fuel>(newFuel);
        associatedSystemID = systemID;
        m_VehicleDataTopic.SetComponent(ptrFuel, topicDatagram);
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t decodedMSG;
        mavlink_msg_command_ack_decode(message,&decodedMSG);
        std::cout<<"The command acknowledgement came from: "<<decodedMSG.command<<std::endl;
        switch(decodedMSG.result)
        {
            case MAV_RESULT_ACCEPTED:
                std::cout<<"MAV result accepted"<<std::endl;
                break;
            case MAV_RESULT_TEMPORARILY_REJECTED:
                std::cout<<"MAV result rejected"<<std::endl;
                break;
            case MAV_RESULT_DENIED:
                std::cout<<"MAV result denied"<<std::endl;
                break;
            case MAV_RESULT_UNSUPPORTED:
                std::cout<<"MAV result unsupported"<<std::endl;
                break;
            case MAV_RESULT_FAILED:
                std::cout<<"MAV result failed"<<std::endl;
                break;
            default:
                std::cout<<"Uknown ack!"<<std::endl;
        }
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(message,&decodedMSG);
        DataGenericItem::DataGenericItem_GPS newItem = DataCOMMS::Generic_COMMSTOMACE::GPS_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPS = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(newItem);
        m_VehicleDataTopic.SetComponent(ptrGPS, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MAVLINK_MSG_ID_GPS_STATUS:
    {
        //This is message definition 25
        //The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        //This is message definition 30
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_attitude_t decodedMSG;
        mavlink_msg_attitude_decode(message,&decodedMSG);
        DataState::StateAttitude newAttitude = DataCOMMS::State_COMMSTOMACE::Attitude_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(newAttitude);
        m_VehicleDataTopic.SetComponent(ptrAttitude, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(message,&decodedMSG);
        DataState::StateLocalPosition newPosition = DataCOMMS::State_COMMSTOMACE::LocalPosition_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrLocalPosition, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(message,&decodedMSG);
        DataState::StateGlobalPosition newPosition = DataCOMMS::State_COMMSTOMACE::GlobalPosition_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(newPosition);

        m_VehicleDataTopic.SetComponent(ptrPosition, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
    {
        //This is message definition 62
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        //This is message definition 74
        //Metrics typically displayed on a HUD for fixed wing aircraft
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t decodedMSG;
        mavlink_msg_command_long_decode(message,&decodedMSG);
        this->ParseCommsCommand(&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        //This is message definition 109
        //Status generated by radio and injected into MAVLink stream.
        mavlink_radio_status_t decodedMSG;
        mavlink_msg_radio_status_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS:
    {
        //This is message definition 125
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        //This is message definition 147
        //Battery information
        mavlink_battery_status_t decodedMSG;
        mavlink_msg_battery_status_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        //This is message definition 253
        mavlink_statustext_t decodedMSG;
        mavlink_msg_statustext_decode(message,&decodedMSG);
        std::cout<<"The status text says: "<<decodedMSG.text<<std::endl;
        DataGenericItem::DataGenericItem_Text newText = DataCOMMS::Generic_COMMSTOMACE::Text_COMMSTOMACE(decodedMSG,systemID);
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);

        m_VehicleDataTopic.SetComponent(ptrStatusText, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
   break;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// HOME BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////
    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        mavlink_set_home_position_t decodedMSG;
        mavlink_msg_set_home_position_decode(message,&decodedMSG);
        DataCOMMS::Mission_COMMSTOMACE missionConvert;
        MissionItem::SpatialHome systemHome;
        missionConvert.Home_COMMSTOMACE(decodedMSG.target_system,decodedMSG,systemHome);
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->Event_SetHomePosition(this, systemHome);
        });
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        mavlink_home_position_t decodedMSG;
        mavlink_msg_home_position_decode(message,&decodedMSG);

        MissionItem::SpatialHome spatialHome;
        spatialHome.position.latitude = decodedMSG.latitude / pow(10,7);
        spatialHome.position.longitude = decodedMSG.longitude / pow(10,7);
        spatialHome.position.altitude = decodedMSG.altitude / 1000;
        spatialHome.setVehicleID(systemID);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->NewVehicleHomePosition(this,spatialHome);
        });

        std::shared_ptr<MissionTopic::MissionHomeTopic> missionTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
        missionTopic->setVehicleID(systemID);
        missionTopic->setHome(spatialHome);

        MaceCore::TopicDatagram topicDatagram;
        m_MissionDataTopic.SetComponent(missionTopic, topicDatagram);
        //notify listneres of topic
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });

        break;
    }
    ////////////////////////////////////////////////////////////////////////////
    /// MISSION BASED EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    case MAVLINK_MSG_ID_MACE_NEW_CURRENT_MISSION:
    {
        mavlink_mace_new_current_mission_t decodedMSG;
        mavlink_msg_mace_new_current_mission_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MACE_NEW_ONBOARD_MISSION:
    {
        mavlink_mace_new_onboard_mission_t decodedMSG;
        mavlink_msg_mace_new_onboard_mission_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MACE_NEW_PROPOSED_MISSION:
    {
        //A vechile instance of MACE should receive this message
        mavlink_mace_new_proposed_mission_t decodedMSG;
        mavlink_msg_mace_new_proposed_mission_decode(message,&decodedMSG);

        //at this point associatedSystemID should be the same as decodedMSG.target_system
        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionTypeState missionState = static_cast<Data::MissionTypeState>(decodedMSG.mission_state);

        MissionItem::MissionList newMissionList(decodedMSG.target_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState,decodedMSG.count);
        MissionItem::MissionList::MissionListStatus status = newMissionList.getMissionListStatus();

        //now we tell the core that we have some sort of new list that external link may be receiving
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_ReceivingMissionQueue(this, newMissionList);
        });

        mavlink_message_t msg;
        mavlink_msg_mace_mission_request_item_pack_chan(decodedMSG.target_system,compID,m_LinkChan,&msg,systemID,decodedMSG.target_system,decodedMSG.mission_creator,decodedMSG.mission_id,decodedMSG.mission_type,0);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        break;
    }
    case MAVLINK_MSG_ID_MACE_ACK_MISSION:
    {
        mavlink_mace_ack_mission_t decodedMSG;
        mavlink_msg_mace_ack_mission_decode(message,&decodedMSG);

        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST_LIST:
    {
        mavlink_mace_mission_request_list_t decodedMSG;
        mavlink_msg_mace_mission_request_list_decode(message,&decodedMSG);

        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionTypeState missionState = static_cast<Data::MissionTypeState>(decodedMSG.mission_state);
        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(decodedMSG.target_system,missionType,missionState,missionList);

        if(!validity) //KEN TODO: Return a message saying that the request is invalid because the item does not exsist...probably enum failure value / validity
            return;

        mavlink_mace_mission_count_t missionCount;
        missionCount.count = missionList.getQueueSize();
        Data::MissionKey key = missionList.getMissionKey();
        missionCount.mission_creator = key.m_creatorID;
        missionCount.mission_id = key.m_missionID;
        missionCount.mission_state = decodedMSG.mission_state;
        missionCount.mission_system = key.m_systemID;
        missionCount.mission_type = decodedMSG.mission_type;
        missionCount.target_system = systemID;

        mavlink_message_t msg;
        mavlink_msg_mace_mission_count_encode_chan(associatedSystemID,compID,m_LinkChan,&msg,&missionCount);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_COUNT:
    {
        //This message indicates that the sender has a new mission for us to handle
        mavlink_mace_mission_count_t decodedMSG;
        mavlink_msg_mace_mission_count_decode(message,&decodedMSG);

        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionTypeState missionState = static_cast<Data::MissionTypeState>(decodedMSG.mission_state);

        MissionItem::MissionList newMissionList(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType,missionState,decodedMSG.count);
        MissionItem::MissionList::MissionListStatus status = newMissionList.getMissionListStatus();

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_ReceivingMissionQueue(this, newMissionList);
        });

        mavlink_message_t msg;
        mavlink_msg_mace_mission_request_item_pack_chan(decodedMSG.target_system,compID,m_LinkChan,&msg,systemID,decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,decodedMSG.mission_type,0);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST_PARTIAL_LIST:
    {
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_WRITE_PARTIAL_LIST:
    {
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mavlink_mace_mission_item_t decodedMSG;
        mavlink_msg_mace_mission_item_decode(message,&decodedMSG);

        DataCOMMS::Mission_COMMSTOMACE missionConvert;
        std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem = missionConvert.Covert_COMMSTOMACE(decodedMSG);

        //Get the MissionItem::MissionList from the data core
        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);
        Data::MissionKey itemKey(decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,missionType);
        MissionItem::MissionList missionList;
        bool valid = this->getDataObject()->getRXMissionList(itemKey,missionList);
        if(valid) //if the mission didnt already exist than something is out of order and we currently cant handle it
        {
            missionList.replaceMissionItemAtIndex(newMissionItem,decodedMSG.seq);
            MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                ptr->ExternalEvent_ReceivingMissionQueue(this, missionList);
            });
            if(status.state == MissionItem::MissionList::INCOMPLETE)
            {
                mavlink_message_t msg;
                mavlink_msg_mace_mission_request_item_pack_chan(decodedMSG.target_system,compID,m_LinkChan,&msg,systemID,decodedMSG.mission_system,decodedMSG.mission_creator,decodedMSG.mission_id,decodedMSG.mission_type,status.remainingItems.at(0));
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
                //should we tell anyone that we have received another mission item however the mission is incomplete
            }else{

                mavlink_message_t msg;
                mavlink_mace_ack_mission_t ackMission;
                ackMission.target_system = systemID;
                ackMission.mission_system = itemKey.m_systemID;
                ackMission.mission_creator = itemKey.m_creatorID;
                ackMission.mission_id = itemKey.m_missionID;
                ackMission.mission_type = (uint8_t)itemKey.m_missionType;
                //KEN TODO: Maybe we have another state reflect that it has been received differnt than onboard (implying received by the aircraft instance)
                ackMission.mission_state = (uint8_t)Data::MissionTypeState::TRANSMITTED;

                if(missionList.getMissionTypeState() == Data::MissionTypeState::PROPOSED)
                {
                    //This case implies that we were receiving the item from a ground module or
                    //someone without directly relating to the vehicle and therefore we should ack

                    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                        ptr->ExternalEvent_FinisedRXProposedQueue(this, missionList);
                    });
                }else if(missionList.getMissionTypeState() == Data::MissionTypeState::ONBOARD)
                {
                    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                        ptr->ExternalEvent_FinisedRXOnboardQueue(this, missionList);
                    });
                }else if(missionList.getMissionTypeState() == Data::MissionTypeState::CURRENT)
                {
                    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                        ptr->ExternalEvent_FinisedRXCurrentQueue(this, missionList);
                    });
                }

                mavlink_msg_mace_ack_mission_encode_chan(itemKey.m_systemID,0,m_LinkChan,&msg,&ackMission);
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST_ITEM:
    {
        //Request the information of the mission item with the sequence number seq.
        //The response of the system to this message should be a MISSION_ITEM message.
        mavlink_mace_mission_request_item_t decodedMSG;
        mavlink_msg_mace_mission_request_item_decode(message,&decodedMSG);

        Data::MissionType missionType = static_cast<Data::MissionType>(decodedMSG.mission_type);

        Data::MissionKey itemKey(decodedMSG.mission_system, decodedMSG.mission_creator, decodedMSG.mission_id, missionType);
        MissionItem::MissionList missionList;

        bool valid = this->getDataObject()->getMissionList(itemKey,missionList);

        if((!valid) || (decodedMSG.seq > missionList.getQueueSize() - 1))
            return;

        DataCOMMS::Mission_MACETOCOMMS missionConvert(decodedMSG.target_system, systemID, itemKey, m_LinkChan);
        std::shared_ptr<MissionItem::AbstractMissionItem> missionItem;
        missionItem = missionList.getMissionItem(decodedMSG.seq);

        mavlink_message_t msg;
        bool msgValid = missionConvert.MACEMissionToCOMMSMission(missionItem, decodedMSG.seq, msg);
        if(msgValid){
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        }
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_SET_CURRENT:
    {
        mavlink_mace_mission_set_current_t decodedMSG;
        mavlink_msg_mace_mission_set_current_decode(message,&decodedMSG);

        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_CURRENT:
    {
        mavlink_mace_mission_current_t decodedMSG;
        mavlink_msg_mace_mission_current_decode(message,&decodedMSG);

        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_CLEAR:
    {
        mavlink_mace_mission_clear_t decodedMSG;
        mavlink_msg_mace_mission_clear_decode(message,&decodedMSG);

        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_ITEM_REACHED:
    {
        mavlink_mace_mission_item_reached_t decodedMSG;
        mavlink_msg_mace_mission_item_reached_decode(message,&decodedMSG);

        break;
    }
    default:
    {
        std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement

}
