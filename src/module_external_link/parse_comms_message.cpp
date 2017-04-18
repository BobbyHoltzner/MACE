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
            ptr->SetVehicleHomePosition(this, systemHome);
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

    case MAVLINK_MSG_ID_MACE_MISSION_ITEM:
    {
        //This is message definition 39
        //Message encoding a mission item. This message is emitted to announce the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
        mavlink_mace_mission_item_t decodedMSG;
        mavlink_msg_mace_mission_item_decode(message,&decodedMSG);
        MACE_MISSION_TYPE missionType = static_cast<MACE_MISSION_TYPE>(decodedMSG.mission_type);

//        mavlink_mission_item_t decodedMSG;
//        mavlink_msg_mission_item_decode(message,&decodedMSG);

        DataCOMMS::Mission_COMMSTOMACE missionConvert;
        std::shared_ptr<MissionItem::AbstractMissionItem> newMissionItem = missionConvert.Covert_COMMSTOMACE(decodedMSG);

        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, decodedMSG.target_system, MissionItem::MissionList::INCOMPLETE, static_cast<Data::MissionType>(decodedMSG.mission_type));
        if(!validity)
            return;

        missionList.replaceMissionItemAtIndex(newMissionItem,decodedMSG.seq);
        MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            mavlink_message_t msg;
            mavlink_msg_mace_mission_request_pack_chan(associatedSystemID,compID,m_LinkChan,&msg,systemID,compID,status.remainingItems.at(0),missionType);
            //mavlink_msg_mission_request_pack_chan(associatedSystemID,compID,m_LinkChan,&msg,systemID,missionType,status.remainingItems.at(0));
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
//KEN FIX
//            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//                ptr->UpdateVehicleMission(this, status, missionList);
//            });
        }else{
            std::cout<<"The mission requested is complete"<<std::endl;

            //First let us send the acknowledgement to the ground
            mavlink_message_t msg;
            mavlink_mace_mission_ack_t ack;
            ack.target_system = systemID;
            ack.target_component = compID;
            ack.mission_type = decodedMSG.mission_type;
            ack.type = MAV_MISSION_ACCEPTED;
            mavlink_msg_mace_mission_ack_encode_chan(associatedSystemID,compID,m_LinkChan,&msg,&ack);
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

            //We should update the core
//KEN FIX
//            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//                ptr->UpdateVehicleMission(this, status, missionList);
//            });

            //We now tell the core to pass that mission to the correct vehicle
            ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
                ptr->TransferMissionToVehicle(this, missionList);
            });

            //KEN FIX: Check that target system matches sysID
            //We should update all listeners
            std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>();
            missionTopic->setMissionList(missionList);

            MaceCore::TopicDatagram topicDatagram;
            m_MissionDataTopic.SetComponent(missionTopic, topicDatagram);
            //notify listneres of topic
            ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), decodedMSG.target_system, MaceCore::TIME(), topicDatagram);
            });
        }
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST:
    {
        //This is message definition 40
        //Request the information of the mission item with the sequence number seq.
        //The response of the system to this message should be a MISSION_ITEM message.
        mavlink_mace_mission_request_t decodedMSG;
        mavlink_msg_mace_mission_request_decode(message,&decodedMSG);
        MACE_MISSION_TYPE missionType = static_cast<MACE_MISSION_TYPE>(decodedMSG.mission_type);

//        mavlink_mission_request_t decodedMSG;
//        mavlink_msg_mission_request_decode(message,&decodedMSG);
        DataCOMMS::Mission_MACETOCOMMS missionConvert(decodedMSG.target_system,missionType);
        std::shared_ptr<MissionItem::AbstractMissionItem> missionItem;

        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, decodedMSG.target_system, MissionItem::MissionList::COMPLETE, static_cast<Data::MissionType>(decodedMSG.mission_type));
        if((!validity) || (decodedMSG.seq > missionList.getQueueSize() - 1))
            return;
        missionItem = missionList.getMissionItem(decodedMSG.seq);

        mavlink_message_t msg;
        bool msgValid = missionConvert.MACEMissionToMAVLINKMission(missionItem, missionType, m_LinkChan,missionType,decodedMSG.seq,msg);
        if(msgValid){
            m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        }
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_REQUEST_LIST:
    {
        //This is message definition 43
        //External item has requested the overall list of mission items from the system/component.
        mavlink_mace_mission_request_list_t decodedMSG;
        mavlink_msg_mace_mission_request_list_decode(message,&decodedMSG);
//        mavlink_mission_request_list_t decodedMSG;
//        mavlink_msg_mission_request_list_decode(message,&decodedMSG);
        MACE_MISSION_TYPE missionType = static_cast<MACE_MISSION_TYPE>(decodedMSG.mission_type);

        //Now we have to respond with the mission count
        MissionItem::MissionList missionList;
        bool validity = this->getDataObject()->getMissionList(missionList, decodedMSG.target_system, MissionItem::MissionList::COMPLETE, static_cast<Data::MissionType>(decodedMSG.mission_type));
        if(!validity)
            return;

        mavlink_mace_mission_count_t missionCount;
        missionCount.target_system = systemID;
        missionCount.target_component = decodedMSG.target_component;
        missionCount.mission_type = missionType;
        missionCount.count = missionList.getQueueSize();

        mavlink_message_t msg;
        mavlink_msg_mace_mission_count_encode_chan(associatedSystemID,compID,m_LinkChan,&msg,&missionCount);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        break;
    }

    case MAVLINK_MSG_ID_MACE_MISSION_COUNT:
    {
        //This is message definition 44
        //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
        //The GCS can then request the individual mission item based on the knowledge of the total number of MISSION
        //This isnt necessarily the best way as the packet gets reset
        mavlink_mace_mission_count_t decodedMSG;
        mavlink_msg_mace_mission_count_decode(message,&decodedMSG);
//        mavlink_mission_count_t decodedMSG;
//        mavlink_msg_mission_count_decode(message,&decodedMSG);
        MACE_MISSION_TYPE missionType = static_cast<MACE_MISSION_TYPE>(decodedMSG.mission_type);

        MissionItem::MissionList newMissionList;
        newMissionList.setMissionType(static_cast<Data::MissionType>(decodedMSG.mission_type));
        newMissionList.setVehicleID(decodedMSG.target_system);
        newMissionList.initializeQueue(decodedMSG.count);
        MissionItem::MissionList::MissionListStatus status = newMissionList.getMissionListStatus();
//KEN FIX
//        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//            ptr->UpdateVehicleMission(this, status, newMissionList);
//        });

        mavlink_message_t msg;
        mavlink_msg_mace_mission_request_pack_chan(associatedSystemID,compID,m_LinkChan,&msg,decodedMSG.target_system,compID,0,missionType);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        break;
    }
    case MAVLINK_MSG_ID_MACE_MISSION_ACK:
    {
        break;
    }

    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement

}
