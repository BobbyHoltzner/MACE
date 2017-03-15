#include "mavlink_parser.h"

namespace DataMAVLINK
{
std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> MAVLINKParser::ParseForData(const mavlink_message_t* message, const std::shared_ptr<const MaceData>){

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;

    switch ((int)message->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        //might want to figure out a way to handle the case of sending an
        //empty heartbeat back just to acknowledge the aircraft is still there
        //then again the streaming other messages may handle this...so maybe
        //timer should be since last time heard.
        mavlink_heartbeat_t decodedMSG;
        mavlink_msg_heartbeat_decode(message,&decodedMSG);

        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrParameters = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
        //ptrParameters->setVehicleType(decodedMSG.type);
        //ptrParameters->setVehicleArmed(decodedMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
        rtnVector.push_back(ptrParameters);
        m_CurrentVehicleState = ptrParameters;
        //check that something has actually changed
//            if(m_CurrentArduVehicleState == NULL || *ptrParameters != *m_CurrentArduVehicleState)
//            {
//                rtnVector.push_back(ptrParameters);
//                m_CurrentArduVehicleState = ptrParameters;
//            }
        heartbeatSeen = true;
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        mavlink_sys_status_t decodedMSG;
        mavlink_msg_sys_status_decode(message,&decodedMSG);

        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Fuel> ptrFuel = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Fuel>();
        ptrFuel->setBatteryVoltage(decodedMSG.voltage_battery/1000.0);
        ptrFuel->setBatteryCurrent(decodedMSG.current_battery/10000.0);
        ptrFuel->setBatteryRemaining(decodedMSG.battery_remaining);
        if(m_CurrentVehicleFuel == NULL || *ptrFuel != *m_CurrentVehicleFuel)
        {
            rtnVector.push_back(ptrFuel);
            m_CurrentVehicleFuel = ptrFuel;
        }
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
        std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>();
        ptrAttitude->setAttitude(decodedMSG.roll,decodedMSG.pitch,decodedMSG.yaw);
        ptrAttitude->setAttitudeRates(decodedMSG.rollspeed,decodedMSG.pitchspeed,decodedMSG.yawspeed);

        if(m_CurrentVehicleAttitude == NULL || *ptrAttitude != *m_CurrentVehicleAttitude)
        {
            rtnVector.push_back(ptrAttitude);
        }
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        //This is message definition 32
        //The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(message,&decodedMSG);
        std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
        ptrLocalPosition->x = decodedMSG.x;
        ptrLocalPosition->y = decodedMSG.y;
        ptrLocalPosition->z = decodedMSG.z;

        //check that something has actually changed
        if(m_CurrentLocalPosition == NULL || *ptrLocalPosition != *m_CurrentLocalPosition)
        {
            rtnVector.push_back(ptrLocalPosition);
            m_CurrentLocalPosition = ptrLocalPosition;
        }

        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(message,&decodedMSG);
        double power = pow(10,7);
        std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
        ptrPosition->setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.alt/1000);
        //check that something has actually changed
        if(m_CurrentGlobalPosition == NULL || *ptrPosition != *m_CurrentGlobalPosition)
        {
            rtnVector.push_back(ptrPosition);
            m_CurrentGlobalPosition = ptrPosition;
        }
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
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
        ptrStatusText->setText(decodedMSG.text);
        switch (decodedMSG.severity) {
        case MAV_SEVERITY_EMERGENCY:

            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_EMERGENCY);
            break;
        case MAV_SEVERITY_ALERT:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_ALERT);
            break;
        case MAV_SEVERITY_CRITICAL:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_CRITICAL);
            break;
        case MAV_SEVERITY_ERROR:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_ERROR);
            break;
        case MAV_SEVERITY_WARNING:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_WARNING);
            break;
        case MAV_SEVERITY_NOTICE:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_NOTICE);
            break;
        case MAV_SEVERITY_INFO:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_INFO);
            break;
        case MAV_SEVERITY_DEBUG:
            ptrStatusText->setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_DEBUG);
            break;
        default:
            break;
        }
        //check that something has actually changed
        if(m_CurrentVehicleText == NULL || *ptrStatusText != *m_CurrentVehicleText)
        {
            rtnVector.push_back(ptrStatusText);
            m_CurrentVehicleText = ptrStatusText;
        }
   break;
    }

    default:
    {
        //std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
    }

    }//end of switch statement



    if(rtnVector.empty())
    {
        return {};
    }
    return {rtnVector};

}

}
