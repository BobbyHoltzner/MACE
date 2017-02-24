#ifndef MAVLINK_PARSER_ARDUPILOT_H
#define MAVLINK_PARSER_ARDUPILOT_H

#include <iostream>
#include <functional>

#include "mavlink.h"

#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_generic_topic/data_vehicle_generic_topic_components.h"

#include "components/vehicle_flightMode.h"
#include "components/vehicle_operating_status.h"

namespace DataVehicleArdupilot
{

class MAVLINKParserArduPilot
{
public:

    MAVLINKParserArduPilot() :
        m_CurrentArduVehicleState(NULL), m_CurrentArduVehicleStatus(NULL), m_CurrentArduGlobalPosition(NULL),
        m_CurrentArduVehicleFuel(NULL), m_CurrentArduVehicleGPS(NULL), m_CurrentArduVehicleText(NULL)
    {

    }

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message){
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

            std::shared_ptr<VehicleFlightMode> ptrParameters = std::make_shared<VehicleFlightMode>();
            ptrParameters->parseMAVLINK(decodedMSG);
            //check that something has actually changed
            if(m_CurrentArduVehicleState == NULL || *ptrParameters != *m_CurrentArduVehicleState)
            {
                rtnVector.push_back(ptrParameters);
                m_CurrentArduVehicleState = ptrParameters;
            }

            std::shared_ptr<VehicleOperatingStatus> ptrStatus = std::make_shared<VehicleOperatingStatus>();
            ptrStatus->setVehicleArmed(decodedMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
            //check that something has actually changed
            if(m_CurrentArduVehicleStatus == NULL || *ptrStatus != *m_CurrentArduVehicleStatus)
            {
                rtnVector.push_back(ptrStatus);
                m_CurrentArduVehicleStatus = ptrStatus;
            }
            heartbeatSeen = true;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t decodedMSG;
            mavlink_msg_sys_status_decode(message,&decodedMSG);

            std::shared_ptr<DataVehicleGenericTopic::DataVehicleGenericTopic_Fuel> ptrFuel = std::make_shared<DataVehicleGenericTopic::DataVehicleGenericTopic_Fuel>();
            ptrFuel->setBatteryVoltage(decodedMSG.voltage_battery/1000.0);
            ptrFuel->setBatteryCurrent(decodedMSG.current_battery/10000.0);
            ptrFuel->setBatteryRemaining(decodedMSG.battery_remaining);
            if(m_CurrentArduVehicleFuel == NULL || *ptrFuel != *m_CurrentArduVehicleFuel)
            {
                rtnVector.push_back(ptrFuel);
                m_CurrentArduVehicleFuel = ptrFuel;
            }
            break;
        }
        case MAVLINK_MSG_ID_LOG_ENTRY:
        {
            mavlink_log_entry_t decodedMSG;
            mavlink_msg_log_entry_decode(message,&decodedMSG);
            std::cout<<"A log entry message was received! ID: "<<decodedMSG.id<<" Out of: "<<decodedMSG.num_logs<<"With a size of: "<<decodedMSG.size<<std::endl;
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
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        {
            //This is message definition 2
            //The system time is the time of the master clock, typically the computer clock of the main onboard computer.
            mavlink_system_time_t decodedMSG;
            mavlink_msg_system_time_decode(message,&decodedMSG);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            //This is message definition 20
            //Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
            mavlink_param_request_read_t decodedMSG;
            mavlink_msg_param_request_read_decode(message,&decodedMSG);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            //This is message definition 21
            //Request all parameters of this component. After this request, all parameters are emitted.
            mavlink_param_request_list_t decodedMSG;
            mavlink_msg_param_request_list_decode(message,&decodedMSG);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            //This is message definition 22
            //Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
            mavlink_param_value_t decodedMSG;
            mavlink_msg_param_value_decode(message,&decodedMSG);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_SET:
        {
            //This is message definition 23
            //Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
            mavlink_param_set_t decodedMSG;
            mavlink_msg_param_set_decode(message,&decodedMSG);
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
        case MAVLINK_MSG_ID_RAW_IMU:
        {
            //This is message definition 27
            break;
        }
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        {
            //This is message definition 29
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

            if(m_CurrentArduVehicleAttitude == NULL || *ptrAttitude != *m_CurrentArduVehicleAttitude)
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

            rtnVector.push_back(ptrLocalPosition);
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
            if(m_CurrentArduGlobalPosition == NULL || *ptrPosition != *m_CurrentArduGlobalPosition)
            {
                rtnVector.push_back(ptrPosition);
                m_CurrentArduGlobalPosition = ptrPosition;
            }
            break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        {
            //This is message definition 35
            break;
        }
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
            //This is message definition 36
            break;
        }
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        {
            //This is message definition 62
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:
        {
            //This is message definition 65
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
        case MAVLINK_MSG_ID_SCALED_IMU2:
        {
            //This is message definition 116
            break;
        }
        case MAVLINK_MSG_ID_POWER_STATUS:
        {
            //This is message definition 125
            break;
        }
        case MAVLINK_MSG_ID_TERRAIN_REPORT:
        {
            //This is message definition 136
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

        case MAVLINK_MSG_ID_TERRAIN_REPORT:
        {
            //This is message definition 136
            break;
        }
        case MAVLINK_MSG_ID_MEMINFO:
        {
            //This is message definition 152
            break;
        }
        case MAVLINK_MSG_ID_AHRS:
        {
            //This is message definition 163
            break;
        }
        case MAVLINK_MSG_ID_SIMSTATE:
        {
            //This is message definition 164
            break;
        }
        case MAVLINK_MSG_ID_HWSTATUS:
        {
            //This is message definition 165
            break;
        }
        case MAVLINK_MSG_ID_AHRS2:
        {
            //This is message definition 178
            break;
        }
        case MAVLINK_MSG_ID_AHRS3:
        {
            //This is message definition 182
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
        {
            //This is message definition 193
            break;
        }
        case MAVLINK_MSG_ID_VIBRATION:
        {
            //This is message definition 241
            //Vibration levels and accelerometer clipping
            mavlink_vibration_t decodedMSG;
            mavlink_msg_vibration_decode(message,&decodedMSG);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            //This is message definition 253
            mavlink_statustext_t decodedMSG;
            mavlink_msg_statustext_decode(message,&decodedMSG);
            std::cout<<"The status text says: "<<decodedMSG.text<<std::endl;
            std::shared_ptr<DataVehicleGenericTopic::DataVehicleGenericTopic_Text> ptrStatusText = std::make_shared<DataVehicleGenericTopic::DataVehicleGenericTopic_Text>();
            ptrStatusText->setText(decodedMSG.text);
            switch (decodedMSG.severity) {
            case MAV_SEVERITY_EMERGENCY:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_EMERGENCY);
                break;
            case MAV_SEVERITY_ALERT:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_ALERT);
                break;
            case MAV_SEVERITY_CRITICAL:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_CRITICAL);
                break;
            case MAV_SEVERITY_ERROR:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_ERROR);
                break;
            case MAV_SEVERITY_WARNING:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_WARNING);
                break;
            case MAV_SEVERITY_NOTICE:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_NOTICE);
                break;
            case MAV_SEVERITY_INFO:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_INFO);
                break;
            case MAV_SEVERITY_DEBUG:
                ptrStatusText->setSeverity(DataVehicleGenericTopic::DataVehicleGenericTopic_Text::STATUS_DEBUG);
                break;
            default:
                break;
            }
            //check that something has actually changed
            if(m_CurrentArduVehicleText == NULL || *ptrStatusText != *m_CurrentArduVehicleText)
            {
                rtnVector.push_back(ptrStatusText);
                m_CurrentArduVehicleText = ptrStatusText;
            }
       break;
        }

        default:
        {
            std::cout<<"I received an unknown supported message with the ID "<<(int)message->msgid<<std::endl;
        }

        }//end of switch statement



        if(rtnVector.empty())
        {
            return {};
        }
        return {rtnVector};

    }


public:

    int getFlightModeFromString(const std::string &flightString){
        return m_CurrentArduVehicleState->getFlightMode(flightString);
    }

    bool heartbeatUpdated(){
        return heartbeatSeen;
    }

private:

    bool heartbeatSeen = false;
    std::shared_ptr<VehicleFlightMode> m_CurrentArduVehicleState;
    std::shared_ptr<VehicleOperatingStatus> m_CurrentArduVehicleStatus;
    std::shared_ptr<DataVehicleGenericTopic::DataVehicleGenericTopic_Fuel> m_CurrentArduVehicleFuel;
    std::shared_ptr<DataVehicleGenericTopic::DataVehicleGenericTopic_GPS> m_CurrentArduVehicleGPS;
    std::shared_ptr<DataVehicleGenericTopic::DataVehicleGenericTopic_Text> m_CurrentArduVehicleText;

    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> m_CurrentArduGlobalPosition;
    std::shared_ptr<DataStateTopic::StateAttitudeTopic> m_CurrentArduVehicleAttitude;

};

}

#endif // MAVLINK_PARSER_ARDUPILOT_H
