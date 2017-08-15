#include "mavlink_parser.h"

namespace DataMAVLINK
{

MAVLINKParser::MAVLINKParser(DataContainer_MAVLINK* dataContainer)
{
    data = dataContainer;
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> MAVLINKParser::ParseForVehicleData(const mavlink_message_t* message){

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;

    switch ((int)message->msgid) {
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        mavlink_sys_status_t decodedMSG;
        mavlink_msg_sys_status_decode(message,&decodedMSG);

        DataGenericItem::DataGenericItem_Battery battery;
        battery.setBatteryVoltage(decodedMSG.voltage_battery/1000.0);
        battery.setBatteryCurrent(decodedMSG.current_battery/10000.0);
        battery.setBatteryRemaining(decodedMSG.battery_remaining);
        if(data->vehicleFuel.set(battery))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrBattery = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(battery);
            rtnVector.push_back(ptrBattery);
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
        Generic_MAVLINKTOMACE parseHelper(message->sysid);

        DataGenericItem::DataGenericItem_GPS gpsStatus = parseHelper.GPS_MAVLINKTOMACE(decodedMSG);

        if(data->vehicleGPSStatus.set(gpsStatus))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(gpsStatus);
            rtnVector.push_back(ptrGPSStatus);
        }

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
        DataState::StateAttitude attitude;
        attitude.setAttitude(decodedMSG.roll,decodedMSG.pitch,decodedMSG.yaw);
        attitude.setAttitudeRates(decodedMSG.rollspeed,decodedMSG.pitchspeed,decodedMSG.yawspeed);

        if(data->vehicleAttitude.set(attitude))
        {
            std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(attitude);
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

        DataState::StateLocalPosition localPosition;
        localPosition.setPositionX(decodedMSG.x);
        localPosition.setPositionY(decodedMSG.y);
        localPosition.setPositionZ(decodedMSG.z);

        //check that something has actually changed

        if(data->vehicleLocalPosition.set(localPosition))
        {
            std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(localPosition);
            rtnVector.push_back(ptrLocalPosition);
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

        DataState::StateGlobalPosition position;
        position.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.relative_alt/1000.0);
        //check that something has actually changed
        if(data->vehicleGlobalPosition.set(position))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(position);
            rtnVector.push_back(ptrPosition);
        }

        DataState::StateGlobalPositionEx positionEx;
        positionEx.setPosition(decodedMSG.lat/power,decodedMSG.lon/power,decodedMSG.alt/1000.0);
        positionEx.heading = (decodedMSG.hdg/100.0)*(3.14/180.0);

        //check that something has actually changed
        if(data->vehicleGlobalPositionEx.set(positionEx))
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> ptrPositionEx = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>(positionEx);
            rtnVector.push_back(ptrPositionEx);
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
        mavlink_vfr_hud_t decodedMSG;
        mavlink_msg_vfr_hud_decode(message,&decodedMSG);

        DataState::StateAirspeed airspeed;
        airspeed.setAirspeed(decodedMSG.airspeed);
        //check that something has actually changed

        if(data->vehicleAirspeed.set(airspeed))
        {
            std::shared_ptr<DataStateTopic::StateAirspeedTopic> ptrAirspeedTopic = std::make_shared<DataStateTopic::StateAirspeedTopic>(airspeed);
            rtnVector.push_back(ptrAirspeedTopic);
        }
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

        DataGenericItem::DataGenericItem_Text statusText;
        statusText.setText(decodedMSG.text);
        switch (decodedMSG.severity) {
        case MAV_SEVERITY_EMERGENCY:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_EMERGENCY);
            break;
        case MAV_SEVERITY_ALERT:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_ALERT);
            break;
        case MAV_SEVERITY_CRITICAL:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_CRITICAL);
            break;
        case MAV_SEVERITY_ERROR:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_ERROR);
            break;
        case MAV_SEVERITY_WARNING:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_WARNING);
            break;
        case MAV_SEVERITY_NOTICE:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_NOTICE);
            break;
        case MAV_SEVERITY_INFO:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_INFO);
            break;
        case MAV_SEVERITY_DEBUG:
            statusText.setSeverity(statusText.STATUS_SEVERITY::STATUS_DEBUG);
            break;
        default:
            break;
        }
        //check that something has actually changed
        if(data->vehicleTextAlert.set(statusText))
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(statusText);
            rtnVector.push_back(ptrStatusText);
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
