#include "generic_mace_to_mavlink.h"

namespace DataMAVLINK {

mavlink_message_t Generic_MACETOMAVLINK::FlightMode_MACETOMAVLINK(std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> flightModeItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
     mavlink_message_t msg;
     mavlink_heartbeat_t heartbeat;
     if(flightModeItem->getAutopilotType() == Data::AutopilotTypes::ARDUPILOT)
         heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
     heartbeat.custom_mode = flightModeItem->getFlightModeInt();
     if(flightModeItem->getVehicleType() == Data::VehicleTypes::PLANE)
     {
         heartbeat.type = MAV_TYPE_FIXED_WING;
     }else
     {
         heartbeat.type = MAV_TYPE_QUADROTOR;
     }
     mavlink_msg_heartbeat_encode_chan(systemID,compID,chan,&msg,&heartbeat);
     return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::Fuel_MACETOMAVLINK(DataGenericItem::DataGenericItem_Fuel fuelItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
    mavlink_message_t msg;
    mavlink_sys_status_t sysStatus;
    sysStatus.current_battery = (int16_t)(fuelItem.getBatteryCurrent() * 10000.0);
    sysStatus.voltage_battery = (uint16_t)(fuelItem.getBatteryVoltage()*1000.0);
    sysStatus.battery_remaining = (int8_t)fuelItem.getBatteryRemaining();
    mavlink_msg_sys_status_encode_chan(systemID,compID,chan,&msg,&sysStatus);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::GPS_MACETOMAVLINK(DataGenericItem::DataGenericItem_GPS GPSItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{

    mavlink_message_t msg;
    mavlink_gps_raw_int_t gpsRaw;
    gpsRaw.satellites_visible = GPSItem.getSatVisible();
    gpsRaw.eph = GPSItem.getHDOP();
    gpsRaw.epv = GPSItem.getVDOP();

    switch(GPSItem.getGPSFix())
    {
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_2DFIX:
        gpsRaw.fix_type = 2;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_3DFIX:
        gpsRaw.fix_type = 3;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_DGPS:
        gpsRaw.fix_type = 4;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_NOFIX:
        gpsRaw.fix_type = 1;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_NOGPS:
        gpsRaw.fix_type = 0;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_RTKFIXED:
        gpsRaw.fix_type = 6;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_RTKFLOAT:
        gpsRaw.fix_type = 5;
        break;
    case DataGenericItem::DataGenericItem_GPS::GPSFIX_STATIC:
        gpsRaw.fix_type = 7;
        break;
    default:
        gpsRaw.fix_type = 0;
        break;
    }

    mavlink_msg_gps_raw_int_encode_chan(systemID,compID,chan,&msg,&gpsRaw);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::Text_MACETOMAVLINK(DataGenericItem::DataGenericItem_Text textItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
    mavlink_message_t msg;
    mavlink_statustext_t statusText;
    strcpy(statusText.text,textItem.getText().c_str());

    switch(textItem.getSeverity()){
    case DataGenericItem::DataGenericItem_Text::STATUS_ALERT:
        statusText.severity = MAV_SEVERITY_ALERT;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_CRITICAL:
        statusText.severity = MAV_SEVERITY_CRITICAL;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_DEBUG:
        statusText.severity = MAV_SEVERITY_DEBUG;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_EMERGENCY:
        statusText.severity = MAV_SEVERITY_EMERGENCY;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_ERROR:
        statusText.severity = MAV_SEVERITY_ERROR;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_INFO:
        statusText.severity = MAV_SEVERITY_INFO;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_NOTICE:
        statusText.severity = MAV_SEVERITY_NOTICE;
        break;
    case DataGenericItem::DataGenericItem_Text::STATUS_WARNING:
        statusText.severity = MAV_SEVERITY_WARNING;
        break;
    default:
        statusText.severity = MAV_SEVERITY_ALERT;
        //probably should throw something here alerting that something went wrong
        break;
    }

    mavlink_msg_statustext_encode_chan(systemID,compID,chan,&msg,&statusText);
    return(msg);
}


} //end of namespace DataMAVLINK
