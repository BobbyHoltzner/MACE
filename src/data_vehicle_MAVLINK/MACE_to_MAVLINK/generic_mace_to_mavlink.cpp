#include "generic_mace_to_mavlink.h"

namespace DataMAVLINK {

Generic_MACETOMAVLINK::Generic_MACETOMAVLINK(const int &systemID, const int &compID):
    mSystemID(systemID),mCompID(compID)
{

}

mavlink_message_t Generic_MACETOMAVLINK::FlightModeTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &topicItem, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_FlightMode newFlightMode = *topicItem.get();
    mavlink_message_t msg = FlightMode_MACETOMAVLINK(newFlightMode, chan);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::FlightMode_MACETOMAVLINK(DataGenericItem::DataGenericItem_FlightMode &flightModeItem, const uint8_t &chan)
{
     mavlink_message_t msg;
     mavlink_heartbeat_t heartbeat;
    //TODO: Kenny fix this heartbeat
     mavlink_msg_heartbeat_encode_chan(mSystemID,mCompID,chan,&msg,&heartbeat);
     return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::BatteryTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &topicItem, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_Battery newBattery = *topicItem.get();
    mavlink_message_t msg = Battery_MACETOMAVLINK(newBattery,chan);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::Battery_MACETOMAVLINK(DataGenericItem::DataGenericItem_Battery fuelItem, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_sys_status_t sysStatus;
    sysStatus.current_battery = (int16_t)(fuelItem.getBatteryCurrent() * 10000.0);
    sysStatus.voltage_battery = (uint16_t)(fuelItem.getBatteryVoltage()*1000.0);
    sysStatus.battery_remaining = (int8_t)fuelItem.getBatteryRemaining();
    mavlink_msg_sys_status_encode_chan(mSystemID,mCompID,chan,&msg,&sysStatus);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::GPSTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &topicItem, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_GPS newGPS = *topicItem.get();
    mavlink_message_t msg = GPS_MACETOMAVLINK(newGPS,chan);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::GPS_MACETOMAVLINK(DataGenericItem::DataGenericItem_GPS GPSItem, const uint8_t &chan)
{

    mavlink_message_t msg;
    mavlink_gps_raw_int_t gpsRaw;
    gpsRaw.satellites_visible = GPSItem.getSatVisible();
    gpsRaw.eph = GPSItem.getHDOP();
    gpsRaw.epv = GPSItem.getVDOP();

    switch(GPSItem.getGPSFix())
    {
    case Data::GPSFixType::GPS_FIX_2D_FIX:
        gpsRaw.fix_type = GPS_FIX_TYPE_2D_FIX;
        break;
    case Data::GPSFixType::GPS_FIX_3D_FIX:
        gpsRaw.fix_type = GPS_FIX_TYPE_3D_FIX;
        break;
    case Data::GPSFixType::GPS_FIX_DGPS:
        gpsRaw.fix_type = GPS_FIX_TYPE_DGPS;
        break;
    case Data::GPSFixType::GPS_FIX_NONE:
        gpsRaw.fix_type = GPS_FIX_TYPE_NO_GPS;
        break;
    case Data::GPSFixType::GPS_FIX_NO_FIX:
        gpsRaw.fix_type = GPS_FIX_TYPE_NO_FIX;
        break;
    case Data::GPSFixType::GPS_FIX_RTK_FIXED:
        gpsRaw.fix_type = GPS_FIX_TYPE_RTK_FIXED;
        break;
    case Data::GPSFixType::GPS_FIX_RTK_FLOAT:
        gpsRaw.fix_type = GPS_FIX_TYPE_RTK_FLOAT;
        break;
    case Data::GPSFixType::GPS_FIX_STATIC:
        gpsRaw.fix_type = GPS_FIX_TYPE_STATIC;
        break;
    default:
        gpsRaw.fix_type = GPS_FIX_TYPE_NO_FIX;
        break;
    }

    mavlink_msg_gps_raw_int_encode_chan(mSystemID,mCompID,chan,&msg,&gpsRaw);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::TextTopicPTR_MACETOMAVLINK(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &topicItem, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_Text newText = *topicItem.get();
    mavlink_message_t msg = Text_MACETOMAVLINK(newText,chan);
    return(msg);
}

mavlink_message_t Generic_MACETOMAVLINK::Text_MACETOMAVLINK(DataGenericItem::DataGenericItem_Text textItem, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_statustext_t statusText;
    strcpy(statusText.text,textItem.getText().c_str());

    switch(textItem.getSeverity()){
    case Data::StatusSeverityType::STATUS_ALERT:
        statusText.severity = MAV_SEVERITY_ALERT;
        break;
    case Data::StatusSeverityType::STATUS_CRITICAL:
        statusText.severity = MAV_SEVERITY_CRITICAL;
        break;
    case Data::StatusSeverityType::STATUS_DEBUG:
        statusText.severity = MAV_SEVERITY_DEBUG;
        break;
    case Data::StatusSeverityType::STATUS_EMERGENCY:
        statusText.severity = MAV_SEVERITY_EMERGENCY;
        break;
    case Data::StatusSeverityType::STATUS_ERROR:
        statusText.severity = MAV_SEVERITY_ERROR;
        break;
    case Data::StatusSeverityType::STATUS_INFO:
        statusText.severity = MAV_SEVERITY_INFO;
        break;
    case Data::StatusSeverityType::STATUS_NOTICE:
        statusText.severity = MAV_SEVERITY_NOTICE;
        break;
    case Data::StatusSeverityType::STATUS_WARNING:
        statusText.severity = MAV_SEVERITY_WARNING;
        break;
    default:
        statusText.severity = MAV_SEVERITY_ALERT;
        //probably should throw something here alerting that something went wrong
        break;
    }

    mavlink_msg_statustext_encode_chan(mSystemID,mCompID,chan,&msg,&statusText);
    return(msg);
}


} //end of namespace DataMAVLINK
