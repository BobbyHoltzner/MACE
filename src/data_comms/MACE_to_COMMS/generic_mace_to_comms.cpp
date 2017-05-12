#include "generic_mace_to_comms.h"

namespace DataCOMMS {


mace_message_t Generic_MACETOCOMMS::HeartbeatTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_Heartbeat heartbeatItem = *topicItem.get();
    mace_message_t msg = Heartbeat_MACETOCOMMS(heartbeatItem,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::Heartbeat_MACETOCOMMS(DataGenericItem::DataGenericItem_Heartbeat heartbeatItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
     mace_message_t msg;
     mace_heartbeat_t heartbeat;
     heartbeat.autopilot = heartbeatItem.getAutopilot();
     heartbeat.mace_companion = heartbeatItem.getCompaion() ? 1 : 0;
     heartbeat.protocol = heartbeatItem.getProtocol();
     heartbeat.type = heartbeatItem.getType();
     mace_msg_heartbeat_encode_chan(systemID,compID,chan,&msg,&heartbeat);
     return(msg);
}

mace_message_t Generic_MACETOCOMMS::SystemArmTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_SystemArm newSystemArm = *topicItem.get();
    mace_message_t msg = SystemArm_MACETOCOMMS(newSystemArm,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::SystemArm_MACETOCOMMS(DataGenericItem::DataGenericItem_SystemArm systemArmItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
     mace_message_t msg;
     mace_vehicle_armed_t armed;
     armed.vehicle_armed = systemArmItem.getSystemArm() ? 1 : 0;
     mace_msg_vehicle_armed_encode_chan(systemID,compID,chan,&msg,&armed);
     return(msg);
}

mace_message_t Generic_MACETOCOMMS::FlightModeTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_FlightMode newFlightMode = *topicItem.get();
    mace_message_t msg = FlightMode_MACETOCOMMS(newFlightMode,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::FlightMode_MACETOCOMMS(DataGenericItem::DataGenericItem_FlightMode flightModeItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
     mace_message_t msg;
     mace_vehicle_mode_t mode;
     flightModeItem.getFlightModeString().copy(mode.vehicle_mode,10,0);
     mace_msg_vehicle_mode_encode_chan(systemID,compID,chan,&msg,&mode);
     return(msg);
}

mace_message_t Generic_MACETOCOMMS::BatteryTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_Battery newBattery = *topicItem.get();
    mace_message_t msg = Battery_MACETOCOMMS(newBattery,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::Battery_MACETOCOMMS(DataGenericItem::DataGenericItem_Battery fuelItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
    mace_message_t msg;
    mace_battery_status_t batteryStatus;
    batteryStatus.current_battery = (int16_t)(fuelItem.getBatteryCurrent() * 10000.0);
    batteryStatus.voltage_battery = (uint16_t)(fuelItem.getBatteryVoltage()*1000.0);
    batteryStatus.battery_remaining = (int8_t)fuelItem.getBatteryRemaining();
    mace_msg_battery_status_encode_chan(systemID,compID,chan,&msg,&batteryStatus);
    return(msg);
}

mace_message_t Generic_MACETOCOMMS::GPSTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_GPS newGPS = *topicItem.get();
    mace_message_t msg = GPS_MACETOCOMMS(newGPS,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::GPS_MACETOCOMMS(DataGenericItem::DataGenericItem_GPS GPSItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
    mace_message_t msg;
    mace_gps_raw_int_t gpsRaw;
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

    mace_msg_gps_raw_int_encode_chan(systemID,compID,chan,&msg,&gpsRaw);
    return(msg);
}

mace_message_t Generic_MACETOCOMMS::TextTopicPTR_MACETOCOMMS(const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> &topicItem, const int &systemID, const uint8_t &compID, const uint8_t &chan)
{
    DataGenericItem::DataGenericItem_Text newText = *topicItem.get();
    mace_message_t msg = Text_MACETOCOMMS(newText,systemID,compID,chan);
    return(msg);
}
mace_message_t Generic_MACETOCOMMS::Text_MACETOCOMMS(DataGenericItem::DataGenericItem_Text textItem, const int &systemID, const uint8_t &chan, const uint8_t &compID)
{
    mace_message_t msg;
    mace_statustext_t statusText;
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

    mace_msg_statustext_encode_chan(systemID,compID,chan,&msg,&statusText);
    return(msg);
}


} //end of namespace DataCOMMS
