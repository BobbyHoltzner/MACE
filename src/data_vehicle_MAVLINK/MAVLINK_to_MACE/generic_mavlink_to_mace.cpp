#include "generic_mavlink_to_mace.h"

namespace DataMAVLINK{

Generic_MAVLINKTOMACE::Generic_MAVLINKTOMACE(const int &systemID):
    mSystemID(systemID)
{

}

DataGenericItem::DataGenericItem_Battery Generic_MAVLINKTOMACE::Battery_MAVLINKTOMACE(const mavlink_sys_status_t &genericItem)
{
    DataGenericItem::DataGenericItem_Battery batteryItem;
    batteryItem.setBatteryVoltage(genericItem.voltage_battery/1000.0);
    batteryItem.setBatteryCurrent(genericItem.current_battery/10000.0);
    batteryItem.setBatteryRemaining(genericItem.battery_remaining);
    return batteryItem;
}

DataGenericItem::DataGenericItem_FlightMode Generic_MAVLINKTOMACE::FlightMode_MAVLINKTOMACE(const mavlink_heartbeat_t &genericItem)
{
    UNUSED(genericItem);
    DataGenericItem::DataGenericItem_FlightMode flightItem;
    return flightItem;
}

DataGenericItem::DataGenericItem_GPS Generic_MAVLINKTOMACE::GPS_MAVLINKTOMACE(const mavlink_gps_raw_int_t &genericItem)
{
    DataGenericItem::DataGenericItem_GPS gpsItem;
    gpsItem.setHDOP(genericItem.eph);
    gpsItem.setVDOP(genericItem.epv);
    gpsItem.setSatVisible(genericItem.satellites_visible);
    switch(genericItem.fix_type)
    {
    case GPS_FIX_TYPE_2D_FIX:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_2D_FIX);
        break;
    case GPS_FIX_TYPE_3D_FIX:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_3D_FIX);
        break;
    case GPS_FIX_TYPE_DGPS:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_DGPS);
        break;
    case GPS_FIX_TYPE_NO_FIX:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NO_FIX);
        break;
    case GPS_FIX_TYPE_NO_GPS:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NONE);
        break;
    case GPS_FIX_TYPE_RTK_FIXED:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_RTK_FIXED);
        break;
    case GPS_FIX_TYPE_RTK_FLOAT:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_RTK_FLOAT);
        break;
    case GPS_FIX_TYPE_STATIC:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_STATIC);
        break;
    default:
        gpsItem.setGPSFix(gpsItem.GPSFixType::GPS_FIX_NO_FIX);
        break;
    }

    return gpsItem;

}

DataGenericItem::DataGenericItem_Heartbeat Generic_MAVLINKTOMACE::Heartbeat_MAVLINKTOMACE(const mavlink_heartbeat_t &genericItem)
{
    //KEN FIX
    DataGenericItem::DataGenericItem_Heartbeat heartbeatItem;
    return heartbeatItem;
}

DataGenericItem::DataGenericItem_SystemArm Generic_MAVLINKTOMACE::SystemArm_MAVLINKTOMACE(const mavlink_heartbeat_t &genericItem)
{
    DataGenericItem::DataGenericItem_SystemArm systemArmItem;
    return systemArmItem;
}

DataGenericItem::DataGenericItem_Text Generic_MAVLINKTOMACE::Text_MAVLINKTOMACE(const mavlink_statustext_t &genericItem)
{
    DataGenericItem::DataGenericItem_Text statusText;
    statusText.setText(genericItem.text);

    switch (genericItem.severity) {
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

    return statusText;
}

} //end of namespace DataMAVLINK
