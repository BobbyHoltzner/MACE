#include "generic_mavlink_to_mace.h"

namespace DataMAVLINK{

Generic_MAVLINKTOMACE::Generic_MAVLINKTOMACE(const int &systemID):
    mSystemID(systemID)
{

}

DataGenericItem::DataGenericItem_FlightMode Generic_MAVLINKTOMACE::FlightMode_MAVLINKTOMACE(const mavlink_heartbeat_t &genericItem)
{
    UNUSED(genericItem);
    DataGenericItem::DataGenericItem_FlightMode flightItem;
    return flightItem;
}

DataGenericItem::DataGenericItem_Battery Generic_MAVLINKTOMACE::Battery_MAVLINKTOMACE(const mavlink_sys_status_t &genericItem)
{
    DataGenericItem::DataGenericItem_Battery batteryItem;
    batteryItem.setBatteryVoltage(genericItem.voltage_battery/1000.0);
    batteryItem.setBatteryCurrent(genericItem.current_battery/10000.0);
    batteryItem.setBatteryRemaining(genericItem.battery_remaining);
    return batteryItem;
}

DataGenericItem::DataGenericItem_GPS Generic_MAVLINKTOMACE::GPS_MAVLINKTOMACE(const mavlink_gps_raw_int_t &genericItem)
{
    DataGenericItem::DataGenericItem_GPS gpsItem;
    gpsItem.setHDOP(genericItem.eph);
    gpsItem.setVDOP(genericItem.epv);
    gpsItem.setSatVisible(genericItem.satellites_visible);
    switch(genericItem.fix_type)
    {
    case 0:
        gpsItem.setGPSFix(gpsItem.GPSFIX_NOGPS);
        break;
    case 1:
        gpsItem.setGPSFix(gpsItem.GPSFIX_NOFIX);
        break;
    case 2:
        gpsItem.setGPSFix(gpsItem.GPSFIX_2DFIX);
        break;
    case 3:
        gpsItem.setGPSFix(gpsItem.GPSFIX_3DFIX);
        break;
    case 4:
        gpsItem.setGPSFix(gpsItem.GPSFIX_DGPS);
        break;
    case 5:
        gpsItem.setGPSFix(gpsItem.GPSFIX_RTKFLOAT);
        break;
    case 6:
        gpsItem.setGPSFix(gpsItem.GPSFIX_RTKFIXED);
        break;
    case 7:
        gpsItem.setGPSFix(gpsItem.GPSFIX_STATIC);
        break;
    default:
        gpsItem.setGPSFix(gpsItem.GPSFIX_NOGPS);
        break;
    }

    return gpsItem;

}

DataGenericItem::DataGenericItem_Text Generic_MAVLINKTOMACE::Text_MAVLINKTOMACE(const mavlink_statustext_t &genericItem)
{
    DataGenericItem::DataGenericItem_Text statusText;
    statusText.setText(genericItem.text);

    switch (genericItem.severity) {
    case MAV_SEVERITY_EMERGENCY:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_EMERGENCY);
        break;
    case MAV_SEVERITY_ALERT:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_ALERT);
        break;
    case MAV_SEVERITY_CRITICAL:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_CRITICAL);
        break;
    case MAV_SEVERITY_ERROR:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_ERROR);
        break;
    case MAV_SEVERITY_WARNING:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_WARNING);
        break;
    case MAV_SEVERITY_NOTICE:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_NOTICE);
        break;
    case MAV_SEVERITY_INFO:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_INFO);
        break;
    case MAV_SEVERITY_DEBUG:
        statusText.setSeverity(DataGenericItem::DataGenericItem_Text::STATUS_DEBUG);
        break;
    default:
        break;
    }

    return statusText;
}

} //end of namespace DataMAVLINK
