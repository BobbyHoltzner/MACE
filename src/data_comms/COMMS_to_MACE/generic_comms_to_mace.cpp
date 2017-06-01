#include "generic_comms_to_mace.h"

namespace DataCOMMS{

Generic_COMMSTOMACE::Generic_COMMSTOMACE()
{

}

DataGenericItem::DataGenericItem_SystemArm Generic_COMMSTOMACE::SystemArm_COMMSTOMACE(const mace_vehicle_armed_t &genericItem, const int &systemID)
{
    UNUSED(systemID);
    DataGenericItem::DataGenericItem_SystemArm armItem;
    armItem.setSystemArm(genericItem.vehicle_armed);
    return armItem;
}

DataGenericItem::DataGenericItem_FlightMode Generic_COMMSTOMACE::SystemMode_COMMSTOMACE(const mace_vehicle_mode_t &genericItem, const int &systemID)
{
    UNUSED(systemID);
    DataGenericItem::DataGenericItem_FlightMode modeItem;
    modeItem.setFlightMode(genericItem.vehicle_mode);
    return modeItem;
}

DataGenericItem::DataGenericItem_Battery Generic_COMMSTOMACE::Battery_COMMSTOMACE(const mace_battery_status_t &genericItem, const int &systemID)
{
    UNUSED(systemID);
    DataGenericItem::DataGenericItem_Battery fuelItem;
    fuelItem.setBatteryVoltage(genericItem.voltage_battery/1000.0);
    fuelItem.setBatteryCurrent(genericItem.current_battery/10000.0);
    fuelItem.setBatteryRemaining(genericItem.battery_remaining);
    return fuelItem;
}

DataGenericItem::DataGenericItem_GPS Generic_COMMSTOMACE::GPS_COMMSTOMACE(const mace_gps_raw_int_t &genericItem, const int &systemID)
{
    UNUSED(systemID);
    DataGenericItem::DataGenericItem_GPS gpsItem;
    gpsItem.setHDOP(genericItem.eph);
    gpsItem.setVDOP(genericItem.epv);
    gpsItem.setSatVisible(genericItem.satellites_visible);

    gpsItem.setGPSFix(static_cast<Data::GPSFixType>(genericItem.fix_type));

    return gpsItem;

}

DataGenericItem::DataGenericItem_Text Generic_COMMSTOMACE::Text_COMMSTOMACE(const mace_statustext_t &genericItem, const int &systemID)
{
    UNUSED(systemID);
    DataGenericItem::DataGenericItem_Text statusText;
    statusText.setText(genericItem.text);

    switch (genericItem.severity) {
    case MAV_SEVERITY_EMERGENCY:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_EMERGENCY);
        break;
    case MAV_SEVERITY_ALERT:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_ALERT);
        break;
    case MAV_SEVERITY_CRITICAL:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_CRITICAL);
        break;
    case MAV_SEVERITY_ERROR:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_ERROR);
        break;
    case MAV_SEVERITY_WARNING:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_WARNING);
        break;
    case MAV_SEVERITY_NOTICE:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_NOTICE);
        break;
    case MAV_SEVERITY_INFO:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_INFO);
        break;
    case MAV_SEVERITY_DEBUG:
        statusText.setSeverity(Data::StatusSeverityType::STATUS_DEBUG);
        break;
    default:
        break;
    }

    return statusText;
}

} //end of namespace DataCOMMS
