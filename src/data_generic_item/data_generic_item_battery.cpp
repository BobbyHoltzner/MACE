#include "data_generic_item_battery.h"

namespace DataGenericItem {

DataGenericItem_Battery::DataGenericItem_Battery() :
    voltage(0.0), current(0.0), batteryRemaing(0.0)
{

}

DataGenericItem_Battery::DataGenericItem_Battery(const DataGenericItem_Battery &copyObj)
{
    this->voltage = copyObj.getBatteryVoltage();
    this->current = copyObj.getBatteryCurrent();
    this->batteryRemaing = copyObj.getBatteryRemaining();
}

DataGenericItem_Battery::DataGenericItem_Battery(const mace_battery_status_t &copyObj)
{
    this->voltage = copyObj.current_battery / 10000.0;
    this->current = copyObj.voltage_battery / 1000.0;
    this->batteryRemaing = copyObj.battery_remaining;
}

mace_battery_status_t DataGenericItem_Battery::getMACECommsObject() const
{
    mace_battery_status_t rtnObj;

    rtnObj.current_battery = (int16_t)(this->current * 10000.0);
    rtnObj.voltage_battery = (uint16_t)(this->voltage*1000.0);
    rtnObj.battery_remaining = (int8_t)this->getBatteryRemaining();

    return rtnObj;
}

} //end of namespace DataGenericItem
