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

} //end of namespace DataGenericItem
