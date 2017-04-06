#include "data_generic_item_fuel.h"

namespace DataGenericItem {

DataGenericItem_Fuel::DataGenericItem_Fuel() :
    voltage(0.0), current(0.0), batteryRemaing(0.0)
{

}

DataGenericItem_Fuel::DataGenericItem_Fuel(const DataGenericItem_Fuel &copyObj)
{
    this->voltage = copyObj.getBatteryVoltage();
    this->current = copyObj.getBatteryCurrent();
    this->batteryRemaing = copyObj.getBatteryRemaining();
}

} //end of namespace DataGenericItem
