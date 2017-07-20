#include "data_generic_item_system_arm.h"

namespace DataGenericItem {

DataGenericItem_SystemArm::DataGenericItem_SystemArm() :
    armed(false)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const bool &arm) :
    armed(arm)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const DataGenericItem_SystemArm &copyObj)
{
    this->armed = copyObj.getSystemArm();
}


mace_vehicle_armed_t DataGenericItem_SystemArm::getMACECommsObject() const
{
    mace_vehicle_armed_t rtnObj;

    rtnObj.vehicle_armed = this->armed ? 1 : 0;

    return rtnObj;
}

} //end of namespace DataGenericItem
