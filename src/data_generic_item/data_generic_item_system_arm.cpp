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

} //end of namespace DataGenericItem
