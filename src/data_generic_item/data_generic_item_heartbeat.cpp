#include "data_generic_item_heartbeat.h"

namespace DataGenericItem {

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat() :
    protocol(MAV_PROTOCOL_MACE),type(MAV_TYPE_GENERIC),autopilot(MAV_AUTOPILOT_GENERIC),maceCompanion(true)
{

}

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const mace_heartbeat_t &heartbeat)
{
    protocol = static_cast<MAV_PROTOCOL>(heartbeat.protocol);
    type = static_cast<MAV_TYPE>(heartbeat.type);
    autopilot = static_cast<MAV_AUTOPILOT>(heartbeat.autopilot);
    maceCompanion = (heartbeat.mace_companion > 0) ? true : false;
}

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj)
{
    this->protocol = copyObj.getProtocol();
    this->type = copyObj.getType();
    this->autopilot = copyObj.getAutopilot();
    this->maceCompanion = copyObj.getCompaion();
}

} //end of namespace DataGenericItem
