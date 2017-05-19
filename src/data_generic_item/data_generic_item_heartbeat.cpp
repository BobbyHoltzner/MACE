#include "data_generic_item_heartbeat.h"

namespace DataGenericItem {

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat() :
    autopilot(Data::AutopilotType::AUTOPILOT_TYPE_GENERIC),protocol(Data::CommsProtocol::COMMS_MACE),type(Data::SystemType::SYSTEM_TYPE_GENERIC),maceCompanion(true)
{

}

//DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const mace_heartbeat_t &heartbeat)
//{
//    protocol = static_cast<MAV_PROTOCOL>(heartbeat.protocol);
//    type = static_cast<MAV_TYPE>(heartbeat.type);
//    autopilot = static_cast<MAV_AUTOPILOT>(heartbeat.autopilot);
//    maceCompanion = (heartbeat.mace_companion > 0) ? true : false;
//}

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj)
{
    this->protocol = copyObj.getProtocol();
    this->type = copyObj.getType();
    this->autopilot = copyObj.getAutopilot();
    this->maceCompanion = copyObj.getCompanion();
}

} //end of namespace DataGenericItem
