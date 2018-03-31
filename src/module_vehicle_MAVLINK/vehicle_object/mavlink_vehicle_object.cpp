#include "mavlink_vehicle_object.h"

MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj)
{
    this->commsLink = commsObj;

    controllerQueue = new Controllers::MessageModuleTransmissionQueue<mavlink_message_t>(2000, 3);

    //inside this object we should construct any of the controllers that are required across all states of the vehicle

//    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKVehicleControllers::CommandLand>(this, queue, m_LinkChan));
//    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKVehicleControllers::CommandTakeoff>(this, queue, m_LinkChan));
//    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKVehicleControllers::CommandARM>(this, queue, m_LinkChan));
//    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKVehicleControllers::CommandRTL>(this, queue, m_LinkChan));
//    m_Controllers.Retreive<MAVLINKVehicleControllers::CommandRTL>().

}

