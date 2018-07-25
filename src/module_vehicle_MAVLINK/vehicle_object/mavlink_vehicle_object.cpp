#include "mavlink_vehicle_object.h"

MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj, const MaceCore::ModuleCharacteristic &module, const int &m_MavlinkID):
    m_CB(nullptr), m_module(module), mavlinkID(m_MavlinkID)
{   
    this->commsLink = commsObj;

    controllerQueue = new TransmitQueue<mavlink_message_t, MavlinkEntityKey>(2000, 3);
    state = new StateData_MAVLINK();
    mission = new MissionData_MAVLINK();
}

int MavlinkVehicleObject::getMAVLINKID() const
{
    return this->mavlinkID;
}


MaceCore::ModuleCharacteristic MavlinkVehicleObject::getModule() const
{
    return m_module;
}

CommsMAVLINK* MavlinkVehicleObject::getCommsObject() const
{
    return this->commsLink;
}

bool MavlinkVehicleObject::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    int MavlinkSysID = msg.sysid;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t, int>*>::iterator it;

    m_ControllersCollection.controllerMutex.lock();
    for(it=m_ControllersCollection.controllers.begin(); it!=m_ControllersCollection.controllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t, int>* obj = it->second;
        consumed = obj->ReceiveMessage(&msg, MavlinkSysID);
    }
    m_ControllersCollection.controllerMutex.unlock();

    return consumed;
}
