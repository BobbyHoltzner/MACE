#include "mavlink_vehicle_object.h"

MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj, const int &ID):
    m_CB(nullptr), mavlinkID(ID)
{
    this->commsLink = commsObj;

    controllerQueue = new Controllers::MessageModuleTransmissionQueue<mavlink_message_t>(2000, 3);
    state = new StateData_MAVLINK();
    mission = new MissionData_MAVLINK();
}

int MavlinkVehicleObject::getMAVLINKID() const
{
    return this->mavlinkID;
}

CommsMAVLINK* MavlinkVehicleObject::getCommsObject() const
{
    return this->commsLink;
}

bool MavlinkVehicleObject::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    int systemID = msg.sysid;

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;

    m_ControllersCollection.controllerMutex.lock();
    for(it=m_ControllersCollection.controllers.begin(); it!=m_ControllersCollection.controllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t>* obj = it->second;
        consumed = obj->ReceiveMessage(&msg, sender);
    }
    m_ControllersCollection.controllerMutex.unlock();

}
