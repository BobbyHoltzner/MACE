#include "mavlink_vehicle_object.h"

MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj, const int &ID, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue):
    m_CB(nullptr), mavlinkID(ID)
{
    this->commsLink = commsObj;

    controllerQueue = queue;
    state = new StateData_MAVLINK();
    mission = new MissionData_MAVLINK();
}

MavlinkVehicleObject::~MavlinkVehicleObject()
{ 
    delete state;
    delete mission;
}

int MavlinkVehicleObject::getMAVLINKID() const
{
    return this->mavlinkID;
}

CommsMAVLINK* MavlinkVehicleObject::getCommsObject() const
{
    return this->commsLink;
}
void MavlinkVehicleObject::updatedOrigins()
{
    if(mission->globalOrigin.hasBeenSet() && mission->vehicleHomePosition.hasBeenSet())
    {
        CommandItem::SpatialHome vehicleHome = mission->vehicleHomePosition.get();
        mace::pose::GeodeticPosition_3D homeCast(vehicleHome.getPosition().getX(),vehicleHome.getPosition().getY(),vehicleHome.getPosition().getZ());
        mace::pose::GeodeticPosition_3D origin = mission->globalOrigin.get();
        bearingShift = origin.polarBearingTo(homeCast);
        distanceShift = origin.distanceBetween2D(homeCast);
    }
}

void MavlinkVehicleObject::updateGlobalOrigin(const GeodeticPosition_3D &refOrigin)
{
    if(mission->globalOrigin.set(refOrigin))
    {
        this->updatedOrigins();
    }
}

void MavlinkVehicleObject::updateVehicleHomePosition(const SpatialHome &refOrigin)
{
    if(mission->vehicleHomePosition.set(refOrigin))
    {
        this->updatedOrigins();
    }
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

    return consumed;
}
