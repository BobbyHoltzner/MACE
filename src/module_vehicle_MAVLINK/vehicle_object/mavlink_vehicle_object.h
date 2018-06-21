#ifndef MAVLINK_VEHICLE_OBJECT_H
#define MAVLINK_VEHICLE_OBJECT_H

#include "commsMAVLINK/comms_mavlink.h"

#include "controllers/generic_controller.h"

#include "../controllers/commands/command_land.h"
#include "../controllers/commands/command_takeoff.h"
#include "../controllers/commands/command_arm.h"
#include "../controllers/commands/command_rtl.h"
#include "../controllers/controller_system_mode.h"
#include "../controllers/controller_collection.h"

#include "state_data_mavlink.h"
#include "mission_data_mavlink.h"

template <typename T, typename TT>
T* Helper_CreateAndSetUp(TT* obj, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue, uint8_t chan)
{
    T* newController = new T(obj, queue, chan);
    //newController->setLambda_DataReceived([obj](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command){obj->ReceivedCommand(sender, command);});
    //newController->setLambda_Finished(FinishedMAVLINKMessage);
    return newController;
}

class CallbackInterface_MAVLINKVehicleObject
{
public:
    virtual void cbi_VehicleStateData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;
    virtual void cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;

    virtual void cbi_VehicleHome(const int &systemID, const CommandItem::SpatialHome &home) = 0;
    virtual void cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList) = 0;
    virtual void cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current) = 0;
};

class MavlinkVehicleObject : public Controllers::IMessageNotifier<mavlink_message_t>
{
public:
    MavlinkVehicleObject(CommsMAVLINK *commsObj, const int &ID, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue);

    ~MavlinkVehicleObject();

    int getMAVLINKID() const;

    CommsMAVLINK* getCommsObject() const;

    void connectCallback(CallbackInterface_MAVLINKVehicleObject *cb)
    {
        m_CB = cb;
    }

    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmit to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const mavlink_message_t &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) const
    {
        UNUSED(target);
        commsLink->TransmitMAVLINKMessage(msg);
    }

    virtual bool parseMessage(const mavlink_message_t *msg);


    Controllers::ControllerCollection<mavlink_message_t>* ControllersCollection()
    {
        return &m_ControllersCollection;
    }

    Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *GetControllerQueue()
    {
        return controllerQueue;
    }

    bool handleMAVLINKMessage(const mavlink_message_t &msg);

public:
    StateData_MAVLINK *state;
    MissionData_MAVLINK *mission;

protected:
    int mavlinkID;

    PointerCollection<
    > m_Controllers;

    CommsMAVLINK *commsLink;

    CallbackInterface_MAVLINKVehicleObject* m_CB;

    Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *controllerQueue;

    Controllers::ControllerCollection<mavlink_message_t> m_ControllersCollection;
};

#endif // MAVLINK_VEHICLE_OBJECT_H
