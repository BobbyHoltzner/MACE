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

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

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
    virtual void cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) const = 0;

    virtual void cbi_VehicleHome(const int &systemID, const CommandItem::SpatialHome &home) = 0;
    virtual void cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList) = 0;
    virtual void cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current) const = 0;
};

class MavlinkVehicleObject : public Controllers::IMessageNotifier<mavlink_message_t, int>
{
public:
    MavlinkVehicleObject(CommsMAVLINK *commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID);

    ~MavlinkVehicleObject();

    int getMAVLINKID() const;

    MaceCore::ModuleCharacteristic getModule() const;

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
    virtual void TransmitMessage(const mavlink_message_t &msg, const OptionalParameter<int> &target) const
    {
        UNUSED(target);
        commsLink->TransmitMAVLINKMessage(msg);
    }

    virtual std::vector<int> GetAllTargets() const
    {
        return {mavlinkID};
    }

    virtual int GetModuleFromMAVLINKVehicleID(int ID) const
    {
        if(mavlinkID != ID)
        {
            throw std::runtime_error("Asked a local vehicle object for the ModuleCharacterstic for a vehicle that isn't it");
        }

        return mavlinkID;
    }


    virtual int GetHostKey() const
    {
        return mavlinkID;
    }


    virtual std::tuple<int, int> GetSysIDAndCompIDFromComponentKey(const int &key) const
    {
        return std::make_tuple(key, 0);
    }


    virtual bool parseMessage(const mavlink_message_t *msg);


    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey>* ControllersCollection()
    {
        return &m_ControllersCollection;
    }

    TransmitQueue<mavlink_message_t, MavlinkEntityKey> *GetControllerQueue()
    {
        return controllerQueue;
    }

    const CallbackInterface_MAVLINKVehicleObject* getCallbackInterface() const
    {
        if(m_CB != nullptr)
            return m_CB;
    }

    bool handleMAVLINKMessage(const mavlink_message_t &msg);

public:
    StateData_MAVLINK *state;
    MissionData_MAVLINK *mission;

protected:
    int mavlinkID;
    MaceCore::ModuleCharacteristic m_module;

    PointerCollection<
    > m_Controllers;

    CommsMAVLINK *commsLink;

    CallbackInterface_MAVLINKVehicleObject* m_CB;

    TransmitQueue<mavlink_message_t, MavlinkEntityKey> *controllerQueue;

    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> m_ControllersCollection;
};

#endif // MAVLINK_VEHICLE_OBJECT_H
