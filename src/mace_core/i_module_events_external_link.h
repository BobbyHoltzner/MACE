#ifndef I_MODULE_EVENTS_EXTERNAL_LINK_H
#define I_MODULE_EVENTS_EXTERNAL_LINK_H

#include "i_module_events_general.h"
#include "i_module_events_vehicle.h"
#include "i_module_events_general_vehicle.h"

namespace MaceCore
{
class IModuleEventsExternalLink : public IModuleEventsGeneral, public IModuleEventsGeneralVehicle
{
public:

    virtual void ExternalEvent_RequestingDataSync(const void *sender, const int &targetID) = 0;
    //!
    //! \brief ExternalEvent_UpdateRemoteID
    //! \param sender
    //! \param remoteID
    //!
    virtual void ExternalEvent_UpdateRemoteID(const void *sender, const int &remoteID) = 0;

    //!
    //! \brief ExternalEvent_NewModule
    //! \param sender
    //! \param newVehicleObserved
    //!
    virtual void ExternalEvent_NewModule(const void *sender, const ModuleCharacteristic &module) = 0;

    //!
    //! \brief ExternalEvent_MissionACK
    //! \param sender
    //! \param missionACK
    //!
    virtual void ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK) = 0;

    //!
    //! \brief ExternalEvent_FinishedRXMissionList function event emitted by an external link module after the controller
    //! had completed receiving a mission. The type and state of the mission are contained in the list object. It will be
    //! up top the core to decypher this information and organize it in the appropriate place.
    //! \param sender pointer to the module emitting the event.
    //! \param missionList reference to the mission queue the module had received. This should be a complete mission
    //! without any holes or gaps in the queue.
    //!
    virtual void ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList) = 0;


    virtual void ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_EXTERNAL_LINK_H
