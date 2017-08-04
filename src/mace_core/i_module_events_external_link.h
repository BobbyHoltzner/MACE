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
    //! \brief ExternalEvent_NewConstructedVehicle
    //! \param sender
    //! \param newVehicleObserved
    //!
    virtual void ExternalEvent_NewConstructedVehicle(const void *sender, const int &newVehicleObserved) = 0;

    //!
    //! \brief ExternalEvent_MissionACK
    //! \param sender
    //! \param missionACK
    //!
    virtual void ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK) = 0;

    //!
    //! \brief External_AppendMissionQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_ReceivingMissionQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_FinisedRXProposedQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_FinishedRXProposedQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_FinisedRXOnboardQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_FinishedRXOnboardQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_FinisedRXCurrentQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_FinishedRXCurrentQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_EXTERNAL_LINK_H
