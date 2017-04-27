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

    //!
    //! \brief UpdateMissionToVehicle
    //! \param sender
    //! \param missionList
    //!
    virtual void TransferMissionToVehicle(const void* sender, const MissionItem::MissionList &missionList) = 0;


    virtual void ExternalEvent_MissionACK(const void* sender, const Data::MissionKey &key, const Data::MissionTypeState &state) = 0;

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
    virtual void ExternalEvent_FinisedRXProposedQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_FinisedRXOnboardQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_FinisedRXOnboardQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief ExternalEvent_FinisedRXCurrentQueue
    //! \param sender
    //! \param missionList
    //!
    virtual void ExternalEvent_FinisedRXCurrentQueue(const void* sender, const MissionItem::MissionList &missionList) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_EXTERNAL_LINK_H
