#ifndef I_RTA_EVENTS_H
#define I_RTA_EVENTS_H

#include "i_module_events_general.h"


namespace MaceCore
{

class IModuleEventsRTA  : public IModuleEventsGeneral
{
public:
    virtual void GSEvent_UploadMission(const void* sender, const MissionItem::MissionList &missionList) = 0;

};

} //End MaceCore Namespace

#endif // I_RTA_EVENTS_H
