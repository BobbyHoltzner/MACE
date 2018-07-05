#ifndef I_GROUND_STATION_EVENTS_H
#define I_GROUND_STATION_EVENTS_H

#include "i_module_events_general.h"

#include "i_module_events_boundary_generator.h"

namespace MaceCore
{

class IModuleEventsGroundStation : public IModuleEventsGeneral, public IModuleEventsBoundaryGenerator
{
public:
    virtual void RequestDummyFunction(const void* sender, const int &vehicleID) = 0;

    virtual void GSEvent_UploadMission(const void* sender, const MissionItem::MissionList &missionList) = 0;
};

} //End MaceCore Namespace

#endif // I_GROUND_STATION_EVENTS_H
