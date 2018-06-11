#ifndef I_RTA_EVENTS_H
#define I_RTA_EVENTS_H

#include "i_module_events_general.h"


namespace MaceCore
{

class IModuleEventsRTA  : public IModuleEventsGeneral
{
public:
    virtual void Event_SetVehicleBoundaryVertices(const ModuleBase *sender, const std::map<int, mace::geometry::Cell_2DC> &vehicleMap) = 0;
};

} //End MaceCore Namespace

#endif // I_RTA_EVENTS_H
