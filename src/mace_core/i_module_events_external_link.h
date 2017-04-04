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

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_EXTERNAL_LINK_H
