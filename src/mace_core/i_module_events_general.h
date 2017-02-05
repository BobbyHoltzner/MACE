#ifndef I_MODULE_EVENTS_GENERAL_H
#define I_MODULE_EVENTS_GENERAL_H

#include "data_generic_mission_item/mission_item_components.h"

namespace MaceCore
{

class IModuleEventsGeneral
{
public:
    virtual void RequestVehicleArm(const void* sender, const MissionItem::ActionArm &arm) = 0;
    virtual void RequestVehicleMode(const void* sender, const MissionItem::ActionChangeMode &changeMode) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_H
