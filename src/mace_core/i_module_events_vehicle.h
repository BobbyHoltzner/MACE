#ifndef I_VEHICLE_EVENTS_H
#define I_VEHICLE_EVENTS_H

namespace MaceCore
{

class IModuleEventsVehicle
{
public:
    //A vehicle module can indicate something has happened
    virtual void NewConstructedVehicle(const void* sender, const int &newVehicleObserved) = 0;


};

} //End MaceCore Namespace

#endif // I_VEHICLE_EVENTS_H
