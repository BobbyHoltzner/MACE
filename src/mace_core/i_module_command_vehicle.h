#ifndef I_VEHICLE_COMMS_H
#define I_VEHICLE_COMMS_H

#include "module_base.h"
#include "metadata_vehicle.h"

#include "i_module_events_vehicle.h"

class IModuleCommandVehicle : public ModuleBase<MetadataVehicle, IModuleEventsVehicle>
{
public:

    IModuleCommandVehicle(MetadataVehicle metadata):
        ModuleBase(metadata)
    {

    }

    virtual std::string ModuleName() const
    {
        return "Vehicle";
    }


public:

};

#endif // I_VEHICLE_COMMS_H
