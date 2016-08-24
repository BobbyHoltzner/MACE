#ifndef I_RTA_H
#define I_RTA_H

#include <string>
#include <map>

#include "module_base.h"
#include "metadata_rta.h"

#include "i_module_events_rta.h"

#include "metadata_vehicle.h"

#include "vehicle_data.h"

namespace MaceCore
{

class IModuleCommandRTA : public ModuleBase<Metadata_RTA, IModuleEventsRTA>
{
public:

    IModuleCommandRTA(Metadata_RTA metadata):
        ModuleBase(metadata)
    {

    }

    virtual std::string ModuleName() const
    {
        return "RTA";
    }

public:
    virtual void NewVehicle(const std::string &ID, const MetadataVehicle &vehicle) = 0;

    virtual void RemoveVehicle(const std::string &ID) = 0;

    virtual void UpdatedPosition(const std::string &vehicleID) = 0;

    virtual void UpdateDynamicsState(const std::string &vehicleID) = 0;

    virtual void UpdatedVehicleLife(const std::string &vehicleID) = 0;

    virtual void UpdatedOccupancyMap() = 0;
};

} //End MaceCore Namespace

#endif // I_RTA_H
