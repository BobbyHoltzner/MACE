#ifndef MODULE_VEHICLE_GENERIC_H
#define MODULE_VEHICLE_GENERIC_H

#include "module_vehicle_generic_global.h"


#include "common/common.h"

#include "mace_core/i_module_command_vehicle.h"

#include "vehicle_data_composites.h"
#include "i_vehicle_data_component.h"



class MODULE_VEHICLE_GENERICSHARED_EXPORT ModuleVehicleGeneric : public MaceCore::IModuleCommandVehicle
{

public:
    ModuleVehicleGeneric();

    virtual std::unordered_map<std::string, MaceCore::TopicStructure> GetTopics();

    void AddComponent(const int &vehicleID, const std::shared_ptr<IVehicleDataComponent> component);

private:

    std::unordered_map<int, VehicleDataComposites> m_Vehicles;
};

#endif // MODULE_VEHICLE_GENERIC_H
