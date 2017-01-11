#ifndef MODULE_VEHICLE_GENERIC_H
#define MODULE_VEHICLE_GENERIC_H

#include "module_vehicle_generic_global.h"

#include <functional>

#include "common/common.h"

#include "mace_core/i_module_command_vehicle.h"

#include "data_vehicle_generic/local_position.h"
#include "data_vehicle_generic/local_velocity.h"
#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/global_velocity.h"


#include "data/topic_data_object_collection.h"


template <typename ...VehicleTopicAdditionalComponents>
class MODULE_VEHICLE_GENERICSHARED_EXPORT ModuleVehicleGeneric : public MaceCore::IModuleCommandVehicle
{

public:
    ModuleVehicleGeneric() :
        MaceCore::IModuleCommandVehicle(),
        m_VehicleDataTopic("vehicleData")
    {

    }

protected:

    Data::TopicDataObjectCollection<
        VehicleTopicAdditionalComponents...,
        DataVehicleGeneric::LocalPosition,
        DataVehicleGeneric::LocalVelocity,
        DataVehicleGeneric::GlobalPosition,
        DataVehicleGeneric::GlobalVelocity> m_VehicleDataTopic;
};

#endif // MODULE_VEHICLE_GENERIC_H
