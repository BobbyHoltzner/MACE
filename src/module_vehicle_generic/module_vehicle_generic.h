#ifndef MODULE_VEHICLE_GENERIC_H
#define MODULE_VEHICLE_GENERIC_H

#include "module_vehicle_generic_global.h"

#include <functional>

#include "common/common.h"


#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_vehicle.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_vehicle_generic/local_position.h"
#include "data_vehicle_generic/local_velocity.h"
#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/global_velocity.h"

#include "data_vehicle_generic/components.h"


template <typename ...VehicleTopicAdditionalComponents>
class MODULE_VEHICLE_GENERICSHARED_EXPORT ModuleVehicleGeneric : public MaceCore::IModuleCommandVehicle
{
public:

    typedef Data::TopicDataObjectCollection<
    VehicleTopicAdditionalComponents...,
    DATA_VEHICLE_GENERIC_TYPES> VehicleDataTopicType;

    ModuleVehicleGeneric() :
        MaceCore::IModuleCommandVehicle(),
        m_VehicleDataTopic("vehicleData")
    {

    }

public:

    VehicleDataTopicType m_VehicleDataTopic;
};



#endif // MODULE_VEHICLE_GENERIC_H
