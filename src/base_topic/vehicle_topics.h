#ifndef VEHICLE_TOPICS_H
#define VEHICLE_TOPICS_H

#include <cassert>

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data/topic_components/altitude.h"
#include "data/topic_components/position_global.h"
#include "data/topic_components/position_local.h"
#include "data/topic_components/topic_component_string.h"
#include "data/topic_components/topic_component_void.h"

#include <functional>

#include "mace_core/topic_collection.h"

#include "common/pointer_collection.h"

#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"



namespace BaseTopic {





namespace VehicleTopicsNames {
    static constexpr char CommandName_Takeoff[] = "command_takeoff";
    static constexpr char CommandName_Land[] = "command_land";
    static constexpr char CommandName_SystemMode[] = "command_systemMode";
}

namespace VehicleTopicsTypes {
    using Topic_Takeoff = MaceCore::NonSpooledTopic<Data::TopicComponents::Void, Data::TopicComponents::Altitude, Data::TopicComponents::PositionGlobal, Data::TopicComponents::LocalPosition>;
    using Topic_Land = MaceCore::NonSpooledTopic<Data::TopicComponents::Void, Data::TopicComponents::PositionGlobal, Data::TopicComponents::LocalPosition>;
    using Topic_SystemMode = MaceCore::NonSpooledTopic<Data::TopicComponents::String>;
}


using VehicleTopicsTopicCollection = MaceCore::TopicCollection<
    MaceCore::NamedTopic<VehicleTopicsNames::CommandName_Takeoff, VehicleTopicsTypes::Topic_Takeoff>,
    MaceCore::NamedTopic<VehicleTopicsNames::CommandName_Land, VehicleTopicsTypes::Topic_Land>,
    MaceCore::NamedTopic<VehicleTopicsNames::CommandName_SystemMode, VehicleTopicsTypes::Topic_SystemMode>
>;


class VehicleTopicsBase : public VehicleTopicsTopicCollection
{
public:



public:

private:

public:

    VehicleTopicsBase()  :
        VehicleTopicsTopicCollection()
    {

    }

};







// Primary template (does not define key_type)
template<bool FORCE_COMPILETIME_IMPLIMENTATION, typename = void>
struct VehicleTopics : public VehicleTopicsBase
{

};

// Specialization using SFINAE to check for the existence of key() const
// (does define key_type)
template<bool FORCE_COMPILETIME_IMPLIMENTATION>
struct VehicleTopics<
    FORCE_COMPILETIME_IMPLIMENTATION,
    typename std::enable_if<!FORCE_COMPILETIME_IMPLIMENTATION>::type
    > : public VehicleTopicsBase
{

};



}

#endif // VEHICLE_TOPICS_H
