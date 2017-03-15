#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H


#include <iostream>
#include <functional>
#include "mace_core/mace_core.h"

#include "mavlink.h"
#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

namespace DataMAVLINK
{


class MAVLINKParser
{
public:

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForData(const mavlink_message_t* message,  const std::shared_ptr<const MaceCore::MaceData>);


    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> Parse(const mavlink_message_t* message) const{
        UNUSED(message);
        return {};
    }

};

}
#endif // I_MAVLINK_PARSER_H

