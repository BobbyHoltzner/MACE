#ifndef I_MAVLINK_PARSER_H
#define I_MAVLINK_PARSER_H


#include <iostream>
#include <functional>

#include "mavlink.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_container_mavlink.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "MAVLINK_to_MACE/generic_mavlink_to_mace.h"
#include "MAVLINK_to_MACE/state_mavlink_to_mace.h"

namespace DataMAVLINK
{

class MAVLINKParser
{
public:
    MAVLINKParser(DataContainer_MAVLINK *dataContainer);
    virtual std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> ParseForVehicleData(const mavlink_message_t* message);

protected:
    DataContainer_MAVLINK* data;
};

}
#endif // I_MAVLINK_PARSER_H

