#ifndef CALLBACK_INTERFACE_DATA_MAVLINK_H
#define CALLBACK_INTERFACE_DATA_MAVLINK_H

#include <memory>

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

namespace DataInterface_MAVLINK{


class CallbackInterface_DataMAVLINK
{
public:
    CallbackInterface_DataMAVLINK();

    virtual cbi_VehicleStateData(std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;
    virtual cbi_VehicleMissionData(std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;


public:

};

} //end of namespace DataInterface_MAVLINK
#endif // CALLBACK_INTERFACE_DATA_MAVLINK_H
