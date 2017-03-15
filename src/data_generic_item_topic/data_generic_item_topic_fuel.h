#ifndef DATA_GENERIC_ITEM_TOPIC_FUEL_H
#define DATA_GENERIC_ITEM_TOPIC_FUEL_H

#include "data_generic_item/data_generic_item_fuel.h"
#include "data/i_topic_component_data_object.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicFuel_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicFuel_structure;

class DataGenericItemTopic_Fuel : public DataGenericItem::DataGenericItem_Fuel, public Data::NamedTopicComponentDataObject<DataGenericItemTopicFuel_name, &DataGenericItemTopicFuel_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_FUEL_H
