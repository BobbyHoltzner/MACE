#ifndef TESTDUMMY_H
#define TESTDUMMY_H

#include "data/i_topic_component_data_object.h"

namespace DataVehicleGeneric
{

extern const char TestDummy_name[];
extern const MaceCore::TopicComponentStructure TestDummy_structure;

class TestDummy : public Data::NamedTopicComponentDataObject<TestDummy_name, &TestDummy_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

};

}


#endif // TESTDUMMY_H
