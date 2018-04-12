#include "topic_component_boolean.h"

#include "common/common.h"

namespace Data {

namespace TopicComponents
{


const char TopicComponts_Boolean_name[] = "bool";
const MaceCore::TopicComponentStructure TopicComponts_Boolean_structure = []{
    MaceCore::TopicComponentStructure structure;

    structure.AddTerminal<bool>("boolean");

    return structure;
}();


MaceCore::TopicDatagram Boolean::GenerateDatagram() const
{
    MaceCore::TopicDatagram diagram;
    diagram.AddTerminal("Boolean", m_value);
    return diagram;
}


void Boolean::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_value = datagram.GetTerminal<bool>("boolean");
}

Boolean::Boolean() :
    m_value(false)
{

}

Boolean::Boolean(const bool value) :
    m_value(value)
{

}


} // BaseTopics

}
