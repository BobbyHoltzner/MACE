#include "local_velocity_topic.h"

#include <math.h>

namespace DataStateTopic
{

const char LocalVelocity_name[] = "local_velocity";
const MaceCore::TopicComponentStructure LocalVelocity_structure = [](){
        MaceCore::TopicComponentStructure structure;
        structure.AddTerminal<double>("x");
        structure.AddTerminal<double>("y");
        structure.AddTerminal<double>("z");

        return structure;
    }();


MaceCore::TopicDatagram LocalVelocityTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("x", x);
    datagram.AddTerminal<double>("y", y);
    datagram.AddTerminal<double>("z", z);

    return datagram;
}


void LocalVelocityTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    x = datagram.GetTerminal<double>("x");
    y = datagram.GetTerminal<double>("y");
    z = datagram.GetTerminal<double>("z");
}

}
