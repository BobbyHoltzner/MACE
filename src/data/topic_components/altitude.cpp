#include "altitude.h"

namespace Data {

namespace TopicComponents
{

const char TopicComponts_Altitude_name[] = "altitude";
const MaceCore::TopicComponentStructure TopicComponts_Altitude_structure = []{
    return TopicComponentPrototypes::TopicPrototype_Altitude_structure;
}();

MaceCore::TopicDatagram Altitude::GenerateDatagram() const
{
    return TopicComponentPrototypes::Altitude::GenerateDatagram();
}

void Altitude::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    TopicComponentPrototypes::Altitude::CreateFromDatagram(datagram);
}

Altitude::Altitude()
{

}

Altitude::Altitude(const double &altitude, const ReferenceAltitude &ref) :
    TopicComponentPrototypes::Altitude(altitude, ref)
{

}

Altitude::Altitude(const Altitude &copyObj) :
    TopicComponentPrototypes::Altitude(copyObj)
{

}

} // TopicComponents

} // Data
