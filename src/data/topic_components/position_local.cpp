#include "position_local.h"

namespace Data {

namespace TopicComponents
{

const char TopicComponts_LocalPosition_name[] = "position_local";
const MaceCore::TopicComponentStructure TopicComponts_LocalPosition_structure = []{
    return TopicComponentPrototypes::PositionCartesian3D_structure;
}();


MaceCore::TopicDatagram LocalPosition::GenerateDatagram() const
{
    return TopicComponentPrototypes::PositionCartesian3D::GenerateDatagram();
}

void LocalPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    TopicComponentPrototypes::PositionCartesian3D::CreateFromDatagram(datagram);
}

LocalPosition::LocalPosition(const double &x, const double &y, const double &z, const ReferenceCartesian &ref) :
    TopicComponentPrototypes::PositionCartesian3D(x,y,z,ref)
{
}

LocalPosition::LocalPosition(const LocalPosition &copyObj) :
    TopicComponentPrototypes::PositionCartesian3D(copyObj)
{

}

} // TopicComponents

} // Data
