#include "vehicle_target_topic.h"

namespace MissionTopic{

const char VehicleTargetTopic_name[] = "VehicleTarget";
const MaceCore::TopicComponentStructure VehicleTargetTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("systemID");
    structure.AddTerminal<DataState::Base3DPosition>("targetPosition");
    structure.AddTerminal<double>("targetDistance");
    return structure;
}();

MaceCore::TopicDatagram VehicleTargetTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("systemID",systemID);
    datagram.AddTerminal<DataState::Base3DPosition>("targetPosition", targetPosition);
    datagram.AddTerminal<double>("targetDistance", targetDistance);
    return datagram;
}

void VehicleTargetTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    systemID = datagram.GetTerminal<int>("systemID");
    targetPosition = datagram.GetTerminal<DataState::Base3DPosition>("targetPosition");
    targetDistance = datagram.GetTerminal<double>("targetDistance");
}

VehicleTargetTopic::VehicleTargetTopic(const int &vehicleID, const DataState::Base3DPosition &position, const double &distance):
    systemID(vehicleID), targetPosition(position), targetDistance(distance)
{

}


VehicleTargetTopic::VehicleTargetTopic(const VehicleTargetTopic &target)
{
    this->systemID = target.systemID;
    this->targetPosition = target.targetPosition;
    this->targetDistance = target.targetDistance;
}

std::ostream& operator<<(std::ostream& os, const VehicleTargetTopic& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Target Topic for system " << t.systemID << ": " << t.targetPosition.getX() << ", "<< t.targetPosition.getY() << ", "<< t.targetPosition.getZ() << ", "<< t.targetDistance<< ".";
    os << stream.str();

    return os;
}


} //end of namespace MissionTopic
