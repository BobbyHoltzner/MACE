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

VehicleTargetTopic::VehicleTargetTopic() :
    systemID(0), targetDistance(0.0)
{

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

VehicleTargetTopic::VehicleTargetTopic(const mace_guided_target_stats_t &obj)
{
    this->targetPosition.setCoordinateFrame(static_cast<Data::CoordinateFrameType>(obj.coordinate_frame));
    this->targetDistance = obj.distance;
    this->targetPosition.setX(obj.x);
    this->targetPosition.setY(obj.y);
    this->targetPosition.setZ(obj.z);
}

mace_guided_target_stats_t VehicleTargetTopic::getMACECommsObject() const
{
    mace_guided_target_stats_t rtn;
    rtn.coordinate_frame = (uint8_t)this->targetPosition.getCoordinateFrame();
    rtn.distance = this->targetDistance;
    rtn.x = this->targetPosition.getX();
    rtn.y = this->targetPosition.getY();
    rtn.z = this->targetPosition.getZ();
    return rtn;
}
mace_message_t VehicleTargetTopic::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_guided_target_stats_t target = getMACECommsObject();
    mace_message_t msg;
    mace_msg_guided_target_stats_encode_chan(systemID,compID,chan,&msg,&target);
    return msg;
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
