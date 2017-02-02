#include "attitude.h"

namespace DataVehicleGeneric
{

const char Attitude_name[] = "attitude";
const MaceCore::TopicComponentStructure Attitude_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("roll");
    structure.AddTerminal<double>("rollRate");
    structure.AddTerminal<double>("pitch");
    structure.AddTerminal<double>("pitchRate");
    structure.AddTerminal<double>("yaw");
    structure.AddTerminal<double>("yawRate");

    return structure;
}();

MaceCore::TopicDatagram Attitude::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("roll", roll);
    datagram.AddTerminal<double>("rollRate", rollRate);
    datagram.AddTerminal<double>("pitch", pitch);
    datagram.AddTerminal<double>("pitchRate", pitchRate);
    datagram.AddTerminal<double>("yaw", yaw);
    datagram.AddTerminal<double>("yawRate", yawRate);
    return datagram;
}

void Attitude::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    roll = datagram.GetTerminal<double>("roll");
    rollRate = datagram.GetTerminal<double>("rollRate");
    pitch = datagram.GetTerminal<double>("pitch");
    pitchRate = datagram.GetTerminal<double>("pitchRate");
    yaw = datagram.GetTerminal<double>("yaw");
    yawRate = datagram.GetTerminal<double>("yawRate");
}

void Attitude::setAttitude(const double &roll, const double &pitch, const double &yaw)
{
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
}

void Attitude::setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate)
{
    this->rollRate = rollRate;
    this->pitchRate =pitchRate;
    this->yawRate = yawRate;
}
} //end of namespace DataVehicleGeneric
