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
    datagram.AddTerminal<double>("roll", m_roll);
    datagram.AddTerminal<double>("rollRate", m_rollRate);
    datagram.AddTerminal<double>("pitch", m_pitch);
    datagram.AddTerminal<double>("pitchRate", m_pitchRate);
    datagram.AddTerminal<double>("yaw", m_yaw);
    datagram.AddTerminal<double>("yawRate", m_yawRate);
    return datagram;
}

void Attitude::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_roll = datagram.GetTerminal<double>("roll");
    m_rollRate = datagram.GetTerminal<double>("rollRate");
    m_pitch = datagram.GetTerminal<double>("pitch");
    m_pitchRate = datagram.GetTerminal<double>("pitchRate");
    m_yaw = datagram.GetTerminal<double>("yaw");
    m_yawRate = datagram.GetTerminal<double>("yawRate");
}

void Attitude::setRoll(const double &roll)
{
    m_roll = roll;
}

double Attitude::getRoll(){
    return m_roll;
}


} //end of namespace DataVehicleGeneric
