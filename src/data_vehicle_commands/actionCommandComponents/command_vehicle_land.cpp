#include "command_vehicle_land.h"

namespace DataVehicleCommands {

const char CommandVehicleLand_Name[] = "CommandVehicleLand";

const MaceCore::TopicComponentStructure CommandVehicleLand_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("x");
    return structure;
}();

template<>
MaceCore::TopicDatagram CommandVehicleLand<DataVehicleGeneric::LocalPosition>::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("pitch", m_pitch);
    datagram.AddTerminal<double>("yawAngle", m_yawAngle);
    datagram.AddTerminal<double>("x", m_Location.x);
    datagram.AddTerminal<double>("y", m_Location.y);
    datagram.AddTerminal<double>("z", m_Location.z);
    return datagram;
}
template<>
MaceCore::TopicDatagram CommandVehicleLand<DataVehicleGeneric::GlobalPosition>::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("pitch", m_pitch);
    datagram.AddTerminal<double>("yawAngle", m_yawAngle);
    datagram.AddTerminal<double>("latitude", m_Location.latitude);
    datagram.AddTerminal<double>("longitude", m_Location.longitude);
    datagram.AddTerminal<std::unordered_map<int, double>>("altitude", m_Location.altitude);
    return datagram;
}


template<>
void CommandVehicleLand<DataVehicleGeneric::GlobalPosition>::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_pitch = datagram.GetTerminal<double>("pitch");
    m_yawAngle = datagram.GetTerminal<double>("yawAngle");
    m_Location.latitude = datagram.GetTerminal<double>("latitude");
    m_Location.longitude = datagram.GetTerminal<double>("longitude");
    m_Location.altitude = datagram.GetTerminal<std::unordered_map<int, double>>("altitude");
}
template<>
void CommandVehicleLand<DataVehicleGeneric::LocalPosition>::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_pitch = datagram.GetTerminal<double>("pitch");
    m_yawAngle = datagram.GetTerminal<double>("yawAngle");
    m_Location.x = datagram.GetTerminal<double>("x");
    m_Location.y = datagram.GetTerminal<double>("y");
    m_Location.z = datagram.GetTerminal<double>("z");
}


template <>
void CommandVehicleLand<DataVehicleGeneric::GlobalPosition>::setLocation(const DataVehicleGeneric::GlobalPosition &location)
{
    m_Location.latitude = location.latitude;
    m_Location.longitude = location.longitude;
    m_Location.altitude = location.altitude;
}
template <>
void CommandVehicleLand<DataVehicleGeneric::LocalPosition>::setLocation(const DataVehicleGeneric::LocalPosition &location)
{
    m_Location.x = location.x;
    m_Location.y = location.y;
    m_Location.z = location.z;
}


}

