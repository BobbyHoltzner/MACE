#include "command_vehicle_land.h"

namespace DataVehicleCommands {
template<>
void CommandVehicleLand<DataVehicleGeneric::GlobalPosition>::CommandVehicleLand(){
    m_PositionFrame = DataVehicleGeneric::PositionFrame::GLOBAL;
}

template<>
void CommandVehicleLand<DataVehicleGeneric::LocalPosition>::CommandVehicleLand(){
    m_PositionFrame = DataVehicleGeneric::PositionFrame::LOCAL;
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

