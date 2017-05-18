#include "spatial_takeoff.h"

namespace CommandItem {
template <class T>
SpatialTakeoff<T>::SpatialTakeoff() :
    positionFlag(false)
{

}

template <class T>
Data::CommandItemType SpatialTakeoff<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_TAKEOFF;
}

template <class T>
std::string SpatialTakeoff<T>::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

template <class T>
bool SpatialTakeoff<T>::hasSpatialInfluence() const
{
    return true;
}

//____________________________________________________________________________
template<>
SpatialTakeoff<DataState::StateGlobalPosition>::SpatialTakeoff():
    positionFlag(false)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialTakeoff<DataState::StateLocalPosition>::SpatialTakeoff():
    positionFlag(false)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________
template<class T>
SpatialTakeoff<T>::SpatialTakeoff(const SpatialTakeoff<T> &obj):
    AbstractCommandItem(0,0),positionFlag(false)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialTakeoff<DataState::StateGlobalPosition>::SpatialTakeoff(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget),positionFlag(false)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialTakeoff<DataState::StateLocalPosition>::SpatialTakeoff(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget),positionFlag(false)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________


}

template class CommandItem::SpatialTakeoff<DataState::StateGlobalPosition>;
template class CommandItem::SpatialTakeoff<DataState::StateLocalPosition>;
