#include "positional_aid.h"

namespace DataState {
//!
//! \brief PositionalAid::PositionalAid
//!
PositionalAid::PositionalAid()
{

}

//!
//! \brief PositionalAid::~PositionalAid
//!
PositionalAid::~PositionalAid()
{

}

//!
//! \brief PositionalAid::GlobalPositionToLocal
//! \param origin
//! \param position
//! \param local
//!
void PositionalAid::GlobalPositionToLocal(const StateGlobalPosition &origin, const StateGlobalPosition &position, StateLocalPosition &local)
{
    Eigen::Vector3f transform;
    origin.translationTransformation3D(position,transform);
    local.setPositionX(transform(0));
    local.setPositionY(transform(1));
    local.setPositionZ(transform(2));
}

//!
//! \brief PositionalAid::LocalPositionToGlobal
//! \param origin
//! \param position
//! \param global
//!
void PositionalAid::LocalPositionToGlobal(const StateGlobalPosition &origin, const StateLocalPosition &position, StateGlobalPosition &global)
{
    double bearing = position.bearingDegreesFromOrigin() * (M_PI/180.0);
    double distance = position.distanceFromOrigin();
    global.NewPositionFromHeadingBearing(distance,bearing,false);
}

} //end of namespace DataState
