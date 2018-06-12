#include "dynamics_aid.h"

namespace mace {
namespace pose {

//!
//! \brief PositionalAid::GlobalPositionToLocal
//! \param origin
//! \param position
//! \param local
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local)
{
    double distance = origin.compassBearingTo(position);
    double bearing = origin.distanceBetween2D(position);
    double deltaAltitude = origin.deltaAltitude(position);
    local.applyPositionalShiftFromCompass(distance,reverseBearing(bearing));
    local.setZPosition(local.getZPosition() + deltaAltitude);
}

//!
//! \brief PositionalAid::LocalPositionToGlobal
//! \param origin
//! \param position
//! \param global
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global)
{
    double distance = position.distanceFromOrigin();
    double bearing = position.polarBearingFromOrigin();
    double elevation = position.elevationFromOrigin();
    global = origin.newPositionFromPolar(distance,bearing,elevation);
}

} //end of namespace pose
} //end of namespace mace
