#include "dynamics_aid.h"

namespace mace {
namespace pose {


// **** 3D to 3D **** //
//!
//! \brief GlobalPositionToLocal Convert global position to local coordinate frame (3D global to 3D local)
//! \param origin 3D Geodetic global origin
//! \param position 3D Geodetic global position
//! \param local 3D Cartesian local position container
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local) {
    DynamicsAid m_this;
    m_this.globalPositionToLocal(origin, position, local);
}

//!
//! \brief LocalPositionToGlobal Convert local position to global coordinate frame (3D local to 3D global)
//! \param origin 3D Geodetic global origin
//! \param position 3D Cartesian local position
//! \param global 3D Geodetic global position container
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global) {
    DynamicsAid m_this;
    m_this.localPositionToGlobal(origin, position, global);
}


// **** 2D to 2D **** //
//!
//! \brief GlobalPositionToLocal Convert global position to local coordinate frame (2D global to 2D local)
//! \param origin 3D Geodetic global origin
//! \param position 2D Geodetic global position
//! \param local 2D Cartesian local position container
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_2D &position, CartesianPosition_2D &local) {
//    GeodeticPosition_3D origin3D(origin.getLatitude(), origin.getLongitude(), 0.0);
    GeodeticPosition_3D position3D(position.getLatitude(), position.getLongitude(), 0.0);
    CartesianPosition_3D local3D;
    DynamicsAid m_this;
    m_this.globalPositionToLocal(origin, position3D, local3D);
    local.setXPosition(local3D.getXPosition());
    local.setYPosition(local3D.getYPosition());
}

//!
//! \brief LocalPositionToGlobal Convert local position to global coordinate frame (2D local to 2D global)
//! \param origin 3D Geodetic global origin
//! \param position 2D Cartesian local position
//! \param global 2D Geodetic global position container
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_2D &position, GeodeticPosition_2D &global) {
//    GeodeticPosition_3D origin3D(origin.getLatitude(), origin.getLongitude(), 0.0);
    CartesianPosition_3D position3D(position.getXPosition(), position.getYPosition(), 0.0);
    GeodeticPosition_3D global3D;
    DynamicsAid m_this;
    m_this.localPositionToGlobal(origin, position3D, global3D);
    global.setLatitude(global3D.getLatitude());
    global.setLongitude(global3D.getLongitude());
}


// **** 3D to 2D **** //
//!
//! \brief GlobalPositionToLocal Convert global position to local coordinate frame (3D global to 2D local)
//! \param origin 3D Geodetic global origin
//! \param position 3D Geodetic global position
//! \param local 2D Cartesian local position container
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_2D &local) {
    CartesianPosition_3D local3D;
    DynamicsAid m_this;
    m_this.globalPositionToLocal(origin, position, local3D);
    local.setXPosition(local3D.getXPosition());
    local.setYPosition(local3D.getYPosition());
}

//!
//! \brief LocalPositionToGlobal Convert local position to global coordinate frame (3D local to 2D global)
//! \param origin 3D Geodetic global origin
//! \param position 3D Cartesian local position
//! \param global 2D Geodetic global position container
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_2D &global) {
    GeodeticPosition_3D global3D;
    DynamicsAid m_this;
    m_this.localPositionToGlobal(origin, position, global3D);
    global.setLatitude(global3D.getLatitude());
    global.setLongitude(global3D.getLongitude());
}


// **** 2D to 3D **** //
//!
//! \brief GlobalPositionToLocal Convert global position to local coordinate frame (2D global to 3D local)
//! \param origin 3D Geodetic global origin
//! \param position 2D Geodetic global position
//! \param local 3D Cartesian local position container
//!
void DynamicsAid::GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_2D &position, CartesianPosition_3D &local) {
//    GeodeticPosition_3D origin3D(origin.getLatitude(), origin.getLongitude(), 0.0);
    GeodeticPosition_3D position3D(position.getLatitude(), position.getLongitude(), 0.0);
    DynamicsAid m_this;
    m_this.globalPositionToLocal(origin, position3D, local);
}

//!
//! \brief LocalPositionToGlobal Convert local position to global coordinate frame (2D local to 3D global)
//! \param origin 3D Geodetic global origin
//! \param position 2D Cartesian local position
//! \param global 3D Geodetic global position container
//!
void DynamicsAid::LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_2D &position, GeodeticPosition_3D &global) {
//    GeodeticPosition_3D origin3D(origin.getLatitude(), origin.getLongitude(), 0.0);
    CartesianPosition_3D position3D(position.getXPosition(), position.getYPosition(), 0.0);
    DynamicsAid m_this;
    m_this.localPositionToGlobal(origin, position3D, global);
}

//!
//! \brief globalPositionToLocal Convert global position to local position
//! \param origin Global origin
//! \param position Global position to convert
//! \param local Local Cartesian position container
//!
void DynamicsAid::globalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local)
{
    double distance = origin.distanceBetween2D(position);
    double bearing = origin.compassBearingTo(position);
    double deltaAltitude = origin.deltaAltitude(position);
    local.applyPositionalShiftFromCompass(distance,convertDegreesToRadians(bearing));
    local.setZPosition(local.getZPosition() + deltaAltitude);
}

//!
//! \brief localPositionToGlobal Convert local position to global position
//! \param origin Global origin
//! \param position Cartesian position to convert
//! \param global Global Geodetic position container
//!
void DynamicsAid::localPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global)
{
    double distance = position.distanceFromOrigin();
    double bearing = position.polarBearingFromOrigin();
    double elevation = position.elevationFromOrigin();
    global = origin.newPositionFromPolar(distance,bearing,elevation);
}

} //end of namespace pose
} //end of namespace mace
