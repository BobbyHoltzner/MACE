#ifndef DYNAMICS_AID_H
#define DYNAMICS_AID_H

#include "../math/helper_pi.h"

#include "cartesian_position_2D.h"
#include "cartesian_position_3D.h"

#include "geodetic_position_2D.h"
#include "geodetic_position_3D.h"

namespace mace {
namespace pose {

class DynamicsAid
{
public:
    DynamicsAid() = default;

    ~DynamicsAid() = default;


    // ******************************* //
    // **** Geodetic/Cartesian **** //
    // ******************************* //
    // **** 3D to 3D **** //
    //!
    //! \brief GlobalPositionToLocal Convert global position to local coordinate frame (3D global to 3D local)
    //! \param origin 3D Geodetic global origin
    //! \param position 3D Geodetic global position
    //! \param local 3D Cartesian local position container
    //!
    static void GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local);

    //!
    //! \brief LocalPositionToGlobal Convert local position to global coordinate frame (3D local to 3D global)
    //! \param origin 3D Geodetic global origin
    //! \param position 3D Cartesian local position
    //! \param global 3D Geodetic global position container
    //!
    static void LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global);


    // **** 2D to 2D **** //
    //!
    //! \brief GlobalPositionToLocal Convert global position to local coordinate frame (2D global to 2D local)
    //! \param origin 3D Geodetic global origin
    //! \param position 2D Geodetic global position
    //! \param local 2D Cartesian local position container
    //!
    static void GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_2D &position, CartesianPosition_2D &local);

    //!
    //! \brief LocalPositionToGlobal Convert local position to global coordinate frame (2D local to 2D global)
    //! \param origin 3D Geodetic global origin
    //! \param position 2D Cartesian local position
    //! \param global 2D Geodetic global position container
    //!
    static void LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_2D &position, GeodeticPosition_2D &global);


    // **** 3D to 2D **** //
    //!
    //! \brief GlobalPositionToLocal Convert global position to local coordinate frame (3D global to 2D local)
    //! \param origin 3D Geodetic global origin
    //! \param position 3D Geodetic global position
    //! \param local 2D Cartesian local position container
    //!
    static void GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_2D &local);

    //!
    //! \brief LocalPositionToGlobal Convert local position to global coordinate frame (3D local to 2D global)
    //! \param origin 3D Geodetic global origin
    //! \param position 3D Cartesian local position
    //! \param global 2D Geodetic global position container
    //!
    static void LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_2D &global);


    // **** 2D to 3D **** //
    //!
    //! \brief GlobalPositionToLocal Convert global position to local coordinate frame (2D global to 3D local)
    //! \param origin 3D Geodetic global origin
    //! \param position 2D Geodetic global position
    //! \param local 3D Cartesian local position container
    //!
    static void GlobalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_2D &position, CartesianPosition_3D &local);

    //!
    //! \brief LocalPositionToGlobal Convert local position to global coordinate frame (2D local to 3D global)
    //! \param origin 3D Geodetic global origin
    //! \param position 2D Cartesian local position
    //! \param global 3D Geodetic global position container
    //!
    static void LocalPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_2D &position, GeodeticPosition_3D &global);

private:

    //!
    //! \brief globalPositionToLocal Convert global position to local position
    //! \param origin Global origin
    //! \param position Global position to convert
    //! \param local Local Cartesian position container
    //!
    void globalPositionToLocal(const GeodeticPosition_3D &origin, const GeodeticPosition_3D &position, CartesianPosition_3D &local);

    //!
    //! \brief localPositionToGlobal Convert local position to global position
    //! \param origin Global origin
    //! \param position Cartesian position to convert
    //! \param global Global Geodetic position container
    //!
    void localPositionToGlobal(const GeodeticPosition_3D &origin, const CartesianPosition_3D &position, GeodeticPosition_3D &global);

};

} //end of namespace pose
} //end of namespace mace

#endif // DYNAMICS_AID_H
