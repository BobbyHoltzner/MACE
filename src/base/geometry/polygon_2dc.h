#ifndef POLYGON_2DC_H
#define POLYGON_2DC_H

#include "base_polygon.h"

#include "base/pose/cartesian_position_2D.h"

namespace mace{
namespace geometry {

using namespace pose;

class Polygon_2DC : public PolygonBase<Position<CartesianPosition_2D>>
{
public:

    Polygon_2DC(const std::string &descriptor = "2D Cartesian Polygon");

    ~Polygon_2DC() = default;

    //!
    //! \brief getBoundingRect
    //! \return
    //!
    Polygon_2DC getBoundingRect() const;

    //!
    //! \brief contains
    //! \param point
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const Position<CartesianPosition_2D> &point, const bool &onLineCheck = false);

    //!
    //! \brief contains
    //! \param x
    //! \param y
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const double &x, const double &y, const bool &onLineCheck = false);


    //!
    //! \brief getCenter
    //! \return
    //!
    Position<CartesianPosition_2D> getCenter() const;
};

} //end of namespace geometry
} //end of namespace mace

#endif // POLYGON_2DC_H
