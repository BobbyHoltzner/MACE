#ifndef POLYGON_2DC_H
#define POLYGON_2DC_H

#include "base_polygon.h"
#include "list"
#include "base/pose/cartesian_position_2D.h"

namespace mace{
namespace geometry {

using namespace pose;

class Polygon_2DC : public PolygonBase<Position<CartesianPosition_2D>>
{
public:

    Polygon_2DC(const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_2DC(const std::vector<Position<CartesianPosition_2D>> &vector, const std::string &descriptor = "2D Cartesian Polygon");

    Polygon_2DC(const Polygon_2DC &copy);


    ~Polygon_2DC() = default;

    //!
    //! \brief getBoundingRect
    //! \return
    //!
    Polygon_2DC getBoundingRect() const;


    void getBoundingValues(double &xMin, double &yMin, double &xMax, double &yMax) const;

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
    bool contains(const double &x, const double &y, const bool &onLineCheck = false) const;

    //!
    //! \brief contains
    //! \param checkVector
    //! \param onLineCheck
    //! \return
    //!
    std::vector<bool> contains(std::vector<Position<CartesianPosition_2D>> &checkVector, const bool &onLineCheck = false);

    //!
    //! \brief getCenter
    //! \return
    //!
    Position<CartesianPosition_2D> getCenter() const;

public:
    double getXMin() const
    {
        return m_xMin;
    }

    double getYMin() const
    {
        return m_yMin;
    }

    double getXMax() const
    {
        return m_xMax;
    }

    double getYMax() const
    {
        return m_yMax;
    }

protected:
    void updateBoundingBox() override;

private:
    double m_xMin, m_xMax;
    double m_yMin, m_yMax;
};

} //end of namespace geometry
} //end of namespace mace

#endif // POLYGON_2DC_H
