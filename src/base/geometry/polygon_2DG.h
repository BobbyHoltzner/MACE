#ifndef POLYGON_2DG_H
#define POLYGON_2DG_H

#include "base_polygon.h"
#include "polygon_2DC.h"

#include "list"
#include "base/pose/geodetic_position_2D.h"
#include "base/pose/dynamics_aid.h"

namespace mace{
namespace geometry {

using namespace pose;

class Polygon_2DG : public PolygonBase<Position<GeodeticPosition_2D>>
{
public:

    Polygon_2DG(const std::string &descriptor = "2D Geodetic Polygon");

    Polygon_2DG(const std::vector<Position<GeodeticPosition_2D>> &vector, const std::string &descriptor = "2D Geodetic Polygon");

    Polygon_2DG(const Polygon_2DG &copy);


    ~Polygon_2DG() = default;

    //!
    //! \brief getBoundingRect
    //! \return
    //!
    Polygon_2DG getBoundingRect() const;


    void getBoundingValues(double &xMin, double &minY, double &maxX, double &maxY) const;

    //!
    //! \brief contains
    //! \param point
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const Position<GeodeticPosition_2D> &point, const bool &onLineCheck = false) const;

    //!
    //! \brief contains
    //! \param x
    //! \param y
    //! \param onLineCheck
    //! \return
    //!
    bool contains(const double &latitude, const double &longitude, const bool &onLineCheck = false) const;

    //!
    //! \brief contains
    //! \param checkVector
    //! \param onLineCheck
    //! \return
    //!
    std::vector<bool> contains(std::vector<Position<GeodeticPosition_2D>> &checkVector, const bool &onLineCheck = false);

    //!
    //! \brief getCenter
    //! \return
    //!
    Position<GeodeticPosition_2D> getCenter() const;

    std::vector<int> findUndefinedVertices() const override
    {
        int index = 0;
        std::vector<int> nullItems;
        for(std::vector<Position<GeodeticPosition_2D>>::const_iterator it = m_vertex.begin(); it != m_vertex.end(); ++it) {
            if(!it->hasLatitudeBeenSet() && !it->hasLongitudeBeenSet())
            {
                //This should see that the value is null
                nullItems.push_back(index);
            }
            index++;
        }
        return nullItems;
    }

public:
    Position<GeodeticPosition_2D> getTopLeft() const override;
    Position<GeodeticPosition_2D> getTopRight() const override;

    Position<GeodeticPosition_2D> getBottomLeft() const override;
    Position<GeodeticPosition_2D> getBottomRight() const override;

    void getCorners(Position<GeodeticPosition_2D> &topLeft, Position<GeodeticPosition_2D> &bottomRight) const override;

    mace::pose::CoordinateFrame getVertexCoordinateFrame() const override;

    void applyCoordinateShift(const double &distance, const double &bearing);

public:
    double getXMin() const
    {
        return xMin;
    }

    double getYMin() const
    {
        return yMin;
    }

    double getXMax() const
    {
        return xMax;
    }

    double getYMax() const
    {
        return yMax;
    }

protected:
    void updateBoundingBox() override;

    /** Assignment Operators **/
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Polygon_2DG& operator = (const Polygon_2DG &rhs)
    {
        PolygonBase::operator =(rhs);
        this->xMin = rhs.xMin;
        this->yMin = rhs.yMin;
        this->xMax = rhs.xMax;
        this->yMax = rhs.yMax;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Polygon_2DG &rhs) const
    {
        if(!PolygonBase<Position<GeodeticPosition_2D>>::operator ==(rhs))
        {
            return false;
        }
        if(this->xMin != rhs.xMin)
        {
            return false;
        }
        if(this->xMax != rhs.xMax)
        {
            return false;
        }
        if(this->yMin != rhs.yMin)
        {
            return false;
        }
        if(this->yMax != rhs.yMax)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Polygon_2DG &rhs) const {
        return !(*this == rhs);
    }

private:
    double xMin, xMax;
    double yMin, yMax;

private:
    Polygon_2DC m_localPolygon;
};

} //end of namespace geometry
} //end of namespace mace

#endif // POLYGON_2DC_H
