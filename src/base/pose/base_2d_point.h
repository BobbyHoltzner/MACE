#ifndef POINT_2D_H
#define POINT_2D_H

#include "point.h"

namespace base {
namespace pose {

class Point2D : public Point<Point2D>
{
public:
    //!
    //! \brief Point2D
    //!
    Point2D():
        BasePoint()
    {

    }

    //!
    //! \brief Point2D
    //! \param copy
    //!
    Point2D(const Point_2D &copy):
        BasePoint(copy)
    {

    }

    //!
    //! \brief Point2D
    //! \param coordinateFrame
    //!
    Point2D(const CoordinateFrameType &coordinateFrame):
        BasePoint(coordinateFrame)
    {

    }

    //!
    //! \brief Point2D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point2D(const double &posX, const double &posY):
        BasePoint(posX,posY)
    {

    }

    //!
    //! \brief Point2D
    //! \param coordinateFrame
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point2D(const CoordinateFrameType &coordinateFrame, const double &posX, const double &posY):
        BasePoint(coordinateFrame, posX, posY)
    {

    }

public:
    void setPosition2D(const double &posX, const double &posY)
    {
        this->setX(posX);
        this->setY(posY);
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    Point2D& operator = (const BasePoint &rhs)
    {
        BasePoint::operator =(rhs);
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const BasePoint &rhs) {
        if(!BasePoint::operator ==(rhs)){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const BasePoint &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace pose
} //end of namespace base

#endif // POINT_2D_H
