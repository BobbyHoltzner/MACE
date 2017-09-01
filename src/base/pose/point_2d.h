#ifndef POINT_2D_H
#define POINT_2D_H

#include "Eigen/Dense"
#include "point.h"

namespace mace {
namespace pose {

class Point2D
{
public:
    //!
    //! \brief Point2D
    //!
    Point2D();

    //!
    ~Point2D();

    //!
    //! \brief Point2D
    //! \param copy
    //!
    Point2D(const Point2D &copy);

    //!
    //! \brief Point2D
    //! \param x
    //! \param y
    //!
    Point2D(const double &x, const double &y);

public:
    /** Common among all point classes */

    bool is3D() const
    {
        return false;
    }


    //!
    //! \brief set2DPosition
    //! \param point2D
    //!
    void set2DPosition(const Point2D &point2D);

    //!
    //! \brief set2DPosition
    //! \param x
    //! \param y
    //!
    void set2DPosition(const double &x, const double &y);

    //!
    //! \brief setX
    //! \param posX
    //!
    void setX(const double &posX)
    {
        this->x = posX;
        this->posXFlag = true;
    }

    //!
    //! \brief getX
    //! \return
    //!
    double getX() const
    {
        return this->x;
    }

    //!
    //! \brief getPosXFlag
    //! \return
    //!
    bool getPosXFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief setY
    //! \param posY
    //!
    void setY(const double &posY)
    {
        this->y = posY;
        this->posYFlag = true;
    }

    //!
    //! \brief getY
    //! \return
    //!
    double getY() const
    {
        return this->y;
    }

    //!
    //! \brief getPosYFlag
    //! \return
    //!
    bool getPosYFlag() const
    {
        return this->posXFlag;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Point2D operator + (const Point2D &that) const
    {
        double newX = this->x + that.x;
        double newY = this->y + that.y;
        Point2D newPoint(newX, newY);
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Point2D operator - (const Point2D &that) const
    {
        double newX = this->x - that.x;
        double newY = this->y - that.y;
        Point2D newPoint(newX, newY);
        return newPoint;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Point2D &rhs) const
    {
        if(this->x >= rhs.x)
            return false;
        if(this->y >= rhs.y)
            return false;

        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Point2D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Point2D &rhs) const
    {
        if(this->x <= rhs.x)
            return false;
        if(this->y <= rhs.y)
            return false;

        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Point2D &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Point2D &rhs) const
    {
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->posXFlag != rhs.posXFlag){
            return false;
        }
        if(this->posYFlag != rhs.posYFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Point2D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Point2D& operator = (const Point2D &rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->posXFlag = rhs.posXFlag;
        this->posYFlag = rhs.posYFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Point2D& operator += (const Point2D &rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Point2D& operator -= (const Point2D &rhs)
    {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }


    /** Protected Members */
protected:
    //!
    //! \brief x
    //!
    double x = 0.0;

    //!
    //! \brief y
    //!
    double y = 0.0;

    //!
    //! \brief posXFlag
    //!
    bool posXFlag = 0.0;

    //!
    //! \brief posYFlag
    //!
    bool posYFlag = 0.0;
};

} //end of namespace pose
} //end of namespace mace

#endif // POINT_2D_H
