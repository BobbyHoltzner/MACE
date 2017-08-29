#ifndef POINT_2D_H
#define POINT_2D_H

#include "abstract_point.h"

namespace base {
namespace pose {

class Point2D : public AbstractPoint
{
public:
    //!
    //! \brief Point2D
    //!
    Point2D();

    ~Point2D();

    //!
    //! \brief Point2D
    //! \param copy
    //!
    Point2D(const Point2D &copy);

    //!
    //! \brief Point2D
    //! \param posX
    //! \param posY
    //!
    Point2D(const double &posX, const double &posY);

public:
    virtual bool is3D() const
    {
       return false;
    }

    virtual Eigen::Vector2d get2DVector() const
    {
        using namespace Eigen;
        Vector2d vector(this->x, this->y);
        return vector;
    }

    virtual Eigen::Vector3d get3DVector() const
    {
        using namespace Eigen;
        Vector3d vector(this->x, this->y, 0.0);
        return vector;
    }
public:
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

    void setPosition2D(const double &posX, const double &posY)
    {
        this->setX(posX);
        this->setY(posY);
    }

    //!
    //! \brief has2DPositionSet
    //! \return
    //!
    bool has2DPositionSet() const
    {
        return this->posXFlag && this->posYFlag;
    }

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
    bool posXFlag = false;

    //!
    //! \brief posYFlag
    //!
    bool posYFlag = false;
};

} //end of namespace pose
} //end of namespace base

#endif // POINT_2D_H
