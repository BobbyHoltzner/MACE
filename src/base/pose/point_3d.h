#ifndef POINT_3D_H
#define POINT_3D_H

#include "Eigen/Dense"
#include "point_2d.h"

namespace mace {
namespace pose {

class Point3D : public Point2D
{

public:
    //!
    //! \brief Point3D
    //!
    Point3D();

    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point2D &copy);

    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point2D &copy, const double &z);


    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point3D &copy);


    //!
    //! \brief Point2D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point3D(const double &posX, const double &posY, const double &posZ);


    Point2D get2D() const;

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Point3D operator + (const Point3D &that) const
    {
        Point2D new2D = Point2D::operator +(that);
        double newZ = this->z + that.z;
        Point3D newPoint(new2D, newZ);
        return newPoint;
    }

    friend Point3D operator + (const Point2D &lhs, const Point3D &rhs);

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Point3D operator - (const Point3D &that) const
    {
        Point2D new2D = Point2D::operator -(that);
        double newZ = this->z - that.z;
        Point3D newPoint(new2D, newZ);
        return newPoint;
    }

    friend Point3D operator - (const Point2D &lhs, const Point3D &rhs);


    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Point3D &rhs) const
    {
        if(!Point2D::operator <(rhs))
            return false;
        if(this->z >= rhs.z)
            return false;
        return true;
    }

    friend bool operator < (const Point2D &lhs, const Point3D &rhs);

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Point3D &rhs) const
    {
        return !(*this < rhs);
    }

    friend bool operator > (const Point2D &lhs, const Point3D &rhs);

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Point3D &rhs) const
    {
        if(!Point2D::operator ==(rhs))
            return false;
        if(this->z != rhs.z){
            return false;
        }
        if(this->posZFlag != rhs.posZFlag){
            return false;
        }
        return true;
    }

    friend bool operator == (const Point2D &lhs, const Point3D &rhs);

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Point3D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Point3D& operator = (const Point3D &rhs)
    {
        Point2D::operator =(rhs);
        this->z = rhs.z;
        this->posZFlag = rhs.posZFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Point3D& operator += (const Point3D &rhs)
    {
        Point2D::operator +=(rhs);
        this->z += rhs.z;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Point3D& operator -= (const Point3D &rhs)
    {
        Point2D::operator -=(rhs);
        this->z -= rhs.z;
        return *this;
    }


public:
    //!
    //! \brief setZ
    //! \param posZ
    //!
    void setZ(const double &posZ)
    {
        this->z = posZ;
        this->posZFlag = true;
    }

    //!
    //! \brief getZ
    //! \return
    //!
    double getZ() const
    {
        return this->z;
    }

    //!
    //! \brief getPosYFlag
    //! \return
    //!
    bool getPosZFlag() const
    {
        return this->posZFlag;
    }

    /** Protected Members */
protected:
    //!
    //! \brief x
    //!
    double z = 0.0;

    //!
    //! \brief posYFlag
    //!
    bool posZFlag = 0.0;

};


} //end of namespace pose
} //end of namespace base

#endif // POINT_3D_H
