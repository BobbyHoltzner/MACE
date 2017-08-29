#ifndef POINT_3D_H
#define POINT_3D_H

#include "point_2d.h"
namespace base {
namespace pose {

class Point3D : public Point2D
{

public:
    //!
    //! \brief Point3D
    //!
    Point3D();

    ~Point3D();

    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point3D &copy);

    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point2D &copy);

    //!
    //! \brief Point3D
    //! \param copy
    //! \param posZ
    //!
    Point3D(const Point2D &copy, const double &posZ);

    //!
    //! \brief Point3D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point3D(const double &posX, const double &posY, const double &posZ);

public:

    virtual bool is3D() const
    {
       return true;
    }

    virtual Eigen::Vector3d get3DVector() const
    {
        using namespace Eigen;
        Vector3d vector(this->x, this->y, this->z);
        return vector;
    }

public:

    //!
    //! \brief setPosition3D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    void setPosition3D(const double &posX, const double &posY, const double &posZ)
    {
        this->setX(posX);
        this->setY(posY);
        this->setZ(posZ);
    }

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
    //! \brief getPosZFlag
    //! \return
    //!
    bool getPosZFlag() const
    {
        return this->posXFlag;
    }

    //!
    //! \brief has3DPositionSet
    //! \return
    //!
    bool has3DPositionSet() const
    {
        return this->posXFlag && this->posYFlag && this->posZFlag;
    }

public:

    friend Point3D operator + (const Point2D &lhs, const Point3D &rhs)
    {
        return rhs + lhs;
    }

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Point3D operator + (const Point3D &that) const
    {
        double newX = this->x + that.x;
        double newY = this->y + that.y;
        double newZ = this->z + that.z;
        Point3D newPoint(newX, newY, newZ);
        return newPoint;
    }

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Point3D operator + (const Point2D &that) const
    {
        Point2D* ptr = (Point2D*)this;
        Point2D new2D = *ptr + that;
        Point3D newPoint(new2D,this->z);
        return newPoint;
    }

    //!
    //! \brief operator =
    //! \param rhs
    //!
    Point3D& operator = (const Point3D &rhs)
    {
        Point2D::operator =(rhs);
        this->z = rhs.z;
        this->posZFlag = rhs.posZFlag;
        return *this;
    }



    bool operator < (const Point3D &rhs) const
    {
        if(this->z >= rhs.z)
            return false;

        return Point2D::operator <(rhs);
    }


    friend bool operator == (const Point2D &lhs, const Point3D &rhs)
    {
        return rhs == lhs;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Point3D &rhs) {

        if(!Point2D::operator ==(rhs)){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        if(this->posZFlag != rhs.posZFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Point3D &rhs) {
        return !(*this == rhs);
    }

protected:
    //!
    //! \brief z
    //!
    double z = 0.0;

    //!
    //! \brief posZFlag
    //!
    bool posZFlag = false;
};

} //end of namespace pose
} //end of namespace base

#endif // POINT_3D_H
