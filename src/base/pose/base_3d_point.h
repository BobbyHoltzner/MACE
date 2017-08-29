#ifndef POINT_3D_H
#define POINT_3D_H

#include "point.h"

namespace base {
namespace pose {

class Point3D : public Point<Point3D>
{

public:
    //!
    //! \brief Point3D
    //!
    Point3D():
        BasePoint()
    {
        this->z = 0.0;
        this->posZFlag =false;
    }

    Point3D(const Point3D &copy):
        BasePoint(copy)
    {
        this->z = copy.z;
        this->posZFlag = copy.posZFlag;
    }

    //!
    //! \brief Point3D
    //! \param coordinateFrame
    //!
    Point3D(const CoordinateFrameType &coordinateFrame):
        BasePoint(coordinateFrame)
    {
        this->
        this->z = 0.0;
        this->posZFlag =false;
    }

    //!
    //! \brief Point3D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point3D(const double &posX, const double &posY, const double &posZ):
        BasePoint(posX,posY)
    {
        this->z = 0.0;
        this->posZFlag =false;

        this->setPosition3D(posX,posY,posZ);
    }

    //!
    //! \brief Point3D
    //! \param coordinateFrame
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point3D(const CoordinateFrameType &coordinateFrame, const double &posX, const double &posY, const double &posZ):
        BasePoint(coordinateFrame,posX,posY)
    {
        this->setPosition3D(posX,posY,posZ);
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
    //! \brief has3DPositionSet
    //! \return
    //!
    bool has3DPositionSet() const
    {
        return this->posXFlag && this->posYFlag && this->posZFlag;
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    Point3D& operator = (const BasePoint &rhs)
    {
        BasePoint::operator =(rhs);
        if(rhs.is3D())
        {
            Base3DPoint cast = static_cast<const Base3DPoint*>(rhs);
            this->z = cast.z;
            this->posZFlag = cast.posZFlag;
        }else{
            this->z = 0.0;
            this->posZFlag = false;
        }

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
        if(rhs.is3D())
        {
            Base3DPoint cast = static_cast<const Base3DPoint*>(rhs);
            if(this->z != cast.z){
                return false;
            }
            if(this->posZFlag != cast.posZFlag){
                return false;
            }
        }
        else{
            return false;
        }
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const BasePoint &rhs) {
        return !(*this == rhs);
    }

public:

};

} //end of namespace pose
} //end of namespace base

#endif // POINT_3D_H
