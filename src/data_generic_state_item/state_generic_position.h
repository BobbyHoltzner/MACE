#ifndef STATE_GENERIC_POSITION_H
#define STATE_GENERIC_POSITION_H

#include "data/coordinate_frame.h"
#include "Eigen/Dense"

namespace DataState {
template<class T>
class StateGenericPosition
{

public:
    StateGenericPosition()
    {
        this->m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
        this->posZFlag =false;
    }

    StateGenericPosition(const Data::CoordinateFrameType &coordinateFrame)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->posXFlag =false;
        this->posYFlag =false;
        this->posZFlag =false;
    }

    StateGenericPosition(const double &posX, const double &posY, const double &posZ)
    {
        setPositionGeneric(posX,posY,posZ);
    }

    StateGenericPosition(const Data::CoordinateFrameType &coordinateFrame, const double &posX, const double &posY, const double &posZ)
    {
        this->m_CoordinateFrame = coordinateFrame;
        this->setPositionGeneric(posX,posY,posZ);
    }

private:
    void setPositionGeneric(const double &posX, const double &posY, const double &posZ)
    {
        this->setX(posX);
        this->setY(posY);
        this->setZ(posZ);
    }

public:
    void setX(const double &posX)
    {
        this->x = posX;
        this->posXFlag = true;
    }

    void setY(const double &posY)
    {
        this->y = posY;
        this->posYFlag = true;
    }

    void setZ(const double &posZ)
    {
        this->z = posZ;
        this->posZFlag = true;
    }

    double getX() const
    {
        return this->x;
    }
    double getY() const
    {
        return this->y;
    }
    double getZ() const
    {
        return this->z;
    }

    void setCoordinateFrame(const Data::CoordinateFrameType &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    Data::CoordinateFrameType getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

public:
    virtual double deltaAltitude(const T &position) const = 0;
    virtual double distanceBetween2D(const T &position) const = 0;
    virtual double distanceBetween3D(const T &position) const = 0;

public:
    virtual double finalBearing(const T &postion) const = 0;
    virtual double initialBearing(const T &postion) const = 0;
    virtual double bearingBetween(const T &position) const = 0;
    virtual T NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const = 0;
    virtual void translationTransformation(const T &position, Eigen::Vector3f &transVec) = 0;

public:
    void operator = (const StateGenericPosition &rhs)
    {
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
        this->posXFlag = rhs.posXFlag;
        this->posYFlag = rhs.posYFlag;
        this->posZFlag = rhs.posZFlag;
    }

    bool operator == (const StateGenericPosition &rhs) {
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        if(this->posXFlag != rhs.posXFlag){
            return false;
        }
        if(this->posYFlag != rhs.posYFlag){
            return false;
        }
        if(this->posZFlag != rhs.posZFlag){
            return false;
        }
        return true;
    }

    bool operator != (const StateGenericPosition &rhs) {
        return !(*this == rhs);
    }

public:
    bool getPosXFlag() const
    {
        return this->posXFlag;
    }

    bool getPosYFlag() const
    {
        return this->posXFlag;
    }

    bool getPosZFlag() const
    {
        return this->posXFlag;
    }

    bool has2DPositionSet() const
    {
        return this->posXFlag && this->posYFlag;
    }

    bool has3DPositionSet() const
    {
        return this->posXFlag && this->posYFlag && this->posZFlag;
    }

protected:
    Data::CoordinateFrameType m_CoordinateFrame;

    double x;
    double y;
    double z;

    bool posXFlag;
    bool posYFlag;
    bool posZFlag;
};

}

#endif // STATE_GENERIC_POSITION_H
