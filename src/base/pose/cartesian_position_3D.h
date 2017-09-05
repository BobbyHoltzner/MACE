#ifndef CARTESIAN_POSITION_3D_H
#define CARTESIAN_POSITION_3D_H

#include <iostream>

#include "Eigen/Dense"

#include "base_position.h"
#include "misc/data_2d.h"
#include "misc/data_3d.h"

namespace mace{
namespace pose {

class CartesianPosition_3D : public Position<misc::Data3D>, public AbstractPosition<CartesianPosition_3D>
{
public:
    CartesianPosition_3D()
    {
        std::cout<<"Default cartesian position 3d constructor"<<std::endl;
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);

    }

    CartesianPosition_3D(const CartesianPosition_3D &copy):
        Position(copy)
    {

    }

    CartesianPosition_3D(const double x, const double &y, const double &z)
    {
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);
        this->position.setData(x,y,z);
    }

    template <class DERIVED>
    CartesianPosition_3D(const Position<DERIVED> &derived):
        Position(derived)
    {

    }

public:

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const CartesianPosition_3D &pos) const
    {
        double deltaX = this->position.getX() - pos.position.getX();
        double deltaY = this->position.getY() - pos.position.getY();
        double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
        return distance;    }

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const CartesianPosition_3D &pos) const
    {
        double _2DDistance = distanceBetween2D(pos);
        double deltaZ = this->position.getZ() - pos.position.getZ();
        return sqrt(pow(_2DDistance, 2) + pow(deltaZ,2));
    }

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double bearingTo(const CartesianPosition_3D &pos) const
    {

    }

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual CartesianPosition_3D newPosition(const double &distance, const double &bearing) const
    {

    }

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    virtual CoordinateFrame getCoordinateFrame() const
    {

    }

    //!
    //! \brief isOfCoordinateFrame
    //! \return
    //!
    virtual bool isOfCoordinateFrame(const CoordinateFrame &coordinateFrame) const
    {

    }

};

} //end of namespace pose
} //end of namespace mace

#endif // CARTESIAN_POSITION_3D_H
