#ifndef CARTESIAN_POSITION_2D_H
#define CARTESIAN_POSITION_2D_H

#include <iostream>

#include "Eigen/Dense"
#include "misc/data_2d.h"

#include "base_position.h"

namespace mace{
namespace pose {

class CartesianPosition_2D : public Position<misc::Data2D>, public AbstractPosition<CartesianPosition_2D>
{
public:
    CartesianPosition_2D()
    {
        std::cout<<"Default cartesian position 2d constructor"<<std::endl;
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);
    }

    CartesianPosition_2D(const CartesianPosition_2D &copy):
        Position(copy)
    {

    }

    CartesianPosition_2D(const double x, const double &y)
    {
        this->setCoordinateFrame(CoordinateFrame::CF_LOCAL_ENU);
        this->position.set2DPosition(x,y);
    }

    template <class DERIVED>
    CartesianPosition_2D(const Position<DERIVED> &derived):
        Position(derived)
    {

    }

public:
    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const CartesianPosition_2D &pos) const
    {
        double deltaX = this->position.getX() - pos.position.getX();
        double deltaY = this->position.getY() - pos.position.getY();
        double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
        return distance;
    }

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    double distanceTo(const CartesianPosition_2D &pos) const
    {
        return this->distanceBetween2D(pos);
    }

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double bearingTo(const CartesianPosition_2D &pos) const
    {

    }

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual CartesianPosition_2D newPosition(const double &distance, const double &bearing) const
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

#endif // CARTESIAN_POSITION_2D_H
