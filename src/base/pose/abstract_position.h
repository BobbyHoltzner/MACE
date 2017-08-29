#ifndef ABSTRACT_POSITION_H
#define ABSTRACT_POSITION_H

#include "abstract_point.h"
#include "point_2d.h"
#include "point_3d.h"

#include "coordinate_frame.h"

namespace base {
namespace pose {

template <class Position, class Point>
class AbstractPosition
{
    static_assert(std::is_base_of<AbstractPoint,Point>::value,"Point must be a descendant of AbstractPoint");

public:
    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const Position &position) const = 0;

public:
    //!
    //! \brief finalBearing
    //! \param postion
    //! \return
    //!
    virtual double finalBearing(const Position &postion) const = 0;

    //!
    //! \brief initialBearing
    //! \param postion
    //! \return
    //!
    virtual double initialBearing(const Position &postion) const = 0;

    //!
    //! \brief bearingBetween
    //! \param position
    //! \return
    //!
    virtual double bearingBetween(const Position &position) const = 0;

    //!
    //! \brief NewPositionFromHeadingBearing
    //! \param distance
    //! \param bearing
    //! \param degreesFlag
    //! \return
    //!
    virtual Position NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const = 0;

    //!
    //! \brief translationTransformation2D
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation2D(const Position &position, Eigen::Vector2f &transVec) const  = 0;

public:

    //!
    //! \brief getCoordinateFrame
    //! \return
    //!
    CoordinateFrame getCoordinateFrame() const = 0;

    //!
    //! \brief isCoordinateFrame
    //! \param comp
    //! \return
    //!
    bool isCoordinateFrame(const CoordinateFrame &comp) const = 0;

public:
    Point point;
};

} //end of namespace pose
} //end of namespace base

#endif // ABSTRACT_POSITION_H
