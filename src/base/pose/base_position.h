#ifndef BASE_POSITION_H
#define BASE_POSITION_H

#include <iostream>

#include "abstract_position.h"

#include "misc/data_2d.h"
#include "misc/data_3d.h"

#include "coordinate_frame.h"

namespace mace{
namespace pose{


template <class DATA_DIMENSION>
class Position{

public:
    Position()
    {

    }

    Position(const Position<DATA_DIMENSION> &copy)
    {
        this->frame = copy.frame;
        this->position = copy.position;
    }

    template <class DERIVED>
    Position(const Position<DERIVED> &derived)
    {
        this->frame = derived.frame;
        this->position = derived.position;
    }

    bool is3D() const
    {
        if(mace::misc::details::DataTypeHelper<DATA_DIMENSION>::static_size > 2)
            return true;
        return false;
    }

    bool both3D(const Position<DATA_DIMENSION> &rhs) const
    {
        if(this->is3D() && rhs.is3D())
            return true;

        return false;
    }

    CoordinateFrame getCoordinateFrame() const
    {
        return frame;
    }

    void setCoordinateFrame(const CoordinateFrame &cf)
    {
        this->frame = cf;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Position operator + (const Position &that) const
    {
        Position newPoint(this->position + that.position);
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Position operator - (const Position &that) const
    {
        Position newPoint(this->position - that.position);
        return newPoint;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Position &rhs) const
    {
        if(this->position >= rhs.position)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Position &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Position &rhs) const
    {
        if(this->position <= rhs.position)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Position &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Position &rhs) const
    {
        if(this->position != rhs.position){
            return false;
        }
        if(this->frame != rhs.frame){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Position &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Position& operator = (const Position &rhs)
    {
        this->position = rhs.position;
        this->frame = rhs.frame;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Position& operator += (const Position &rhs)
    {
        this->position += rhs.position;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Position& operator -= (const Position &rhs)
    {
        this->position -= rhs.position;
        return *this;
    }

public:
    CoordinateFrame frame = CoordinateFrame::CF_UNKNOWN;

    DATA_DIMENSION position;
};

} // end of namespace pose
} // end of namespace mace

#endif // BASE_POSITION_H
