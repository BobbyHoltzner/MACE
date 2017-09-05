#ifndef ABSTRACT_POSITION_H
#define ABSTRACT_POSITION_H

namespace mace{
namespace pose{

template <class T>
class AbstractPosition
{
public:

    //!
    //! \brief distanceBetween2D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween2D(const T &position) const = 0;

    //!
    //! \brief distanceTo
    //! \param position
    //! \return
    //!
    virtual double distanceTo(const T &position) const = 0;

    //!
    //! \brief bearingTo
    //! \param position
    //! \return
    //!
    virtual double bearingTo(const T &position) const = 0;

    //!
    //! \brief newPosition
    //! \param distance
    //! \param bearing
    //! \return
    //!
    virtual T newPosition(const double &distance, const double &bearing) const = 0;
};

} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_POSITION_H
