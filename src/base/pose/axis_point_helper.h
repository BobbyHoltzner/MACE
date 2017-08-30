#ifndef AXIS_POINT_HELPER_H
#define AXIS_POINT_HELPER_H

#include "point_forward_definition.h"

namespace mace{
namespace pose {

template <class DERIVEDCLASS, bool is3DVal>
class AxisPointHelper;

template <class DERIVEDCLASS>
class AxisPointHelper<DERIVEDCLASS, true>
{
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
        return this->posZFlag;
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

template <class DERIVEDCLASS>
class AxisPointHelper<DERIVEDCLASS, false>
{

};

} //end of namespace pose
} //end of namepsace base

#endif // AXIS_POINT_HELPER_H
