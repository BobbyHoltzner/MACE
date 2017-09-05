#include "data_3d.h"

namespace mace{
namespace misc{

//!
//! \brief Data3D
//!
Data3D::Data3D()
{

}

Data3D::Data3D(const Data3D &copy):
    Data2D(copy)
{
    std::cout<<"copy constructor Data3D"<<std::endl;
    this->z = copy.z;
    this->dataZFlag = copy.dataZFlag;
}

Data3D::Data3D(const Data2D &copy):
    Data2D(copy)
{

}

Data3D::Data3D(const Data2D &copy, const double &z):
    Data2D(copy)
{
    this->setZ(z);
}

//!
//! \brief Data3D::Data3D
//! \param compX
//! \param compY
//! \param compZ
//!
Data3D::Data3D(const double &compX, const double &compY, const double &compZ):
    Data2D(compX, compY)
{

    this->setZ(compZ);
}


Data2D Data3D::get2DData() const
{
    return Data2D(*this);
}

/** The following are defined in point_2d.h as friend functions*/

//!
//! \brief operator +
//! \param lhs
//! \param rhs
//! \return
//!
Data3D operator + (const Data2D &lhs, const Data3D &rhs)
{
    return lhs + rhs;
}

//!
//! \brief operator +
//! \param lhs
//! \param rhs
//! \return
//!
Data3D operator - (const Data2D &lhs, const Data3D &rhs)
{
    return lhs - rhs;
}

//!
//! \brief operator ==
//! \param lhs
//! \param rhs
//! \return
//!
bool operator == (const Data2D &lhs, const Data3D &rhs)
{
    return lhs == rhs;
}

//!
//! \brief operator <
//! \param lhs
//! \param rhs
//! \return
//!
bool operator < (const Data2D &lhs, const Data3D &rhs)
{
    return lhs < rhs.get2DData();
}

//!
//! \brief operator >
//! \param lhs
//! \param rhs
//! \return
//!
bool operator > (const Data2D &lhs, const Data3D &rhs)
{
    return lhs > rhs.get2DData();
}

} //end of namespace misc
} //end of namespace mace
