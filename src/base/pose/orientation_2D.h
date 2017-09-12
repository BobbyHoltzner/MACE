#ifndef ORIENTATION_2D_H
#define ORIENTATION_2D_H

#include <cmath>

#include "Eigen/Core"

namespace mace {
namespace pose {

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 2. The storage of this value is often referred to as a special orthogonal
 * matrix SO(2).
 */

class Orientation_2D
{
public:
    //!
    //! \brief Orientation_2D
    //!
    Orientation_2D();

    ~Orientation_2D();

    //!
    //! \brief Orientation_2D
    //! \param copy
    //!
    Orientation_2D(const Orientation_2D &copy);

    //!
    //! \brief Orientation_2D
    //! \param angle
    //!
    Orientation_2D(const double &angle);

public:
    //!
    //! \brief setPhi
    //! \param angle
    //!
    void setPhi(const double &angle);

    //!
    //! \brief getPhi
    //! \return
    //!
    double getPhi() const;

    //!
    //! \brief getRotationMatrix
    //! \return
    //!
    //!
    void getRotationMatrix(Eigen::Matrix2d &rotM) const;

    //!
    //! \brief getRotationMatrix
    //! \return
    //!
    void getRotationMatrix(Eigen::Matrix3d &rotM) const;

private:
    //!
    //! \brief updateTrigCache
    //!
    inline void updateTrigCache() const
    {
        if(updatedTrig)
            return;
        cosPhi = std::cos(this->phi);
        sinPhi = std::sin(this->phi);
    }


    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Orientation_2D operator + (const Orientation_2D &that) const
    {
        double newPhi = this->phi + that.phi;
        Orientation_2D newObj(newPhi);
        return newObj;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Orientation_2D operator - (const Orientation_2D &that) const
    {
        double newPhi = this->phi - that.phi;
        Orientation_2D newObj(newPhi);
        return newObj;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Orientation_2D &rhs) const
    {
        if(this->phi >= rhs.phi)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Orientation_2D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Orientation_2D &rhs) const
    {
        if(this->phi <= rhs.phi)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Orientation_2D &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Orientation_2D &rhs) const
    {
        if(this->phi != rhs.phi){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Orientation_2D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Orientation_2D& operator = (const Orientation_2D &rhs)
    {
        this->phi = rhs.phi;
        this->angleFlag = rhs.angleFlag;
        this->updatedTrig = false;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Orientation_2D& operator += (const Orientation_2D &rhs)
    {
        this->phi += rhs.phi;
        this->updatedTrig = false;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Orientation_2D& operator -= (const Orientation_2D &rhs)
    {
        this->phi -= rhs.phi;
        this->updatedTrig = false;
        return *this;
    }


    /** Protected Members */
protected:
    //!
    //! \brief updatedTrig
    //!
    mutable bool updatedTrig = false;

    //!
    //! \brief cosPhi
    //!
    mutable double cosPhi = 0.0;

    //!
    //! \brief sinPhi
    //!
    mutable double sinPhi = 0.0;

    //!
    //! \brief angleFlag
    //!
    bool angleFlag = false;

    //!
    //! \brief phi
    //!
    double phi = 0.0;
};

} //end of namespace pose
} //end of namespace mace

#endif // ORIENTATION_2D_H
