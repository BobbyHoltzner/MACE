#ifndef DATA_2D_H
#define DATA_2D_H

#include <iostream>
#include <cmath>

#include "abstract_data.h"

namespace mace {
namespace misc {

class Data2D
{
public:
    //!
    //! \brief Data2D
    //!
    Data2D() = default;

    //!
    virtual ~Data2D() = default;

    //!
    //! \brief Data2D
    //! \param copy
    //!
    Data2D(const Data2D &copy);

    //!
    //! \brief Data2D
    //! \param x
    //! \param y
    //!
    Data2D(const double &x, const double &y);


    Data2D norm() const
    {
        double length = sqrt(x*x + y*y);
        if(length == 0)
            return Data2D();
        else
            return Data2D(x / length, y / length);
    }

    /** Implied through inheritance of AbstractPoint */
public:
    virtual bool is3D() const
    {
        return false;
    }

    //!
    //! \brief getDataYFlag
    //! \return
    //!
    bool getDataXFlag() const
    {
        return this->dataXFlag;
    }

    //!
    //! \brief getDataYFlag
    //! \return
    //!
    bool getDataYFlag() const
    {
        return this->dataYFlag;
    }

    /** Common among all point classes */
public:

    //!
    //! \brief setData
    //! \param data2D
    //!
    void setData(const Data2D &data2D);

    //!
    //! \brief setData
    //! \param x
    //! \param y
    //!
    void setData(const double &x, const double &y);

    //!
    //! \brief setX
    //! \param posX
    //!
    void setX(const double &posX)
    {
        this->x = posX;
        this->dataXFlag = true;
    }

    //!
    //! \brief getX
    //! \return
    //!
    double getX() const
    {
        return this->x;
    }

    //!
    //! \brief setY
    //! \param posY
    //!
    void setY(const double &posY)
    {
        this->y = posY;
        this->dataYFlag = true;
    }

    //!
    //! \brief getY
    //! \return
    //!
    double getY() const
    {
        return this->y;
    }

    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Data2D operator + (const Data2D &that) const
    {
        double newX = this->x + that.x;
        double newY = this->y + that.y;
        Data2D newPoint(newX, newY);
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Data2D operator - (const Data2D &that) const
    {
        double newX = this->x - that.x;
        double newY = this->y - that.y;
        Data2D newPoint(newX, newY);
        return newPoint;
    }

    Data2D operator * (const double &value) const
    {
        Data2D newPoint(x*value, y*value);
        return newPoint;
    }

    Data2D operator / (const double &value) const
    {
        Data2D newPoint(x/value, y/value);
        return newPoint;
    }

    double dot(const Data2D &that) const
    {
        return x * that.x + y * that.y;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const Data2D &rhs) const
    {
        if(this->x >= rhs.x)
            return false;
        if(this->y >= rhs.y)
            return false;

        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const Data2D &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const Data2D &rhs) const
    {
        if(this->x <= rhs.x)
            return false;
        if(this->y <= rhs.y)
            return false;

        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const Data2D &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Data2D &rhs) const
    {
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->dataXFlag != rhs.dataXFlag){
            return false;
        }
        if(this->dataYFlag != rhs.dataYFlag){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Data2D &rhs) const{
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Data2D& operator = (const Data2D &rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->dataXFlag = rhs.dataXFlag;
        this->dataYFlag = rhs.dataYFlag;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Data2D& operator += (const Data2D &rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Data2D& operator -= (const Data2D &rhs)
    {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }


    /** Protected Members */
protected:
    //!
    //! \brief x
    //!
    double x = 0.0;

    //!
    //! \brief y
    //!
    double y = 0.0;

    //!
    //! \brief dataXFlag
    //!
    bool dataXFlag = false;

    //!
    //! \brief dataYFlag
    //!
    bool dataYFlag = false;
};

} //end of namespace misc
} //end of namespace mace

#endif // POINT_2D_H
