#ifndef BASE_POSITION_H
#define BASE_POSITION_H

#include <iostream>
#include <cmath>
#include <Eigen/Core>

#include "abstract_position.h"
#include "base/misc/data_2d.h"
#include "base/misc/data_3d.h"

#include "base/math/helper_pi.h"

namespace mace{
namespace pose{


class CartesianPosition
{

};

template<typename T>
class Position : public T
{
public:

    Position() = default;

    template <typename NEWT>
    Position(const Position<NEWT> &ref):
        T(ref)
    {
        std::cout<<"In the copy constuctor"<<std::endl;
        this->name = ref.name;
    }

    template<typename ... Arg>
    Position(const char *str, const Arg ... arg):
        T(arg ...),
        name(str)
    {
        std::cout<<"In the crazy constuctor"<<std::endl;
    }

public:
    std::string getName() const
    {
        return this->name;
    }

public:
    Position& operator = (const Position &copy)
    {
        T::operator=(copy);
        this->name = copy.name;
        return *this;
    }


private:
    std::string name;
};



} // end of namespace pose
} // end of namespace mace

#endif // BASE_POSITION_H
