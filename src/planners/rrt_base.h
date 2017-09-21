#ifndef RRT_BASE_H
#define RRT_BASE_H

#include "flann/flann.hpp"

class Value
{
    Value():
        value(0.0)
    {

    }

    Value(const Value &copy)
    {
        this->value = copy.value;
    }



private:
    double value = 0.0;
};

class RRT_Base
{
public:
    RRT_Base();
    flann::Index<flann::L2_Simple<double>> m_Tree;

};

#endif // RRT_BASE_H
