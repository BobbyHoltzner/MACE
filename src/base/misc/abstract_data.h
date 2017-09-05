#ifndef ABSTRACT_DATA_H
#define ABSTRACT_DATA_H

#include "data_forward_definition.h"

namespace mace {
namespace misc {

namespace details {

template<class DATABASE>
struct DataTypeHelper;

template<>
struct DataTypeHelper<Data2D>
{
public:
    static const int static_size = 2;
};

template <>
struct DataTypeHelper<Data3D>
{
public:
    static const int static_size = 3;
};

} //end of namespace details

} //end of namespace misc
} //end of namespace mace
#endif // ABSTRACT_DATA_H
