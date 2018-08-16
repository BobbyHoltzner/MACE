#ifndef OBJECTMACEMSGTUPLE_H
#define OBJECTMACEMSGTUPLE_H

#include "common/object_int_tuple.h"

namespace ExternalLink{

//!
//! \brief ObjectMaceMsgIDTuple And class that wraps up some object with an integer identifying the mace message ID
//!
template <typename T>
using ObjectMaceMsgIDTuple = ObjectIntTuple<T>;

}

#endif // OBJECTMACEMSGTUPLE_H
