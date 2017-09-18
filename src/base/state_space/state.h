#ifndef STATE_H
#define STATE_H

#include "base_global.h"

//This class is intended to be abstract as well

namespace mace {
namespace state_space {

class BASESHARED_EXPORT State{

public:
    State() = default;

    virtual ~State() = default;

    /** \brief Disable copy-constructor */
    State(const State &) = delete;

    /** \brief Disable copy operator */
    const State &operator=(const State &) = delete;

public:
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }
};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_H
