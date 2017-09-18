#ifndef SPACE_INFORMATION_H
#define SPACE_INFORMATION_H

#include <functional>

#include "base_global.h"

#include "state.h"

//This class is intended to be abstract as well

namespace mace {
namespace state_space {

typedef std::function<bool(const State* state)> StateValidityFunction;

class BASESHARED_EXPORT SpaceInformation
{

public:
    /** \brief Constructor. Sets the instance of the state space to plan with. */
    SpaceInformation(StateSpacePtr space);

    virtual ~SpaceInformation() = default;

    SpaceInformation(const SpaceInformation &) = delete;
    SpaceInformation &operator=(const SpaceInformation &) = delete;
public:

    /** \brief Check if a given state is valid or not */
    bool isValid(const State *state) const
    {
        return stateValidityChecker_->isValid(state);
    }


public:
    /** \brief Cast this instance to a desired type. */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /** \brief Cast this instance to a desired type. */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }
};

} //end of namespace state_space
} //end of namespace mace

#endif // SPACE_INFORMATION_H
