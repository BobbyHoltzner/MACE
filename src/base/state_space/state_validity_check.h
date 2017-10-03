#ifndef STATE_VALIDITY_CHECK_H
#define STATE_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/class_forward.h"

#include "state_space.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(StateValidityCheck);

class BASESHARED_EXPORT StateValidityCheck{

public:
    StateValidityCheck(StateSpacePtr space):
        m_stateSpace(space)
    {

    }

    virtual ~StateValidityCheck() = default;


    virtual bool isValid(const State *state) const = 0;
protected:
    StateSpacePtr m_stateSpace;

};

class BASESHARED_EXPORT AllValidStateChecker: public StateValidityCheck{

public:
    AllValidStateChecker(StateSpacePtr space):
        StateValidityCheck(space)
    {

    }

    virtual ~AllValidStateChecker() = default;


    bool isValid(const State *state) const override
    {
        return true;
    }

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_VALIDITY_CHECK_H
