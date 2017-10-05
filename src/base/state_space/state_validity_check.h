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
        m_stateSpace(space.get())
    {

    }

    StateValidityCheck(const StateValidityCheck &) = delete;
    StateValidityCheck &operator=(const StateValidityCheck &) = delete;

    virtual ~StateValidityCheck() = default;

    const void updateStateSpace(const StateSpacePtr &space)
    {
        m_stateSpace = space.get();
    }

    virtual bool isValid(const State *state) const = 0;
protected:
    /**
     * @brief m_stateSpace
     */
    StateSpace* m_stateSpace;

};

MACE_CLASS_FORWARD(AllValidStateCheck);

class BASESHARED_EXPORT AllValidStateCheck: public StateValidityCheck{

public:
    AllValidStateCheck(StateSpacePtr space):
        StateValidityCheck(space)
    {

    }

    virtual ~AllValidStateCheck() = default;


    bool isValid(const State *state) const override
    {
        return true;
    }

};

} //end of namespace state_space
} //end of namespace mace

#endif // STATE_VALIDITY_CHECK_H
