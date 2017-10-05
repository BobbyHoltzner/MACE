#ifndef MOTION_VALIDITY_CHECK_H
#define MOTION_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/class_forward.h"

#include "state_space.h"

namespace mace {
namespace state_space {

MACE_CLASS_FORWARD(MotionValidityCheck);

class MotionValidityCheck
{
public:
    MotionValidityCheck(StateSpacePtr space):
        m_stateSpace(space.get())
    {

    }

    virtual ~MotionValidityCheck() = default;

    MotionValidityCheck(const MotionValidityCheck &) = delete;
    MotionValidityCheck &operator=(const MotionValidityCheck &) = delete;

    const void updateStateSpace(const StateSpacePtr &space)
    {
        m_stateSpace = space.get();
    }

    virtual bool isValid(const State* begin, const State *end) const = 0;
protected:
    /**
     * @brief m_stateSpace reference to the state space in which this validation
     * class will be working within. It is not the responsibility of this class
     * to destroy this pointer.
     */
    StateSpace* m_stateSpace;

};

} //end of state_space
} //end of mace

#endif // MOTION_VALIDITY_CHECK_H
