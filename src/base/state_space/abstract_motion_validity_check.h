#ifndef ABSTRACT_MOTION_VALIDITY_CHECK_H
#define ABSTRACT_MOTION_VALIDITY_CHECK_H

#include "base/base_global.h"
#include "common/class_forward.h"

#include "state_space.h"

namespace mace {
namespace state_space {

/**
 *
 */
MACE_CLASS_FORWARD(AbstractMotionValidityCheck);

/**
 * @brief The AbstractMotionValidityCheck class
 */
class AbstractMotionValidityCheck
{
public:
    /**
     * @brief AbstractMotionValidityCheck
     * @param space
     */
    AbstractMotionValidityCheck(const StateSpacePtr &space):
        m_stateSpace(space.get())
    {

    }

    /**
      */
    virtual ~AbstractMotionValidityCheck() = default;

    /**
     * @brief AbstractMotionValidityCheck
     */
    AbstractMotionValidityCheck(const AbstractMotionValidityCheck &) = delete;

    /**
     * @brief operator =
     * @return
     */
    AbstractMotionValidityCheck &operator=(const AbstractMotionValidityCheck &) = delete;

    /**
     * @brief updateStateSpace
     * @param space
     */
    const void updateStateSpace(const StateSpacePtr &space)
    {
        m_stateSpace = space.get();
    }

public:
    /**
     * @brief isValid
     * @param begin
     * @param end
     * @return
     */
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
