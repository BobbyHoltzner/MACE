#ifndef SPACE_INFORMATION_H
#define SPACE_INFORMATION_H

#include <functional>

#include "common/class_forward.h"
#include "base/base_global.h"

#include "base/state_space/state_space.h"
#include "base/state_space/state_sampler.h"
#include "base/state_space/state_validity_check.h"
#include "base/state_space/motion_validity_check.h"

namespace mace {
namespace state_space {

typedef std::function<bool(const State* state)> StateValidityFunction;

MACE_CLASS_FORWARD(SpaceInformation);

class BASESHARED_EXPORT SpaceInformation
{

public:
    /** \brief Constructor. Sets the instance of the state space to plan with. */
    SpaceInformation(const StateSpacePtr &space);

    virtual ~SpaceInformation() = default;

    SpaceInformation(const SpaceInformation &) = delete;
    SpaceInformation &operator=(const SpaceInformation &) = delete;

    void updateStateSpace(const StateSpacePtr &space);

    const StateSpacePtr& getStateSpace() const;

public:

    bool isStateValid(const State* state) const;

    bool isEdgeValid(const State* lhs, const State* rhs) const;

    double distanceBetween(const State* lhs, const State* rhs) const;

    /** \brief Check if a given state is valid or not */
    bool isValid(const State *state) const
    {
        return true;
        //return m_stateValidCheck.isValid(state);
    }

    void setStateSampler(const StateSamplerPtr &sampler);

    StateSamplerPtr getStateSampler() const;

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

public:
    //!
    //! \brief getNewState
    //! \return
    //!
    State* getNewState() const;

    //!
    //! \brief removeState
    //! \param state
    //!
    void removeState(State* state) const;

    //!
    //! \brief copyState
    //! \param state
    //! \return
    //!
    State* copyState(const State *state) const;

    //!
    //! \brief removeStates
    //! \param states
    //!
    void removeStates(std::vector<State*> states) const;

private:
    bool isSetup;

protected:
    StateSpacePtr m_stateSpace;

    StateValidityCheckPtr m_stateValidCheck;
    MotionValidityCheckPtr m_motionValidCheck;
    StateSamplerPtr m_stateSampler;
};

} //end of namespace state_space
} //end of namespace mace

#endif // SPACE_INFORMATION_H
