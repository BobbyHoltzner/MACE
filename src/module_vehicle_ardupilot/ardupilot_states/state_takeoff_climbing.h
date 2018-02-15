#ifndef STATE_TAKEOFF_CLIMBING_H
#define STATE_TAKEOFF_CLIMBING_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_TakeoffTransitioning;

class State_TakeoffClimbing : public AbstractStateArdupilot
{
public:
    State_TakeoffClimbing();

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    void handleCommand() override;

    void Update() override;

    void OnEnter() override;

};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_TAKEOFF_CLIMBING_H
