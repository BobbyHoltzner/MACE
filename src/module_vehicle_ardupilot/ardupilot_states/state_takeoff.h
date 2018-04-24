#ifndef STATE_TAKEOFF_H
#define STATE_TAKEOFF_H

#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

class State_Grounded;
class State_TakeoffClimbing;
class State_TakeoffTransitioning;

class State_Takeoff : public AbstractStateArdupilot
{
public:
    State_Takeoff();

public:
    AbstractStateArdupilot* getClone() const override;

    void getClone(AbstractStateArdupilot** state) const override;

public:
    hsm::Transition GetTransition() override;

public:
    bool handleCommand(const AbstractCommandItem* command) override;

    void Update() override;

    void OnEnter() override;

    void OnEnter(const AbstractCommandItem* command) override;

private:

};

} //end of namespace ardupilot
} //end of namespace state

#endif // STATE_TAKEOFF_H
