#include "state_takeoff_climbing.h"

namespace ardupilot{
namespace state{

State_TakeoffClimbing::State_TakeoffClimbing():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
    this->desiredState = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
}

AbstractStateArdupilot* State_TakeoffClimbing::getClone() const
{
    return (new State_TakeoffClimbing(*this));
}

void State_TakeoffClimbing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffClimbing(*this);
}

hsm::Transition State_TakeoffClimbing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentState != desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            rtn = hsm::SiblingTransition<State_TakeoffTransitioning>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_TakeoffClimbing::handleCommand(const AbstractCommandItem* command)
{

}

void State_TakeoffClimbing::Update()
{

}

void State_TakeoffClimbing::OnEnter()
{

}

void State_TakeoffClimbing::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_takeoff_transitioning.h"

