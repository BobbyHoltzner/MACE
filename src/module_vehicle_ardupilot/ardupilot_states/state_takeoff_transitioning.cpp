#include "state_takeoff_transitioning.h"

namespace ardupilot{
namespace state{

State_TakeoffTransitioning::State_TakeoffTransitioning():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
    this->desiredState = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
}

AbstractStateArdupilot* State_TakeoffTransitioning::getClone() const
{
    return (new State_TakeoffTransitioning(*this));
}

void State_TakeoffTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffTransitioning(*this);
}

hsm::Transition State_TakeoffTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentState != desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_TakeoffTransitioning::handleCommand()
{

}

void State_TakeoffTransitioning::Update()
{

}

void State_TakeoffTransitioning::OnEnter()
{

}

} //end of namespace ardupilot
} //end of namespace state

