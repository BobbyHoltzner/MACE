#include "state_grounded_idle.h"

namespace ardupilot{
namespace state{

State_GroundedIdle::State_GroundedIdle():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_IDLE"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_IDLE;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_IDLE;
}

AbstractStateArdupilot* State_GroundedIdle::getClone() const
{
    return (new State_GroundedIdle(*this));
}

void State_GroundedIdle::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedIdle(*this);
}

hsm::Transition State_GroundedIdle::GetTransition()
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

void State_GroundedIdle::handleCommand()
{

}

void State_GroundedIdle::Update()
{

}

void State_GroundedIdle::OnEnter()
{

}

} //end of namespace ardupilot
} //end of namespace state
