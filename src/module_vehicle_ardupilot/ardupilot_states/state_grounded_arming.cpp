#include "state_grounded_arming.h"

namespace ardupilot{
namespace state{

State_GroundedArming::State_GroundedArming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
}

AbstractStateArdupilot* State_GroundedArming::getClone() const
{
    return (new State_GroundedArming(*this));
}

void State_GroundedArming::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedArming(*this);
}

hsm::Transition State_GroundedArming::GetTransition()
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

void State_GroundedArming::handleCommand()
{

}

void State_GroundedArming::Update()
{

}

void State_GroundedArming::OnEnter()
{

}

} //end of namespace ardupilot
} //end of namespace state
