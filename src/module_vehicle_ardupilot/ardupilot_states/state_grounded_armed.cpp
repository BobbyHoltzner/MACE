#include "state_grounded_armed.h"

namespace ardupilot{
namespace state{

State_GroundedArmed::State_GroundedArmed():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMED"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_ARMED;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMED;
}

AbstractStateArdupilot* State_GroundedArmed::getClone() const
{
    return (new State_GroundedArmed(*this));
}

void State_GroundedArmed::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedArmed(*this);
}

hsm::Transition State_GroundedArmed::GetTransition()
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

void State_GroundedArmed::handleCommand()
{

}

void State_GroundedArmed::Update()
{

}

void State_GroundedArmed::OnEnter()
{

}

} //end of namespace ardupilot
} //end of namespace state
