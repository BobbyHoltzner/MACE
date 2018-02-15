#include "state_takeoff.h"

namespace ardupilot{
namespace state{

State_Takeoff::State_Takeoff():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_TAKEOFF;
    this->desiredState = ArdupilotFlightState::STATE_TAKEOFF;
}

AbstractStateArdupilot* State_Takeoff::getClone() const
{
    return (new State_Takeoff(*this));
}

void State_Takeoff::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Takeoff(*this);
}

hsm::Transition State_Takeoff::GetTransition()
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

void State_Takeoff::handleCommand()
{

}

void State_Takeoff::Update()
{

}

void State_Takeoff::OnEnter()
{

}

} //end of namespace ardupilot
} //end of namespace state

