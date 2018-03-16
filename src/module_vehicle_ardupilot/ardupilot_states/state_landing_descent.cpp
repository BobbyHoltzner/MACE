#include "state_landing_descent.h"

namespace ardupilot{
namespace state{

State_LandingDescent::State_LandingDescent():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_LANDING;
    this->desiredState = ArdupilotFlightState::STATE_LANDING;
}

AbstractStateArdupilot* State_LandingDescent::getClone() const
{
    return (new State_LandingDescent(*this));
}

void State_LandingDescent::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingDescent(*this);
}

hsm::Transition State_LandingDescent::GetTransition()
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

void State_LandingDescent::handleCommand(const AbstractCommandItem* command)
{

}

void State_LandingDescent::Update()
{

}

void State_LandingDescent::OnEnter()
{

}

void State_LandingDescent::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
