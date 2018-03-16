#include "state_landing_transitioning.h"

namespace ardupilot{
namespace state{

State_LandingTransitioning::State_LandingTransitioning():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING_TRANSITIONING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
    this->desiredState = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
}

AbstractStateArdupilot* State_LandingTransitioning::getClone() const
{
    return (new State_LandingTransitioning(*this));
}

void State_LandingTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingTransitioning(*this);
}

hsm::Transition State_LandingTransitioning::GetTransition()
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

void State_LandingTransitioning::handleCommand(const AbstractCommandItem* command)
{

}

void State_LandingTransitioning::Update()
{

}

void State_LandingTransitioning::OnEnter()
{

}

void State_LandingTransitioning::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
