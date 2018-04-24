#include "state_landing_transitioning.h"

namespace ardupilot{
namespace state{

State_LandingTransitioning::State_LandingTransitioning():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING_TRANSITIONING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
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

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingTransitioning::handleCommand(const AbstractCommandItem* command)
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
