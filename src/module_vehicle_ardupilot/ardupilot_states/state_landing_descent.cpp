#include "state_landing_descent.h"

namespace ardupilot{
namespace state{

State_LandingDescent::State_LandingDescent():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
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

bool State_LandingDescent::handleCommand(const AbstractCommandItem* command)
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
