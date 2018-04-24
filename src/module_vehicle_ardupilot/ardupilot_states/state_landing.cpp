#include "state_landing.h"

namespace ardupilot{
namespace state{

State_Landing::State_Landing():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_LANDING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
}

AbstractStateArdupilot* State_Landing::getClone() const
{
    return (new State_Landing(*this));
}

void State_Landing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Landing(*this);
}

hsm::Transition State_Landing::GetTransition()
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

bool State_Landing::handleCommand(const AbstractCommandItem* command)
{

}

void State_Landing::Update()
{

}

void State_Landing::OnEnter()
{

}

void State_Landing::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
