#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided(ControllerFactory *controllerFactory):
    AbstractStateArdupilot(controllerFactory)
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
}

AbstractStateArdupilot* State_FlightGuided::getClone() const
{
    return (new State_FlightGuided(*this));
}

void State_FlightGuided::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided(*this);
}

hsm::Transition State_FlightGuided::GetTransition()
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

bool State_FlightGuided::handleCommand(const AbstractCommandItem* command)
{

}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{

}

void State_FlightGuided::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

