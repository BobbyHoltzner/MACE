#include "state_flight.h"

namespace ardupilot{
namespace state{

State_Flight::State_Flight():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
}

AbstractStateArdupilot* State_Flight::getClone() const
{
    return (new State_Flight(*this));
}

void State_Flight::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Flight(*this);
}

hsm::Transition State_Flight::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {

        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_FLIGHT."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_Flight::handleCommand(const AbstractCommandItem* command)
{
    COMMANDITEM commandType = command->getCommandType();
    ardupilot::state::AbstractStateArdupilot* currentState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
    switch (commandType) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:

        break;
    default:
        break;
    }
    currentState->handleCommand(command);
}

void State_Flight::Update()
{

}

void State_Flight::OnEnter()
{

}

void State_Flight::OnEnter(const AbstractCommandItem *command)
{
    if(command != nullptr)
    {
        this->OnEnter();
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_auto.h"
#include "ardupilot_states/state_flight_brake.h"
#include "ardupilot_states/state_flight_guided.h"
#include "ardupilot_states/state_flight_manual.h"
#include "ardupilot_states/state_landing.h"

