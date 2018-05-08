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

void State_Flight::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleMode.RemoveNotifier(this);
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
        case ArdupilotFlightState::STATE_FLIGHT_AUTO:
        {
            rtn = hsm::InnerEntryTransition<State_FlightAuto>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_BRAKE:
        {
            rtn = hsm::InnerEntryTransition<State_FlightBrake>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided>();
            break;
        }
        case ArdupilotFlightState::STATE_LANDING:
        {
            rtn = hsm::SiblingTransition<State_Landing>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_MANUAL:
        {
            rtn = hsm::InnerEntryTransition<State_FlightManual>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_RTL:
        {
            rtn = hsm::InnerEntryTransition<State_FlightRTL>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_UNKNOWN:
        {
            rtn = hsm::InnerEntryTransition<State_FlightUnknown>();
            break;
        }
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
    switch (commandType) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        AbstractStateArdupilot::handleCommand(command);
        break;
    }
    case COMMANDITEM::CI_NAV_LAND:
    {
        currentCommand = command->getClone();
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_LAND;
        break;
    }
    default:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        currentInnerState->handleCommand(command);
        break;
    }
    } //end of switch statement
}

void State_Flight::Update()
{
    //mode changes are directly handled via add notifier events established in the OnEnter() method
}

void State_Flight::OnEnter()
{
    //This will only handle when the mode has actually changed
    Owner().state->vehicleMode.AddNotifier(this,[this]
    {
        std::string currentModeString = Owner().state->vehicleMode.get().getFlightModeString();
        checkTransitionFromMode(currentModeString);
    });

    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().state->vehicleMode.get().getFlightModeString();
    checkTransitionFromMode(currentModeString);
}

void State_Flight::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
        delete command;//we handle this deletion as we know it had to come from previous command of state
    }
}

void State_Flight::checkTransitionFromMode(const std::string &mode)
{
    if(mode == "AUTO")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_AUTO;
    }
    else if(mode == "BRAKE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_BRAKE;
    }
    else if(mode == "GUIDED")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    }
    else if(mode == "LAND")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we are
        desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
    }
    else if(mode == "STABILIZE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_MANUAL;
    }
    else if(mode == "RTL")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_RTL;
    }
    else{
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_UNKNOWN;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_auto.h"
#include "ardupilot_states/state_flight_brake.h"
#include "ardupilot_states/state_flight_guided.h"
#include "ardupilot_states/state_flight_land.h"
#include "ardupilot_states/state_flight_manual.h"
#include "ardupilot_states/state_flight_rtl.h"
#include "ardupilot_states/state_flight_unknown.h"

#include "ardupilot_states/state_landing.h"



