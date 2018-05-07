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
        case ArdupilotFlightState::STATE_FLIGHT_MANUAL:
        {
            rtn = hsm::InnerEntryTransition<State_FlightManual>();
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
        ardupilot::state::AbstractStateArdupilot* childState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        childState->handleCommand(command);

        auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->setLambda_Finished([this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {
                checkTransitionFromMode();
            }
            controllerSystemMode->Shutdown();
        });

        controllerSystemMode->setLambda_Shutdown([this,controllerSystemMode]()
        {
            currentControllerMutex.lock();
            currentControllers.erase("modeController");
            delete controllerSystemMode;
            currentControllerMutex.unlock();
        });

        MaceCore::ModuleCharacteristic target;
        target.ID = Owner().getMAVLINKID();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MaceCore::ModuleCharacteristic sender;
        sender.ID = 255;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MAVLINKVehicleControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = target.ID;
        commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString(command->as<CommandItem::ActionChangeMode>()->getRequestMode());
        controllerSystemMode->Send(commandMode,sender,target);
        currentControllers.insert({"modeController",controllerSystemMode});

        break;
    }
    default:
        break;
    }
}

void State_Flight::Update()
{

}

void State_Flight::OnEnter()
{
    checkTransitionFromMode();
}

void State_Flight::OnEnter(const AbstractCommandItem *command)
{
    if(command != nullptr)
    {
        this->OnEnter();
    }
}

void State_Flight::checkTransitionFromMode()
{
    std::string currentModeString = Owner().state->vehicleMode.get().getFlightModeString();

    if(currentModeString == "AUTO")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_AUTO;
    }
    else if(currentModeString == "BRAKE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_BRAKE;
    }
    else if(currentModeString == "GUIDED")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    }
    else if(currentModeString == "STABILIZE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_MANUAL;
    }
    else if(currentModeString == "RTL")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_RTL;
    }
    else if(currentModeString == "LAND")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we
        //are going to land or the
    }
    else{

    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_auto.h"
#include "ardupilot_states/state_flight_brake.h"
#include "ardupilot_states/state_flight_guided.h"
#include "ardupilot_states/state_flight_manual.h"
#include "ardupilot_states/state_landing.h"

