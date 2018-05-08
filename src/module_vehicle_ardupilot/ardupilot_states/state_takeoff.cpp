#include "state_takeoff.h"

namespace ardupilot{
namespace state{

State_Takeoff::State_Takeoff():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF;
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

    if(currentStateEnum != desiredStateEnum)
    {
        if(IsInInnerState<State_TakeoffComplete>())
        {
            rtn = hsm::SiblingTransition<State_Flight>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (desiredStateEnum) {
            case ArdupilotFlightState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<State_Grounded>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffClimbing>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffTransitioning>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_FLIGHT:
            {
                rtn = hsm::SiblingTransition<State_Flight>(currentCommand);
            }
            default:
                std::cout<<"I dont know how we eneded up in this transition state from STATE_TAKEOFF."<<std::endl;
                break;
            }
        }
    }
    return rtn;
}

bool State_Takeoff::handleCommand(const AbstractCommandItem* command)
{
    switch(command->getCommandType()) {
    case COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        AbstractStateArdupilot::handleCommand(command);
        MAVLINKVehicleControllers::ControllerSystemMode* modeController = (MAVLINKVehicleControllers::ControllerSystemMode*)currentControllers.at("modeController");
        modeController->setLambda_Finished([this,modeController](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {
                //if a mode change was issued while in the takeoff sequence we may have to handle it in a specific way based on the conditions
                desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
            }
            else
            {
                //we got issues?
            }
            modeController->Shutdown();
        });
        break;
    }
    default:
        break;
    } //end of switch statement
}

void State_Takeoff::Update()
{
    StateData_MAVLINK* vehicleData = Owner().state;

    if(!vehicleData->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
    else
    {
        if(vehicleData->vehicleMode.get().getFlightModeString() == "GUIDED")
            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
    }
}

void State_Takeoff::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->setLambda_Finished([this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        controllerSystemMode->Shutdown();
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
        else
            desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
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
    commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString("GUIDED");
    controllerSystemMode->Send(commandMode,sender,target);
    currentControllers.insert({"modeController",controllerSystemMode});
}



void State_Takeoff::OnEnter(const AbstractCommandItem *command)
{
    if(command != nullptr)
    {
        this->OnEnter();
        this->clearCommand();
        currentCommand = command->getClone();
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded.h"
#include "ardupilot_states/state_takeoff_climbing.h"
#include "ardupilot_states/state_takeoff_transitioning.h"
#include "ardupilot_states/state_takeoff_complete.h"
#include "ardupilot_states/state_flight.h"
