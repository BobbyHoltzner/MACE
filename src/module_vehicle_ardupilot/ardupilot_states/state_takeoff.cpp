#include "state_takeoff.h"

namespace ardupilot{
namespace state{

State_Takeoff::State_Takeoff():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_TAKEOFF;
    this->desiredState = ArdupilotFlightState::STATE_TAKEOFF;
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

    if(currentState != desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
        {
            return hsm::InnerEntryTransition<State_TakeoffClimbing>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            return hsm::InnerEntryTransition<State_TakeoffTransitioning>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_TAKEOFF."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_Takeoff::handleCommand(const AbstractCommandItem* command)
{

}

void State_Takeoff::Update()
{

}

void State_Takeoff::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    auto controllerSystemMode = new MAVLINKVehicleControllers::ControllerSystemMode(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->setLambda_Finished([this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        std::cout<<"The mode has changed"<<std::endl;
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
    this->OnEnter();
    this->clearCommand();
    currentCommand = command->getClone();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_takeoff_climbing.h"
#include "ardupilot_states/state_takeoff_transitioning.h"
