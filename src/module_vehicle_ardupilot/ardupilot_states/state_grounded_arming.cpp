#include "state_grounded_arming.h"

namespace ardupilot{
namespace state{

State_GroundedArming::State_GroundedArming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
    armingCheck = false;
}

AbstractStateArdupilot* State_GroundedArming::getClone() const
{
    return (new State_GroundedArming(*this));
}

void State_GroundedArming::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedArming(*this);
}

hsm::Transition State_GroundedArming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentState != desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMED:
        {
            return hsm::SiblingTransition<State_GroundedArmed>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

void State_GroundedArming::handleCommand(const AbstractCommandItem* command)
{
    const AbstractCommandItem* copyCommand = command->getClone();
    this->clearCommand();

    switch (copyCommand->getCommandType()) {
    case COMMANDITEM::CI_NAV_TAKEOFF:
    {
        this->desiredState = ArdupilotFlightState::STATE_TAKEOFF;
        currentCommand = copyCommand;
        break;
    }
    default:
        break;
    }
}

void State_GroundedArming::Update()
{
    /** We basically will wait in this state until the vehicle is armed
     * or an additional error has occured forcing the state to advance.
     * However, given that once an acknowledgement has been passed at this
     * point, in all likelyhood a mavlink vehicle is going to arm.
      */

    if(Owner().state->vehicleArm.get().getSystemArm())
    {
        this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMED;
        if(currentCommand != nullptr)
            handleCommand(currentCommand);
    }
}

void State_GroundedArming::OnEnter()
{
    //when calling this function that means our intent is to arm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
//    auto commandArm = new MAVLINKVehicleControllers::CommandARM(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
//    commandArm->setLambda_Finished([this,commandArm](const bool completed, const uint8_t finishCode){
//        if(finishCode == MAV_RESULT_ACCEPTED)
//            armingCheck = true;
//        else
//            desiredState = ArdupilotFlightState::STATE_GROUNDED_IDLE;
//    });

//    MaceCore::ModuleCharacteristic target;
//    target.ID = Owner().getMAVLINKID();
//    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
//    MaceCore::ModuleCharacteristic sender;
//    sender.ID = 255;
//    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
//    CommandItem::ActionArm action(255,target.ID);
//    action.setVehicleArm(true);
//    commandArm->Send(action,sender,target);
//    currentControllers.insert({"armController",commandArm});

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

void State_GroundedArming::OnEnter(const AbstractCommandItem* command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        //in this case we dont want to handle the command right away, this is because we have to wait for the vehicle to arm
        this->clearCommand();
        currentCommand = command->getClone();
    }
}


} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_idle.h"
#include "ardupilot_states/state_grounded_armed.h"
