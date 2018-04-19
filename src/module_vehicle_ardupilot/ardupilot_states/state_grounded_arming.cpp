#include "state_grounded_arming.h"

namespace ardupilot{
namespace state{

State_GroundedArming::State_GroundedArming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMING"<<std::endl;
    this->currentState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
    this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMING;
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
        destroyCurrentControllers();
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredState) {
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::InnerEntryTransition<State_GroundedIdle>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMED:
        {
            return hsm::InnerEntryTransition<State_GroundedArmed>();
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
    COMMANDITEM commandType = command->getCommandType();
    switch (commandType) {
    case COMMANDITEM::CI_ACT_ARM: //This command is to be handled when in this state
    {

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

//    if(Owner().state->vehicleArm.get().getSystemArm())
//        this->desiredState = ArdupilotFlightState::STATE_GROUNDED_ARMED;
}

void State_GroundedArming::OnEnter()
{
    //when calling this function that means our intent is to arm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    auto commandArm = new MAVLINKVehicleControllers::CommandARM(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
    commandArm->setLambda_Finished([this,commandArm](const bool completed, const uint8_t finishCode){
        std::cout<<"We are in the lambda finished function: "<<completed<<std::endl;
    });

    MaceCore::ModuleCharacteristic target;
    target.ID = Owner().getMAVLINKID();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
    MaceCore::ModuleCharacteristic sender;
    sender.ID = 255;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
    CommandItem::ActionArm action(255,target.ID);
    action.setVehicleArm(true);
    commandArm->Send(action,sender,target);
    currentControllers.insert({"armController",commandArm});
}

void State_GroundedArming::OnEnter(const AbstractCommandItem* command)
{
    this->OnEnter();
}


} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_idle.h"
#include "ardupilot_states/state_grounded_armed.h"
