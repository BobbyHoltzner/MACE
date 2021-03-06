#include "state_grounded_disarming.h"

namespace ardupilot{
namespace state{

State_GroundedDisarming::State_GroundedDisarming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_DISARMING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_GROUNDED_DISARMING;
    desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_DISARMING;
}

AbstractStateArdupilot* State_GroundedDisarming::getClone() const
{
    return (new State_GroundedDisarming(*this));
}

void State_GroundedDisarming::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedDisarming(*this);
}

hsm::Transition State_GroundedDisarming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_GroundedDisarming."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_GroundedDisarming::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    COMMANDITEM type = command->getCommandType();
    switch (type) {
    case COMMANDITEM::CI_ACT_ARM:
    {
        break;
    }
    default:
        clearCommand();
        currentCommand = command->getClone();
        break;
    }
}

void State_GroundedDisarming::Update()
{
    if(Owner().state->vehicleArm.get().getSystemArm() == false)
    {
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_IDLE;
    }
}

void State_GroundedDisarming::OnEnter()
{
    //when calling this function that means our intent is to disarm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerArm = new MAVLINKVehicleControllers::CommandARM(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerArm->AddLambda_Finished(this, [this,controllerArm](const bool completed, const uint8_t finishCode){
        controllerArm->Shutdown();
        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
            desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_ARMED;
    });

    controllerArm->setLambda_Shutdown([this, collection]()
    {
        auto ptr = collection->Remove("disarmController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    CommandItem::ActionArm action(255, Owner().getMAVLINKID());
    action.setVehicleArm(false);
    controllerArm->Send(action,sender,target);
    printf("Adding disarmController %x\n", controllerArm);
    collection->Insert("disarmController",controllerArm);
}

void State_GroundedDisarming::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_idle.h"
#include "ardupilot_states/state_grounded_armed.h"
