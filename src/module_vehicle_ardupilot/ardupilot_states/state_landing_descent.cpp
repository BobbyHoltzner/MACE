#include "state_landing_descent.h"

namespace ardupilot{
namespace state{

State_LandingDescent::State_LandingDescent():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(0,10,10);
    std::cout<<"We are in the constructor of STATE_LANDING_DESCENT"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
}

void State_LandingDescent::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_LandingDescent::getClone() const
{
    return (new State_LandingDescent(*this));
}

void State_LandingDescent::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingDescent(*this);
}

hsm::Transition State_LandingDescent::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_LANDING_COMPLETE:
        {
            rtn = hsm::SiblingTransition<State_LandingComplete>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_LANDING_DESCENT."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingDescent::handleCommand(const AbstractCommandItem* command)
{
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_NAV_LAND:
    {
        currentCommand = command->getClone();
        const CommandItem::SpatialLand* cmd = currentCommand->as<CommandItem::SpatialLand>();
        StateGlobalPosition cmdPos(cmd->getPosition().getX(),cmd->getPosition().getY(),cmd->getPosition().getZ());
        cmdPos.setCoordinateFrame(cmd->getPosition().getCoordinateFrame());
        Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd,cmdPos]
        {
            if(cmdPos.getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
            {
                StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
                double distance = fabs(currentPosition.deltaAltitude(cmdPos));
                MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), cmdPos, distance);
                Owner().callTargetCallback(vehicleTarget);
                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                if(guidedState == Data::ControllerState::ACHIEVED)
                {
                    this->clearCommand();
                    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_COMPLETE;
                }
            }
        });


        auto controllerDescent = new MAVLINKVehicleControllers::CommandLand(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
        controllerDescent->setLambda_Finished([this,controllerDescent](const bool completed, const uint8_t finishCode){
            if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                GetImmediateOuterState()->setDesiredStateEnum(ArdupilotFlightState::STATE_FLIGHT);
            controllerDescent->Shutdown();
        });

        controllerDescent->setLambda_Shutdown([this,controllerDescent]()
        {
            currentControllerMutex.lock();
            currentControllers.erase("landingDescent");
            delete controllerDescent;
            currentControllerMutex.unlock();
        });

        MaceCore::ModuleCharacteristic target;
        target.ID = Owner().getMAVLINKID();
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
        MaceCore::ModuleCharacteristic sender;
        sender.ID = 255;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        controllerDescent->Send(*cmd,sender,target);
        currentControllers.insert({"landingDescent",controllerDescent});
        break;
    }
    default:
        break;
    }
}

void State_LandingDescent::Update()
{

}

void State_LandingDescent::OnEnter()
{

}

void State_LandingDescent::OnEnter(const AbstractCommandItem *command)
{
    if(command != nullptr)
    {
        handleCommand(command);
        delete command;
    }
    else
        OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_landing_complete.h"
