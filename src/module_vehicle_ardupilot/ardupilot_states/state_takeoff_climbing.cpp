#include "state_takeoff_climbing.h"

namespace ardupilot{
namespace state{

State_TakeoffClimbing::State_TakeoffClimbing():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(2,10,10);
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
}

void State_TakeoffClimbing::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_TakeoffClimbing::getClone() const
{
    return (new State_TakeoffClimbing(*this));
}

void State_TakeoffClimbing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffClimbing(*this);
}

hsm::Transition State_TakeoffClimbing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            rtn = hsm::SiblingTransition<State_TakeoffTransitioning>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF_COMPLETE:
        {
            rtn = hsm::SiblingTransition<State_TakeoffComplete>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_TakeoffClimbing::handleCommand(const AbstractCommandItem* command)
{
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_NAV_TAKEOFF:
    {
        const CommandItem::SpatialTakeoff* cmd = command->getClone()->as<CommandItem::SpatialTakeoff>();
        if(cmd->getPosition().getPosZFlag())
        {
            StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
            StateGlobalPosition targetPosition(currentPosition.getX(), currentPosition.getY(), cmd->getPosition().getZ());
            double distance = fabs(currentPosition.deltaAltitude(targetPosition));
            MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), targetPosition, distance);
            Owner().callTargetCallback(vehicleTarget);

            Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd,targetPosition]
            {
                if(cmd->getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
                {
                    StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.deltaAltitude(targetPosition));
                    MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), targetPosition, distance);
                    Owner().callTargetCallback(vehicleTarget);

                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        if(cmd->getPosition().has3DPositionSet())
                        {
                            this->currentCommand = cmd;
                            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
                        }
                        else
                        {
                            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_COMPLETE;
                        }
                    }

                }
            });


            auto controllerClimb = new MAVLINKVehicleControllers::CommandTakeoff(&Owner(), controllerQueue, Owner().getCommsObject()->getLinkChannel());
            controllerClimb->setLambda_Finished([this,controllerClimb](const bool completed, const uint8_t finishCode){
                if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                    GetImmediateOuterState()->setDesiredStateEnum(ArdupilotFlightState::STATE_GROUNDED);
                controllerClimb->Shutdown();
            });

            controllerClimb->setLambda_Shutdown([this,controllerClimb]()
            {
                currentControllerMutex.lock();
                currentControllers.erase("takeoffClimb");
                delete controllerClimb;
                currentControllerMutex.unlock();
            });

            MaceCore::ModuleCharacteristic target;
            target.ID = Owner().getMAVLINKID();
            target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            MaceCore::ModuleCharacteristic sender;
            sender.ID = 255;
            sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            controllerClimb->Send(*cmd,sender,target);
            currentControllers.insert({"takeoffClimb",controllerClimb});
        }
        break;
    }
    default:
        break;
    }
}

void State_TakeoffClimbing::Update()
{

}

void State_TakeoffClimbing::OnEnter()
{
    //By default I dont think there are any actions that we need to do
}

void State_TakeoffClimbing::OnEnter(const AbstractCommandItem *command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
        delete command;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_takeoff_transitioning.h"
#include "ardupilot_states/state_takeoff_complete.h"
