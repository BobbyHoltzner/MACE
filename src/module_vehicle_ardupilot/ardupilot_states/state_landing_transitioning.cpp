#include "state_landing_transitioning.h"

namespace ardupilot{
namespace state{

State_LandingTransitioning::State_LandingTransitioning():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(2,10,10);
    std::cout<<"We are in the constructor of STATE_LANDING_TRANSITIONING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
}

void State_LandingTransitioning::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_LandingTransitioning::getClone() const
{
    return (new State_LandingTransitioning(*this));
}

void State_LandingTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingTransitioning(*this);
}

hsm::Transition State_LandingTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingTransitioning::handleCommand(const AbstractCommandItem* command)
{
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_NAV_LAND:
    {
        currentCommand = command->getClone();
        const CommandItem::SpatialLand* cmd = currentCommand->as<CommandItem::SpatialLand>();
        if(cmd->getPosition().has3DPositionSet())
        {
            Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd]
            {
                if(cmd->getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
                {
                    StateGlobalPosition cmdPos(cmd->getPosition().getX(),cmd->getPosition().getY(),cmd->getPosition().getZ());
                    StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.distanceBetween2D(cmdPos));
                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                    std::cout<<"The distance to the target is approximately: "<<distance<<std::endl;
                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
                    }

                }
            });

            Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();
            auto landingTransitioning = new MAVLINKVehicleControllers::ControllerGuidedMissionItem<CommandItem::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
            landingTransitioning->AddLambda_Finished(this, [this,landingTransitioning](const bool completed, const uint8_t finishCode){
                if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                    std::cout<<"We are not going to perform the transition portion of the landing."<<std::endl;
                landingTransitioning->Shutdown();
            });

            landingTransitioning->setLambda_Shutdown([this, collection]()
            {
                auto ptr = collection->Remove("landingTransition");
                delete ptr;
            });

            MaceCore::ModuleCharacteristic target;
            target.ID = cmd->getTargetSystem();
            target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            MaceCore::ModuleCharacteristic sender;
            sender.ID = 255;
            sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            Base3DPosition cmdPosition = cmd->getPosition();
            CommandItem::SpatialWaypoint landingTarget(255,cmd->getTargetSystem());
            landingTarget.setPosition(cmdPosition);
            landingTransitioning->Send(landingTarget,sender,target);
            collection->Insert("landingTransition",landingTransitioning);
        }
        break;
    }
    default:
        break;
    }
}

void State_LandingTransitioning::Update()
{

}

void State_LandingTransitioning::OnEnter()
{

}

void State_LandingTransitioning::OnEnter(const AbstractCommandItem *command)
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
