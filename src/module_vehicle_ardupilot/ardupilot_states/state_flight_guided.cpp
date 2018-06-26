#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArdupilot(), guidedTimeout(nullptr), currentQueue(nullptr)
{
    guidedTimeout = new GuidedTimeoutController(this, 1000);
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
}

void State_FlightGuided::OnExit()
{
    guidedTimeout->stop();
    delete guidedTimeout;

    Owner().state->vehicleLocalPosition.RemoveNotifier(this);
    Owner().mission->currentDynamicQueue_LocalCartesian.RemoveNotifier(this);

    Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();
//    auto globalPtr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global<MAVLINKVehicleControllers::TargetControllerStructGlobal>*>(collection->At("globalGuidedController"));
//    if(globalPtr != nullptr)
//        globalPtr->Shutdown();
    auto localPtr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Local<MAVLINKVehicleControllers::TargetControllerStructLocal>*>(collection->At("localGuidedController"));
    if(localPtr != nullptr)
        localPtr->Shutdown();

    delete currentQueue;
}

AbstractStateArdupilot* State_FlightGuided::getClone() const
{
    return (new State_FlightGuided(*this));
}

void State_FlightGuided::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided(*this);
}

hsm::Transition State_FlightGuided::GetTransition()
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

bool State_FlightGuided::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_MISSIONCOMMAND:
    {
        std::cout<<"We saw that we are going to handle the command in guided mode."<<std::endl;
        guidedTimeout->start();
        //Once we get the command that we can go, we need to announce the current mission item

        currentQueue = new TargetItem::DynamicMissionQueue(Owner().mission->currentDynamicQueue_LocalCartesian.get());
        this->initializeNewTargetList();

        Owner().mission->currentDynamicQueue_LocalCartesian.AddNotifier(this,[this]{
    //        std::lock_guard<std::mutex> guard(MUTEXTargetQueue);
            currentQueue = new TargetItem::DynamicMissionQueue(Owner().mission->currentDynamicQueue_LocalCartesian.get());
            this->initializeNewTargetList();
        });

        Owner().state->vehicleLocalPosition.AddNotifier(this,[this]
        {
            if((currentQueue->getDynamicTargetList()->listSize() > 0) && (currentQueue->getDynamicTargetList()->getNextIncomplete() != nullptr))
            {
                //std::lock_guard<std::mutex> guard(MUTEXTargetQueue);
                DataState::StateLocalPosition CP = Owner().state->vehicleLocalPosition.get();
                //first we need to rotate this if it is applicable
                if(CP.getCoordinateFrame() == Data::CoordinateFrameType::CF_LOCAL_NED)
                {
                    CP.setPosition(CP.getY(),CP.getX(),-CP.getZ());
                    CP.setCoordinateFrame(Data::CoordinateFrameType::CF_LOCAL_ENU);
                }
                mace::pose::CartesianPosition_3D currentPosition(CP.getPositionX(),CP.getPositionY(),CP.getPositionZ());
                //std::cout<<"The current position here is: "<<CP.getPositionX()<<" "<<CP.getPositionY()<<std::endl;
                unsigned int currentTargetIndex = currentQueue->getDynamicTargetList()->getActiveTargetItem();
                //std::cout<<"The active target item here is: "<<currentTargetIndex<<std::endl;
                const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getTargetPointerAtIndex(currentTargetIndex);
                double distance = currentPosition.distanceBetween3D(target->getPosition());
                //std::cout<<"The current target position here is: "<<target->getPosition().getXPosition()<<" "<<target->getPosition().getYPosition()<<" "<<target->getPosition().getZPosition()<<std::endl;

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                handleGuidedState(currentPosition, currentTargetIndex, guidedState, distance);
            }
        });


//        MissionItem::MissionKey testKey(1,1,1,MissionItem::MISSIONTYPE::GUIDED);
//        TargetItem::DynamicMissionQueue availableQueue(testKey,1);

//        TargetItem::CartesianDynamicTarget target;
//        target.setPosition(mace::pose::CartesianPosition_3D(0,100,15));
//        availableQueue.getDynamicTargetList()->appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

//        target.setPosition(mace::pose::CartesianPosition_3D(100,100,15));
//        availableQueue.getDynamicTargetList()->appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

//        target.setPosition(mace::pose::CartesianPosition_3D(100,-100,15));
//        availableQueue.getDynamicTargetList()->appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

//        target.setPosition(mace::pose::CartesianPosition_3D(-100,-100,15));
//        availableQueue.getDynamicTargetList()->appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

//        target.setPosition(mace::pose::CartesianPosition_3D(-100,100,15));
//        availableQueue.getDynamicTargetList()->appendDynamicTarget(target,TargetItem::DynamicTargetStorage::INCOMPLETE);

//        Owner().mission->currentDynamicQueue.set(availableQueue);

        break;
    }
    default:
        break;
    }
}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{
    Controllers::ControllerCollection<mavlink_message_t> *collection = Owner().ControllersCollection();

    //The following code is how we eventaully would like this to perform
    //However, there are current inconsistencies in the ardupilot branch
    //about what/how this functions and as well as the report of it
    //For now we are going to take a different route here

    //let us get the controllers ready for transmitting
//    auto controllerGuided_Global = new MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global<MAVLINKVehicleControllers::TargetControllerStructGlobal>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
//    controllerGuided_Global->AddLambda_Finished(this, [this, controllerGuided_Global](const bool completed, const uint8_t finishCode){
//        if(!completed)
//        {
//            std::cout<<"The ardupilot rejected this command."<<std::endl;
//        }
//        else
//        {
//            std::cout<<"The ardupilot is heading towards the target."<<std::endl;
//        }
//    });

//    controllerGuided_Global->setLambda_Shutdown([this, collection]()
//    {
//        auto ptr = collection->Remove("globalGuidedController");
//        delete ptr;
//    });

//    collection->Insert("globalGuidedController", controllerGuided_Global);

    auto controllerGuided_Local = new MAVLINKVehicleControllers::ControllerGuidedTargetItem_Local<MAVLINKVehicleControllers::TargetControllerStructLocal>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerGuided_Local->setLambda_Shutdown([this, collection]()
    {
        auto ptr = collection->Remove("localGuidedController");
        delete ptr;
    });

    collection->Insert("localGuidedController", controllerGuided_Local);


}

void State_FlightGuided::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

void State_FlightGuided::initializeNewTargetList()
{
    MissionItem::MissionKey associatedKey = currentQueue->getAssociatedMissionKey();
    unsigned int associatedIndex = currentQueue->getAssociatedMissionItem();

    MissionItem::MissionItemCurrent currentMissionItem(associatedKey,associatedIndex);
    Owner().getCallbackInterface()->cbi_VehicleMissionItemCurrent(currentMissionItem);

    unsigned int activeTargetIndex = currentQueue->getDynamicTargetList()->getActiveTargetItem();
    const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getNextIncomplete();

    if(target != nullptr)
    {
        guidedTimeout->updateTarget(*target);
        this->cbiArdupilotTimeout_TargetLocal(*target);
    }
    else
    {
        MissionItem::MissionItemAchieved achievement(currentQueue->getAssociatedMissionKey(),currentQueue->getAssociatedMissionItem());
        std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(achievement);
        Owner().getCallbackInterface()->cbi_VehicleMissionData(Owner().getMAVLINKID(),ptrMissionTopic);
    }
}

void State_FlightGuided::handleGuidedState(const mace::pose::CartesianPosition_3D currentPosition, const unsigned int currentTargetIndex,
                                           const Data::ControllerState &state, const double targetDistance)
{
    if(state == Data::ControllerState::ACHIEVED)
    {

        const TargetItem::CartesianDynamicTarget* newTarget = currentQueue->getDynamicTargetList()->markCompletionState(currentTargetIndex,TargetItem::DynamicTargetStorage::TargetCompletion::COMPLETE);
        if(newTarget == nullptr)
        {
            std::cout<<"The are no more points in the queue"<<std::endl;
            //if there are no more points in the queue this mission item is completed
            MissionItem::MissionItemAchieved achievement(currentQueue->getAssociatedMissionKey(),currentQueue->getAssociatedMissionItem());
            std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(achievement);
            Owner().getCallbackInterface()->cbi_VehicleMissionData(Owner().getMAVLINKID(),ptrMissionTopic);
        }
        else //there is a new target
        {
            std::cout<<"The is another point in the queue"<<std::endl;
            unsigned int currentTargetIndex = currentQueue->getDynamicTargetList()->getActiveTargetItem();
            const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getTargetPointerAtIndex(currentTargetIndex);

            //update the people that we have a new target
            guidedTimeout->updateTarget(*target);
            this->cbiArdupilotTimeout_TargetLocal(*target);

            double distance = currentPosition.distanceBetween3D(target->getPosition());
            Data::ControllerState guidedState = guidedProgress.newTargetItem(distance);
            handleGuidedState(currentPosition, currentTargetIndex, guidedState, distance);
        }
        //advance to the next desired dynamic state
    }
    else //we are either hunting or tracking the state
    {
        if(Owner().mission->vehicleHomePosition.hasBeenSet())
        {
            const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getTargetPointerAtIndex(currentTargetIndex);
            announceTargetState(*target,targetDistance);
        }

    }
}

void State_FlightGuided::announceTargetState(const TargetItem::CartesianDynamicTarget &target, const double &targetDistance)
{
    CommandItem::SpatialHome home = Owner().mission->vehicleHomePosition.get();
    mace::pose::GeodeticPosition_3D homePos(home.getPosition().getX(),home.getPosition().getY(),home.getPosition().getZ());
    mace::pose::GeodeticPosition_3D targetPos;
    DynamicsAid::LocalPositionToGlobal(homePos,target.getPosition(),targetPos);
    if(targetPos.getAltitude() < 1)
        std::cout<<"We need to check this."<<std::endl;
    Base3DPosition targetPositionCast(targetPos.getLatitude(),targetPos.getLongitude(),targetPos.getAltitude());
    targetPositionCast.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
    MissionTopic::VehicleTargetTopic currentTarget(Owner().getMAVLINKID(),targetPositionCast, targetDistance);
    Owner().callTargetCallback(currentTarget);
}

} //end of namespace ardupilot
} //end of namespace state

