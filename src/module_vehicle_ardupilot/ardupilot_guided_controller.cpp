#include "ardupilot_guided_controller.h"


Ardupilot_GuidedController::Ardupilot_GuidedController(Comms::CommsMarshaler *commsMarshaler) :
    m_LinkMarshaler(commsMarshaler),mToExit(false),vehicleMode(""),executionState(false),
    currentPosition(DataState::StateGlobalPosition()),currentAttitude(DataState::StateAttitude())
{
    vehicleMissionState = ArdupilotMissionState(3,10,10);

    std::cout << "Constructor on guidance controller" << std::endl;
}

void Ardupilot_GuidedController::updateAttitudeTopic(const DataStateTopic::StateAttitudeTopic &attitudeTopic)
{
    attitudeUpdated = true;
}

void Ardupilot_GuidedController::updateGlobalPositionTopic(const DataStateTopic::StateGlobalPositionTopic &globalPositionTopic)
{
    positionUpdated = true;
}

void Ardupilot_GuidedController::updatedMission(const MissionItem::MissionList &updatedMission)
{
    m_CurrentMission = updatedMission;
}

void Ardupilot_GuidedController::initializeMissionSequence()
{
    executionState = true;
}

double Ardupilot_GuidedController::distanceToTarget(){
    std::shared_ptr<MissionItem::AbstractMissionItem> currentMissionItem = m_CurrentMission.getActiveMissionItem();
    double distance = 0.0;
    switch(currentMissionItem->getMissionType())
    {
    case(MissionItem::MissionItemType::WAYPOINT):
    {
        if(currentMissionItem->getPositionalFrame() == Data::PositionalFrame::GLOBAL)
        {
            std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> castItem = std::dynamic_pointer_cast<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>(currentMissionItem);
            distance = castItem->position.distanceBetween3D(currentPosition);
        }
    break;
    }
    default:
    {
        std::cout<<"I do not understand this type of mission item. I am moving on."<<std::endl;
    }
    }

    return distance;
}

void Ardupilot_GuidedController::generateControl(const Data::MissionState &currentState)
{
    switch(currentState){
    case Data::MissionState::ROUTING:
    {
        std::cout<<"I am still routing to the waypoint"<<std::endl;
        break;
    }
    case Data::MissionState::HUNTING:
    {
        std::cout<<"I am hunting for the waypoint"<<std::endl;
        break;
    }
    case Data::MissionState::ACHIEVED:
    {
        m_CurrentMission.setActiveIndex(m_CurrentMission.getActiveIndex() + 1);
        std::cout<<"I have acheived the waypoint"<<std::endl;
        break;
    }
    }
}


void Ardupilot_GuidedController::run()
{
    while(true)
    {
        if(mToExit == true) {
            break;
        }
        if(positionUpdated)
        {
            if(executionState){
                //let us see how close we are to our target
                double distance = distanceToTarget();
                Data::MissionState currentState = vehicleMissionState.updateMissionState(distance);
                generateControl(currentState);
            }
            std::cout<<"The position has been updated in the thread"<<std::endl;
            positionUpdated = false;

        }
        if(attitudeUpdated)
        {
            attitudeUpdated = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
