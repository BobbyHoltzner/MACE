#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK():
    mToExit(false), currentCommsState(NEUTRAL), currentRetry(0), maxRetries(5), responseTimeout(1000)
{

}

void MissionController_MAVLINK::transmitMission(const MissionItem::MissionList &missionQueue)
{
    this->missionList = missionQueue;

    currentCommsState = TRANSMITTING;
    mavlink_message_t msg;
    mavlink_mission_count_t count;
    count.count = this->missionList.getQueueSize();
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.target_component = 0;
    count.target_system = 0;
    //prevTransmit = new commsObject<mavlink_mission_count_t>(commsObject::ITEMCOUNT, count);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count);
}

void MissionController_MAVLINK::requestMission()
{
    currentCommsState = RECEIVING;
    mTimer.start();
    currentRetry = 0;

    mavlink_mission_request_list_t request;
    request.mission_type = MAV_MISSION_TYPE_FENCE;
    request.target_component = 0;
    request.target_system = 0;

    m_CB->cbiMissionController_TransmitMissionReq();
}

void MissionController_MAVLINK::run()
{
    while(true)
    {
        double timeElapsed = mTimer.elapsedMilliseconds();
        if(mToExit == true) {
            break;
        }
        if(timeElapsed > responseTimeout)
        {
            currentRetry++;

            switch(currentCommsState)
            {
            case NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                mTimer.stop();
                mToExit = true;
             break;
            }
            case TRANSMITTING:
            {
                break;
            }
            case RECEIVING:
            {
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MissionController_MAVLINK::receivedMissionCount(const mavlink_mission_count_t &missionCount)
{
    mTimer.stop();
    currentRetry = 0;
    this->missionList.initializeQueue(missionCount.count);

    mavlink_mission_request_t request;
    request.mission_type = MAV_MISSION_TYPE_MISSION;
    request.seq = 0;
    request.target_system = 0;
    request.target_component = 0;
    mTimer.start();
    m_CB->cbiMissionController_TransmitMissionReq(request);
}

void MissionController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionAck)
{
    mTimer.stop();
    currentRetry = 0;
}

void MissionController_MAVLINK::recievedMissionItem(const mavlink_mission_item_t &missionItem)
{
    mTimer.stop();
    currentRetry = 0;
    int index = missionItem.seq;

    if(index == 0) //This implies we recieved the home position according to the mission
    {
        //This is the home position item associated with the vehicle
        CommandItem::SpatialHome newHome;
        newHome.position.setX(decodedMSG.x);
        newHome.position.setY(decodedMSG.y);
        newHome.position.setZ(decodedMSG.z);
        newHome.setOriginatingSystem(sysID);
        m_CB->cbiMissionController_ReceviedHome(newHome);
    }else{
        int adjustedIndex = index - 1; //we decrement 1 only here because ardupilot references home as 0 and we 0 index in our mission queue
        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = helperMAVtoMACE.Convert_MAVLINKTOMACE(0,missionItem);
        this->missionList.replaceMissionItemAtIndex(newMissionItem,adjustedIndex);
    }

    MissionItem::MissionList::MissionListStatus status = this->missionList.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        int indexRequest = status.remainingItems.at(0)+1;
        std::cout << "Mission Controller Requesting: " << indexRequest << std::endl;

        mavlink_mission_request_t request;
        request.mission_type = MAV_MISSION_TYPE_MISSION;
        request.seq = indexRequest;
        request.target_system = 0;
        request.target_component = 0;

        mTimer.start();
        m_CB->cbiMissionController_TransmitMissionReq(request);
    }else{
        m_CB->cbiMissionController_ReceivedMission(this->missionList);
    }
}

} //end of namespace DataInterface_MAVLINK
