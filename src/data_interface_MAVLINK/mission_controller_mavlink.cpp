#include "mission_controller_mavlink.h"

namespace DataInterface_MAVLINK {

MissionController_MAVLINK::MissionController_MAVLINK(const int &targetID, const int &originatingID):
    mToExit(false), currentCommsState(NEUTRAL), currentRetry(0), maxRetries(5), responseTimeout(1000), prevTransmit(NULL)
{
    systemID = targetID;
    transmittingID = originatingID;
}


void MissionController_MAVLINK::clearPreviousTransmit()
{
    delete prevTransmit;
    prevTransmit = NULL;
}

void MissionController_MAVLINK::requestMission()
{
    currentCommsState = RECEIVING;

    mavlink_mission_request_list_t request;
    request.mission_type = MAV_MISSION_TYPE_MISSION;
    request.target_system = systemID;
    request.target_component = 0;

    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mavlink_mission_request_list_t>(commsItemEnum::ITEM_RXLIST, request);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReqList(request);
    currentRetry = 0;
    this->start();
    mTimer.start();
}

void MissionController_MAVLINK::transmitMission(const MissionItem::MissionList &missionQueue)
{
    if(missionQueue.getVehicleID() == this->systemID)
    {
        this->missionList = missionQueue;
    }

    currentCommsState = TRANSMITTING;
    mavlink_mission_count_t count;
    count.count = this->missionList.getQueueSize();
    count.mission_type = MAV_MISSION_TYPE_MISSION;
    count.target_component = 0;
    count.target_system = systemID;

    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mavlink_mission_count_t>(commsItemEnum::ITEM_TXCOUNT, count);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count);

    currentRetry = 0;
    this->start();
    mTimer.start();
}

void MissionController_MAVLINK::transmitMissionItem(const mavlink_mission_request_t &missionRequest)
{
    m_LambdasToRun.push_back([this, &missionRequest]{
        mTimer.stop();
        int index = missionRequest.seq;

        currentCommsState = TRANSMITTING;
        mavlink_mission_item_t missionItem;


        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->missionList.getMissionItem(index - 1);
        DataMAVLINK::Helper_MissionMACEtoMAVLINK::MACEMissionToMAVLINKMission(ptrItem,index,missionItem);

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mavlink_mission_item_t>(commsItemEnum::ITEM_TXITEM, missionItem);

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionItem(missionItem);
        currentRetry = 0;
        mTimer.start();
    });
}


void MissionController_MAVLINK::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPreviousTransmit();
            mTimer.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock shoud
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = mTimer.elapsedMilliseconds();

        if(timeElapsed > responseTimeout)
        {
            commsItemEnum type = prevTransmit->getType();
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
            case RECEIVING:
            {
                if(type == commsItemEnum::ITEM_RXLIST)
                {
                    std::cout<<"Mission Controller attempt "<<currentRetry<<" for "<<getCommsItemEnumString(type)<<std::endl;
                    PreviousTransmission<mavlink_mission_request_list_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_list_t>*>(prevTransmit);
                    mavlink_mission_request_list_t msgTransmit = tmp->getData();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReqList(msgTransmit);
                    mTimer.start();
                }
                else if(type == commsItemEnum::ITEM_RXITEM)
                {
                    std::cout<<"Mission Controller attempt "<<currentRetry<<" for "<<getCommsItemEnumString(type)<<std::endl;
                    PreviousTransmission<mavlink_mission_request_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_t>*>(prevTransmit);
                    mavlink_mission_request_t msgTransmit = tmp->getData();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit);
                    mTimer.start();
                }
                break;
            }
            case TRANSMITTING:
            {

                if(type == commsItemEnum::ITEM_TXCOUNT)
                {
                    std::cout<<"Mission Controller attempt "<<currentRetry<<" for "<<getCommsItemEnumString(type)<<std::endl;
                    PreviousTransmission<mavlink_mission_count_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_count_t>*>(prevTransmit);
                    mavlink_mission_count_t msgTransmit = tmp->getData();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionCount(msgTransmit);
                    mTimer.start();
                }
                else if(type == commsItemEnum::ITEM_TXITEM)
                {
                    std::cout<<"Mission Controller attempt "<<currentRetry<<" for "<<getCommsItemEnumString(type)<<std::endl;
                    PreviousTransmission<mavlink_mission_item_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_item_t>*>(prevTransmit);
                    mavlink_mission_item_t msgTransmit = tmp->getData();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionItem(msgTransmit);
                    mTimer.start();
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MissionController_MAVLINK::receivedMissionCount(const mavlink_mission_count_t &missionCount)
{
//    m_LambdasToRun.push_back([this, &missionCount]{
//        mTimer.stop();
//        this->missionList.initializeQueue(missionCount.count);

//        mavlink_mission_request_t request;
//        request.mission_type = MAV_MISSION_TYPE_MISSION;
//        request.seq = 0;
//        request.target_system = systemID;
//        request.target_component = 0;

//        clearPreviousTransmit();
//        prevTransmit = new PreviousTransmission<mavlink_mission_request_t>(PreviousTransmissionBase::ITEM_RXITEM, request);

//        if(m_CB)
//            m_CB->cbiMissionController_TransmitMissionReq(request);
//        currentRetry = 0;
//        mTimer.start();
//    });
}


void MissionController_MAVLINK::receivedMissionACK(const mavlink_mission_ack_t &missionAck)
{
    mTimer.stop();
    currentRetry = 0;
}

void MissionController_MAVLINK::recievedMissionItem(const mavlink_mission_item_t &missionItem)
{
    m_LambdasToRun.push_back([this, &missionItem]{
    mTimer.stop();
    currentRetry = 0;
    clearPreviousTransmit();

    int index = missionItem.seq;

    if(index == 0) //This implies we recieved the home position according to the mission
    {
        //This is the home position item associated with the vehicle
        CommandItem::SpatialHome newHome;
        newHome.position.setX(missionItem.x);
        newHome.position.setY(missionItem.y);
        newHome.position.setZ(missionItem.z);
        newHome.setOriginatingSystem(transmittingID);
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
        request.target_system = systemID;
        request.target_component = 0;

        prevTransmit = new PreviousTransmission<mavlink_mission_request_t>(commsItemEnum::ITEM_RXITEM, request);

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request);
        currentRetry = 0;
        mTimer.start();

    }else{
        m_CB->cbiMissionController_ReceivedMission(this->missionList);
    }
    });
}

} //end of namespace DataInterface_MAVLINK
