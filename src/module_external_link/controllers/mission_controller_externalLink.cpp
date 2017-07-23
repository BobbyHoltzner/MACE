#include "mission_controller_externalLink.h"

namespace ExternalLink {

MissionController_ExternalLink::MissionController_ExternalLink(const int &targetID, const int &originatingID):
    systemID(targetID), transmittingID(originatingID),
    mToExit(false), currentRetry(0), maxRetries(5), responseTimeout(5000),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL),
    helperMAVtoMACE(targetID),helperMACEtoMAV(originatingID,0)
{
    mLog = spdlog::get("Log_Vehicle" + std::to_string(this->systemID));
    missionList.setCreatorID(systemID);
    missionList.setVehicleID(systemID);
}


void MissionController_ExternalLink::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void MissionController_ExternalLink::requestMission()
{
    mLog->info("Mission Controller has seen a request mission.");
    missionList.setCreatorID(systemID);
    missionList.setVehicleID(systemID);
    this->missionList.clearQueue();
    currentCommsState = Data::ControllerCommsState::RECEIVING;

    mavlink_mission_request_list_t request;
    request.mission_type = MAV_MISSION_TYPE_MISSION;
    request.target_system = systemID;
    request.target_component = 0;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mavlink_mission_request_list_t>(commsItemEnum::ITEM_RXLIST, request);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReqList(request);
    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();
}

void MissionController_ExternalLink::transmitMission(const MissionItem::MissionList &missionQueue)
{
    mLog->info("Mission Controller has been instructed to transmit a mission.");

    if(missionQueue.getVehicleID() == this->systemID)
    {
        this->missionList = missionQueue;
    }

    Data::MissionKey key = missionQueue.getMissionKey();

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    mace_mission_count_t count;

    missionProposed.mission_state = static_cast<MAV_MISSION_STATE>(missionList.getMissionTXState());
    missionProposed.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);

    count.count = this->missionList.getQueueSize();
    count.target_system = systemID;
    count.mission_system = key.m_missionID;
    count.mission_creator = key.m_creatorID;
    count.mission_id = key.m_missionID;
    count.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
    count.mission_state = static_cast<MAV_MISSION_STATE>(missionQueue.getMissionTXState());

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_count_t>(commsItemEnum::ITEM_TXCOUNT, count);

    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count);
}

void MissionController_ExternalLink::transmitMissionItem(const mace_mission_request_item_t &missionRequest)
{
    m_LambdasToRun.push_back([this, missionRequest]{
        mTimer.stop();
        int index = missionRequest.seq;

        mLog->info("Mission Controller has seen a request for mission item number " + std::to_string(index) + ".");

        currentCommsState = Data::ControllerCommsState::TRANSMITTING;
        mace_mission_item_t missionItem;

        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->missionList.getMissionItem(index - 1);
        helperMACEtoMAV.MACEMissionToMAVLINKMission(ptrItem,index,missionItem);

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mavlink_mission_item_t>(commsItemEnum::ITEM_TXITEM, missionItem);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionItem(missionItem);
    });
}


void MissionController_ExternalLink::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
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
            case Data::ControllerCommsState::NEUTRAL:
            {
                //This case we should terminate this because there is nothing we should be doing apparently
                clearPreviousTransmit();
                mTimer.stop();
                mToExit = true;
             break;
            }
            case Data::ControllerCommsState::RECEIVING:
            {
                if(type == commsItemEnum::ITEM_RXLIST)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_request_list_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_list_t>*>(prevTransmit);
                    mavlink_mission_request_list_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReqList(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_RXITEM)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mavlink_mission_request_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_t>*>(prevTransmit);
                    mavlink_mission_request_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit);
                }
                break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == commsItemEnum::ITEM_TXCOUNT)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_count_t> *tmp = static_cast<PreviousTransmission<mace_mission_count_t>*>(prevTransmit);
                    mace_mission_count_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionCount(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_TXITEM)
                {
                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_item_t> *tmp = static_cast<PreviousTransmission<mace_mission_item_t>*>(prevTransmit);
                    mace_mission_item_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionItem(msgTransmit);
                }
                break;
            }
            }

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MissionController_ExternalLink::receivedMissionCount(const mavlink_mission_count_t &missionCount)
{
    m_LambdasToRun.push_back([this, missionCount]{
        mLog->info("Mission Controller received a mission count of " + std::to_string(missionCount.count));
        mTimer.stop();
        this->missionList.initializeQueue(missionCount.count - 1);

        mace_mission_request_item_t request;
        request.mission_type = MAV_MISSION_TYPE_MISSION;
        request.seq = 0;
        request.target_system = systemID;
        request.target_component = 0;

        mLog->info("Mission Controller is requesting mission item " + std::to_string(0));

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request);
        currentRetry = 0;
        mTimer.start();
        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request);
    });
}


void MissionController_ExternalLink::receivedMissionACK(const mavlink_mission_ack_t &missionACK)
{    
    m_LambdasToRun.push_back([this, missionACK]{
        mTimer.stop();
        currentRetry = 0;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        if(m_CB)
            m_CB->cbiMissionController_MissionACK(missionACK);
    });

}

void MissionController_ExternalLink::recievedMissionItem(const mace_mission_item_t &missionItem)
{
    m_LambdasToRun.push_back([this, missionItem]{
    mTimer.stop();
    currentRetry = 0;
    int index = missionItem.seq;
    mLog->info("Mission Controller received mission item " + std::to_string(index));

    if(index > (this->missionList.getQueueSize() + 1))
    {
        mTimer.start();
        return;
    }

    else{
        int adjustedIndex = index;
        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = helperMAVtoMACE.Convert_MAVLINKTOMACE(missionItem);
        this->missionList.replaceMissionItemAtIndex(newMissionItem,adjustedIndex);
    }

    MissionItem::MissionList::MissionListStatus status = this->missionList.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        int indexRequest = status.remainingItems.at(0)+1;
        mLog->info("Mission Controller is requesting mission item " + std::to_string(indexRequest));
        mace_mission_request_item_t request;
        request.mission_type = MAV_MISSION_TYPE_MISSION;
        request.seq = indexRequest;
        request.target_system = systemID;
        request.target_component = 0;

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request);
    }else{
        mLog->info("Mission Controller has received the entire mission of " + std::to_string(this->missionList.getQueueSize()));
        clearPendingTasks();
        mToExit = true;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        m_CB->cbiMissionController_ReceivedMission(this->missionList);
    }
    });
}

} //end of namespace DataInterface_MAVLINK
