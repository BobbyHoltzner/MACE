#include "mission_controller_externalLink.h"

namespace ExternalLink {

MissionController_ExternalLink::MissionController_ExternalLink(const int &targetID, const int &originatingID):
    systemID(targetID), transmittingID(originatingID),
    mToExit(false), currentRetry(0), maxRetries(5), responseTimeout(5000),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL),
    helperCOMMStoMACE(targetID),helperMACEtoCOMMS(originatingID,0)
{
    //mLog = spdlog::get("Log_Vehicle" + std::to_string(this->systemID));
}


void MissionController_ExternalLink::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

///////////////////////////////////////////////////////////////////////////////
/// GENERAL TRANSMISSION EVENTS: These events are related to sending a mission
/// to a remote instance of MACE.
///////////////////////////////////////////////////////////////////////////////

void MissionController_ExternalLink::transmitMission(const MissionItem::MissionList &missionQueue)
{
    //mLog->info("Mission Controller has been instructed to transmit a mission.");

    if(missionQueue.getVehicleID() == this->systemID)
    {
        this->missionList = missionQueue;
    }
    else
    {
        std::cout<<"We cannot transmit the mission using this external link module."<<std::endl;
        return;
    }

    Data::MissionKey key = missionQueue.getMissionKey();

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;
    mace_mission_count_t count;

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
        Data::MissionKey key(missionRequest.target_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<Data::MissionType>(missionRequest.mission_type));

        if(key != this->missionList.getMissionKey()) //this indicates for some reason the other system requested a different mission?
            return;

        int index = missionRequest.seq;
        if(index > this->missionList.getQueueSize()) //this indicates that RX system requested something OOR
            return;

        mTimer.stop();
        //mLog->info("Mission Controller has seen a request for mission item number " + std::to_string(index) + ".");


        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->missionList.getMissionItem(index);
        mace_mission_item_t missionItem;
        helperMACEtoCOMMS.MACEMissionToCOMMSMission(ptrItem,index,missionItem);

        currentCommsState = Data::ControllerCommsState::TRANSMITTING;
        clearPendingTasks();
        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_mission_item_t>(commsItemEnum::ITEM_TXITEM, missionItem);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionItem(missionItem);
    });
}

void MissionController_ExternalLink::receivedMissionACK(const mace_mission_ack_t &missionACK)
{
    m_LambdasToRun.push_back([this, missionACK]{
        mTimer.stop();
        currentRetry = 0;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        if(m_CB)
            m_CB->cbiMissionController_MissionACK(missionACK);
    });

}

///////////////////////////////////////////////////////////////////////////////
/// GENERAL RECEIVING EVENTS: These events are related to receiving a mission
/// from a remote instance of MACE.
///////////////////////////////////////////////////////////////////////////////

void MissionController_ExternalLink::receivedMissionCount(const mace_mission_count_t &mission)
{
    //mLog->info("Mission Controller has seen a newly available mission.");

    //this is like the equivalent of receiving a mission count
    Data::MissionKey key(mission.target_system,mission.mission_creator,mission.mission_id,static_cast<Data::MissionType>(mission.mission_type));
    this->missionList.setMissionKey(key);
    this->missionList.initializeQueue(mission.count);

    mace_mission_request_item_t request;
    request.mission_creator = mission.mission_creator;
    request.mission_id = mission.mission_id;
    request.mission_system = mission.mission_system;
    request.mission_type = mission.mission_type;
    request.target_system = systemID;
    request.seq = 0;

    //mLog->info("Mission Controller is requesting mission item " + std::to_string(0));

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReq(request);
    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();
}

void MissionController_ExternalLink::recievedMissionItem(const mace_mission_item_t &missionItem)
{
    m_LambdasToRun.push_back([this, missionItem]{
        Data::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<Data::MissionType>(missionItem.mission_type));

        if(key != this->missionList.getMissionKey()) //this indicates for some reason the other system requested a different mission?
        {
            //mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
            return;
        }

        int index = missionItem.seq;
        if(index > (this->missionList.getQueueSize() - 1)) //this should never happen
        {
            //mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
            return;
        }

        mTimer.stop();
        currentRetry = 0;

        //mLog->info("Mission Controller received mission item " + std::to_string(index));

        helperCOMMStoMACE.Convert_COMMSTOMACE(missionItem);
        std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = helperCOMMStoMACE.Convert_COMMSTOMACE(missionItem);
        this->missionList.replaceMissionItemAtIndex(newMissionItem,index);
        MissionItem::MissionList::MissionListStatus status = this->missionList.getMissionListStatus();

        if(status.state == MissionItem::MissionList::INCOMPLETE)
        {
            int indexRequest = status.remainingItems.at(0);
            //mLog->info("Mission Controller is requesting mission item " + std::to_string(indexRequest));

            mace_mission_request_item_t request;
            request.target_system = systemID;
            request.mission_creator = key.m_creatorID;
            request.mission_id = key.m_missionID;
            request.mission_system = key.m_systemID;
            request.mission_type = (uint8_t)key.m_missionType;
            request.seq = indexRequest;

            clearPreviousTransmit();
            prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request);
            currentRetry = 0;
            mTimer.start();

            if(m_CB)
                m_CB->cbiMissionController_TransmitMissionReq(request);
        }else{
            //mLog->info("Mission Controller has received the entire mission of " + std::to_string(this->missionList.getQueueSize()));
            clearPendingTasks();
            mToExit = true;
            currentCommsState = Data::ControllerCommsState::NEUTRAL;
            m_CB->cbiMissionController_ReceivedMission(this->missionList);
        }
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
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_request_list_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_list_t>*>(prevTransmit);
                    mace_mission_request_list_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReqList(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_RXITEM)
                {
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_request_item_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_item_t>*>(prevTransmit);
                    mace_mission_request_item_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit);
                }
//                else if(type == commsItemEnum::ITEM_RXHOME)
//                {
//                    mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
//                    PreviousTransmission<mavlink_mission_request_t> *tmp = static_cast<PreviousTransmission<mavlink_mission_request_t>*>(prevTransmit);
//                    mavlink_mission_request_t msgTransmit = tmp->getData();
//                    mTimer.start();
//                    if(m_CB)
//                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit);
//                }
                break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == commsItemEnum::ITEM_TXCOUNT)
                {
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_count_t> *tmp = static_cast<PreviousTransmission<mace_mission_count_t>*>(prevTransmit);
                    mace_mission_count_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionCount(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_TXITEM)
                {
                    //mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
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

/*
void MissionController_ExternalLink::requestMission(const Data::MissionKey &key)
{
    mLog->info("Mission Controller has seen a request mission.");
    missionList.setCreatorID(systemID);
    missionList.setVehicleID(systemID);
    this->missionList.clearQueue();
    currentCommsState = Data::ControllerCommsState::RECEIVING;

    mace_mission_request_list_t request;
    request.mission_creator = key.m_creatorID;
    request.mission_id = key.m_missionID;
    request.mission_system = key.m_systemID;
    request.mission_type = (uint8_t)key.m_missionType;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_list_t>(commsItemEnum::ITEM_RXLIST, request);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReqList(request);
    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();
}

void MissionController_ExternalLink::receivedSystemHome(const mace_home_position_t &home)
{
    m_LambdasToRun.push_back([this, home]{
        mTimer.stop();
        currentRetry = 0;
        mLog->info("Mission Controller received system home item item.");

        //This is the home position item associated with the vehicle
        CommandItem::SpatialHome newHome;
        newHome.position.setX(missionItem.x);
        newHome.position.setY(missionItem.y);
        newHome.position.setZ(missionItem.z);
        newHome.setOriginatingSystem(systemID);
        newHome.setTargetSystem(systemID);

        clearPendingTasks();
        mToExit = true;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;

        m_CB->cbiMissionController_ReceviedHome(newHome);
    });
}

void MissionController_ExternalLink::requestHome(const int &systemID)
{
    mLog->info("Mission Controller has seen a request home.");
    currentCommsState = Data::ControllerCommsState::RECEIVING;

    mace_mission_request_home_t request;
    request.target_system = systemID;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_home_t>(commsItemEnum::ITEM_RXHOME, request);

    if(m_CB)
        m_CB->cbiMissionController_TransmitHomeReq(request);

    currentRetry = 0;
    mToExit = false;
    this->start();
    mTimer.start();
}
*/
} //end of namespace DataInterface_MAVLINK
