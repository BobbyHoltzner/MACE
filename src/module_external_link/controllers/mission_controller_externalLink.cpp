#include "mission_controller_externalLink.h"

namespace ExternalLink {

MissionController_ExternalLink::MissionController_ExternalLink(MissionController_Interface *cb):
    targetID(0), transmittingID(0),mLog(NULL),
    currentRetry(0), maxRetries(5), responseTimeout(5000),\
    currentCommsState(Data::ControllerCommsState::NEUTRAL),
    m_CB(NULL), prevTransmit(NULL)
{
    connectCallback(cb);
}

void MissionController_ExternalLink::updateLogging(const bool &toLog, const std::string &name)
{
    UNUSED(toLog);
    mLog = spdlog::get(name);
}

void MissionController_ExternalLink::clearPreviousTransmit()
{
    if(prevTransmit)
    {
        delete prevTransmit;
        prevTransmit = NULL;
    }
}

void MissionController_ExternalLink::updateTransmittingJobs(const MaceCore::ModuleCharacteristic &target)
{
    std::map<int, std::list<MissionItem::MissionList>>::iterator it = missionList.begin();
    int ID = (int)it->first;
    MissionItem::MissionList list = it->second.front();
    it->second.pop_front();
    if(it->second.empty())
    {
        missionList.erase(it);
    }
    transmitMission(list, target);
}

void MissionController_ExternalLink::updateIDS(const int &targetSystem, const int &originSystem)
{
    this->targetID = targetSystem; //this is to whom this mission would be going to
    this->transmittingID = originSystem; //this is to whom this mission is from

    helperMACEtoCOMMS.updateIDS(originSystem,0);

}

///////////////////////////////////////////////////////////////////////////////
/// GENERAL TRANSMISSION EVENTS: These events are related to sending a mission
/// to a remote instance of MACE.
///////////////////////////////////////////////////////////////////////////////

void MissionController_ExternalLink::transmitMission(const int &vehicleMissionsOn, const std::list<MissionItem::MissionList> &missionQueue, const MaceCore::ModuleCharacteristic &target)
{
    if(target.Class == MaceCore::ModuleClasses::VEHICLE_COMMS && vehicleMissionsOn == target.ID)
    {
        std::cout<<"This doesn't make sense since this is ourselves."<<std::endl;
        return;
    }

    if(missionQueue.size() > 0)
    {
        std::list<MissionItem::MissionList> copyList = missionQueue;
        std::list<MissionItem::MissionList> currentList = missionList[vehicleMissionsOn];
        currentList.splice(currentList.end(),copyList);
        missionList[vehicleMissionsOn] = currentList;

        if(!isThreadActive())
        {
            updateTransmittingJobs(target);
        }
    }
}


void MissionController_ExternalLink::transmitMission(const MissionItem::MissionList &missionQueue, const MaceCore::ModuleCharacteristic &target)
{
    if(mLog)
    {
        std::stringstream buffer;
        buffer << missionQueue;

        mLog->debug("Mission Controller has been instructed to transmit a mission.");
        mLog->info(buffer.str());
    }

    if(target.Class == MaceCore::ModuleClasses::VEHICLE_COMMS && missionQueue.getVehicleID() == target.ID)
    {
        std::cout<<"This doesn't make sense since this is ourselves."<<std::endl;
        return;
    }

    MaceCore::ModuleCharacteristic sender;
    sender.ID = missionQueue.getVehicleID();
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    currentCommsState = Data::ControllerCommsState::TRANSMITTING;

    this->missionQueue = missionQueue;

    MissionItem::MissionKey key = missionQueue.getMissionKey();

    mace_mission_count_t count;
    count.count = this->missionQueue.getQueueSize();
    count.target_system = missionQueue.getVehicleID();
    count.mission_system = key.m_systemID;
    count.mission_creator = key.m_creatorID;
    count.mission_id = key.m_missionID;
    count.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
    count.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_count_t>(commsItemEnum::ITEM_TXCOUNT, count, sender, target);

    currentRetry = 0;
    this->start();
    mTimer.start();

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionCount(count, sender, target);
}

void MissionController_ExternalLink::transmitMissionItem(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &target)
{
    //m_LambdasToRun.push_back([this, missionRequest, target]{

        MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));

        MaceCore::ModuleCharacteristic sender;
        sender.ID = missionRequest.target_system;
        sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(key != this->missionQueue.getMissionKey())
        {
            //this indicates for some reason the other system requested a different mission?
            if(mLog)
                mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
            return;
        }

        int index = missionRequest.seq;
        if(index > this->missionQueue.getQueueSize())
        {
            //this indicates that RX system requested something OOR
            if(mLog)
                mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return;
        }

        if(mLog)
            mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

        mTimer.stop();

        std::shared_ptr<CommandItem::AbstractCommandItem> ptrItem = this->missionQueue.getMissionItem(index);
        mace_mission_item_t missionItem;
        helperMACEtoCOMMS.MACEMissionToCOMMSMission(ptrItem,index,missionItem);
        helperMACEtoCOMMS.updateMissionKey(key,missionItem);

        currentCommsState = Data::ControllerCommsState::TRANSMITTING;
        clearPendingTasks();
        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_mission_item_t>(commsItemEnum::ITEM_TXITEM, missionItem, sender, target);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionItem(missionItem, sender, target);
//    });
}

void MissionController_ExternalLink::receivedMissionACK(const mace_mission_ack_t &missionACK)
{
    if(!isThreadActive())
    {
        std::cout<<"The thread is not currently active so this must be something else"<<std::endl;
        return;
    }

    m_LambdasToRun.push_back([this, missionACK]{
        mToExit = true;
        mTimer.stop();
        currentRetry = 0;
        clearPendingTasks();
        clearPreviousTransmit();
        currentCommsState = Data::ControllerCommsState::NEUTRAL;
        if(m_CB)
            m_CB->cbiMissionController_MissionACK(missionACK);

    });

}

///////////////////////////////////////////////////////////////////////////////
/// GENERAL RECEIVING EVENTS: These events are related to receiving a mission
/// from a remote instance of MACE.
///////////////////////////////////////////////////////////////////////////////

void MissionController_ExternalLink::receivedMissionCount(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender)
{
    MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));

    MaceCore::ModuleCharacteristic target = sender;

    if(m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        throw std::runtime_error("Given missionKey hasn't been requested to be downloaded");
    }
    MaceCore::ModuleCharacteristic requester = m_MissionsBeingFetching.at(key).requester;

    m_MissionsBeingFetching.at(key).missionList.initializeQueue(mission.count);

    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("Mission Controller has received a mission count of " + std::to_string(mission.count) + " with a key of " + buffer.str() + ".");
    }


    mace_mission_request_item_t request;
    request.mission_creator = mission.mission_creator;
    request.mission_id = mission.mission_id;
    request.mission_system = mission.mission_system;
    request.mission_type = mission.mission_type;
    request.mission_state = mission.mission_state;
    request.target_system = targetID;
    request.seq = 0;


    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request, requester, target);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReq(request, requester, target);
    currentRetry = 0;
    this->start();
    mTimer.start();
}

void MissionController_ExternalLink::recievedMissionItem(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender)
{
    MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));

    //check if mission item received is part of a mission we are activly downloading
    if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        if(mLog)
            mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
        return;
    }

    MaceCore::ModuleCharacteristic requester = m_MissionsBeingFetching.at(key).requester;

    int seqReceived = missionItem.seq;
    if(seqReceived > (m_MissionsBeingFetching[key].missionList.getQueueSize() - 1)) //this should never happen
    {
        if(mLog)
            mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
        return;
    }

    mTimer.stop();
    currentRetry = 0;

    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("Mission Controller has received item " + std::to_string(seqReceived) + " for mission " + buffer.str() + ".");
    }

    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(missionItem, sender);
    m_MissionsBeingFetching[key].missionList.replaceMissionItemAtIndex(newMissionItem, seqReceived);

    MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].missionList.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        int indexRequest = status.remainingItems.at(0);

        if(mLog)
        {
            std::stringstream buffer;
            buffer << key;
            mLog->info("Mission Controller is requesting mission item " + std::to_string(indexRequest) + " for mission " + buffer.str() + ".");
        }

        mace_mission_request_item_t request;
        request.target_system = targetID;
        request.mission_creator = key.m_creatorID;
        request.mission_id = key.m_missionID;
        request.mission_system = key.m_systemID;
        request.mission_type = (uint8_t)key.m_missionType;
        request.mission_state = (uint8_t)key.m_missionState;
        request.seq = indexRequest;

        clearPreviousTransmit();
        prevTransmit = new PreviousTransmission<mace_mission_request_item_t>(commsItemEnum::ITEM_RXITEM, request, requester, sender);
        currentRetry = 0;
        mTimer.start();

        if(m_CB)
            m_CB->cbiMissionController_TransmitMissionReq(request, requester, sender);
    }else{
        if(mLog)
        {
            std::stringstream buffer;
            buffer << key;
            mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].missionList.getQueueSize()) + " for mission " + buffer.str() + ".");
        }

        mace_mission_ack_t ackMission;
        ackMission.mission_system = key.m_systemID;
        ackMission.mission_creator = key.m_creatorID;
        ackMission.mission_id = key.m_missionID;
        ackMission.mission_type = (uint8_t)key.m_missionType;
        ackMission.prev_mission_state = (uint8_t)key.m_missionState;

        //KEN This is a hack but for now
        if(key.m_missionState == MissionItem::MISSIONSTATE::PROPOSED)
        {
            ackMission.cur_mission_state = (uint8_t)MissionItem::MISSIONSTATE::RECEIVED;
            m_MissionsBeingFetching[key].missionList.setMissionTXState(MissionItem::MISSIONSTATE::RECEIVED);
        }
        else
        {
            ackMission.cur_mission_state = (uint8_t)key.m_missionState;
        }

        m_CB->cbiMissionController_TransmitMissionACK(ackMission, requester, sender);

        clearPendingTasks();
        clearPreviousTransmit();
        mToExit = true;
        currentCommsState = Data::ControllerCommsState::NEUTRAL;

        MissionList finishedList = m_MissionsBeingFetching[key].missionList;
        m_MissionsBeingFetching.erase(key);

        m_CB->cbiMissionController_ReceivedMission(finishedList);
    }
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

        if(prevTransmit == NULL)
        {
            mTimer.stop();
            break;
        }

        this->RunPendingTasks();

        //Check to see if any of the pending tasks have said that we can quit
        if(mToExit)
            continue;

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
                    if(mLog)
                        mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_request_list_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_list_t>*>(prevTransmit);
                    mace_mission_request_list_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReqList(msgTransmit);
                }
                if(type == commsItemEnum::ITEM_RXGENLIST)
                {
                    if(mLog)
                        mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_request_list_generic_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_list_generic_t>*>(prevTransmit);
                    mace_mission_request_list_generic_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionGenericReqList(msgTransmit, tmp->Sender());
                }
                else if(type == commsItemEnum::ITEM_RXITEM)
                {
                    if(mLog)
                        mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_request_item_t> *tmp = static_cast<PreviousTransmission<mace_mission_request_item_t>*>(prevTransmit);
                    mace_mission_request_item_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionReq(msgTransmit, tmp->Sender(), tmp->Target());
                }
                break;
            }
            case Data::ControllerCommsState::TRANSMITTING:
            {

                if(type == commsItemEnum::ITEM_TXCOUNT)
                {
                    if(mLog)
                        mLog->error("Mission Controller is on attempt " + std::to_string(currentRetry) + " for " + getCommsItemEnumString(type) + ".");
                    PreviousTransmission<mace_mission_count_t> *tmp = static_cast<PreviousTransmission<mace_mission_count_t>*>(prevTransmit);
                    mace_mission_count_t msgTransmit = tmp->getData();
                    mTimer.start();
                    if(m_CB)
                        m_CB->cbiMissionController_TransmitMissionCount(msgTransmit);
                }
                else if(type == commsItemEnum::ITEM_TXITEM)
                {
                    if(mLog)
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

void MissionController_ExternalLink::requestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
{
    currentCommsState = Data::ControllerCommsState::RECEIVING;
    if(mLog)
    {
        std::stringstream buffer;
        buffer << key;
        mLog->info("MissionController_ExternalLink is requesting the current mission of " + buffer.str() + ".");
    }

    mace_mission_request_list_t request;
    request.mission_creator = key.m_creatorID;
    request.mission_id = key.m_missionID;
    request.mission_system = key.m_systemID;
    request.mission_type = (uint8_t)key.m_missionType;
    request.mission_state = (uint8_t)key.m_missionState;

    if(m_MissionsBeingFetching.find(key) != m_MissionsBeingFetching.cend())
    {
        throw std::runtime_error("Mission is already being downloaded");
    }

    MissionItem::MissionList newList;
    newList.setMissionKey(key);
    newList.clearQueue();
    MissionRequestStruct requestStruct;
    requestStruct.missionList = newList;
    requestStruct.requester = sender;
    m_MissionsBeingFetching.insert({key, requestStruct});

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_list_t>(commsItemEnum::ITEM_RXLIST, request, target, sender);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionReqList(request, sender, target);
    currentRetry = 0;
    this->start();
    mTimer.start();
}

void MissionController_ExternalLink::requestCurrentMission(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    currentCommsState = Data::ControllerCommsState::RECEIVING;
    if(mLog)
        mLog->info("MissionController_ExternalLink is requesting the current mission from system " + std::to_string(targetSystem) + ".");
    mace_mission_request_list_generic_t request;
    request.mission_system = targetSystem;
    request.mission_type = (uint8_t)MissionItem::MISSIONSTATE::CURRENT;
    request.mission_state = 0;

    clearPendingTasks();
    clearPreviousTransmit();
    prevTransmit = new PreviousTransmission<mace_mission_request_list_generic_t>(commsItemEnum::ITEM_RXGENLIST, request, sender);

    if(m_CB)
        m_CB->cbiMissionController_TransmitMissionGenericReqList(request, sender);
    currentRetry = 0;
    this->start();
    mTimer.start();
}
} //end of namespace DataInterface_MAVLINK
