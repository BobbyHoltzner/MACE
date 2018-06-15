#include "controller_boundary.h"

namespace ExternalLink{

/*
    //!
    //! \brief Called when building mavlink packet initial request to a boundary
    //! \param data
    //! \param cmd
    //!
    bool ControllerBoundary::Construct_Send(const BoundaryItem::BoundaryKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_boundary_request_list_t &cmd, BoundaryItem::BoundaryKey &queueObj)
    {
        queueObj = data;

        cmd.boundary_creator = data.m_creatorID;
        cmd.boundary_system = data.m_systemID;
        cmd.boundary_type = (uint8_t)data.m_boundaryType;

        if(m_BoundariesBeingFetching.find(data) != m_BoundariesBeingFetching.cend())
        {
            throw std::runtime_error("Boundary is already being downloaded");
            return false;
        }

        BoundaryItem::BoundaryList newList;
        newList.setBoundaryKey(data);
        newList.clearQueue();
        BoundaryRequestStruct newItem;
        newItem.boundary = newList;
        newItem.requester = sender;
        m_BoundariesBeingFetching.insert({data, newItem});

        std::cout << "Boundary Controller: Sending Boundary Request List" << std::endl;

        return true;
    }

    bool ControllerBoundary::BuildData_Send(const mace_boundary_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_boundary_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj)
    {
        UNUSED(sender);
        BoundaryItem::BoundaryKey key(cmd.boundary_system, cmd.boundary_creator, static_cast<BoundaryItem::BOUNDARYTYPE>(cmd.boundary_type));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        std::vector<std::tuple<BoundaryKey, BoundaryList>> boundaries;
        CONTROLLER_BOUNDARY_TYPE::FetchDataFromKey(key, boundaries);

        if(boundaries.size() == 0)
        {
            return false;
        }
        if(boundaries.size() > 1)
        {
            throw std::runtime_error("Multiple boundaries assigned to the same key returned, This is a non-op");
        }
        if(std::get<0>(boundaries.at(0)) != key)
        {
            throw std::runtime_error("Requesting a specific boundary key did not return the same key, This is a non-op");
        }


        if(m_BoundariesUploading.find(key) != m_BoundariesUploading.cend())
        {
            std::cout << "Boundary Upload Progress: The boundary that was requested to be transmitted is already being transmitted" << std::endl;
            return false;
        }
        BoundaryList boundary = std::get<1>(boundaries.at(0));
        m_BoundariesUploading.insert({key, boundary});


        rtn.count = m_BoundariesUploading.at(key).getQueueSize();
        rtn.boundary_creator = key.m_creatorID;
        rtn.boundary_system = key.m_systemID;
        rtn.boundary_type = static_cast<BOUNDARY_TYPE>(key.m_boundaryType);

        std::cout << "Boundary Controller: Sending Boundary Count" << std::endl;

        return true;
    }

    bool ControllerBoundary::BuildData_Send(const mace_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj)
    {
        UNUSED(sender);
        BoundaryItem::BoundaryKey key(boundary.boundary_system,boundary.boundary_creator,static_cast<BoundaryItem::BOUNDARYTYPE>(boundary.boundary_type));
        receiveQueueObj = key;
        respondQueueObj = key;


        if(m_BoundariesBeingFetching.find(key) == m_BoundariesBeingFetching.cend())
        {
            return false;
        }

        vehicleObj = m_BoundariesBeingFetching[key].requester;

        m_BoundariesBeingFetching.at(key).boundary.initializeBoundary(boundary.count);

        request.boundary_creator = boundary.boundary_creator;
        request.boundary_system = boundary.boundary_system;
        request.boundary_type = boundary.boundary_type;
        request.seq = 0;

        std::cout << "Boundary Controller: Requesting Item " << 0 << std::endl;

        return true;
    }


    bool ControllerBoundary::BuildData_Send(const mace_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj)
    {
        UNUSED(sender);
        BoundaryItem::BoundaryKey key(boundary.boundary_system,boundary.boundary_creator,static_cast<BoundaryItem::BOUNDARYTYPE>(boundary.boundary_type));
        receiveQueueObj = sender;
        respondQueueObj = key;


        if(m_BoundariesBeingFetching.find(key) != m_BoundariesBeingFetching.cend())
        {
            return false;
        }

        if(m_GenericRequester.IsSet() == false) {
            throw std::runtime_error("Do not know what module requested a boundary");
        }

        BoundaryItem::BoundaryList newList;
        newList.setBoundaryKey(key);
        newList.clearQueue();
        BoundaryRequestStruct newItem;
        newItem.boundary = newList;
        newItem.requester = m_GenericRequester();
        m_BoundariesBeingFetching.insert({key, newItem});

        m_GenericRequester = OptionalParameter<MaceCore::ModuleCharacteristic>();

        vehicleObj = m_BoundariesBeingFetching[key].requester;

        m_BoundariesBeingFetching.at(key).boundary.initializeBoundary(boundary.count);

        request.boundary_creator = boundary.boundary_creator;
        request.boundary_system = boundary.boundary_system;
        request.boundary_type = boundary.boundary_type;
        request.seq = 0;

        std::cout << "Boundary Controller: Requesting Item " << 0 << std::endl;

        return true;
    }

    bool ControllerBoundary::BuildData_Send(const mace_boundary_request_item_t &boundaryRequest, const MaceCore::ModuleCharacteristic &sender, mace_boundary_item_t &boundaryItem, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj)
    {
        UNUSED(sender);
        BoundaryItem::BoundaryKey key(boundaryRequest.boundary_system,boundaryRequest.boundary_creator,static_cast<BoundaryItem::BOUNDARYTYPE>(boundaryRequest.boundary_type));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        if(m_BoundariesUploading.find(key) == m_BoundariesUploading.cend())
        {
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("BoundaryController_ExternalLink has been told to transmit a boundary item from a boundary which keys dont match the contained.");
            return false;
        }

        int index = boundaryRequest.seq;
        if(index >= m_BoundariesUploading[key].getQueueSize())
        {
            //this indicates that RX system requested something OOR
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("BoundaryController_ExternalLink has been told to transmit a boundary item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
            return false;
        }

        if(CONTROLLER_BOUNDARY_TYPE::mLog)
            CONTROLLER_BOUNDARY_TYPE::mLog->info("BoundaryController_ExternalLink has been told to transmit a boundary item with index " + std::to_string(index) + ".");

        mace::pose::Position<mace::pose::CartesianPosition_2D> ptrItem = this->m_BoundariesUploading[key].getBoundaryItemAtIndex(index);

        boundaryItem.boundary_creator = key.m_creatorID;
        boundaryItem.boundary_system = key.m_systemID;
        boundaryItem.boundary_type = (uint8_t)key.m_boundaryType;
        boundaryItem.frame = MAV_FRAME_LOCAL_ENU;
        boundaryItem.seq = index;
        boundaryItem.x = ptrItem.getXPosition();
        boundaryItem.y = ptrItem.getYPosition();
        boundaryItem.z = 0.0;

        std::cout << "Boundary Controller: Sending Item " << index << std::endl;

        return true;
    }

    bool ControllerBoundary::BuildData_Send(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = boundaryItem.boundary_system;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        BoundaryItem::BoundaryKey key(boundaryItem.boundary_system,boundaryItem.boundary_creator,static_cast<BoundaryItem::BOUNDARYTYPE>(boundaryItem.boundary_type));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj = m_BoundariesBeingFetching[key].requester;

        //check if boundary item received is part of a boundary we are activly downloading
        if(this->m_BoundariesBeingFetching.find(key) == m_BoundariesBeingFetching.cend())
        {
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = boundaryItem.seq;
        if(seqReceived > (m_BoundariesBeingFetching[key].boundary.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Boundary download Error: received a boundary item with an index greater than available in the queue" << std::endl;
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with an index greater than available in the queue.");
            return false;
        }
        //execution will only continue if not last item
        if(seqReceived == (m_BoundariesBeingFetching[key].boundary.getQueueSize() - 1))
        {
            return false;
        }

        Position<CartesianPosition_2D> newVertex;
        newVertex.setXPosition(boundaryItem.x);
        newVertex.setYPosition(boundaryItem.y);

        m_BoundariesBeingFetching[key].boundary.replaceVertexItemAtIndex(newVertex, seqReceived);

        BoundaryItem::BoundaryList::BoundaryListStatus status = m_BoundariesBeingFetching[key].boundary.getBoundaryListStatus();
        if(status.state == BoundaryItem::BoundaryList::COMPLETE)
        {
            throw std::runtime_error("Still have more items to request, but boundary is full");
        }


        int indexRequest = status.remainingItems.at(0);

        request.boundary_creator = key.m_creatorID;
        request.boundary_system = key.m_systemID;
        request.boundary_type = (uint8_t)key.m_boundaryType;
        request.seq = indexRequest;

        std::cout << "Boundary Controller: Requesting Item " << indexRequest << std::endl;

        return true;
    }

    bool ControllerBoundary::Construct_FinalObjectAndResponse(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_ack_t &ackBoundary, std::shared_ptr<BoundaryItem::BoundaryList> &finalList, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &queueObj)
    {
        UNUSED(sender);
        MaceCore::ModuleCharacteristic target;
        target.ID = boundaryItem.boundary_system;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        BoundaryItem::BoundaryKey key(boundaryItem.boundary_system,boundaryItem.boundary_creator,static_cast<BoundaryItem::BOUNDARYTYPE>(boundaryItem.boundary_type));
        queueObj = key;

        vehicleObj.ID = key.m_systemID;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        //check if boundary item received is part of a boundary we are activly downloading
        if(this->m_BoundariesBeingFetching.find(key) == m_BoundariesBeingFetching.cend())
        {
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with a key that is not equal to the one we were originally told.");
            return false;
        }

        int seqReceived = boundaryItem.seq;
        if(seqReceived > (m_BoundariesBeingFetching[key].boundary.getQueueSize() - 1)) //this should never happen
        {
            std::cout << "Boundary download Error: received a boundary item with an index greater than available in the queue" << std::endl;
            if(CONTROLLER_BOUNDARY_TYPE::mLog)
                CONTROLLER_BOUNDARY_TYPE::mLog->error("Boundary controller received a boundary item with an index greater than available in the queue.");
            return false;
        }

        //execution will only continue if last item
        if(seqReceived < (m_BoundariesBeingFetching[key].boundary.getQueueSize() - 1))
        {
            return false;
        }

        Position<CartesianPosition_2D> newVertex;
        newVertex.setXPosition(boundaryItem.x);
        newVertex.setYPosition(boundaryItem.y);

        m_BoundariesBeingFetching[key].boundary.replaceVertexItemAtIndex(newVertex, seqReceived);

        BoundaryItem::BoundaryList::BoundaryListStatus status = m_BoundariesBeingFetching[key].boundary.getBoundaryListStatus();
        if(status.state == BoundaryItem::BoundaryList::INCOMPLETE)
        {
            throw std::runtime_error("Reached end of request but boundaries are not completed");
        }

        if(CONTROLLER_BOUNDARY_TYPE::mLog)
        {
            std::stringstream buffer;
            buffer << key;
            CONTROLLER_BOUNDARY_TYPE::mLog->info("Boundary Controller has received the entire boundary of " + std::to_string(m_BoundariesBeingFetching[key].boundary.getQueueSize()) + " for mission " + buffer.str() + ".");
        }

        ackBoundary.boundary_system = key.m_systemID;
        ackBoundary.boundary_creator = key.m_creatorID;
        ackBoundary.boundary_type = (uint8_t)key.m_boundaryType;
        ackBoundary.boundary_result = BOUNDARY_ACCEPTED;

        finalList = std::make_shared<BoundaryItem::BoundaryList>(m_BoundariesBeingFetching[key].boundary);
        m_BoundariesBeingFetching.erase(key);

        std::cout << "Boundary Controller: Sending Final ACK" << std::endl;

        return true;
    }

    bool ControllerBoundary::Finish_Receive(const mace_boundary_ack_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, BoundaryItem::BoundaryKey &queueObj)
    {
        UNUSED(sender);

        BoundaryItem::BoundaryKey key(boundaryItem.boundary_system, boundaryItem.boundary_creator, static_cast<BoundaryItem::BOUNDARYTYPE>(boundaryItem.boundary_type));
        queueObj = key;

        ack = boundaryItem.boundary_result;

        std::cout << "Boundary Controller: Received Final ACK" << std::endl;

        return true;
    }

    void ControllerBoundary::Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_boundary_request_list_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        msg.boundary_system = target.ID;
        msg.boundary_creator = sender.ID;
        msg.boundary_type = 0;
        m_GenericRequester = sender;

        queueObj = target;

        std::cout << "Boundary Controller: Sending boundary request" << std::endl;
    }

    ControllerBoundary::ControllerBoundary(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan) :
        CONTROLLER_BOUNDARY_TYPE(cb, queue, linkChan),
        SendBoundaryHelper_RequestDownload(this, mace_msg_boundary_request_list_encode_chan),
        SendBoundaryHelper_RequestList(this, mace_msg_boundary_request_list_decode, mace_msg_boundary_count_encode_chan),
        SendBoundaryHelper_ReceiveCountRespondItemRequest(this, mace_msg_boundary_count_decode, mace_msg_boundary_request_item_encode_chan),
        SendBoundaryHelper_ReceiveCountRespondItemRequest_FromRequest(this, mace_msg_boundary_count_decode, mace_msg_boundary_request_item_encode_chan),
        SendBoundaryHelper_RequestItem(this, mace_msg_boundary_request_item_decode, mace_msg_boundary_item_encode_chan),
        SendBoundaryHelper_ReceiveItem(this,
                                                    [this](const mace_boundary_request_item_t &A, const MaceCore::ModuleCharacteristic &B, const BoundaryItem::BoundaryKey &C, const MaceCore::ModuleCharacteristic &D){SendBoundaryHelper_ReceiveCountRespondItemRequest::NextTransmission(A,B,C,D);},
    mace_msg_boundary_item_decode),
        SendBoundaryHelper_Final(this, mace_msg_boundary_item_decode, mace_msg_boundary_ack_encode_chan),
        SendBoundaryHelper_FinalFinal(this, mace_msg_boundary_ack_decode)
//        Action_RequestCurrentBoundary_Initiate(this, mace_msg_boundary_request_list_encode_chan),
//        Action_RequestCurrentBoundary_Response(this,
//                                [this](const mace_boundary_count_t &A, const MaceCore::ModuleCharacteristic &B, const BoundaryItem::BoundaryKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RequestList::NextTransmission(A,B,C,D);},
//                                mace_msg_boundary_request_list_decode),
//        Action_RequestCurrentBoundary_NoBoundaryResponse(this,
//                                [this](const mace_boundary_ack_t &A, const MaceCore::ModuleCharacteristic &B, const BoundaryItem::BoundaryKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_Final::FinalResponse(A,B,C,D);},
//                                mace_msg_boundary_request_list_decode)
    {

    }


    void ControllerBoundary::RequestBoundary(const BoundaryItem::BoundaryKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        SendBoundaryHelper_RequestDownload::Send(key, sender, target);
    }

    void ControllerBoundary::RequestCurrentBoundary(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        //Action_RequestCurrentBoundary_Initiate::Request(sender, target);
    }
*/
}
