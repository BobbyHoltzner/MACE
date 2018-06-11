#ifndef CONTROLLER_BOUNDARY_H
#define CONTROLLER_BOUNDARY_H

#include "base/pose/cartesian_position_2D.h"
#include "data_generic_command_item/command_item_components.h"

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"

using namespace BoundaryItem;

namespace ExternalLink{

using CONTROLLER_BOUNDARY_TYPE = Controllers::GenericController<
    mace_message_t,
    TransmitQueueWithKeys<Controllers::MessageModuleTransmissionQueue<mace_message_t>, ObjectIntTuple<MaceCore::ModuleCharacteristic>, ObjectIntTuple<BoundaryItem::BoundaryKey>>,
    uint8_t,
    Controllers::DataItem<BoundaryItem::BoundaryKey, BoundaryList>
>;

using SendBoundaryHelper_RequestDownload = Controllers::ActionSend<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryKey,
    mace_boundary_request_list_t,
    MACE_MSG_ID_BOUNDARY_COUNT
>;


using SendBoundaryHelper_RequestList = Controllers::ActionIntermediate<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryKey,
    mace_boundary_request_list_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_LIST,
    mace_boundary_count_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM
>;


using SendBoundaryHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryKey,
    mace_boundary_count_t,
    MACE_MSG_ID_BOUNDARY_COUNT,
    mace_boundary_request_item_t,
    MACE_MSG_ID_BOUNDARY_ITEM
>;

using SendBoundaryHelper_ReceiveCountRespondItemRequest_FromRequest = Controllers::ActionIntermediate<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    MaceCore::ModuleCharacteristic,
    BoundaryItem::BoundaryKey,
    mace_boundary_count_t,
    MACE_MSG_ID_BOUNDARY_COUNT,
    mace_boundary_request_item_t,
    MACE_MSG_ID_BOUNDARY_ITEM
>;


using SendBoundaryHelper_RequestItem = Controllers::ActionIntermediate<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryKey,
    mace_boundary_request_item_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM,
    mace_boundary_item_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM,
    MACE_MSG_ID_BOUNDARY_ACK
>;


using SendBoundaryHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryKey,
    mace_boundary_item_t,
    MACE_MSG_ID_BOUNDARY_ITEM,
    mace_boundary_request_item_t
>;

using SendBoundaryHelper_Final = Controllers::ActionFinalReceiveRespond<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    BoundaryItem::BoundaryList,
    mace_boundary_item_t,
    mace_boundary_ack_t,
    MACE_MSG_ID_BOUNDARY_ITEM
>;

using SendBoundaryHelper_FinalFinal = Controllers::ActionFinish<
    mace_message_t,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryItem::BoundaryKey,
    uint8_t,
    mace_boundary_ack_t,
    MACE_MSG_ID_BOUNDARY_ACK
>;

//template <typename MESSAGETYPE>
//using Action_RequestCurrentBoundary_Initiate = ActionRequest<
//    MESSAGETYPE,
//    CONTROLLER_BOUNDARY_TYPE,
//    MaceCore::ModuleCharacteristic,
//    mace_boundary_request_list_t,
//    MACE_MSG_ID_BOUNDARY_COUNT,
//    MACE_MSG_ID_BOUNDARY_ACK
//>;

//template <typename MESSAGETYPE>
//using Action_RequestCurrentBoundary_Response = ActionIntermediateReceive<
//    MESSAGETYPE,
//    CONTROLLER_BOUNDARY_TYPE,
//    BoundaryItem::BoundaryKey,
//    BoundaryItem::BoundaryKey,
//    mace_boundary_request_list_t,
//    MACE_MSG_ID_BOUNDARY_REQUEST_LIST,
//    mace_boundary_count_t
//>;

//template <typename MESSAGETYPE>
//using Action_RequestCurrentBoundary_NoBoundaryResponse = ActionIntermediateReceive<
//    MESSAGETYPE,
//    CONTROLLER_BOUNDARY_TYPE,
//    BoundaryItem::BoundaryKey,
//    BoundaryItem::BoundaryKey,
//    mace_boundary_request_list_t,
//    MACE_MSG_ID_BOUNDARY_REQUEST_LIST,
//    mace_boundary_ack_t
//>;

class ControllerBoundary : public CONTROLLER_BOUNDARY_TYPE,
        public SendBoundaryHelper_RequestDownload,
        public SendBoundaryHelper_RequestList,
        public SendBoundaryHelper_ReceiveCountRespondItemRequest,
        public SendBoundaryHelper_ReceiveCountRespondItemRequest_FromRequest,
        public SendBoundaryHelper_RequestItem,
        public SendBoundaryHelper_ReceiveItem,
        public SendBoundaryHelper_Final,
        public SendBoundaryHelper_FinalFinal
//        public Action_RequestCurrentBoundary_Initiate,
//        public Action_RequestCurrentBoundary_Response,
//        public Action_RequestCurrentBoundary_NoBoundaryResponse
{

private:

    struct BoundaryRequestStruct
    {
        BoundaryItem::BoundaryList boundary;
        MaceCore::ModuleCharacteristic requester;
    };

    OptionalParameter<MaceCore::ModuleCharacteristic> m_GenericRequester;
    std::unordered_map<BoundaryItem::BoundaryKey, BoundaryRequestStruct> m_BoundariesBeingFetching;

    std::unordered_map<BoundaryItem::BoundaryKey, BoundaryItem::BoundaryList> m_BoundariesUploading;

protected:

    //!
    //! \brief Called when building mavlink packet initial request to a boundary
    //! \param data
    //! \param cmd
    //!
    virtual bool Construct_Send(const BoundaryItem::BoundaryKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_boundary_request_list_t &cmd, BoundaryItem::BoundaryKey &queueObj);

    virtual bool BuildData_Send(const mace_boundary_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_boundary_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj);

    virtual bool BuildData_Send(const mace_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj);


    virtual bool BuildData_Send(const mace_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj);

    virtual bool BuildData_Send(const mace_boundary_request_item_t &boundaryRequest, const MaceCore::ModuleCharacteristic &sender, mace_boundary_item_t &boundaryItem, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj);

    virtual bool BuildData_Send(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &receiveQueueObj, BoundaryItem::BoundaryKey &respondQueueObj);

    virtual bool Construct_FinalObjectAndResponse(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_ack_t &ackBoundary, std::shared_ptr<BoundaryItem::BoundaryList> &finalList, MaceCore::ModuleCharacteristic &vehicleObj, BoundaryItem::BoundaryKey &queueObj);

    virtual bool Finish_Receive(const mace_boundary_ack_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, BoundaryItem::BoundaryKey &queueObj);

    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_boundary_request_list_t &msg, MaceCore::ModuleCharacteristic &queueObj);

public:

    ControllerBoundary(const Controllers::IMessageNotifier<mace_message_t> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan);


    void RequestBoundary(const BoundaryItem::BoundaryKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);

    void RequestCurrentBoundary(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);

};

}

#endif // CONTROLLER_BOUNDARY_H
