#ifndef CONTROLLER_BOUNDARY_H
#define CONTROLLER_BOUNDARY_H

#include "common/watchdog.h"

#include "base/pose/cartesian_position_2D.h"
#include "data_generic_command_item/command_item_components.h"

#include "controllers/generic_controller.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_intermediate_respond.h"
#include "controllers/actions/action_intermediate_receive.h"
#include "controllers/actions/action_intermediate.h"
#include "controllers/actions/action_intermediate_unsolicited.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_request.h"

#include "controllers/actions/action_broadcast.h"
#include "controllers/actions/action_broadcast_reliable.h"
#include "controllers/actions/action_unsolicited_receive.h"

#include "../pair_module_boundary_identifier.h"


using namespace BoundaryItem;


//!
//! \brief Structure of data to send when notifying the presense of a border.
//!
//! Because mavlink messages can not send vectors without a significant protocol
//! a single vehicle is sent per packet.
//!
//! If a boundary contains multiple vehicles, the packet will have to be sent multiple times
//! with different vehicleApplicableTo. The recevier will then use uniqueIdentifier to assemble them into one.
//!
struct BoundaryNotificationData
{
    BoundaryItem::BoundaryCharacterisic characteristic; //! Characterstic of boundary
    uint8_t uniqueIdentifier; //! Unique number to identify the boundary ON THE HOST
};



namespace ExternalLink{

using CONTROLLER_BOUNDARY_TYPE = Controllers::GenericController<
    mace_message_t, MaceCore::ModuleCharacteristic,
    TransmitQueueWithKeys<Controllers::MessageModuleTransmissionQueue<mace_message_t>, ObjectIntTuple<ModuleBoundaryIdentifier>, ObjectIntTuple<ObjectIntTuple<MaceCore::ModuleCharacteristic>>>,
    uint8_t,
    Controllers::DataItem<MaceCore::ModuleCharacteristic, BoundaryNotificationData>,
    Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>

>;

using SendBoundaryHelper_RequestDownload = Controllers::ActionSend<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    uint8_t,
    mace_boundary_request_list_t,
    MACE_MSG_ID_BOUNDARY_COUNT
>;


using BoundaryControllerAction_ReceiveUnsolicitedRequestList_SendCount = Controllers::ActionIntermediateUnsolicited<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    mace_boundary_request_list_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_LIST,
    mace_boundary_count_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM
>;


using SendBoundaryHelper_ReceiveCountRespondItemRequest = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mace_boundary_count_t,
    MACE_MSG_ID_BOUNDARY_COUNT,
    mace_boundary_request_item_t,
    MACE_MSG_ID_BOUNDARY_ITEM
>;


using SendBoundaryHelper_RequestItem = Controllers::ActionIntermediate<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mace_boundary_request_item_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM,
    mace_boundary_item_t,
    MACE_MSG_ID_BOUNDARY_REQUEST_ITEM,
    MACE_MSG_ID_BOUNDARY_ACK
>;


using SendBoundaryHelper_ReceiveItem = Controllers::ActionIntermediateReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    ModuleBoundaryIdentifier,
    mace_boundary_item_t,
    MACE_MSG_ID_BOUNDARY_ITEM,
    mace_boundary_request_item_t
>;

using SendBoundaryHelper_Final = Controllers::ActionFinalReceiveRespond<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    BoundaryItem::BoundaryList,
    mace_boundary_item_t,
    mace_boundary_ack_t,
    MACE_MSG_ID_BOUNDARY_ITEM
>;

using SendBoundaryHelper_FinalFinal = Controllers::ActionFinish<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    ModuleBoundaryIdentifier,
    uint8_t,
    mace_boundary_ack_t,
    MACE_MSG_ID_BOUNDARY_ACK
>;



using BroadcastNewBoundaryNotification = Controllers::ActionBroadcastReliable<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryNotificationData,
    mace_new_boundary_object_t,
    MACE_MSG_ID_BOUNDARY_ACK
>;

using UsolicitedReceiveNewBoundaryNotification = Controllers::ActionUnsolicitedReceive<
    mace_message_t, MaceCore::ModuleCharacteristic,
    CONTROLLER_BOUNDARY_TYPE,
    BoundaryNotificationData,
    mace_new_boundary_object_t,
    MACE_MSG_ID_NEW_BOUNDARY_OBJECT
>;



class ControllerBoundary : public CONTROLLER_BOUNDARY_TYPE,
        public SendBoundaryHelper_RequestDownload,
        public BoundaryControllerAction_ReceiveUnsolicitedRequestList_SendCount,
        public SendBoundaryHelper_ReceiveCountRespondItemRequest,
        public SendBoundaryHelper_RequestItem,
        public SendBoundaryHelper_ReceiveItem,
        public SendBoundaryHelper_Final,
        public SendBoundaryHelper_FinalFinal,
        public BroadcastNewBoundaryNotification,
        public UsolicitedReceiveNewBoundaryNotification
//        public Action_RequestCurrentBoundary_Initiate,
//        public Action_RequestCurrentBoundary_Response,
//        public Action_RequestCurrentBoundary_NoBoundaryResponse
{

private:

    OptionalParameter<MaceCore::ModuleCharacteristic> m_GenericRequester;

    //! Object that maps HostModule,BoundaryID pair to Destination,Object pair
    std::map<ModuleBoundaryIdentifier, std::tuple<MaceCore::ModuleCharacteristic, BoundaryItem::BoundaryList>> m_BoundariesBeingFetching;

    std::map<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList> m_BoundariesUploading;

    //! Member to hold the vehicles in a boundary. Will be built up as the boundary is learned about.
    std::unordered_map<ModuleBoundaryIdentifier, std::vector<int>> m_VehiclesInBoundary;
    std::unordered_map<ModuleBoundaryIdentifier, std::shared_ptr<Watchdog>> m_BoundaryBuilderWatchdogs;

protected:

    //!
    //! \brief Download Action - Initiate download with a request list message
    //! \param data Data given to send.
    //! \param sender Module sending data
    //! \param target Module targeting
    //! \param cmd Message to contruct
    //! \param queueObj Queue object to contruct identifitying this transmission
    //! \return True if transmission should continue
    //!
    virtual bool Construct_Send(const uint8_t &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_boundary_request_list_t &cmd, ModuleBoundaryIdentifier &queueObj);


    //!
    //! \brief Upload Action - Receive request list and respond with count
    //! \param cmd Message received asking for count of a boundary
    //! \param sender Module that sent request
    //! \param rtn Message to construct containing count
    //! \param moduleUploadingFrom Module to contruct that contains the boundary to upload
    //! \param respondQueueObj Object to to construct that indicates how the message should be queued
    //! \return True if transmission should continue
    //!
    virtual bool IntermediateUnsolicitedReceive(const mace_boundary_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_boundary_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Function on downloading side that receives a count and makes request for first item
    //! \param msg Count message received
    //! \param sender Module that contains the boundary
    //! \param rtn Object to set containing message to send after following up.
    //! \param moduleDownloadingTo Object to set indicating what module requested the boundary
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message is to be used
    //!
    virtual bool BuildData_Send(const mace_boundary_count_t &boundary, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Upload Action - Receive a item request and respond with item
    //! \param msg Request Item message received
    //! \param sender Module that sent message
    //! \param boundaryItem Message to construct to send back
    //! \param moduleUploadingFrom Object to construct indicating what module boundary is downloading from
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message should be used
    //!
    virtual bool BuildData_Send(const mace_boundary_request_item_t &boundaryRequest, const MaceCore::ModuleCharacteristic &sender, mace_boundary_item_t &boundaryItem, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Receive an item and make request for next item.
    //!
    //! This action will NOT be used for the final action
    //! \param msg Item message received
    //! \param sender Module that sent message
    //! \param request Message to construct requesting next item
    //! \param moduleDownloadingTo Module that boundary is downloading to
    //! \param receiveQueueObj Object to set that will remove any queued transmission
    //! \param respondQueueObj Object to set to queue next transmission
    //! \return True if message should be used
    //!
    virtual bool BuildData_Send(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_request_item_t &request, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &receiveQueueObj, ModuleBoundaryIdentifier &respondQueueObj);


    //!
    //! \brief Download Action - Receive final item and configure ack
    //!
    //! This action will ONLY be used on last item
    //! \param msg Item message recieved
    //! \param sender Module that sent message
    //! \param ackBoundary Ack to construct to indicate the success/failure of boundary transmission
    //! \param finalList Final boundary downloaded to be returned to module
    //! \param moduleDownloadingTo Module boundary is being downloaded to
    //! \param queueObj Object to set that will remove any queued transmission
    //! \return True if message should be used
    //!
    virtual bool Construct_FinalObjectAndResponse(const mace_boundary_item_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, mace_boundary_ack_t &ackBoundary, std::shared_ptr<BoundaryItem::BoundaryList> &finalList, MaceCore::ModuleCharacteristic &vehicleObj, ModuleBoundaryIdentifier &queueObj);


    //!
    //! \brief Upload Action - Receive final ACK and end the previous transmission
    //! \param msg Ack message received
    //! \param sender Module that sent message
    //! \param ack Ack code to return
    //! \param queueObj Object to set that will remove any queued transmission
    //! \return
    //!
    virtual bool Finish_Receive(const mace_boundary_ack_t &boundaryItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, ModuleBoundaryIdentifier &queueObj);


    virtual void Construct_BroadcastReliable(const BoundaryNotificationData &data, const MaceCore::ModuleCharacteristic &sender, std::vector<mace_new_boundary_object_t> &vec);


    //!
    //! \brief Notify Receive - Receive the notification of a new boundary
    //!
    //! When being notified of a boundary, a seperate message will be received for each vehicle.
    //! This function is to receive those and assemble them into a single BoundaryCharacterstic.
    //!
    //! A watchdog is kicked off to re-request if a full BoundaryCharacterstic isn't received.
    //!
    //! \param msg Message received indicating a new boundary was received
    //! \param sender Module that generated the boundary on the remote machine
    //! \param data Data to return to local instance when boundary is fully received
    //! \return True if boundary is fully received, false otherwise
    //!
    virtual bool Construct_FinalObject(const mace_new_boundary_object_t &msg, const MaceCore::ModuleCharacteristic &sender, std::shared_ptr<BoundaryNotificationData> &data);

public:

    ControllerBoundary(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, int linkChan);


    //!
    //! \brief Request a boundary from a remote instance
    //! \param key Boundary identifer on remote
    //! \param sender Module making request
    //! \param target Target module that contains the boundary to download
    //!
    void RequestBoundary(const uint8_t &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target);


private:


    //!
    //! \brief Function to be called when the watchdog for receiving vehicles in a boundary expires
    //!
    //! This would indicates that messages where lost on transmission.
    //! \param pair Details on whom and what boundary was incomplete
    //!
    void BoundaryBuilderWatchdogExpired(const ModuleBoundaryIdentifier &pair);

};

}


#endif // CONTROLLER_BOUNDARY_H
