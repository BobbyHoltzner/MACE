#ifndef ACTION_INTERMEDIATE_H
#define ACTION_INTERMEDIATE_H

#include "action_base.h"
#include "action_intermediate_receive.h"
#include "action_intermediate_respond.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename RECEIVE_QUEUE_TYPE, typename RESPOND_QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID, typename ACK_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediate :
        public ActionIntermediateReceive<MESSAGE_TYPE, CONTROLLER_TYPE, RECEIVE_QUEUE_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>,
        public ActionIntermediateRespond<MESSAGE_TYPE, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>
{

public:

    ActionIntermediate(CONTROLLER_TYPE *controller,
                                    const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode,
                                    const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> &encode_ack_chan) :
        ActionIntermediateReceive<MESSAGE_TYPE, CONTROLLER_TYPE, RECEIVE_QUEUE_TYPE, RESPOND_QUEUE_TYPE, MSG_TYPE, MESSAGE_REQUEST_ID, ACK_TYPE>(controller,
                                  [this](const ACK_TYPE &A, const MaceCore::ModuleCharacteristic &B, const RESPOND_QUEUE_TYPE &C, const MaceCore::ModuleCharacteristic &D){ActionIntermediateRespond<MESSAGE_TYPE, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>::NextTransmission(A,B,C,D);},
                                  decode),
        ActionIntermediateRespond<MESSAGE_TYPE, CONTROLLER_TYPE, ACK_TYPE, MESSAGE_ACK_ID...>(controller, encode_ack_chan)
    {
    }
};

}

#endif // ACTION_INTERMEDIATE_H
