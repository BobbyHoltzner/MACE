#ifndef ACTION_SEND_H
#define ACTION_SEND_H

#include "action_base.h"

#include "common/optional_parameter.h"

namespace Controllers {


template<typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename DATA_TYPE, typename MSG_TYPE, const int MESSAGE_ACK_ID>
class ActionSend :
        public ActionBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:
    virtual void Construct_Send(const DATA_TYPE &, const MaceCore::ModuleCharacteristic &sender, MSG_TYPE &, QUEUE_TYPE &) = 0;

public:

    ActionSend(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    void Send(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        Construct_Send(commandItem, sender, cmd, queueObj);

        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_SEND_H
