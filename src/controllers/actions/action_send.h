#ifndef ACTION_SEND_H
#define ACTION_SEND_H

#include "action_base.h"

#include "common/optional_parameter.h"

namespace Controllers {

template<typename DATA_TYPE>
class IActionSend
{
public:
    virtual void Send(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target) = 0;
};


template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename DATA_TYPE, typename MSG_TYPE, const int MESSAGE_ACK_ID>
class ActionSend :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>,
        public IActionSend<DATA_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:
    virtual void Construct_Send(const DATA_TYPE &, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, MSG_TYPE &, QUEUE_TYPE &) = 0;

public:

    ActionSend(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    virtual void Send(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        Construct_Send(commandItem, sender, target, cmd, queueObj);

        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_SEND_H
