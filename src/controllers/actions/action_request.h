#ifndef ACTION_REQUEST_H
#define ACTION_REQUEST_H

#include "action_base.h"

namespace Controllers {

template<typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_ACK_ID>
class ActionRequest_TargetedWithResponse :
        public ActionBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:
    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, MSG_TYPE &, QUEUE_TYPE &) = 0;

public:


    ActionRequest_TargetedWithResponse(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    void Request(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        Request_Construct(sender, target, cmd, queueObj);

        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_REQUEST_H
