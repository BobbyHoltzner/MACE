#ifndef ACTION_REQUEST_H
#define ACTION_REQUEST_H

#include "action_base.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename MSG_TYPE, const int ...MESSAGE_ACK_ID>
class ActionRequest_TargetedWithResponse :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:
    virtual void Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, MSG_TYPE &, QUEUE_TYPE &) = 0;

public:


    ActionRequest_TargetedWithResponse(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    void Request(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        Request_Construct(sender, target, cmd, queueObj);

        std::vector<int> expectedResponses { { MESSAGE_ACK_ID... } };

        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, expectedResponses, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_REQUEST_H
