#ifndef ACTION_RESPONSE_INTERMEDIATE_H
#define ACTION_RESPONSE_INTERMEDIATE_H

#include "action_base.h"

namespace Controllers {

template<typename CONTROLLER_TYPE, typename MSG_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediateRespond :
        public ActionBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

public:

    ActionIntermediateRespond()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionIntermediateRespond(CONTROLLER_TYPE *controller,
                                  const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {
    }

protected:

    template<typename QUEUE_TYPE>
    void NextTransmission(const MSG_TYPE &cmd, const MaceCore::ModuleCharacteristic &sender, const QUEUE_TYPE &queueObj, const MaceCore::ModuleCharacteristic &target)
    {
        if(target.ID == 0)
        {
            throw std::runtime_error("Target ID is 0. This don't make sense when responding");
        }

        std::vector<int> expectedResponses { { MESSAGE_ACK_ID... } };
        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, expectedResponses, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_RESPONSE_INTERMEDIATE_H
