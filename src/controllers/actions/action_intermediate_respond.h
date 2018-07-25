#ifndef ACTION_RESPONSE_INTERMEDIATE_H
#define ACTION_RESPONSE_INTERMEDIATE_H

#include "action_base.h"

namespace Controllers {

template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename MSG_TYPE, const int ...MESSAGE_ACK_ID>
class ActionIntermediateRespond :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;

public:

    ActionIntermediateRespond()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionIntermediateRespond(CONTROLLER_TYPE *controller,
                                  const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {
    }

protected:

    template<typename QUEUE_TYPE>
    void NextTransmission(const MSG_TYPE &cmd, const COMPONENT_KEY &sender, const QUEUE_TYPE &queueObj, const COMPONENT_KEY &target)
    {

        std::vector<int> expectedResponses { { MESSAGE_ACK_ID... } };
        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, expectedResponses, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_RESPONSE_INTERMEDIATE_H
