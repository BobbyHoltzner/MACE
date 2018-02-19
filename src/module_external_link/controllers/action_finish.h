#ifndef ACTION_FINISH_H
#define ACTION_FINISH_H

#include "action_base.h"

namespace ExternalLink {

template<typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID>
class ActionFinish :
        public ActionBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    virtual bool Finish_Receive(const MSG_TYPE &, const MaceCore::ModuleCharacteristic &sender, QUEUE_TYPE &queueObj) = 0;

public:

    ActionFinish()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionFinish(CONTROLLER_TYPE *controller,
                           const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        ActionBase<CONTROLLER_TYPE, MSG_TYPE>(controller, [](uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*){}, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    QUEUE_TYPE queueObj;
                    bool valid = this-> template Finish_Receive(msg, sender, queueObj);
                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);
                    }
                }
        );
    }

protected:
};

}

#endif // ACTION_FINISH_H
