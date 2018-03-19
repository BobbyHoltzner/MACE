#ifndef ACTION_RECEIVE_INTERMEDIATE_H
#define ACTION_RECEIVE_INTERMEDIATE_H

#include "action_base.h"

namespace Controllers {

template<typename CONTROLLER_TYPE, typename RECEIVE_QUEUE_TYPE, typename RESPOND_QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID, typename ACK_TYPE>
class ActionIntermediateReceive :
        public ActionBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    virtual bool BuildData_Send(const MSG_TYPE &, const MaceCore::ModuleCharacteristic &sender, ACK_TYPE &, MaceCore::ModuleCharacteristic &vehicleObj, RECEIVE_QUEUE_TYPE &receiveQueueObj, RESPOND_QUEUE_TYPE &respondQueueObj)= 0;

public:

    ActionIntermediateReceive()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    ActionIntermediateReceive(CONTROLLER_TYPE *controller,
                                  const std::function<void(const ACK_TYPE &, const MaceCore::ModuleCharacteristic &, const RESPOND_QUEUE_TYPE &, const MaceCore::ModuleCharacteristic &)> &nextStep,
                                  const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        ActionBase<CONTROLLER_TYPE, MSG_TYPE>(controller, {}, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this, nextStep](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    ACK_TYPE ack;
                    RECEIVE_QUEUE_TYPE receiveQueueObj;
                    RESPOND_QUEUE_TYPE respondQueueObj;

                    bool valid = this-> template BuildData_Send(msg, sender, ack, vehicleFrom, receiveQueueObj, respondQueueObj);


                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(receiveQueueObj, MESSAGE_REQUEST_ID);

                        //BASE::m_Controller->Set(ack, vehicleFrom, queueObj, target);
                        nextStep(ack, vehicleFrom, respondQueueObj, target);
                    }
                }
        );
    }
};

}

#endif // ACTION_RECEIVE_INTERMEDIATE_H
