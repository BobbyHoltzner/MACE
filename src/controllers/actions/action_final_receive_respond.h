#ifndef ACTION_RESPONSE_FINAL_DELIVERY_H
#define ACTION_RESPONSE_FINAL_DELIVERY_H

#include "action_base.h"

namespace Controllers {


template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename FINAL_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID>
class ActionFinalReceiveRespond :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;

    std::function<void(uint8_t, uint8_t, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> m_encode_ack_chan;

protected:

    virtual bool Construct_FinalObjectAndResponse(const MSG_TYPE &, const MaceCore::ModuleCharacteristic &sender, ACK_TYPE &, std::shared_ptr<FINAL_TYPE> &, MaceCore::ModuleCharacteristic &vehicleObj, QUEUE_TYPE &queueObj)= 0;

public:

    ActionFinalReceiveRespond(CONTROLLER_TYPE *controller,
                           const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode,
                           const std::function<void(uint8_t, uint8_t, uint8_t, MESSAGE_TYPE*, const ACK_TYPE*)> &encode_ack_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, [](uint8_t, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*){}, decode),
        m_encode_ack_chan(encode_ack_chan)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this, encode_ack_chan](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    ACK_TYPE ack;
                    std::shared_ptr<FINAL_TYPE> finalObj;
                    QUEUE_TYPE queueObj;

                    bool valid = this-> template Construct_FinalObjectAndResponse(msg, sender, ack, finalObj, vehicleFrom, queueObj);
                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);
                        BASE::m_Controller->onDataReceived(queueObj, finalObj);
                        this->template FinalResponse(ack, vehicleFrom, queueObj, target);
                    }
                }
        );
    }

    void FinalResponse(const ACK_TYPE &cmd, const MaceCore::ModuleCharacteristic &sender, const QUEUE_TYPE &queueObj, const MaceCore::ModuleCharacteristic &target)
    {
        UNUSED(queueObj);
        BASE::m_Controller->template EncodeMessage(m_encode_ack_chan, cmd, sender, target);
    }

protected:
};

}

#endif // ACTION_RESPONSE_FINAL_DELIVERY_H
