#ifndef ACTION_UNSOLICITED_RECEIVE_H
#define ACTION_UNSOLICITED_RECEIVE_H

#include "action_base.h"

#include "../base_data_item.h"

namespace Controllers {

//!
//! \brief Action to receive an message type and generates NO response to the sender
//!
//! Use of this method requires the caller to set a lambda in setLambda_DataReceived with approriate FINAL_TYPE object.
//! The lambda set will be called when data is received by this method.
//!
//! \template MESSAGE_TYPE Underlaying generic message type that all communication is done through
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template FINAL_TYPE Type of object that will be sent up.
//! \template MSG_TYPE Type of communications messsage that is to be transmitted out
//! \template MESSAGE_REQUEST_ID Integer code for message that is to kick off this action
//!
template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename FINAL_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID>
class ActionUnsolicitedReceive :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    //!
    //! \brief Method to be implimented that defines how to translate between received message and outgoing data
    //! \param msg Received message
    //! \param sender Module sending message
    //! \param data Data to be generated by message
    //! \return True if message is to be consumed, false if ignored (and possibly consumed by another action)
    //!
    virtual bool Construct_FinalObject(const MSG_TYPE &msg, const MaceCore::ModuleCharacteristic &sender, std::shared_ptr<FINAL_TYPE> &data)= 0;

public:

    ActionUnsolicitedReceive(CONTROLLER_TYPE *controller,
                           const std::function<void(const MESSAGE_TYPE*, MSG_TYPE*)> &decode) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, [](uint8_t, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*){}, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    std::shared_ptr<FINAL_TYPE> finalObj;

                    bool valid = this-> template Construct_FinalObject(msg, sender, finalObj);
                    if(valid == true)
                    {
                        ((Controllers::DataItem<MaceCore::ModuleCharacteristic, FINAL_TYPE>*)BASE::m_Controller)->onDataReceived(sender, finalObj);
                    }
                }
        );
    }

protected:
};

template <typename ...T>
using ActionFinalReceive = ActionUnsolicitedReceive<T...>;

}

#endif // ACTION_UNSOLICITED_RECEIVE_H
