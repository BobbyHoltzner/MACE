#ifndef ACTION_BROADCAST_H
#define ACTION_BROADCAST_H

#include "action_base.h"

#include "common/optional_parameter.h"

namespace Controllers {

//!
//! \brief Sets up an action to broadcast data
//!
//! Unlike ActionSend or ActionRequest, no transmission is queued.
//! This is because it is a broadcast and thus there is no knowledge of who should acknowledge.
//!
//! \template MESSAGE_TYPE Underlaying generic message type that all communication is done through
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template DATA_TYPE Incomming data type of data that is to be sent. This is the data that is stored/used interanally in the module.
//! \template MSG_TYPE Type of communications messsage that is to be transmitted out
//!
template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE>
class ActionBroadcast :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:

    //!
    //! \brief Method that is to be implimented for this action to generate broadcasted message
    //! \param data Incomming data to translate, given in the Broadcast function
    //! \param sender Module emitting this action, given in the Send function
    //! \param msg Communications message to send to comms interface
    //!
    virtual void Construct_Broadcast(const DATA_TYPE &data, const MaceCore::ModuleCharacteristic &sender, MSG_TYPE &msg) = 0;

public:


    ActionBroadcast(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    /**
     * @brief Broadcast data
     * @param commandItem Data to broadcast
     * @param sender Module sending
     */
    void Broadcast(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        Construct_Broadcast(commandItem, sender, cmd);

        BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender);
    }
};

}

#endif // ACTION_BROADCAST_H
