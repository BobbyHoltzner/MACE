#ifndef ACTION_BROADCAST_H
#define ACTION_BROADCAST_H

#include "action_base.h"

#include "common/optional_parameter.h"

namespace Controllers {

template<typename DATA_TYPE>
class IActionBroadcast
{
public:
    virtual void Broadcast(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender) = 0;
};

template<typename MESSAGE_TYPE, typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE>
//!
//! \brief The ActionBroadcast class
//!
class ActionBroadcast :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>,
        public IActionBroadcast<DATA_TYPE>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:

    virtual void Construct_Broadcast(const DATA_TYPE &, const MaceCore::ModuleCharacteristic &sender, MSG_TYPE &) = 0;

public:


    ActionBroadcast(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    /**
     * @brief Broadcast data
     * @param commandItem Data to broadcast
     * @param sender Module sending
     */
    virtual void Broadcast(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        Construct_Broadcast(commandItem, sender, cmd);

        BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender);
    }
};

}

#endif // ACTION_BROADCAST_H
