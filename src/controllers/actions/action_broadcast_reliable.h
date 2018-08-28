#ifndef ACTION_BROADCAST_RELIABLE_H
#define ACTION_BROADCAST_RELIABLE_H

#include "action_base.h"
#include <vector>

#include "common/optional_parameter.h"
#include "common/object_int_tuple.h"

namespace Controllers {


template<typename DATA_TYPE, typename COMPONENT_KEY>
class IActionBroadcastReliable
{
public:
    virtual void BroadcastReliable(const DATA_TYPE &commandItem, const COMPONENT_KEY &sender, bool UniqueDestinationsOnly) = 0;
};


//!
//! \brief Sets up an action that does a reliable broadcast of data.
//!
//! \template MESSAGE_TYPE Underlaying generic message type that all communication is done through
//! \template COMPONENT_KEY Type that identifies actors on the network
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template DATA_TYPE Incomming data type of data that is to be sent. This is the data that is stored/used interanally in the module.
//! \template MSG_TYPE Type of communications messsage that is to be transmitted out
//! \template MESSAGE_ACK_ID intenger ID that identifies the message that is to stop transmission
//!
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, const int MESSAGE_ACK_ID>
class ActionBroadcastReliable :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>,
        public IActionBroadcastReliable<DATA_TYPE, COMPONENT_KEY>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:

    //!
    //! \brief Function that prepares for the reliable broadcast
    //! \param data Incomming data to translate, given in the BroadcastReliable function
    //! \param sender Module emitting this action, given in the Send function
    //! \param msg Message to send out
    //!
    virtual void Construct_BroadcastReliable(const DATA_TYPE &data, const COMPONENT_KEY &sender, std::vector<MSG_TYPE> &vec) = 0;

public:


    ActionBroadcastReliable(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    /**
     * @brief Broadcast data
     * @param commandItem Data to broadcast
     * @param sender Module sending
     */
    virtual void BroadcastReliable(const DATA_TYPE &commandItem, const COMPONENT_KEY &sender, bool UniqueDestinationsOnly)
    {
        std::vector<MSG_TYPE> vec;
        Construct_BroadcastReliable(commandItem, sender, vec);


        std::vector<COMPONENT_KEY> targets = BASE::m_Controller-> template GetAllTargets();


        int i = 0;
        for(auto it = vec.cbegin() ; it != vec.cend() ; ++it)
        {

            MSG_TYPE msg = *it;
            for(auto itt = targets.cbegin() ; itt != targets.cend() ; ++itt)
            {
                COMPONENT_KEY target = *itt;
                ObjectIntTuple<COMPONENT_KEY> queue(target, i);
                BASE::m_Controller-> template QueueTransmission<ObjectIntTuple<COMPONENT_KEY>>(queue, MESSAGE_ACK_ID, [this, msg, sender, target](){
                    BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, msg, sender, target);
                });
            }
            i++;
        }

    }
};

}

#endif // ACTION_BROADCAST_RELIABLE_H
