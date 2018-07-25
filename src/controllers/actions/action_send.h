#ifndef ACTION_SEND_H
#define ACTION_SEND_H

#include "action_base.h"

#include "common/optional_parameter.h"

#include <vector>

namespace Controllers {

template<typename DATA_TYPE, typename COMPONENT_KEY>
class IActionSend
{
public:
    virtual void Send(const DATA_TYPE &commandItem, const COMPONENT_KEY &sender, const COMPONENT_KEY &target) = 0;
};


//!
//! \brief Sets up an action that sends a data type to an underlying communications paradigm.
//!
//! When sending an action the transmit will be queued on the controller until the exepcted response is heard.
//! When the given response type is heard by the expected object the queued transmission will be removed.
//!
//! \template MESSAGE_TYPE Underlaying generic message type that all communication is done through
//! \template COMPONENT_KEY Type that identifies actors on the network
//! \template CONTROLLER_TYPE Type of controller being used by this action, will be used to queue transmissions.
//! \template QUEUE_TYPE Type of object that will establish uniqueness in the queue.
//!   This ultimatly allows a controller to have two identical messages going out to two entities.
//! \template DATA_TYPE Incomming data type of data that is to be sent. This is the data that is stored/used interanally in the module.
//! \template MSG_TYPE Type of communications messsage that is to be transmitted out
//! \template MESSAGE_ACK_ID intenger ID that identifies the message that is to stop transmission
//!
template<typename MESSAGE_TYPE, typename COMPONENT_KEY, typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename DATA_TYPE, typename MSG_TYPE, const int MESSAGE_ACK_ID>
class ActionSend :
        public ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>,
        public IActionSend<DATA_TYPE, COMPONENT_KEY>
{

    typedef ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:

    //!
    //! \brief Method that is to be implimented for this action that translates a data of type DATA_TYPE and generate corrisponding message of type MSG_TYPE.
    //! This method also realizes the queue object to identify the transmission
    //!
    //! \param data Incomming data to translate, given in the Send function
    //! \param sender Module emitting this action, given in the Send function
    //! \param target Module targeted by action
    //! \param msg Communications message to send to comms interface
    //! \param queue Queue object to identifiy this tranmissions when ack is returned
    //! \return True is to procede with transmission
    //!
    virtual bool Construct_Send(const DATA_TYPE &data, const COMPONENT_KEY &sender, const COMPONENT_KEY &target, MSG_TYPE &msg, QUEUE_TYPE &queue) = 0;

public:

    ActionSend(CONTROLLER_TYPE *controller,
               const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGE_TYPE*, const MSG_TYPE*)> &encode_chan) :
        ActionBase<MESSAGE_TYPE, CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, {})
    {

    }


    //!
    //! \brief Send data to a given target
    //! \param data Data to send
    //! \param sender Module sending data
    //! \param target Module that is to receive the data
    //!
    void Send(const DATA_TYPE &data, const COMPONENT_KEY &sender, const COMPONENT_KEY &target)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        if(Construct_Send(data, sender, target, cmd, queueObj) == false)
        {
            return;
        }

        BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
        });
    }
};

}

#endif // ACTION_SEND_H
