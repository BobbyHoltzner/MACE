#ifndef HELPER_CONTROLLER_BROADCAST_H
#define HELPER_CONTROLLER_BROADCAST_H

#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "mace_core/module_characteristics.h"

#include "generic_mace_controller.h"

namespace ExternalLink {


template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID>
class HelperControllerBroadcast
{
private:

    CONTROLLER_TYPE *m_Controller;

    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const mace_message_t*, MSG_TYPE*)> m_DecodeFunc;



protected:

    virtual void BuildMessage_Broadcast(const DATA_TYPE &, MSG_TYPE &) = 0;

    virtual void BuildData_Broadcast(const MSG_TYPE &, std::shared_ptr<DATA_TYPE>, const MaceCore::ModuleCharacteristic &) const= 0;

public:

    HelperControllerBroadcast(CONTROLLER_TYPE *controller,
                            const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan,
                            const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        m_Controller(controller),
        m_EncodeChanFunc(encode_chan),
        m_DecodeFunc(decode)
    {

        m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    std::shared_ptr<DATA_TYPE> tmp = std::make_shared<DATA_TYPE>();
                    this-> template BuildData_Broadcast(msg, tmp, sender);

                    m_Controller->template onDataReceived(sender, tmp);
                }
        );
    }

    void Broadcast(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        BuildMessage_Broadcast(commandItem, cmd);

        m_Controller-> template EncodeMessage(m_EncodeChanFunc, cmd, sender);
    }

};

}

#endif // HELPER_CONTROLLER_BROADCAST_H
