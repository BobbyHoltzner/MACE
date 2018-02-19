#ifndef GENERIC_REQUEST_CONTROLLER_H
#define GENERIC_REQUEST_CONTROLLER_H

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


template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID>
class HelperControllerRequest
{
private:

    CONTROLLER_TYPE *m_Controller;

    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const mace_message_t*, MSG_TYPE*)> m_DecodeFunc;
    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> m_EncodeAckChanFunc;



protected:

    virtual void BuildMessage_Request(const MaceCore::ModuleCharacteristic &target, MSG_TYPE &) = 0;

    virtual std::vector<ACK_TYPE> BuildData_Request(const MSG_TYPE &, std::shared_ptr<DATA_TYPE>) const= 0;

public:

    HelperControllerRequest(CONTROLLER_TYPE *controller, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan, const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> encode_ack_chan) :
        m_Controller(controller),
        m_EncodeChanFunc(encode_chan),
        m_DecodeFunc(decode),
        m_EncodeAckChanFunc(encode_ack_chan)
    {

        m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){
                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    vehicleFrom.ID = msg.target_system;
                    vehicleFrom.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

                    std::shared_ptr<DATA_TYPE> tmp = std::make_shared<DATA_TYPE>();
                    std::vector<ACK_TYPE> ack = this-> template BuildData_Request(msg, tmp);

                    for(auto it = ack.cbegin() ; it != ack.cend() ; ++it)
                    {
                        m_Controller->template EncodeMessage(m_EncodeAckChanFunc, *it, vehicleFrom, target);
                    }
                }
        );
    }

    void Request(const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        BuildMessage_Request(target, cmd);

        m_Controller-> template QueueTransmission<MaceCore::ModuleCharacteristic>(target, MESSAGE_ACK_ID, [this, cmd, sender, target](){
            m_Controller-> template EncodeMessage(m_EncodeChanFunc, cmd, sender, target);
        });
    }

};

}

#endif // GENERIC_REQUEST_CONTROLLER_H
