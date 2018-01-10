#ifndef GENERIC_MACE_CONTROLLER_H
#define GENERIC_MACE_CONTROLLER_H

#include "generic_controller.h"
#include "common/fsm.h"
#include "common/pointer_collection.h"

#include <tuple>

namespace ExternalLink {

typedef GenericController<mace_message_t, MaceCore::ModuleCharacteristic> GenericMACEController;

template<typename RequestInterface, typename ResponseInterface>
class GenericMACEController_RequestResponse : public GenericMACEController
{
private:

    int m_LinkChan;


    ResponseInterface *m_CB_Response;
    RequestInterface *m_CB_Request;

    PointerCollection<ResponseInterface, RequestInterface> m_cb;

protected:

    std::vector<std::tuple<std::function<bool(MaceCore::ModuleCharacteristic, const mace_message_t*)>, std::function<void(MaceCore::ModuleCharacteristic, const mace_message_t*)>>> m_MessageBehaviors;

public:

    template <typename T>
    T* Callback()
    {
        T* ptr;
        m_cb.Get(ptr);
        return ptr;
    }

    AddMessageLogic(const std::function<bool(MaceCore::ModuleCharacteristic, const mace_message_t*)> &critera, const std::function<void(MaceCore::ModuleCharacteristic, const mace_message_t*)> &action)
    {
        auto newItem = std::make_tuple(critera, action);
        m_MessageBehaviors.push_back(newItem);
    }

    GenericMACEController_RequestResponse(RequestInterface *cb_requestCB, ResponseInterface *cb_response, int linkChan) :
        GenericMACEController(),
        m_LinkChan(linkChan)
    {
        m_cb.Set(cb_requestCB);
        m_cb.Set(cb_response);
    }

    template <const int I>
    static std::function<bool(MaceCore::ModuleCharacteristic, const mace_message_t*)> MaceMessageIDEq()
    {
        return [](MaceCore::ModuleCharacteristic sender, const mace_message_t* message){
            return message->msgid ==I;
        };
    }

    template <typename DECODE_TYPE, typename DECODE_FUNC>
    static std::function<void(MaceCore::ModuleCharacteristic, const mace_message_t*)> MaceProcessFSMState(DECODE_FUNC decode_func, const std::function<void(const DECODE_TYPE &msg, const MaceCore::ModuleCharacteristic &sender)> &func)
    {
        return [decode_func, func](MaceCore::ModuleCharacteristic sender, const mace_message_t* message)
        {
            DECODE_TYPE decodedMSG;
            decode_func(message, &decodedMSG);

            func(decodedMSG, sender);
        };
    }




    virtual bool ReceiveMessage(const mace_message_t *message)
    {
        int systemID = message->sysid;
        int compID = message->compid;

        MaceCore::ModuleCharacteristic sender;
        sender.ID = systemID;
        sender.Class = (MaceCore::ModuleClasses)compID;

        for(auto it = m_MessageBehaviors.cbegin() ; it != m_MessageBehaviors.cend() ; ++it)
        {
            bool criteraEvaluation = std::get<0>(*it)(sender, message);
            if(criteraEvaluation)
            {
                std::get<1>(*it)(sender, message);
                return true;
            }
        }
    }


protected:


    template <typename FUNC, typename TT>
    EncodeMessage(FUNC func, TT requestItem, OptionalParameter<MaceCore::ModuleCharacteristic> sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(sender.IsSet() == false) {
            throw std::runtime_error("no sender given");
        }

        mace_message_t msg;
        func(sender().ID, (int)sender().Class, m_LinkChan, &msg, &requestItem);
        GenericMACEController::TransmitMessage(msg, target);
    }

};


}

#endif // GENERIC_MACE_CONTROLLER_H
