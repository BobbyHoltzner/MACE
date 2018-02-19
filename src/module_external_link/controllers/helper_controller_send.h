#ifndef HELPER_CONTROLLER_SEND_H
#define HELPER_CONTROLLER_SEND_H
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



template<typename CONTROLLER_TYPE, typename MSG_TYPE>
class _HelperControllerSendBase
{
protected:

    CONTROLLER_TYPE *m_Controller;

    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const mace_message_t*, MSG_TYPE*)> m_DecodeFunc;



protected:


public:

    _HelperControllerSendBase()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    _HelperControllerSendBase(CONTROLLER_TYPE *controller,
                              const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan,
                              const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        m_Controller(controller),
        m_EncodeChanFunc(encode_chan),
        m_DecodeFunc(decode)
    {
    }



};


template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID, typename QUEUE_TYPE>
class _HelperControllerSendInitial :
        public _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE> BASE;
protected:
    virtual void BuildMessage_Send(const DATA_TYPE &, const MaceCore::ModuleCharacteristic &sender, MSG_TYPE &, QUEUE_TYPE &, MaceCore::ModuleCharacteristic &target) = 0;

    virtual bool BuildData_Send(const MSG_TYPE &, ACK_TYPE &, MaceCore::ModuleCharacteristic &vehicleObj, QUEUE_TYPE &queueObj)= 0;

public:

    _HelperControllerSendInitial()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    _HelperControllerSendInitial(CONTROLLER_TYPE *controller,
                                 const std::function<void(const ACK_TYPE &, const MaceCore::ModuleCharacteristic &, const QUEUE_TYPE &, const MaceCore::ModuleCharacteristic &)> &nextStep,
                                 const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan,
                                 const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, decode)
    {

        BASE::m_Controller->template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this, nextStep](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    ACK_TYPE ack;
                    QUEUE_TYPE queueObj;
                    MaceCore::ModuleCharacteristic vehicleFrom;

                    bool valid = this-> template BuildData_Send(msg, ack, vehicleFrom, queueObj);
                    if(valid == true)
                    {
                        nextStep(ack, vehicleFrom, queueObj, target);
                    }
                }
        );
    }


    void Set(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        MaceCore::ModuleCharacteristic target;
        BuildMessage_Send(commandItem, sender, cmd, queueObj, target);


        if(target.ID == 0)
        {
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender);
        }
        else
        {
            BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
                BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
            });
        }
    }
};



template<typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int ...MESSAGE_ACK_ID>
class _HelperControllerIntermediate :
        public _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    virtual bool BuildData_Send(const MSG_TYPE &, ACK_TYPE &, MaceCore::ModuleCharacteristic &vehicleObj, QUEUE_TYPE &queueObj)= 0;

public:

    _HelperControllerIntermediate()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    _HelperControllerIntermediate(CONTROLLER_TYPE *controller,
                                  const std::function<void(const ACK_TYPE &, const MaceCore::ModuleCharacteristic &, const QUEUE_TYPE &, const MaceCore::ModuleCharacteristic &)> &nextStep,
                                  const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan,
                                  const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>(controller, encode_chan, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this, nextStep](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    ACK_TYPE ack;
                    QUEUE_TYPE queueObj;

                    bool valid = this-> template BuildData_Send(msg, ack, vehicleFrom, queueObj);


                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);

                        //BASE::m_Controller->Set(ack, vehicleFrom, queueObj, target);
                        nextStep(ack, vehicleFrom, queueObj, target);
                    }
                }
        );
    }

protected:

    void Set(const MSG_TYPE &cmd, const MaceCore::ModuleCharacteristic &sender, const QUEUE_TYPE &queueObj, const MaceCore::ModuleCharacteristic &target)
    {
        if(target.ID == 0)
        {
            BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender);
        }
        else
        {
            std::vector<int> expectedResponses { { MESSAGE_ACK_ID... } };
            BASE::m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, expectedResponses, [this, cmd, sender, target](){
                BASE::m_Controller-> template EncodeMessage(BASE::m_EncodeChanFunc, cmd, sender, target);
            });
        }
    }
};


template<typename CONTROLLER_TYPE, typename FINAL_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID, typename QUEUE_TYPE>
class _HelperControllerFinal :
        public _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    virtual bool BuildData_Send(const MSG_TYPE &, ACK_TYPE &, std::shared_ptr<FINAL_TYPE> &, MaceCore::ModuleCharacteristic &vehicleObj, QUEUE_TYPE &queueObj)= 0;

public:

    _HelperControllerFinal()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    _HelperControllerFinal(CONTROLLER_TYPE *controller,
                           const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode,
                           const std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> &encode_ack_chan) :
        _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>(controller, [](uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*){}, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this, encode_ack_chan](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    ACK_TYPE ack;
                    QUEUE_TYPE queueObj;
                    std::shared_ptr<FINAL_TYPE> finalObj;

                    bool valid = this-> template BuildData_Send(msg, ack, finalObj, vehicleFrom, queueObj);
                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);
                        BASE::m_Controller->onDataReceived(queueObj, finalObj);
                        BASE::m_Controller->template EncodeMessage(encode_ack_chan, ack, vehicleFrom, target);
                    }
                }
        );
    }

protected:
};


template<typename CONTROLLER_TYPE, typename QUEUE_TYPE, typename MSG_TYPE, const int MESSAGE_REQUEST_ID>
class _HelperControllerFinalNoResponse :
        public _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>
{

    typedef _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE> BASE;

protected:

    virtual bool BuildData_Send(const MSG_TYPE &, QUEUE_TYPE &queueObj) = 0;

public:

    _HelperControllerFinalNoResponse()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    _HelperControllerFinalNoResponse(CONTROLLER_TYPE *controller,
                           const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode) :
        _HelperControllerSendBase<CONTROLLER_TYPE, MSG_TYPE>(controller, [](uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*){}, decode)
    {


        BASE::m_Controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( BASE::m_DecodeFunc,
                [this](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){

                    QUEUE_TYPE queueObj;
                    bool valid = this-> template BuildData_Send(msg, queueObj);
                    if(valid == true)
                    {
                        BASE::m_Controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);
                    }
                }
        );
    }

protected:
};









































template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID, typename QUEUE_TYPE>
class HelperControllerSendBase
{
private:

    CONTROLLER_TYPE *m_Controller;

    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> m_EncodeChanFunc;
    std::function<void(const mace_message_t*, MSG_TYPE*)> m_DecodeFunc;
    std::function<void(uint8_t, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> m_EncodeAckChanFunc;



protected:

    virtual void BuildMessage_Send(const DATA_TYPE &, const MaceCore::ModuleCharacteristic &sender, MSG_TYPE &, QUEUE_TYPE &, MaceCore::ModuleCharacteristic &target) = 0;

public:

    HelperControllerSendBase()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    HelperControllerSendBase(CONTROLLER_TYPE *controller, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan, const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> encode_ack_chan) :
        m_Controller(controller),
        m_EncodeChanFunc(encode_chan),
        m_DecodeFunc(decode),
        m_EncodeAckChanFunc(encode_ack_chan)
    {
    }

    void Set(const DATA_TYPE &commandItem, const MaceCore::ModuleCharacteristic &sender)
    {
        MSG_TYPE cmd;
        QUEUE_TYPE queueObj;
        MaceCore::ModuleCharacteristic target;
        BuildMessage_Send(commandItem, sender, cmd, queueObj, target);


        if(commandItem.getTargetSystem() == 0)
        {
            m_Controller-> template EncodeMessage(m_EncodeChanFunc, cmd, sender);
        }
        else
        {
            m_Controller-> template QueueTransmission<QUEUE_TYPE>(queueObj, MESSAGE_ACK_ID, [this, cmd, sender, target](){
                m_Controller-> template EncodeMessage(m_EncodeChanFunc, cmd, sender, target);
            });
        }
    }

};


template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID, typename QUEUE_TYPE, typename NOTIFY_DATA_TYPE = void>
class HelperControllerSend :
        public HelperControllerSendBase<CONTROLLER_TYPE, DATA_TYPE, MSG_TYPE, ACK_TYPE, MESSAGE_REQUEST_ID, MESSAGE_ACK_ID, QUEUE_TYPE>
{
protected:
    virtual bool BuildData_Send(const MSG_TYPE &, std::shared_ptr<NOTIFY_DATA_TYPE>, ACK_TYPE &, MaceCore::ModuleCharacteristic &vehicleFrom, QUEUE_TYPE &queueObj) = 0;

public:

    HelperControllerSend()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    HelperControllerSend(CONTROLLER_TYPE *controller, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan, const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> encode_ack_chan) :
        HelperControllerSendBase<CONTROLLER_TYPE, DATA_TYPE, MSG_TYPE, ACK_TYPE, MESSAGE_REQUEST_ID, MESSAGE_ACK_ID, QUEUE_TYPE>(controller, encode_chan, decode, encode_ack_chan)
    {

        controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( decode,
                [this, controller, &encode_ack_chan](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){
                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    std::shared_ptr<NOTIFY_DATA_TYPE> tmp = std::make_shared<NOTIFY_DATA_TYPE>();
                    ACK_TYPE ack;

                    QUEUE_TYPE queueObj;
                    bool valid = this-> template BuildData_Send(msg, tmp, ack, vehicleFrom, queueObj);

                    controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);

                    if(valid == true)
                    {
                        controller->onDataReceived(sender, tmp);
                        controller->template EncodeMessage(encode_ack_chan, ack, vehicleFrom, target);
                    }
                }
        );
    }
};



template<typename CONTROLLER_TYPE, typename DATA_TYPE, typename MSG_TYPE, typename ACK_TYPE, const int MESSAGE_REQUEST_ID, const int MESSAGE_ACK_ID, typename QUEUE_TYPE>
class HelperControllerSend<CONTROLLER_TYPE, DATA_TYPE, MSG_TYPE, ACK_TYPE, MESSAGE_REQUEST_ID, MESSAGE_ACK_ID, QUEUE_TYPE> :
        public HelperControllerSendBase<CONTROLLER_TYPE, DATA_TYPE, MSG_TYPE, ACK_TYPE, MESSAGE_REQUEST_ID, MESSAGE_ACK_ID, QUEUE_TYPE>
{
protected:
    virtual bool BuildData_Send(const MSG_TYPE &, ACK_TYPE &, MaceCore::ModuleCharacteristic &vehicleFrom, QUEUE_TYPE &queueObj)= 0;

public:

    HelperControllerSend()
    {
        throw std::runtime_error("Default Constructor not supported");
    }

    HelperControllerSend(CONTROLLER_TYPE *controller, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const MSG_TYPE*)> &encode_chan, const std::function<void(const mace_message_t*, MSG_TYPE*)> &decode, const std::function<void(uint8_t system_id, uint8_t, uint8_t, mace_message_t*, const ACK_TYPE*)> encode_ack_chan) :
        HelperControllerSendBase<CONTROLLER_TYPE, DATA_TYPE, MSG_TYPE, ACK_TYPE, MESSAGE_REQUEST_ID, MESSAGE_ACK_ID, QUEUE_TYPE>(controller, encode_chan, decode, encode_ack_chan)
    {

        controller-> template AddTriggeredLogic<MESSAGE_REQUEST_ID, MSG_TYPE>( decode,
                [this, controller, &encode_ack_chan](const MSG_TYPE  &msg, const MaceCore::ModuleCharacteristic &sender){
                    MaceCore::ModuleCharacteristic target = sender;

                    MaceCore::ModuleCharacteristic vehicleFrom;
                    ACK_TYPE ack;

                    QUEUE_TYPE queueObj;
                    bool valid = this-> template BuildData_Send(msg, ack, vehicleFrom, queueObj);

                    controller->RemoveTransmission(queueObj, MESSAGE_REQUEST_ID);

                    if(valid == true)
                    {
                        controller->template EncodeMessage(encode_ack_chan, ack, vehicleFrom, target);
                    }
                }
        );
    }
};



}

#endif // HELPER_CONTROLLER_SEND_H
