#ifndef GENERIC_MACE_CONTROLLER_H
#define GENERIC_MACE_CONTROLLER_H

#include "common/common.h"

#include <unordered_map>

#include "mace_core/module_characteristics.h"
#include "I_controller.h"
#include "common/pointer_collection.h"

#include <spdlog/spdlog.h>

#include <tuple>
#include <functional>

#include "common/optional_parameter.h"
#include "common/chain_inheritance.h"
#include "common/object_int_tuple.h"

#include "I_message_notifier.h"

#include "base_data_item.h"

#include "base_module_queue.h"

namespace Controllers {


//!
//! \brief Sets up basic operations of a controller
//!
//! The purpose of the controller is to receive messages and queue transmissions.
//!
//! Queued transmissions are initiated with parameters that signify what reception satifisfies the transmissions.
//! i.e. when the transmission doesn't need to transmit anymore.
//!
//! The controller can also set up the actions to take when a message is received, which may be a further transmission or finish off the exchange.
//!
//! \template MESSAGETYPE The type of message the controller is to injest/output
//! \template TransmitQueueType Queue object the controller is to use when sending messages
//! \template DataItems Data items the controller is to transmit
//!
template<typename MESSAGETYPE, typename COMPONENT_KEY, typename TransmitQueueType, typename FINISH_CODE, typename ...DataItems>
class GenericController : public IController<MESSAGETYPE, COMPONENT_KEY>, public TransmitQueueType, public ChainInheritance<DataItems...>
{
private:

    int m_LinkChan;

    const IMessageNotifier<MESSAGETYPE, COMPONENT_KEY>* m_CB;

protected:

    std::shared_ptr<spdlog::logger> mLog;

    std::vector<std::tuple<
        std::function<bool(COMPONENT_KEY, const MESSAGETYPE*)>,
        std::function<void(COMPONENT_KEY, const MESSAGETYPE*)>
    >> m_MessageBehaviors;

    std::mutex m_MessageBehaviorsMutex;

    std::unordered_map<void*, std::function<void(const bool completed, const FINISH_CODE finishCode)>> m_FinishLambda;
    std::unordered_map<void*, std::function<void()>> m_ShutdownLambda;

    std::mutex m_MutexFinishLambda;
    std::mutex m_MutexShutdownLambda;

public:

    GenericController(const IMessageNotifier<MESSAGETYPE, COMPONENT_KEY>* cb, TransmitQueue<MESSAGETYPE, COMPONENT_KEY>* queue, int linkChan) :
        m_LinkChan(linkChan),
        m_CB(cb)
    {
        TransmitQueueType::SetQueue(queue);
    }

    virtual ~GenericController() = default;

    //!
    //! \brief Remove all action tied to the given host
    //! \param ptr Pointer to host that actions attributed to are to be removed
    //!
    virtual void RemoveHost(void* ptr)
    {
        m_MutexFinishLambda.lock();
        m_FinishLambda.erase(ptr);
        m_MutexFinishLambda.unlock();

        m_MutexShutdownLambda.lock();
        m_ShutdownLambda.erase(ptr);
        m_MutexShutdownLambda.unlock();
    }

    void setLambda_Finished(const std::function<void(const bool completed, const FINISH_CODE finishCode)> &lambda){
        m_MutexFinishLambda.lock();

        if(m_FinishLambda.find(0) != m_FinishLambda.cend())
        {
            printf("Warning!!!! A finish procedure already exists, replacing old with new\n");
            m_FinishLambda.erase(0);
        }

        m_FinishLambda.insert({0, lambda});
        m_MutexFinishLambda.unlock();
    }

    //!
    //! \brief Add a behavior to do when controller is finished
    //!
    //! This behavior is tied to a "host", which can be removed should the controller persist beyond the host
    //!
    //! \param host Pointer to host of behavior
    //! \param lambda Action to perform
    //!
    void AddLambda_Finished(void* host, const std::function<void(const bool completed, const FINISH_CODE finishCode)> &lambda){
        m_MutexFinishLambda.lock();
        m_FinishLambda.insert({host, lambda});
        m_MutexFinishLambda.unlock();
    }



    void onFinished(const bool completed, const FINISH_CODE finishCode = FINISH_CODE()){

        m_MutexFinishLambda.lock();
        for(auto it = m_FinishLambda.cbegin() ; it != m_FinishLambda.cend() ; ++it)
        {
            it->second(completed, finishCode);
        }
        m_MutexFinishLambda.unlock();
    }

    //!
    //! \brief Sets a lambda to perform some shutdown action.
    //!
    //! The shutdown action will be performed on its own thread when Shutdown is called.
    //! See Shutdown method for more discussion on this behavior.
    //! \param lambda Lambda to set
    //!
    void setLambda_Shutdown(const std::function<void()> &lambda){

        if(m_ShutdownLambda.find(0) != m_ShutdownLambda.cend())
        {
            printf("Warning!!!! A shutdown procedure already exists, replacing old with new\n");
            m_ShutdownLambda[0] = lambda;
        }
        else
        {
            m_ShutdownLambda.insert({0, lambda});
        }


    }


    //!
    //! \brief Sets a lambda to perform some shutdown action.
    //!
    //! The shutdown action will be performed on its own thread when Shutdown is called.
    //! See Shutdown method for more discussion on this behavior.
    //! \param lambda Lambda to set
    //!
    void AddLambda_Shutdown(void* sender, const std::function<void()> &lambda){
        m_MutexShutdownLambda.lock();
        m_ShutdownLambda.insert({sender, lambda});
        m_MutexShutdownLambda.unlock();
    }


    //!
    //! \brief Shutdown the controller.
    //!
    //! Will call the lambda set by setLambda_Shutdown on a seperate thread.
    //! This allows a mutex to lock out resources that will probably be locked when other lambdas are called.
    //!   i.e. if the controller is removed from a list when onFinished is called, any mutex protecting that removal will probably already be locked from receiving.
    //!
    void Shutdown()
    {
        std::thread thread([this](){
            this->onShutdown();
        });
        thread.detach();
    }

private:

    void onShutdown(){

        m_MessageBehaviorsMutex.lock();
        m_MessageBehaviors.clear();
        m_MessageBehaviorsMutex.unlock();


        std::vector<std::function<void()>> lambdasToCall = {};
        for(auto it = m_ShutdownLambda.cbegin() ; it != m_ShutdownLambda.cend() ; ++it)
        {
            lambdasToCall.push_back(it->second);
        }

        for(auto it = lambdasToCall.cbegin() ; it != lambdasToCall.cend() ; ++it)
        {
            (*it)();
        }
    }

public:


    //!
    //! \brief Queue a transmission
    //!
    //! \template T Unique key type of entity that is transmitting this message
    //! \param key key of entity that is transmitting this message
    //! \param messageID ID of message that is expected to respond to this message
    //! \param transmitAction Action to take to do transmissions
    //!
    template <typename T>
    void QueueTransmission(const T &key, const int &messageID, const std::function<void()> &transmitAction)
    {
        auto lambda = [this](){
            onFinished(false);
        };
        TransmitQueueType::QueueTransmission(ObjectIntTuple<T>(key, messageID), transmitAction, lambda);
    }

    template <typename T>
    void QueueTransmission(const T &key, const std::vector<int> &messagesID, const std::function<void()> &transmitAction)
    {
        auto lambda = [this](){
            onFinished(false);
        };
        std::vector<ObjectIntTuple<T>> vec;
        for(auto it = messagesID.cbegin() ; it != messagesID.cend() ; ++it)
        {
            vec.push_back(ObjectIntTuple<T>(key, *it));
        }
        TransmitQueueType::QueueTransmission(vec, transmitAction, lambda);
    }

    template <typename T>
    void RemoveTransmission(const T &key, const int &messageID)
    {
        TransmitQueueType::RemoveTransmission(ObjectIntTuple<T>(key, messageID));
    }



    //!
    //! \brief Add logic that is response to a message already sent out by this object.
    //!
    //! This function will set up logic that will automatically check and delete queued messages that where created by QueueTransmission
    //!
    //! \template I Message id this logic pertains to
    //! \template KEY Datatype that can hold unique identifier that received message is associated with
    //! \template DECODE_TYPE type that received message is decoded to
    //! \template DECODE_FUNC function to decode received message to DECODE_TYPE
    //! \param func function to decode the message received.
    //! \param keyExtractor function to extract key from message
    //! \param action Action to partake with message.
    //!
    template <const int I, typename KEY, typename DECODE_TYPE, typename DECODE_FUNC>
    void AddResponseLogic(DECODE_FUNC func, const std::function<KEY(const DECODE_TYPE&, const COMPONENT_KEY &sender)> &keyExtractor, const std::function<void(const DECODE_TYPE &msg, KEY &key, const COMPONENT_KEY &sender)> &action)
    {

        auto newItem = std::make_tuple(MaceMessageIDEq<I>(),
                                       MaceProcessFSMState<DECODE_TYPE>(func, [this, action, keyExtractor](const DECODE_TYPE &msg, const COMPONENT_KEY &sender)
                                        {
                                            KEY key = keyExtractor(msg, sender);
                                            TransmitQueueType::RemoveTransmission(ObjectIntTuple<KEY>(key, I));

                                            action(msg, key, sender);
                                        }));

        std::lock_guard<std::mutex> lock(m_MessageBehaviorsMutex);
        m_MessageBehaviors.push_back(newItem);
    }


    //!
    //! \brief Add logic that is triggered by unsolicited message
    //!
    //! The logic added by this function does NOT delete any queued transmission.
    //!
    //! \template I Message id this logic pertains to
    //! \template DECODE_TYPE type that received message is decoded to
    //! \template DECODE_FUNC function to decode received message to DECODE_TYPE
    //! \param func function to decode the message received.
    //! \param action Action to partake with message.
    //!
    template <const int I, typename DECODE_TYPE, typename DECODE_FUNC>
    void AddTriggeredLogic(DECODE_FUNC func, const std::function<void(const DECODE_TYPE &msg, const COMPONENT_KEY &sender)> &action)
    {

        auto newItem = std::make_tuple(MaceMessageIDEq<I>(),
                                       MaceProcessFSMState<DECODE_TYPE>(func, [this, action](const DECODE_TYPE &msg, const COMPONENT_KEY &sender)
                                        {
                                            action(msg, sender);
                                        }));

        std::lock_guard<std::mutex> lock(m_MessageBehaviorsMutex);
        m_MessageBehaviors.push_back(newItem);
    }



    virtual bool ReceiveMessage(const MESSAGETYPE *message, const COMPONENT_KEY &sender)
    {
        std::lock_guard<std::mutex> lock(m_MessageBehaviorsMutex);
        bool usedMessage = false;
        for(auto it = m_MessageBehaviors.cbegin() ; it != m_MessageBehaviors.cend() ; ++it)
        {
            bool criteraEvaluation = std::get<0>(*it)(sender, message);
            if(criteraEvaluation)
            {
                std::get<1>(*it)(sender, message);
                usedMessage = true;
            }
        }

        return usedMessage;
    }



public:


    template <typename FUNC, typename TT>
    void EncodeMessage(FUNC func, TT requestItem, const COMPONENT_KEY &sender, const OptionalParameter<COMPONENT_KEY> &target = OptionalParameter<COMPONENT_KEY>())
    {

        std::tuple<int, int> keys = m_CB->GetSysIDAndCompIDFromComponentKey(sender);
        int sysid = std::get<0>(keys);
        int compid = std::get<1>(keys);

        MESSAGETYPE msg;
        func(sysid, compid, m_LinkChan, &msg, &requestItem);
        m_CB->TransmitMessage(msg, target);
    }

    std::vector<COMPONENT_KEY> GetAllTargets() const
    {
        return m_CB->GetAllTargets();
    }

    COMPONENT_KEY GetModuleFromMAVLINKVehicleID(int ID) const
    {
        return m_CB->GetModuleFromMAVLINKVehicleID(ID);
    }


    COMPONENT_KEY GetHostKey() const
    {
        return m_CB->GetHostKey();
    }


private:

    template <const int I>
    static std::function<bool(COMPONENT_KEY, const MESSAGETYPE*)> MaceMessageIDEq()
    {
        return [](COMPONENT_KEY sender, const MESSAGETYPE* message){
            UNUSED(sender);
            return message->msgid ==I;
        };
    }

    template <typename DECODE_TYPE, typename DECODE_FUNC>
    static std::function<void(COMPONENT_KEY, const MESSAGETYPE*)> MaceProcessFSMState(DECODE_FUNC decode_func, const std::function<void(const DECODE_TYPE &msg, const COMPONENT_KEY &sender)> &func)
    {
        return [decode_func, func](COMPONENT_KEY sender, const MESSAGETYPE* message)
        {
            DECODE_TYPE decodedMSG;
            decode_func(message, &decodedMSG);

            func(decodedMSG, sender);
        };
    }

    GenericController(const GenericController &rhs);
    GenericController& operator=(const GenericController&);

};



}


#endif // GENERIC_MACE_CONTROLLER_H
