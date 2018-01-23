#ifndef GENERIC_MACE_CONTROLLER_H
#define GENERIC_MACE_CONTROLLER_H

#include "generic_controller.h"
#include "common/fsm.h"
#include "common/pointer_collection.h"
#include "spdlog/spdlog.h"

#include <tuple>
#include <functional>

#include "common/transmit_queue.h"

namespace ExternalLink {

//typedef GenericController<mace_message_t, MaceCore::ModuleCharacteristic> GenericMACEController;


typedef TransmitQueue<mace_message_t, MaceCore::ModuleCharacteristic> MACETransmissionQueue;

/*
template <typename ...T>
class GenericMACEControllerTransmitTracking;

template <typename Head, typename ...T>
class GenericMACEControllerTransmitTracking<Head, T...> : public GenericMACEControllerTransmitTracking<T...>
{
public:
    using GenericMACEControllerTransmitTracking<T...>::QueueTransmission;
    using GenericMACEControllerTransmitTracking<T...>::RemoveTransmission;

private:



    std::unordered_map<Head, int> m_ActiveTransmissions;

public:
    void QueueTransmission(const Head &key, const std::function<void()> &transmitAction)
    {
        int num = GenericMACEController::QueueTransmission(transmitAction);
        m_ActiveTransmissions.insert({key, num});
        std::cout << "Transmission Queued: " << num << std::endl;
    }

    void RemoveTransmission(const Head &key)
    {
        if(m_ActiveTransmissions.find(key) != m_ActiveTransmissions.cend())
        {
            std::cout << "Transmission Removed: " << m_ActiveTransmissions.at(key) << std::endl;
            GenericMACEController::RemoveTransmissionFromQueue(m_ActiveTransmissions.at(key));
            m_ActiveTransmissions.erase(key);
        }
    }

};

template <>
class GenericMACEControllerTransmitTracking<> : public GenericMACEController
{
public:

    void QueueTransmission()
    {

    }

    void RemoveTransmission()
    {

    }
};
*/


template <typename T>
class KeyWithInt
{
public:

    T m_obj;
    int m_int;

public:
    KeyWithInt(const T &obj, const int &integer) :
        m_obj(obj),
        m_int(integer)
    {

    }

    bool operator== (const KeyWithInt &rhs) const
    {
        if(this->m_obj != rhs.m_obj)
            return false;
        if(this->m_int != rhs.m_int) {
            return false;
        }
        return true;
    }
};


template< typename Key, typename Type>
class DataItem
{
public:

    typedef std::vector<std::tuple<Key, Type>> FetchKeyReturn;
    typedef std::vector<std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<Key, Type>>>> FetchModuleReturn;

private:

    OptionalParameter<std::function<void(const Key &, const Type &)>> m_lambda_DataRecieved;
    OptionalParameter<std::function<FetchKeyReturn(const OptionalParameter<Key> &)>> m_lambda_FetchDataFromKey;
    OptionalParameter<std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)>> m_lambda_FetchAll;
public:


    void setLambda_DataReceived(const std::function<void(const Key &, const Type &)> &lambda){
        m_lambda_DataRecieved = lambda;
    }

    void onDataReceived(const Key &key, const Type &data){
        if(m_lambda_DataRecieved.IsSet() == false) {
            throw std::runtime_error("Data Received Lambda not set!");
        }

        m_lambda_DataRecieved()(key, data);
    }




    void setLambda_FetchDataFromKey(const std::function<FetchKeyReturn(const OptionalParameter<Key> &)> &lambda){
        m_lambda_FetchDataFromKey = lambda;
    }

    void FetchDataFromKey(const OptionalParameter<Key> &key, FetchKeyReturn &data){
        if(m_lambda_FetchDataFromKey.IsSet() == false) {
            throw std::runtime_error("FetchKey Lambda not set!");
        }

        data = m_lambda_FetchDataFromKey()(key);
    }




    void setLambda_FetchAll(const std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)> &lambda){
        m_lambda_FetchAll = lambda;
    }

    void FetchFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &target, FetchModuleReturn &data) {
        if(m_lambda_FetchAll.IsSet() == false) {
            throw std::runtime_error("FetchFromModule Lambda not set!");
        }

        data = m_lambda_FetchAll()(target);
        return;
    }
};



template <typename ...T>
class A;

template<typename HEAD, typename ...T>
class A<HEAD, T...> : public A<T...>, public HEAD
{

};

template<>
class A<>
{

};


class MACEControllerInterface
{
public:
    virtual void TransmitMessage(const mace_message_t &, const OptionalParameter<MaceCore::ModuleCharacteristic> &) const = 0;
};


//template<typename TransmitQueueType, typename ...DataItems>
//class GenericMACEController : public TransmitQueueType, public A<DataItems...>, public GenericController
template<typename TransmitQueueType, typename ...DataItems>
class GenericMACEController : public GenericController, public TransmitQueueType, public A<DataItems...>
{
public:

private:



    int m_LinkChan;

    const MACEControllerInterface* m_CB;

protected:

    std::shared_ptr<spdlog::logger> mLog;

    std::vector<std::tuple<
        std::function<bool(MaceCore::ModuleCharacteristic, const mace_message_t*)>,
        std::function<void(MaceCore::ModuleCharacteristic, const mace_message_t*)>
    >> m_MessageBehaviors;

public:

    AddMessageLogic(const std::function<bool(MaceCore::ModuleCharacteristic, const mace_message_t*)> &critera, const std::function<void(MaceCore::ModuleCharacteristic, const mace_message_t*)> &action)
    {
        auto newItem = std::make_tuple(critera, action);
        m_MessageBehaviors.push_back(newItem);
    }

    template <const int I, typename KEY, typename DECODE_TYPE, typename DECODE_FUNC>
    AddMaceMessagLogic(DECODE_FUNC func, const std::function<KEY(const DECODE_TYPE&, const MaceCore::ModuleCharacteristic &sender)> &keyExtractor, const std::function<void(const DECODE_TYPE &msg, KEY &key, const MaceCore::ModuleCharacteristic &sender)> &action)
    {

        auto newItem = std::make_tuple(MaceMessageIDEq<I>(),
                                       MaceProcessFSMState<DECODE_TYPE>(func, [this, action, keyExtractor](const DECODE_TYPE &msg, const MaceCore::ModuleCharacteristic &sender)
                                        {
                                            KEY key = keyExtractor(msg, sender);
                                            TransmitQueueType::RemoveTransmission(KeyWithInt<KEY>(key, I));

                                            action(msg, key, sender);
                                        }));

        m_MessageBehaviors.push_back(newItem);
    }


    template <const int I, typename DECODE_TYPE, typename DECODE_FUNC>
    AddMaceMessagLogic(DECODE_FUNC func, const std::function<void(const DECODE_TYPE &msg, const MaceCore::ModuleCharacteristic &sender)> &action)
    {

        auto newItem = std::make_tuple(MaceMessageIDEq<I>(),
                                       MaceProcessFSMState<DECODE_TYPE>(func, [this, action](const DECODE_TYPE &msg, const MaceCore::ModuleCharacteristic &sender)
                                        {
                                            action(msg, sender);
                                        }));

        m_MessageBehaviors.push_back(newItem);
    }

    GenericMACEController(const MACEControllerInterface* cb, int linkChan) :
        m_LinkChan(linkChan),
        m_CB(cb)
    {
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


    template <typename T>
    void QueueTransmission(const T &key, const int &messageID, const std::function<void()> &transmitAction)
    {
        TransmitQueueType::QueueTransmission(KeyWithInt<T>(key, messageID), transmitAction);
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

        return false;
    }



protected:


    template <typename FUNC, typename TT>
    void EncodeMessage(FUNC func, TT requestItem, OptionalParameter<MaceCore::ModuleCharacteristic> sender, const MaceCore::ModuleCharacteristic &target)
    {
        if(sender.IsSet() == false) {
            throw std::runtime_error("no sender given");
        }

        mace_message_t msg;
        func(sender().ID, (int)sender().Class, m_LinkChan, &msg, &requestItem);
        m_CB->TransmitMessage(msg, target);
    }

private:
    GenericMACEController(const GenericMACEController &rhs);
    GenericMACEController& operator=(const GenericMACEController&);

};



}



namespace std
{

template <typename T>
struct hash<ExternalLink::KeyWithInt<T>>
{
    std::size_t operator()(const ExternalLink::KeyWithInt<T>& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;



      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

        std::size_t const h1 ( std::hash<T>{}(k.m_obj) );
        std::size_t const h2 ( std::hash<int>{}((int)k.m_int) );
        return h1 ^ (h2 << 1);
    }
};
}

#endif // GENERIC_MACE_CONTROLLER_H
