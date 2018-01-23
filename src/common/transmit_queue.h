#ifndef TRANSMIT_QUEUE_H
#define TRANSMIT_QUEUE_H

#include "thread_manager.h"

#include <mutex>
#include <functional>
#include <unordered_map>

template <typename ...TRANSMITARGS>
class TransmitQueue : public Thread
{
private:

    static const int DEFAULT_RESPONSE_WAIT_IN_MS = 2000000000;

private:

    class TransmitTask
    {
    public:

        TransmitTask(std::function<void()> action) :
            numTries(0),
            transmitAction(action)
        {
        }

        int numTries;
        std::function<void()> transmitAction;
        std::chrono::time_point<std::chrono::system_clock> lastTransmit;
    };


private:

    std::mutex m_ActiveTransmitsMutex;
    std::unordered_map<int, TransmitTask> m_ActiveTransmits;


public:

    TransmitQueue()
    {
        start();
    }

    ~TransmitQueue()
    {
    }

    /*
    template <typename FUNC, typename TT>
    EncodeMessage(FUNC func, TT requestItem, TRANSMITARGS... args)
    {
        if(sender.IsSet() == false) {
            throw std::runtime_error("no sender given");
        }

        MESSAGETYPE msg;
        func(sender().ID, (int)sender().Class, m_LinkChan, &msg, &requestItem);
        m_TransmitFunction(msg, target);
    }

    void transmitMessage(const mace_message_t &msg, OptionalParameter<MaceCore::ModuleCharacteristic> target)
    {
        if(m_CB == NULL)
        {
            throw std::runtime_error("Callback not Set!");
        }
        m_CB->transmitMessage(msg, target);
    }
    */


    int QueueTransmission(std::function<void()> transmitAction)
    {
        m_ActiveTransmitsMutex.lock();
        int num;
        do
        {
            num = std::rand();
        }
        while(this->m_ActiveTransmits.find(num) != m_ActiveTransmits.cend());

        m_ActiveTransmits.insert({num, TransmitTask(transmitAction)});
        m_ActiveTransmitsMutex.unlock();

        return num;
    }


    void RemoveTransmissionFromQueue(int ID)
    {
        m_ActiveTransmitsMutex.lock();
        if(m_ActiveTransmits.find(ID) != m_ActiveTransmits.cend())
        {
            m_ActiveTransmits.erase(ID);
        }
        m_ActiveTransmitsMutex.unlock();
    }

    void run()
    {
        while(true)
        {
            if(isThreadActive() == false)
            {
                break;
            }

            if(m_ActiveTransmitsMutex.try_lock())
            {
                for(auto it = m_ActiveTransmits.begin() ; it != m_ActiveTransmits.end() ; ++it)
                {
                    std::chrono::time_point<std::chrono::system_clock> currTime = std::chrono::system_clock::now();
                    int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(currTime - it->second.lastTransmit).count();
                    if(it->second.numTries == 0 || elapsed_ms > DEFAULT_RESPONSE_WAIT_IN_MS)
                    {
                        it->second.lastTransmit = currTime;
                        it->second.numTries++;
                        it->second.transmitAction();
                    }
                }
                m_ActiveTransmitsMutex.unlock();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }


    }
};

template <typename Queue, typename ...T>
class TransmitQueueWithKeys;


template <typename Queue, typename Head, typename ...T>
class TransmitQueueWithKeys<Queue, Head, T...> : public TransmitQueueWithKeys<Queue, T...>
{
public:
    using TransmitQueueWithKeys<Queue, T...>::QueueTransmission;
    using TransmitQueueWithKeys<Queue, T...>::RemoveTransmission;

private:



    std::unordered_map<Head, int> m_ActiveTransmissions;

public:

    void QueueTransmission(const Head &key, const std::function<void()> &transmitAction)
    {
        int num = Queue::QueueTransmission(transmitAction);
        m_ActiveTransmissions.insert({key, num});
        //std::cout << "Transmission Queued: " << num << std::endl;
    }

    void RemoveTransmission(const Head &key)
    {
        if(m_ActiveTransmissions.find(key) != m_ActiveTransmissions.cend())
        {
            //std::cout << "Transmission Removed: " << m_ActiveTransmissions.at(key) << std::endl;
            Queue::RemoveTransmissionFromQueue(m_ActiveTransmissions.at(key));
            m_ActiveTransmissions.erase(key);
        }
    }

};

template <typename Queue>
class TransmitQueueWithKeys<Queue> : public Queue
{
public:

    void QueueTransmission()
    {

    }

    void RemoveTransmission()
    {

    }
};


#endif // TRANSMIT_QUEUE_H
