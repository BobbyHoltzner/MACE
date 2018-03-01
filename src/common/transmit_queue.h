#ifndef TRANSMIT_QUEUE_H
#define TRANSMIT_QUEUE_H

#include "thread_manager.h"

#include <mutex>
#include <functional>
#include <unordered_map>
#include <vector>

template <typename ...TRANSMITARGS>
class TransmitQueue : public Thread
{
private:

    static const int DEFAULT_RESPONSE_WAIT_IN_MS = 2000;

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

        printf("Added Transmision - Number active: %d\n", m_ActiveTransmits.size());

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

        printf("Removed Transmision - Number active: %d\n", m_ActiveTransmits.size());
    }

    void run()
    {
        while(true)
        {
            if(isThreadActive() == false)
            {
                printf("!!!!!!!!!!!!SHUTTING DOWN TRANSMIT QUEUE!!!!!!\n");
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
    using TransmitQueueWithKeys<Queue, T...>::m_Queue;

private:



    std::unordered_map<Head, int> m_ActiveTransmissions;

    std::unordered_map<int, std::vector<Head>> m_ActiveTransmissionsToKeyMap;

public:

    void QueueTransmission(const Head &key, const std::function<void()> &transmitAction)
    {
        int num = m_Queue->QueueTransmission(transmitAction);
        m_ActiveTransmissions.insert({key, num});
        m_ActiveTransmissionsToKeyMap.insert({num, {key}});
        //std::cout << "Transmission Queued: " << num << std::endl;
    }

    void QueueTransmission(const std::vector<Head> &keys, const std::function<void()> &transmitAction)
    {
        int num = m_Queue->QueueTransmission(transmitAction);
        for(auto it = keys.cbegin() ; it != keys.cend() ; ++it)
        {
            m_ActiveTransmissions.insert({*it, num});
        }
        m_ActiveTransmissionsToKeyMap.insert({num, keys});
        //std::cout << "Transmission Queued: " << num << std::endl;
    }

    void RemoveTransmission(const Head &key)
    {
        if(m_ActiveTransmissions.find(key) != m_ActiveTransmissions.cend())
        {
            int num = m_ActiveTransmissions.at(key);
            //std::cout << "Transmission Removed: " << m_ActiveTransmissions.at(key) << std::endl;
            m_Queue->RemoveTransmissionFromQueue(num);

            for(auto it = m_ActiveTransmissionsToKeyMap.at(num).cbegin() ; it != m_ActiveTransmissionsToKeyMap.at(num).cend() ; ++it)
            {
                m_ActiveTransmissions.erase(*it);
            }
            m_ActiveTransmissionsToKeyMap.erase(num);
        }
    }

};

template <typename Queue>
class TransmitQueueWithKeys<Queue>
{
protected:

    Queue* m_Queue;

public:

    void SetQueue(Queue* queue) {
        m_Queue = queue;
    }

    void QueueTransmission()
    {

    }

    void RemoveTransmission()
    {

    }
};


#endif // TRANSMIT_QUEUE_H
