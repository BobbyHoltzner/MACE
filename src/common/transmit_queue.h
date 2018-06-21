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

    static const int DEFAULT_RESPONSE_WAIT_IN_MS = 2000000000;
    static const int DEFAULT_NUM_RETRIES = 3;

private:

    class TransmitTask
    {
    public:

        TransmitTask(std::function<void()> action, std::function<void()> failure) :
            numTries(0),
            transmitAction(action),
            failureAction(failure),
            active(true)
        {
        }

        int numTries;
        std::function<void()> transmitAction;
        std::function<void()> failureAction;
        std::chrono::time_point<std::chrono::system_clock> lastTransmit;
        bool active;
    };


private:

    std::mutex m_ActiveTransmitsMutex;
    std::unordered_map<int, TransmitTask> m_ActiveTransmits;

    int m_WaitForRetries;
    int m_NumRetries;


public:

    TransmitQueue(int waitForRetries = DEFAULT_RESPONSE_WAIT_IN_MS, int numRetries = DEFAULT_NUM_RETRIES)
    {
        m_WaitForRetries = waitForRetries;
        m_NumRetries = numRetries;

        if(waitForRetries > 1000000000)
        {
            printf("WARNING!!! Wait for retransmit is unreasonably high. Is is left on a debug value?\n");
        }
        start();
    }

    virtual ~TransmitQueue() = default;


    int QueueTransmission(std::function<void()> transmitAction, const std::function<void()> onFailure = [](){})
    {
        m_ActiveTransmitsMutex.lock();
        int num;
        do
        {
            num = std::rand();
        }
        while(this->m_ActiveTransmits.find(num) != m_ActiveTransmits.cend());

        m_ActiveTransmits.insert({num, TransmitTask(transmitAction, onFailure)});
        m_ActiveTransmitsMutex.unlock();

        printf("Added Transmision - Number active: %d\n", m_ActiveTransmits.size());

        return num;
    }


    void RemoveTransmissionFromQueue(int ID)
    {
        m_ActiveTransmitsMutex.lock();
        if(m_ActiveTransmits.find(ID) != m_ActiveTransmits.cend())
        {
            m_ActiveTransmits.at(ID).active = false;
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
                    if(it->second.active == false)
                    {
                        continue;
                    }
                    std::chrono::time_point<std::chrono::system_clock> currTime = std::chrono::system_clock::now();
                    int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(currTime - it->second.lastTransmit).count();
                    if(it->second.numTries == 0 || elapsed_ms > m_WaitForRetries)
                    {
                        //if we have exceeded number of retries then remove and fail out
                        if(it->second.numTries >= m_NumRetries)
                        {
                            it->second.failureAction();
                            it->second.active = false;
                            std::thread thread([this, it](){
                                RemoveTransmissionFromQueue(it->first);
                            });
                            thread.detach();
                            continue;
                        }

                        if(it->second.numTries >= 1)
                        {
                            // +1 to include original transmission
                            printf("Retransmitting for %d time\n", it->second.numTries+1);
                        }

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

    void QueueTransmission(const Head &key, const std::function<void()> &transmitAction, const std::function<void()> onFailure = [](){})
    {
        int num = m_Queue->QueueTransmission(transmitAction, onFailure);
        m_ActiveTransmissions.insert({key, num});
        m_ActiveTransmissionsToKeyMap.insert({num, {key}});
        //std::cout << "Transmission Queued: " << num << std::endl;
    }

    void QueueTransmission(const std::vector<Head> &keys, const std::function<void()> &transmitAction, const std::function<void()> onFailure = [](){})
    {
        int num = m_Queue->QueueTransmission(transmitAction, onFailure);
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

    virtual ~TransmitQueueWithKeys() = default;

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
