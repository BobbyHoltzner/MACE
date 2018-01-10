#ifndef GENERIC_CONTROLLER_H
#define GENERIC_CONTROLLER_H

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "commsMACEHelper/comms_mace_helper.h"

#include <functional>

namespace ExternalLink {



template <typename ...TRANSMITARGS>
class GenericController : public Thread
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

    std::function<void(TRANSMITARGS...)> m_TransmitFunction;


public:

    //!
    //! \brief Receive a message for the controller
    //! \param message Message to receive
    //! \return True if action was taken, false if this module didnt' care about message
    //!
    virtual bool ReceiveMessage(const mace_message_t* message) = 0;


    GenericController(const OptionalParameter<const std::function<void(TRANSMITARGS...)>> &func = OptionalParameter<const std::function<void(TRANSMITARGS...)>>())
    {
        if(func.IsSet())
        {
            SetTransmit(func.Value());
        }
        start();
    }

    ~GenericController()
    {
    }


    void SetTransmit(const std::function<void(TRANSMITARGS...)> &func)
    {
        m_TransmitFunction = func;
    }


    void TransmitMessage(TRANSMITARGS ...args)
    {
        m_TransmitFunction(args...);
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

}

#endif // GENERIC_CONTROLLER_H
