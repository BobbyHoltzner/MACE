#ifndef MODULE_ABSTRACT_EVENT_LISTENERS_H
#define MODULE_ABSTRACT_EVENT_LISTENERS_H

#include "abstract_module_base.h"

#include "command_marshler.h"

namespace MaceCore
{

//!
//! \brief Build up of ModuleBase that adds event listening helper functions
//!
template<typename T, typename I, typename CT>
class AbstractModule_EventListeners : public ModuleBase
{
public:

    AbstractModule_EventListeners() :
        m_LoopSleepTime(std::chrono::milliseconds(10)),
        m_DefaultMarshalCommandOnEventLoop(true)
    {

    }

    void SetEventLoopRate(const std::chrono::milliseconds &duration)
    {
        m_LoopSleepTime = duration;
    }

    virtual void start()
    {
        while(true)
        {
            m_EventLooper.ExecuteQueuedCommands();

            std::this_thread::sleep_for (m_LoopSleepTime);
        }
    }

    void setModuleMetaData(const T &data)
    {
        m_MetaData = data;
    }

    T getModuleMetaData()
    {
        return m_MetaData;
    }


    void addListener(const I *listener)
    {
        m_Listeners.push_back(listener);
    }

    void NotifyListeners(const std::function<void(I*)> &func)
    {
        for(auto it = m_Listeners.cbegin() ; it != m_Listeners.cend() ; ++it)
        {
            func(*it);
        }
    }


    //!
    //! \brief Set time between calls of a specific command
    //! \param command Command to set interval of
    //! \param interval Interval in milliseconds between call
    //!
    void SetCallInterval(CT command, std::chrono::milliseconds interval)
    {
        m_EventLooper.SetCallInterval(command, interval);
    }


public:

    //!
    //! \brief Define default behavior as to commands are to invoked on event loop
    //!
    //! If set to true, the execution of a command will occur on the module's thread at a rate that doesn't exceed the module's/command's rates.
    //! Code can directly be placed into module's methods and their code will be executed on module's thread.
    //!
    //! If set to false, the executation of a command will occur on the thread that decided the command must be called.
    //! To execute code on modules thread in this case, the start() method will need to be overidden and a thread masharling scheme will need to implimented.
    //! \param value True if are to marshal onto event loop
    //!
    void InvokeCommandOnModuleEventLoop(const bool &value)
    {
        m_DefaultMarshalCommandOnEventLoop = value;
    }


    //!
    //! \brief Define for a specific command if it should be invoked on event loop
    //!
    //! If set to true, the execution of a command will occur on the module's thread at a rate that doesn't exceed the module's/command's rates.
    //! Code can directly be placed into module's methods and their code will be executed on module's thread.
    //!
    //! If set to false, the executation of a command will occur on the thread that decided the command must be called.
    //! To execute code on modules thread in this case, the start() method will need to be overidden and a thread masharling scheme will need to implimented.
    //!
    //! If come commands are to be invoked on module's event loop and other not, the start() method will need to be overidden with a basic envent loop.
    //! In that event loop, any queued commands who are configured to be executed on event loop can be ran with "m_EventLooper.ExecuteQueuedCommands();" command.
    //!
    //! \param command Command to modify behavior of.
    //! \param value True if are to marshal onto event loop
    //!
    void InvokeCommandOnModuleEventLoop(const CT command, const bool &value)
    {
        m_DefaultMarshalCommandOnEventLoop[command] = value;
    }

protected:


    void MarshalCommand(CT enumValue)
    {
        bool IssueOnThread;
        if(m_MarshalCommandsOnEventLoop.find(enumValue) == m_MarshalCommandsOnEventLoop.cend())
            IssueOnThread = m_DefaultMarshalCommandOnEventLoop;
        else
            IssueOnThread = m_MarshalCommandsOnEventLoop.at(enumValue);

        if(IssueOnThread == true)
            this->m_EventLooper.QueueCommand(enumValue);
        else
            this->m_EventLooper.ImmediatlyCallCommand(enumValue);
    }


    template<typename P1T>
    void MarshalCommand(CT enumValue, const P1T &value)
    {
        bool IssueOnModuleEventLoop;
        if(m_MarshalCommandsOnEventLoop.find(enumValue) == m_MarshalCommandsOnEventLoop.cend())
            IssueOnModuleEventLoop = m_DefaultMarshalCommandOnEventLoop;
        else
            IssueOnModuleEventLoop = m_MarshalCommandsOnEventLoop.at(enumValue);

        if(IssueOnModuleEventLoop == true)
            this->m_EventLooper.QueueCommand(enumValue, value);
        else
            this->m_EventLooper.ImmediatlyCallCommand(enumValue, value);
    }



private:

    T m_MetaData;

    std::vector<I*> m_Listeners;

    std::chrono::milliseconds m_LoopSleepTime;


    bool m_DefaultMarshalCommandOnEventLoop;


    std::unordered_map<CT, bool, EnumClassHash> m_MarshalCommandsOnEventLoop;

protected:


    CommandMarshler<CT> m_EventLooper;


};

} // END MaceCore Namespace

#endif // MODULE_ABSTRACT_EVENT_LISTENERS_H
