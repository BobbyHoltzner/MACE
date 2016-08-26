#ifndef MODULE_ABSTRACT_EVENT_LISTENERS_H
#define MODULE_ABSTRACT_EVENT_LISTENERS_H

#include "abstract_module_base.h"

namespace MaceCore
{

//!
//! \brief Build up of ModuleBase that adds event listening helper functions
//!
template<typename T, typename I>
class AbstractModule_EventListeners : public ModuleBase
{
public:

    AbstractModule_EventListeners()
    {
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



private:

    T m_MetaData;

    std::vector<I*> m_Listeners;


};

} // END MaceCore Namespace

#endif // MODULE_ABSTRACT_EVENT_LISTENERS_H
