#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>

#include "mace_core_global.h"

namespace MaceCore
{

template<typename T, typename I>
class ModuleBase
{
public:

    ModuleBase(const T &metaData) :
        m_MetaData(metaData)
    {
    }

    T getModuleMetaData()
    {
        return m_MetaData;
    }


    virtual std::string ModuleName() const = 0;


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


} //End MaceCore Namespace


#endif // MODULE_INTERFACE_H
