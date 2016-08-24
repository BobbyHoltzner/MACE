#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>

#include "mace_core_global.h"

#include "mace_data.h"

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


    //!
    //! \brief function that is to kick off the event loop of the module
    //!
    virtual void start() = 0;


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


    void setDataObject(const std::shared_ptr<MaceData> &data)
    {
        m_Data = data;
    }

    std::shared_ptr<const MaceData> getDataObject() const
    {
        return m_Data;
    }


private:

    T m_MetaData;

    std::vector<I*> m_Listeners;

    std::shared_ptr<const MaceData> m_Data;
};


} //End MaceCore Namespace


#endif // MODULE_INTERFACE_H
