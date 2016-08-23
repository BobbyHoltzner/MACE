#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>

#include "resource_number_generator.h"

static ResourceNumberGenerator G_RNGenerator;

template<typename T, typename I>
class ModuleBase
{
public:

    ModuleBase(const T &metaData) :
        m_MetaData(metaData)
    {
        m_RN = G_RNGenerator.GenerateNumber(ModuleName());
    }

    T getModuleMetaData()
    {
        return m_MetaData;
    }


    virtual std::string ModuleName() const = 0;

    std::string ResourceName() const
    {
        return m_RN;
    }

    void addListener(const I &listener)
    {
        m_Listeners.push_back(listener);
    }

private:

    T m_MetaData;

    std::string m_RN;

    std::vector<I> m_Listeners;
};


#endif // MODULE_INTERFACE_H
