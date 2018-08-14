#ifndef I_LINK_EVENTS_MACE_H
#define I_LINK_EVENTS_MACE_H

#include "commsmace_global.h"

#include <cstdlib>
#include <vector>
#include <string>

namespace CommsMACE
{

//!
//! \brief Describes a targetable resource, either on local machine or remote
//!
//! A resource is defined as a list of components and assosiated list of ID's
//! Examples:
//!   [MaceInstance(1)]
//!   [MaceInstance(1) Vehicle(1)]
//!   [MaceInstance(1) Vehicle(2)]
//!
class Resource
{
private:
    std::vector<std::string> m_componentNames;
    std::vector<int> m_IDs;
public:

    void Add(const std::string &name, const int ID)
    {
        m_componentNames.push_back(name);
        m_IDs.push_back(ID);
    }


    template<const char* ...N, typename ...I>
    void Set(I... ids)
    {
        static_assert(sizeof...(N) == sizeof...(ids), "Name and Resource values length must be the same");


        m_componentNames =  { N... };
        m_IDs = { ids... };
    }

    size_t Size() const
    {
        return m_IDs.size();
    }

    std::string NameAt(int i) const
    {
        return m_componentNames.at(i);
    }

    int IDAt(int i) const
    {
        return m_IDs.at(i);
    }

    bool operator ==(const Resource &rhs)
    {
        if(this->m_IDs.size() != rhs.m_componentNames.size()) return false;
        if(this->m_componentNames.size() != rhs.m_componentNames.size()) return false;

        for(std::size_t i = 0 ; i < m_IDs.size() ; i++)
        {
            if(m_IDs.at(i) != rhs.m_IDs.at(i)) return false;
        }

        for(std::size_t i = 0 ; i < m_componentNames.size() ; i++)
        {
            if(m_componentNames.at(i) != rhs.m_componentNames.at(i)) return false;
        }

        return true;
    }

};



class ILink;

class ILinkEvents
{
public:

    virtual void AddedExternalResource(ILink *link_ptr, const Resource &resource) = 0;

    virtual void RemovedExternalResource(ILink *link_ptr, const Resource &resource) const = 0;

    virtual void ReceiveData(ILink *link_ptr, const std::vector<uint8_t> &buffer) const = 0;

    virtual void CommunicationError(const ILink* link_ptr, const std::string &type, const std::string &msg) const = 0;

    virtual void CommunicationUpdate(const ILink *link_ptr, const std::string &name, const std::string &msg) const = 0;

    virtual void Connected(const ILink* link_ptr) const = 0;

    virtual void ConnectionRemoved(const ILink *link_ptr) const = 0;
};

} //END Comms

#endif // I_LINK_EVENTS_H
