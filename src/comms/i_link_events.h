#ifndef I_LINK_EVENTS_H
#define I_LINK_EVENTS_H

#include <cstdlib>
#include <vector>
#include <string>

namespace Comms
{

class ILinkEvents
{
public:

    virtual void ReceiveData(const void* sender, const std::vector<u_int8_t> &buffer) const = 0;

    virtual void CommunicationError(const std::string &type, const std::string &msg) const = 0;

    virtual void CommunicationUpdate(const std::string &name, const std::string &msg) const = 0;

    virtual void Connected() const = 0;

    virtual void ConnectionRemoved(const void *sender) const = 0;
};

} //END Comms

#endif // I_LINK_EVENTS_H
