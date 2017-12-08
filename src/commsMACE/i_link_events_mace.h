#ifndef I_LINK_EVENTS_MACE_H
#define I_LINK_EVENTS_MACE_H

#include "commsmace_global.h"

#include <cstdlib>
#include <vector>
#include <string>

namespace CommsMACE
{

class ILink;

class ILinkEvents
{
public:

    virtual void AddedExternalVehicle(ILink *link_ptr, int vehicleID) const = 0;

    virtual void RemovedExternalVehicle(ILink *link_ptr, int vehicleID) const = 0;

    virtual void ReceiveData(ILink *link_ptr, const std::vector<uint8_t> &buffer) const = 0;

    virtual void CommunicationError(const ILink* link_ptr, const std::string &type, const std::string &msg) const = 0;

    virtual void CommunicationUpdate(const ILink *link_ptr, const std::string &name, const std::string &msg) const = 0;

    virtual void Connected(const ILink* link_ptr) const = 0;

    virtual void ConnectionRemoved(const ILink *link_ptr) const = 0;
};

} //END Comms

#endif // I_LINK_EVENTS_H
