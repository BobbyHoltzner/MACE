#ifndef COMMS_EVENTS_H
#define COMMS_EVENTS_H

#include "mavlink.h"
#include <string>

namespace Comms
{

class CommsEvents
{
public:
    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg) const
    {

    }
};

}

#endif // COMMS_EVENTS_H
