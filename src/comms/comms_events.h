#ifndef COMMS_EVENTS_H
#define COMMS_EVENTS_H

#include "mavlink.h"
#include <string>

namespace Comms
{

class CommsEvents
{
public:

    /////////////////////////////////////////////////////////
    /// Link Events
    /////////////////////////////////////////////////////////


    virtual void LinkCommunicationError(const std::string &linkName, const std::string &type, const std::string &msg) const
    {

    }

    virtual void LinkCommunicationUpdate(const std::string &linkName, const std::string &name, const std::string &msg) const
    {

    }

    virtual void LinkConnected(const std::string &linkName) const
    {

    }

    virtual void LinkConnectionRemoved(const std::string &linkName) const
    {

    }



    /////////////////////////////////////////////////////////
    /// Generic Protocol Events
    /////////////////////////////////////////////////////////


    virtual void ProtocolStatusMessage(const std::string &linkName, const std::string &title, const std::string &message) const
    {

    }


    virtual void ReceiveLossPercentChanged(const std::string &linkName, int uasId, float lossPercent) const
    {
    }

    virtual void ReceiveLossTotalChanged(const std::string &linkName, int uasId, int totalLoss) const
    {

    }


    /////////////////////////////////////////////////////////
    /// MAVLINK Protocol Events
    /////////////////////////////////////////////////////////


    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg)
    {

    }


    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
    {
    }


    //!
    //! \brief A new radio status packet received
    //! \param link
    //! \param rxerrors
    //! \param fixed
    //! \param rssi
    //! \param remrssi
    //! \param txbuf
    //! \param noise
    //! \param remnoise
    //!
    virtual void RadioStatusChanged(const std::string &linkName, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const
    {

    }

};

}

#endif // COMMS_EVENTS_H
