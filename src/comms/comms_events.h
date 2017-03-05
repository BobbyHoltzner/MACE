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
        Q_UNUSED(linkName);
        Q_UNUSED(type);
        Q_UNUSED(msg);
    }

    virtual void LinkCommunicationUpdate(const std::string &linkName, const std::string &name, const std::string &msg) const
    {
        Q_UNUSED(linkName);
        Q_UNUSED(name);
        Q_UNUSED(msg);
    }

    virtual void LinkConnected(const std::string &linkName) const
    {
        Q_UNUSED(linkName);
    }

    virtual void LinkConnectionRemoved(const std::string &linkName) const
    {
        Q_UNUSED(linkName);
    }



    /////////////////////////////////////////////////////////
    /// Generic Protocol Events
    /////////////////////////////////////////////////////////


    virtual void ProtocolStatusMessage(const std::string &linkName, const std::string &title, const std::string &message) const
    {
        Q_UNUSED(linkName);
        Q_UNUSED(title);
        Q_UNUSED(message);
    }


    virtual void ReceiveLossPercentChanged(const std::string &linkName, int uasId, float lossPercent) const
    {
        Q_UNUSED(linkName);
        Q_UNUSED(uasId);
        Q_UNUSED(lossPercent);
    }

    virtual void ReceiveLossTotalChanged(const std::string &linkName, int uasId, int totalLoss) const
    {
        Q_UNUSED(linkName);
        Q_UNUSED(uasId);
        Q_UNUSED(totalLoss);
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
        Q_UNUSED(linkName);
        Q_UNUSED(msg);
    }


    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
    {
        Q_UNUSED(linkName);
        Q_UNUSED(vehicleId);
        Q_UNUSED(vehicleMavlinkVersion);
        Q_UNUSED(vehicleFirmwareType);
        Q_UNUSED(vehicleType);
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
        Q_UNUSED(linkName);
        Q_UNUSED(rxerrors);
        Q_UNUSED(fixed);
        Q_UNUSED(rssi);
        Q_UNUSED(remrssi);
        Q_UNUSED(txbuf);
        Q_UNUSED(noise);
        Q_UNUSED(remnoise);
    }

};

}

#endif // COMMS_EVENTS_H
