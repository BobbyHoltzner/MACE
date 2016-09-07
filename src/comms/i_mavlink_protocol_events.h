#ifndef I_MAVLINK_COMMS_EVENTS_H
#define I_MAVLINK_COMMS_EVENTS_H

#include <string>
#include <memory>

#include "mavlink.h"

#include "i_link.h"


namespace Comms
{

//!
//! \brief Interface that it to be implimented by users of MavlinkComms to listen for any events it fired
//!
class IMavlinkCommsEvents
{
public:

    //!
    //! \brief A message about protocol has been generated
    //! \param title
    //! \param message
    //!
    virtual void ProtocolStatusMessage(const std::string &title, const std::string &message) const = 0;

    //!
    //! \brief A Message has been received over Mavlink protocol
    //! \param message Message that has been received
    //!
    virtual void MessageReceived(const mavlink_message_t &message) const = 0;

    //!
    //! \brief Heartbeat of vehicle received
    //! \param link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const = 0;

    virtual void ReceiveLossPercentChanged(int uasId, float lossPercent) const = 0;
    virtual void ReceiveLossTotalChanged(int uasId, int totalLoss) const = 0;


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
    virtual void RadioStatusChanged(const std::string &linkName, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const = 0;
};


} //END MAVLINKComms

#endif // I_MAVLINK_COMMS_EVENTS_H
