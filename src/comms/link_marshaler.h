#ifndef LINKMARSHALER_H
#define LINKMARSHALER_H

#include "common/common.h"

#include "comms_global.h"

#include <unordered_map>

#include "i_link.h"
#include "serial_link.h"
#include "mavlink_protocol.h"

#include "i_link_events.h"

namespace Comms
{

enum class LinkTypes
{
    SERIAL
};

enum class Protocols
{
    MAVLINK
};


//!
//! \brief The link marshaler is to coordinate a collection of links and the protocols which those links are using.
//!
//! There should only exists one protocol object for each type.
//! But there may exists multiple links of the same type and may be using the same protocol.
//!
//! When multiple links are using the same protol, they utilize protocol channels to differiciante themselves.
//!
class LinkMarshaler : private ILinkEvents
{
public:

    //////////////////////////////////////////////////////////////
    /// Setup
    //////////////////////////////////////////////////////////////

    LinkMarshaler();

    //!
    //! \brief Create a mavlink protocol to be used as transport layer of a link
    //! \param config Configuration of mavlink
    //! \param ptr Listener object to issue events onto
    //!
    void AddProtocol(const MavlinkConfiguration &config, IMavlinkCommsEvents * ptr);


    //!
    //! \brief Adds a serial link that can be used
    //! \param name Name of link for use when referencing it later
    //! \param config Configuration of serial link
    //!
    void AddLink(const std::string &name, const SerialConfiguration &config);


    //!
    //! \brief Set the protocol which a link is to use
    //! \param linkName Link name to set protocol of
    //! \param protocol Protocol type that link is to use
    //!
    void SetProtocolForLink(const std::string &linkName, Protocols protocol);


    //!
    //! \brief Connect to an already created link
    //! \param linkName Name of link to connect to
    //!
    void ConnectToLink(const std::string &linkName);


    //////////////////////////////////////////////////////////////
    /// Query
    //////////////////////////////////////////////////////////////

    //!
    //! \brief Get the channel being used by the given link to communicate
    //! \param link Link to be used
    //! \return Channel for that link
    //!
    uint8_t GetProtocolChannel(const std::string &linkName) const;


    //!
    //! \brief Issue a message to a given link
    //!
    //! The type used in the shall be an underlaying type which the protocol understands
    //! \param link Link to send message on
    //! \param message Message to send
    //!
    template <typename T>
    void SendMessage(const std::string &linkName, const T& message);



private:

    //////////////////////////////////////////////////////////////
    /// React to Link Events
    //////////////////////////////////////////////////////////////

    virtual void ReceiveData(const void *sender, const std::vector<uint8_t> &buffer) const;

    virtual void CommunicationError(const std::string &type, const std::string &msg) const;

    virtual void CommunicationUpdate(const std::string &name, const std::string &msg) const;

    virtual void Connected() const;

    virtual void ConnectionRemoved(const void *sender) const;

private:

    std::unordered_map<Protocols, std::shared_ptr<IProtocol>, EnumClassHash> m_ProtocolObjects;

    std::unordered_map<std::string, std::shared_ptr<ILink>> m_CreatedLinks;

    std::unordered_map<ILink*, Protocols> m_LinksProtocol;

private:

    int m_MavlinkChannelsUsedBitMask;

    std::unordered_map<ILink*, uint8_t> m_MavlinkChannels;

};

}//END Comms

#endif // LINKMARSHALER_H
