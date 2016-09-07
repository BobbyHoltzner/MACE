#include "link_marshaler.h"

namespace Comms
{


//////////////////////////////////////////////////////////////
/// Setup
//////////////////////////////////////////////////////////////

LinkMarshaler::LinkMarshaler() :
    m_MavlinkChannelsUsedBitMask(1)
{

}


//!
//! \brief Create a mavlink protocol to be used as transport layer of a link
//! \param config Configuration of mavlink
//! \param ptr Listener object to issue events onto
//!
void LinkMarshaler::AddProtocol(const MavlinkConfiguration &config, IMavlinkCommsEvents *ptr)
{
    if(m_ProtocolObjects.find(Protocols::MAVLINK) != m_ProtocolObjects.cend())
        throw std::runtime_error("Mavlink protocol has already been created");

    std::shared_ptr<MavlinkProtocol> protocol = std::make_shared<MavlinkProtocol>(config);

    protocol->AddListner(ptr);

    m_ProtocolObjects.insert({Protocols::MAVLINK, protocol});
}


//!
//! \brief Adds a serial link that can be used
//! \param name Name of link for use when referencing it later
//! \param config Configuration of serial link
//!
void LinkMarshaler::AddLink(const std::string &name, const SerialConfiguration &config)
{
    if(m_CreatedLinks.find(name) != m_CreatedLinks.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<SerialLink>(config);

    m_CreatedLinks.insert({name, link});
    link->AddListener(this);
}


//!
//! \brief Set the protocol which a link is to use
//! \param linkName Link name to set protocol of
//! \param protocol Protocol type that link is to use
//!
void LinkMarshaler::SetProtocolForLink(const std::string &linkName, Protocols protocol)
{
    if(m_CreatedLinks.find(linkName) == m_CreatedLinks.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinks.at(linkName);

    ILink* link_ptr = link.get();
    if(m_LinksProtocol.find(link_ptr) == m_LinksProtocol.cend())
        m_LinksProtocol.insert({link_ptr, protocol});
    else
        m_LinksProtocol[link_ptr] = protocol;

    std::shared_ptr<IProtocol> protocolObj = m_ProtocolObjects[protocol];
    protocolObj->ResetMetadataForLink(link_ptr);


    // if mavlink then set the channel
    if(protocol == Protocols::MAVLINK)
    {

        std::shared_ptr<MavlinkProtocol> mavlinkProtocol = std::static_pointer_cast<MavlinkProtocol>(protocolObj);
        bool channelSet = false;
        for (int i=0; i<32; i++) {
            if (!(m_MavlinkChannelsUsedBitMask & 1 << i)) {
                mavlink_reset_channel_status(i);
                protocolObj->SetChannel(link_ptr, i);
                m_MavlinkChannelsUsedBitMask |= 1 << i;
                channelSet = true;
                break;
            }
        }

        if (!channelSet) {
            throw std::runtime_error("Ran out of MAVLINK channels");
        }
    }

}


//!
//! \brief Connect to an already created link
//! \param linkName Name of link to connect to
//!
void LinkMarshaler::ConnectToLink(const std::string &linkName)
{
    if(m_CreatedLinks.find(linkName) == m_CreatedLinks.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinks.at(linkName);

    link->Connect();
}






//////////////////////////////////////////////////////////////
/// Query
//////////////////////////////////////////////////////////////


//!
//! \brief Get the channel being used by the given link to communicate
//! \param link Link to be used
//! \return Channel for that link
//!
uint8_t LinkMarshaler::GetProtocolChannel(const std::string &linkName) const
{
    if(m_CreatedLinks.find(linkName) == m_CreatedLinks.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinks.at(linkName);

    Protocols protocol = m_LinksProtocol.at(link.get());
    std::shared_ptr<IProtocol> protocolObj =  m_ProtocolObjects.at(protocol);

    return protocolObj->GetChannel(link.get());
}


//!
//! \brief Issue a message to a given link
//!
//! The type used in the shall be an underlaying type which the protocol understands
//! \param link Link to send message on
//! \param message Message to send
//!
template <typename T>
void LinkMarshaler::SendMessage(const std::string &linkName, const T& message)
{
    if(m_CreatedLinks.find(linkName) == m_CreatedLinks.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinks.at(linkName);

    switch(m_LinksProtocol.at(link.get()))
    {
    case Protocols::MAVLINK:
    {
        std::shared_ptr<MavlinkProtocol> protocol = std::static_pointer_cast<MavlinkProtocol>(m_ProtocolObjects.at(Protocols::MAVLINK));
        protocol->SendMessage(link.get(), message);
        break;
    }
    default:
        throw std::runtime_error("Attempting to send a message on an unknown protocol");
    }
}





//////////////////////////////////////////////////////////////
/// React to Link Events
//////////////////////////////////////////////////////////////

void LinkMarshaler::ReceiveData(const void *sender, const std::vector<uint8_t> &buffer) const
{
    ILink *link= (ILink*)sender;

    if(m_LinksProtocol.find(link) == m_LinksProtocol.cend())
        throw std::runtime_error("Protocol is not set for given link");
    Protocols protocol = m_LinksProtocol.at(link);

    if(m_ProtocolObjects.find(protocol) == m_ProtocolObjects.cend())
        throw std::runtime_error("Object has not be set for link's protocol");
    std::shared_ptr<IProtocol> protocolObj = m_ProtocolObjects.at(protocol);

    protocolObj->ReceiveData(link, buffer);
}


void LinkMarshaler::CommunicationError(const std::string &type, const std::string &msg) const
{
    std::cout << "Error Type:" << type << "  Message:" << msg << std::endl;
}

void LinkMarshaler::CommunicationUpdate(const std::string &name, const std::string &msg) const
{
    std::cout << "Update Type:" << name << "  Message:" << msg << std::endl;
}

void LinkMarshaler::Connected() const
{
    std::cout << "Connected" << std::endl;
}

void LinkMarshaler::ConnectionRemoved(const void *sender) const
{
    std::cout << "Connection removed" << std::endl;
}

template void LinkMarshaler::SendMessage<mavlink_message_t>(const std::string &, const mavlink_message_t&);

}//END Comms
