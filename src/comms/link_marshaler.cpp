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


void LinkMarshaler::AddProtocol(const Protocols &type, const std::shared_ptr<IProtocol> protocol)
{
    m_ProtocolObjects.insert({type, protocol});
}

void LinkMarshaler::AddLink(std::shared_ptr<ILink> link)
{
    m_Links.push_back(link);
    link->AddListener(this);
}

void LinkMarshaler::SetProtocolForLink(std::shared_ptr<ILink> link, Protocols protocol)
{
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



        //configure version on channel
        mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(protocolObj->GetChannel(link_ptr));
        switch (mavlinkProtocol->m_version) {
        case MavlinkProtocol::MavlinkVersion::MavlinkVersion2IfVehicle2:
            if (mavlinkStatus->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
                mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                break;
            }
            // Fallthrough to set version 2
        case MavlinkProtocol::MavlinkVersion::MavlinkVersionAlways2:
            mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            break;
        default:
        case MavlinkProtocol::MavlinkVersion::MavlinkVersionAlways1:
            mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            break;
        }
    }

}






//////////////////////////////////////////////////////////////
/// Query
//////////////////////////////////////////////////////////////


//!
//! \brief Get the channel being used by the given link to communicate
//! \param link Link to be used
//! \return Channel for that link
//!
uint8_t LinkMarshaler::GetProtocolChannel(std::shared_ptr<ILink> link) const
{
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
void LinkMarshaler::SendMessage(std::shared_ptr<ILink> link, const T& message)
{
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

}//END Comms
