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

class LinkMarshaler : public ILinkEvents
{
public:


    LinkMarshaler() :
        m_MavlinkChannelsUsedBitMask(1)
    {

    }

    void AddProtocol(const Protocols &type, const std::shared_ptr<IProtocol> protocol)
    {
        m_ProtocolObjects.insert({type, protocol});
    }

    void AddLink(std::shared_ptr<ILink> link)
    {
        m_Links.push_back(link);
        link->AddListener(this);
    }

    void SetProtocolForLink(std::shared_ptr<ILink> link, Protocols protocol)
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

            std::shared_ptr<MavlinkComms> mavlinkProtocol = std::static_pointer_cast<MavlinkComms>(protocolObj);
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
            case MavlinkComms::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (mavlinkStatus->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
                    mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                    break;
                }
                // Fallthrough to set version 2
            case MavlinkComms::MavlinkVersion::MavlinkVersionAlways2:
                mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                break;
            default:
            case MavlinkComms::MavlinkVersion::MavlinkVersionAlways1:
                mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                break;
            }
        }

    }


    uint8_t GetProtocolChannel(std::shared_ptr<ILink> link)
    {
        Protocols protocol = m_LinksProtocol.at(link.get());
        std::shared_ptr<IProtocol> protocolObj =  m_ProtocolObjects.at(protocol);

        return protocolObj->GetChannel(link.get());
    }


    template <typename T>
    void SendMessage(std::shared_ptr<ILink> link, const T& message)
    {
        switch(m_LinksProtocol.at(link.get()))
        {
        case Protocols::MAVLINK:
        {
            std::shared_ptr<MavlinkComms> protocol = std::static_pointer_cast<MavlinkComms>(m_ProtocolObjects.at(Protocols::MAVLINK));
            protocol->SendMessage(link.get(), message);
            break;
        }
        default:
            throw std::runtime_error("Attempting to send a message on an unknown protocol");
        }
    }




    virtual void ReceiveData(const void *sender, const std::vector<uint8_t> &buffer) const
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


    virtual void CommunicationError(const std::string &type, const std::string &msg) const
    {
        std::cout << "Error Type:" << type << "  Message:" << msg << std::endl;
    }

    virtual void CommunicationUpdate(const std::string &name, const std::string &msg) const
    {
        std::cout << "Update Type:" << name << "  Message:" << msg << std::endl;
    }

    virtual void Connected() const
    {
        std::cout << "Connected" << std::endl;
    }

    virtual void ConnectionRemoved(const void *sender) const
    {
        std::cout << "Connection removed" << std::endl;
    }



    std::unordered_map<Protocols, std::shared_ptr<IProtocol>, EnumClassHash> m_ProtocolObjects;
    std::vector<std::shared_ptr<ILink>> m_Links;

    std::unordered_map<ILink*, Protocols> m_LinksProtocol;

private:

    int m_MavlinkChannelsUsedBitMask;

    std::unordered_map<ILink*, uint8_t> m_MavlinkChannels;

};

}//END Comms

#endif // LINKMARSHALER_H
