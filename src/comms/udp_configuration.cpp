#include "udp_configuration.h"

namespace Comms
{

UdpConfiguration::UdpConfiguration(const std::string& listenAddress, const int &listenPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
//    _senderAddress       = senderAddress;
//    _senderPortNumber    = senderPort;
}

UdpConfiguration::UdpConfiguration(const std::string& listenAddress, const int &listenPort, const std::string& senderAddress, const int &senderPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
    _senderAddress       = senderAddress;
    _senderPortNumber    = senderPort;
}

UdpConfiguration::UdpConfiguration(UdpConfiguration* copy)
{
    _listenAddress       = copy->listenAddress();
    _listenPortNumber    = copy->listenPortNumber();
    _senderAddress       = copy->senderAddress();
    _senderPortNumber    = copy->senderPortNumber();
}

void UdpConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    UdpConfiguration* ssource = dynamic_cast<UdpConfiguration*>(source);
    Q_ASSERT(ssource != NULL);
    _listenAddress               = ssource->listenAddress();
    _listenPortNumber            = ssource->listenPortNumber();
    _senderAddress               = ssource->senderAddress();
    _senderPortNumber            = ssource->senderPortNumber();
}

void UdpConfiguration::setListenAddress(std::string address)
{
    _listenAddress = address;
}

void UdpConfiguration::setListenPortNumber(int portNumber)
{
    _listenPortNumber = portNumber;
}

void UdpConfiguration::setSenderAddress(std::string address)
{
    _senderAddress = address;
}

void UdpConfiguration::setSenderPortNumber(int portNumber)
{
    _senderPortNumber = portNumber;
}


}
