#include "udp_configuration.h"

namespace Comms
{

UdpConfiguration::UdpConfiguration(const std::string& address, const int &port)
{
    _address       = address;
    _portNumber    = port;
}

UdpConfiguration::UdpConfiguration(UdpConfiguration* copy)
{
    _address       = copy->address();
    _portNumber    = copy->portNumber();
}

void UdpConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    UdpConfiguration* ssource = dynamic_cast<UdpConfiguration*>(source);
    Q_ASSERT(ssource != NULL);
    _address               = ssource->address();
    _portNumber            = ssource->portNumber();
}

void UdpConfiguration::setAddress(std::string address)
{
    _address = address;
}

void UdpConfiguration::setPortNumber(int portNumber)
{
    _portNumber = portNumber;
}


}
