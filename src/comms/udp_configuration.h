#ifndef UDPCONFIGURATION_H
#define UDPCONFIGURATION_H

#include <QUdpSocket>
#include <string>

#include "link_configuration.h"

#include "comms_global.h"

namespace Comms
{

class COMMSSHARED_EXPORT UdpConfiguration : public LinkConfiguration
{
public:

    UdpConfiguration(const std::__cxx11::string &listenAddress, const int &listenPort);
    UdpConfiguration(const std::string& listenAddress, const int &listenPort, const std::string& senderAddress, const int &senderPort);
    UdpConfiguration(UdpConfiguration* copy);

    const int listenPortNumber() const { return _listenPortNumber; }
    const std::string listenAddress() const { return _listenAddress; }
    const int senderPortNumber() const { return _senderPortNumber; }
    const std::string senderAddress() const { return _senderAddress; }

    void setListenPortNumber(int portNumber);
    void setListenAddress(std::string address);
    void setSenderPortNumber(int portNumber);
    void setSenderAddress(std::string address);

    /// From LinkConfiguration
    void        copyFrom        (LinkConfiguration* source);
    //void        loadSettings    (Settings& settings, const QString& root);
    //void        saveSettings    (Settings& settings, const QString& root);
    //void        updateSettings  ();
    //QString     settingsURL     () { return "SerialSettings.qml"; }

private:
    int _listenPortNumber;
    std::string _listenAddress;
    int _senderPortNumber;
    std::string _senderAddress;
};

}

#endif // UDPCONFIGURATION_H
