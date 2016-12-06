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

    UdpConfiguration(const std::string& address, const int& port);
    UdpConfiguration(UdpConfiguration* copy);

    const int portNumber() const { return _portNumber; }
    const std::string address() const { return _address; }

    void setPortNumber(int portNumber);
    void setAddress(std::string address);

    /// From LinkConfiguration
    void        copyFrom        (LinkConfiguration* source);
    //void        loadSettings    (Settings& settings, const QString& root);
    //void        saveSettings    (Settings& settings, const QString& root);
    //void        updateSettings  ();
    //QString     settingsURL     () { return "SerialSettings.qml"; }

private:
    int _portNumber;
    std::string _address;
};

}

#endif // UDPCONFIGURATION_H
