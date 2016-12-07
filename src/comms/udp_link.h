#ifndef UDPLINK_H
#define UDPLINK_H

#include "comms_global.h"

#include <mutex>
#include <iostream>
#include <thread>

#include <QThread>
#include <QUdpSocket>

#include "udp_configuration.h"

#include "i_link.h"


namespace Comms
{



class COMMSSHARED_EXPORT UdpLink : public ILink
{
public:

    UdpLink(const UdpConfiguration &config);

    ~UdpLink();

    virtual void RequestReset();
    virtual uint64_t getConnectionSpeed() const;

    virtual void WriteBytes(const char *bytes, int length) const;

    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;

    std::string getAddress() const;
    int getPortNumber() const;



    void _emitLinkError(const std::string& errorMsg) const;

    LinkConfiguration getLinkConfiguration();



    virtual bool Connect(void);

    virtual void Disconnect(void);

private:


    /// Performs the actual hardware port connection.
    ///     @param[out] error if failed
    ///     @param[out] error string if failed
    /// @return success/fail
    bool _hardwareConnect(QAbstractSocket::SocketError& error, QString& errorString);

private:


    void processPendingDatagrams(void);

    void linkError(QUdpSocket::SocketError error);

private:

    QUdpSocket* m_socket;
    quint64 m_bytesRead;
    int     m_timeout;
    std::thread *m_CommsThread;
    QThread *m_ListenThread;
    std::mutex  m_dataMutex;       // Mutex for reading data from _port
    std::mutex  m_writeMutex;      // Mutex for accessing the _transmitBuffer.


    volatile bool        m_stopp;
    volatile bool        m_reqReset;
    std::mutex           m_stoppMutex;      // Mutex for accessing _stopp
    UdpConfiguration _config;
};

}

#endif // UDPLINK_H
