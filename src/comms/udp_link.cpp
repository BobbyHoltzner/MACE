#include "udp_link.h"
#include <QCoreApplication>

namespace Comms
{


//!
//! \brief This class defines a thread such that a QObject can run in peace.
//!
class ReceiverThread : public QThread
{
public:
    ReceiverThread(const std::function<void(void)> &func):
        m_func(func)
    {
        if(QCoreApplication::instance() == NULL)
        {
            int argc = 0;
            char * argv[] = {(char *)"sharedlib.app"};
            pApp = new QCoreApplication(argc, argv);
        }
    }

    virtual void run()
    {
        while(true)
        {
            QCoreApplication::processEvents();
            m_func();
        }
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
};


UdpLink::UdpLink(const UdpConfiguration &config) :
    _config(config)
{
    m_socket    = NULL;
    m_bytesRead = 0;
    m_stopp    = false;
    m_reqReset = false;

    std::cout << "Create UdpLink: " << config.address() << config.portNumber() << std::endl;
}

UdpLink::~UdpLink()
{
    Disconnect();
    if(m_socket) delete m_socket;
    m_socket = NULL;
}

void UdpLink::RequestReset()
{
    m_stoppMutex.lock();
    m_reqReset = true;
    m_stoppMutex.unlock();
}
uint64_t UdpLink::getConnectionSpeed() const
{
    // TODO: Figure out a way to have a serial ilink and udp ilink separated, as udp doesnt need connectionspeed
    return 0;
}

bool UdpLink::Connect(void)
{
    Disconnect();

    QUdpSocket::SocketError error;
    QString errorString;

    // Initialize the connection
    if (!_hardwareConnect(error, errorString)) {
        if (_config.isAutoConnect()) {
            // Be careful with spitting out open error related to trying to open a busy port using autoconnect
            if (error == QUdpSocket::AddressInUseError) {
                // Device already open, ignore and fail connect
                return false;
            }
        }

        _emitLinkError("Error connecting: Could not create port. " + errorString.toStdString());
        return false;
    }
    return true;
}

void UdpLink::Disconnect(void)
{
    if (m_socket) {
        m_socket->close();
        delete m_socket;
        m_socket = NULL;
    }
}


/// Performs the actual hardware port connection.
///     @param[out] error if failed
///     @param[out] error string if failed
/// @return success/fail
bool UdpLink::_hardwareConnect(QAbstractSocket::SocketError &error, QString& errorString)
{
    if (m_socket) {
        std::cout << "UdpLink:" << QString::number((long)this, 16).toStdString() << "closing port" << std::endl; // TODO: What is this doing??
        m_socket->close();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        delete m_socket;
        m_socket = NULL;
    }

    std::cout << "UdpLink: hardwareConnect to " << _config.address() << ":" << _config.portNumber() << std::endl;

    m_socket = new QUdpSocket();
    m_socket->bind(_config.portNumber(), QUdpSocket::ShareAddress);


    // TODO: Figure out the alternative to this:
    EmitEvent([this](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, getAddress(), "Opened port!");});
    EmitEvent([this](const ILinkEvents *ptr){ptr->Connected(this);});

    std::cout << "Connection UdpLink: " << "with settings " << _config.address() << ":" << _config.portNumber() << std::endl;


    m_ListenThread = new ReceiverThread([&](){
        if(m_socket->waitForReadyRead(300))
            this->processPendingDatagrams();

        if(m_socket->errorString() != "")
        {
//            std::cout << "Socket error: " << m_socket->errorString().toStdString() << std::endl;
        }
    });
    m_socket->moveToThread(m_ListenThread);
    m_ListenThread->start();

    return true; // successful connection
}


void UdpLink::WriteBytes(const char *bytes, int length) const
{
    QByteArray data(bytes, length);
    if(m_socket && m_socket->isOpen()) {
        //_logOutputDataRate(data.size(), QDateTime::currentMSecsSinceEpoch());
        m_socket->write(data);
    } else {
        // Error occured
        _emitLinkError("Could not send data - link " + getAddress() + ":" + std::to_string(getPortNumber()) + " is disconnected!");
    }
}



//!
//! \brief Determine the connection status
//! \return True if the connection is established, false otherwise
//!
bool UdpLink::isConnected() const
{
    bool isConnected = false;

    if (m_socket) {
        isConnected = m_socket->isOpen();
    }

    return isConnected;
}


void UdpLink::_emitLinkError(const std::string& errorMsg) const
{
    std::string msg = "Error on link " + getAddress() + ":" + std::to_string(getPortNumber()) + " - " + errorMsg;
    EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationError(this, "Link Error", msg);});
}

LinkConfiguration UdpLink::getLinkConfiguration()
{
    return _config;
}

std::string UdpLink::getAddress() const
{
    return _config.address();
}

int UdpLink::getPortNumber() const
{
    return _config.portNumber();
}

void UdpLink::processPendingDatagrams(void)
{
    // TODO: Read bytes from UDP socket. Readdatagram???
//    while (m_socket->hasPendingDatagrams()) {
//        QByteArray datagram;
//        datagram.resize(m_socket->pendingDatagramSize());
//        QHostAddress sender;
//        quint16 senderPort;

//        m_socket->readDatagram(datagram.data(), datagram.size(),
//                                &sender, &senderPort);

//        std::cout << "UDP Data: " << datagram.data() << " - Received from: " << sender.toString().toStdString() << std::endl;
//    }

    while (m_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_socket->pendingDatagramSize());
        m_socket->readDatagram(datagram.data(), datagram.size());

        // TODO: TEST THIS!!!
        std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(datagram.begin(), datagram.end());
        EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
    }
}

void UdpLink::linkError(QUdpSocket::SocketError error)
{
    // TODO: Handle link errors:
    switch (error) {
    case QUdpSocket::AddressInUseError:
        EmitEvent([this](const ILinkEvents *ptr){ptr->ConnectionRemoved(this);});
        break;
    default:
        break;
    }
}


}
