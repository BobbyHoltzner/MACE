#include <iostream>

#include "ardupilot_position.h"

namespace Ardupilot{

/*
ArdupilotPosition::ArdupilotPosition()
{
    m_LocalPosition = new ArdupilotLocalPosition();
    m_GlobalPosition = new ArdupilotGlobalPosition();

    fixStatus = 0;
    numberOfSats = 0;
    horizontalDOP = 0;
    verticalDOP = 0;
}

void ArdupilotPosition::getGlobalPosition(Data::GlobalPosition &positionVector)
{
    m_GlobalPosition->getGlobalPosition(positionVector);
}

void ArdupilotPosition::getGPSStatus(int &fixCode, int &numSats)
{
    fixCode = this->fixStatus;
    numSats = this->numberOfSats;
}

void ArdupilotPosition::handleMAVLINKMessage(const mavlink_message_t &posMSG)
{
    int messageID = posMSG.msgid;
    switch (messageID)
    {
    case MAVLINK_MSG_ID_GPS_STATUS:
    {
        //This is message definition 25
        //The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
        mavlink_gps_status_t decodedMSG;
        mavlink_msg_gps_status_decode(&posMSG,&decodedMSG);
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        //This is message definition 24
        //The global position, as returned by the Global Positioning System (GPS). This is NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
        mavlink_gps_raw_int_t decodedMSG;
        mavlink_msg_gps_raw_int_decode(&posMSG,&decodedMSG);
        fixStatus = decodedMSG.fix_type;
        numberOfSats = decodedMSG.satellites_visible;
        horizontalDOP = decodedMSG.eph;
        verticalDOP = decodedMSG.epv;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        //This is message definition 33
        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(&posMSG,&decodedMSG);
        m_GlobalPosition->updateFromMavlink(decodedMSG);
        break;
    }
    default:
        std::cout << "I found another type of message" << messageID << std::endl;
    }
}
*/
} //end of namespace Ardupilot
