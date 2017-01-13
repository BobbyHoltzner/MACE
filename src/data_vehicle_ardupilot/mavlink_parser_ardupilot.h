#ifndef MAVLINK_PARSER_ARDUPILOT_H
#define MAVLINK_PARSER_ARDUPILOT_H

#include "mavlink.h"

#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "components/vehicle_operating_parameters.h"

namespace DataVehicleArdupilot
{


class MAVLINKParserArduPilot
{
public:

    MAVLINKParserArduPilot() :
        m_CurrentArduVehicleState(NULL)
    {

    }


    template<typename T>
    std::shared_ptr<MaceCore::TopicDatagram> Parse(const mavlink_message_t* message, const T* topic){

        switch ((int)message->msgid) {

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                mavlink_heartbeat_t decodedMSG;
                mavlink_msg_heartbeat_decode(message,&decodedMSG);

                std::shared_ptr<VehicleOperatingParameters> ptr = std::make_shared<VehicleOperatingParameters>();

                //m_FlightMode->setVehicleType(decodedMSG.type);
                ptr->setPlatform((Arduplatforms)decodedMSG.type);
                ptr->setFlightMode(decodedMSG.custom_mode);


                if(m_CurrentArduVehicleState == NULL || *ptr != *m_CurrentArduVehicleState)
                {
                    std::shared_ptr<MaceCore::TopicDatagram> datagram = std::make_shared<MaceCore::TopicDatagram>();
                    ((Data::TopicDataObjectCollection<VehicleOperatingParameters>*)topic)->SetComponent(ptr, datagram.get());
                    m_CurrentArduVehicleState = ptr;
                    return datagram;
                }
                return NULL;
            }
            default:
                return NULL;
        }

    }


private:

    std::shared_ptr<VehicleOperatingParameters> m_CurrentArduVehicleState;

};

}

#endif // MAVLINK_PARSER_ARDUPILOT_H
