#ifndef MAVLINK_PARSER_ARDUPILOT_H
#define MAVLINK_PARSER_ARDUPILOT_H

#include "mavlink.h"

#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "components/vehicle_operating_parameters.h"
#include "components/vehicle_operating_status.h"


namespace DataVehicleArdupilot
{


class MAVLINKParserArduPilot
{
public:

    MAVLINKParserArduPilot() :
        m_CurrentArduVehicleState(NULL), m_CurrentArduVehicleStatus(NULL)
    {

    }


    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> Parse(const mavlink_message_t* message){
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
        switch ((int)message->msgid) {

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                mavlink_heartbeat_t decodedMSG;
                mavlink_msg_heartbeat_decode(message,&decodedMSG);

                std::shared_ptr<VehicleOperatingParameters> ptrParameters = std::make_shared<VehicleOperatingParameters>();
                //m_FlightMode->setVehicleType(decodedMSG.type);
                ptrParameters->setPlatform((Arduplatforms)decodedMSG.type);
                ptrParameters->setFlightMode(decodedMSG.custom_mode);
                //check that something has actually changed
                if(m_CurrentArduVehicleState == NULL || *ptrParameters != *m_CurrentArduVehicleState)
                {
                    rtnVector.push_back(ptrParameters);
                }

                std::shared_ptr<VehicleOperatingStatus> ptrStatus = std::make_shared<VehicleOperatingStatus>();
                ptrStatus->setVehicleArmed(decodedMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
                //check that something has actually changed
                if(m_CurrentArduVehicleStatus == NULL || *ptrStatus != *m_CurrentArduVehicleStatus)
                {
                    rtnVector.push_back(ptrStatus);
                }

                if(rtnVector.empty())
                {
                    return {};
                }
                return {rtnVector};



            }
            default:
                return {};
        }

    }


private:

    std::shared_ptr<VehicleOperatingParameters> m_CurrentArduVehicleState;
    std::shared_ptr<VehicleOperatingStatus> m_CurrentArduVehicleStatus;


};

}

#endif // MAVLINK_PARSER_ARDUPILOT_H
