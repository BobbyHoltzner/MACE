#ifndef ARDUPILOTPOSITION_H
#define ARDUPILOTPOSITION_H

#include <Eigen/Dense>

#include "mavlink.h"
#include "ardupilot_global_position.h"
#include "ardupilot_local_position.h"

//This class will handle the local, global and home positioning elements of a vehicle
namespace Ardupilot{

class ArdupilotPosition
{
public:
    ArdupilotPosition();
    void handleMAVLINKMessage(const mavlink_message_t &posMSG);
    void getGlobalPosition(Eigen::Vector3d &positionVector);


private:
    ArdupilotLocalPosition* m_LocalPosition;
    ArdupilotGlobalPosition* m_GlobalPosition;
};

} //end of namespace Ardupilot

#endif // ARDUPILOTPOSITION_H
