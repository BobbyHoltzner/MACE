#ifndef I_MODULE_EVENTS_ROS_H
#define I_MODULE_EVENTS_ROS_H

#include "i_module_events_general.h"

namespace MaceCore
{

class IModuleEventsROS  : public IModuleEventsGeneral
{

public:
    virtual void ROS_NewLaserScan(const octomap::Pointcloud* obj) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_ROS_H
