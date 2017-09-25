#include <module_ROS.h>

#include "mace_core/module_factory.h"

#include <iostream>

ModuleROS::ModuleROS() :
    MaceCore::IModuleCommandROS()
{
#ifdef ROS_EXISTS
    this->setupROS();
#endif
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleROS::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleROS::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
}

void ModuleROS::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    UNUSED(topicName);
    UNUSED(senderID);
    UNUSED(componentsUpdated);
}

void ModuleROS::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
}

////! ========================================================================
////! ======================  ROS Specific functions:  =======================
////! ========================================================================
#ifdef ROS_EXISTS

void ModuleROS::setupROS() {

    // Subscribers
    laserSub = nh.subscribe <sensor_msgs::LaserScan> ("/scan", 500, &ModuleROS::newLaserScan, this);

    ros::spin();
}

void ModuleROS::newLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::cout << "Ranges size: " << msg.ranges.size() << std::endl;
}

#endif
