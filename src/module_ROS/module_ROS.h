#ifndef MODULE_ROS_H
#define MODULE_ROS_H

#include "common/common.h"
#include "module_ROS_global.h"

#include "mace_core/i_module_command_ROS.h"

#include <memory>


#ifdef ROS_EXISTS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#endif

#include "rosTimer.h"


class MODULE_ROSSHARED_EXPORT ModuleROS : public MaceCore::IModuleCommandROS
{

public:
    ModuleROS();

    ~ModuleROS();

    //!
    //! \brief start Start ROS loop
    //!
    void start();

    // ============================================================================= //
    // ================= Default methods for module configuration ================== //
    // ============================================================================= //
public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
    {
        UNUSED(ptr);
    }

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    //!
    //! \brief NewTopic New topic available from MACE Core
    //! \param topicName Topic name that has been published
    //! \param senderID Topic sender ID
    //! \param componentsUpdated List of MACE core components that have updated data
    //!
    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGroundStation ======== //
    // ============================================================================= //
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID);


    // ============================================================================= //
    // ========================  ROS Specific functions:  ========================== //
    // ============================================================================= //
public:

#ifdef ROS_EXISTS

    //!
    //! \brief setupROS Setup ROS subscribers, publishers, and node handler
    //!
    void setupROS();

    //!
    //! \brief newLaserScan Callback for ROS laser scan subscriber
    //! \param msg Laser scan message
    //!
    void newLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);

    //!
    //! \brief newOccupancyGrid Callback for ROS occupancy grid subscriber
    //! \param msg Occupancy grid message
    //!
    void newOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    //!
    //! \brief publishVehiclePosition Publish vehicle position to ROS
    //! \param vehicleID Vehicle ID for which to set the new position
    //! \param localPos New vehicle position
    //!
    void publishVehiclePosition(const int &vehicleID, const DataState::StateLocalPosition &localPos);

    //!
    //! \brief addSensorsToROS Spawn sensor models in ROS
    //!
    void addSensorsToROS();
#endif

private:

#ifdef ROS_EXISTS
    //!
    //! \brief laserSub Subscriber for ROS laser scan messages
    //!
    ros::Subscriber laserSub;

    //!
    //! \brief mapSub Subscriber for occupancy grid messages
    //!
    ros::Subscriber mapSub;

    //!
    //! \brief velocityPub Publisher for velocity/Twist messages
    //!
    ros::Publisher velocityPub;

    //!
    //! \brief vehicleSensors Vector of vehicle sensors that can be spawned in ROS
    //!
    std::vector<std::string> vehicleSensors;
#endif

    //!
    //! \brief m_timer Timer that fires and spins the ROS loop to fire any publishers or subscribers stuck in the queue
    //!
    std::shared_ptr<ROSTimer> m_timer;

    //!
    //! \brief m_vehicleID Vehicle ID attached to this ROS module instance
    //!
    int m_vehicleID;

    //!
    //! \brief airborneInstance Flag denoting if this ROS module is attached to an airborne instance
    //!
    bool airborneInstance;

    //!
    //! \brief m_sensors List of sensors that will be spawned into the ROS environment
    //!
    std::vector<std::tuple<std::string, std::string> > m_sensors;

    // TESTING:
    int counter;
    // END TESTING
};

#endif // MODULE_ROS_H
