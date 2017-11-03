#ifndef MODULE_ROS_H
#define MODULE_ROS_H

#include "common/common.h"
#include "module_ROS_global.h"

#include "mace_core/i_module_command_ROS.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include <memory>

#ifdef ROS_EXISTS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#endif

#include "rosTimer.h"


class MODULE_ROSSHARED_EXPORT ModuleROS : public MaceCore::IModuleCommandROS
{

public:
    ModuleROS();

    ~ModuleROS();

    void start();

    //! Default methods for module configuration:
public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
    {
        ptr->Subscribe(this,m_PlanningStateTopic.Name());
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

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

    //! Virtual functions as defined by IModuleCommandROS
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);


    //! ========================================================================
    //! ======================  ROS Specific functions:  =======================
    //! ========================================================================
public:

#ifdef ROS_EXISTS

    void setupROS();

    void newLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);

    void publishVehiclePosition(const int &vehicleID, const DataState::StateLocalPosition &localPos);

    void renderState(const mace::pose::CartesianPosition_2D &state);

    void renderEdge(const mace::geometry::Line_2DC &edge);
#endif

private:

#ifdef ROS_EXISTS
    ros::Subscriber laserSub;
    ros::Publisher velocityPub, markerPub;
    visualization_msgs::Marker points, line_strip, line_list;
#endif

    std::shared_ptr<ROSTimer> m_timer;

    // TESTING:
    int counter;
    // END TESTING

private:
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;
};

#endif // MODULE_ROS_H
