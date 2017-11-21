#ifndef MODULE_ROS_H
#define MODULE_ROS_H

#include "common/common.h"
#include "module_ROS_global.h"

#include "mace_core/i_module_command_ROS.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include <memory>

#ifdef ROS_EXISTS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
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
        ptr->Subscribe(this, m_PlanningStateTopic.Name());
        ptr->Subscribe(this, m_VehicleDataTopic.Name());
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

    void newLaserScan(const ros::MessageEvent<sensor_msgs::LaserScan const>& event);

    void newPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void pixelTo3DPoint(const sensor_msgs::PointCloud2::ConstPtr& pCloud, const int u, const int v, geometry_msgs::Point &p);

    void publishVehiclePosition(const int &vehicleID, const DataState::StateLocalPosition &localPos);

    void renderState(const mace::pose::CartesianPosition_2D &state);

    void renderEdge(const mace::geometry::Line_2DC &edge);

    void updatePositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &component);
    void updateAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component);
#endif

private:

#ifdef ROS_EXISTS
    ros::Subscriber laserSub;
    ros::Subscriber pointCloudSub;
    ros::Publisher velocityPub, markerPub;
    visualization_msgs::Marker points, line_strip, line_list;

    ros::ServiceClient m_client;
    tf::TransformBroadcaster m_broadcaster;
    tf::Transform m_transform;
    gazebo_msgs::ModelState m_modelState;
    gazebo_msgs::SetModelState m_srv;
#endif

    std::map<int, std::tuple<DataState::StateLocalPosition, DataState::StateAttitude> > m_vehicleMap;

    std::shared_ptr<ROSTimer> m_timer;

    // TESTING:
    int counter;
    const double degree =  M_PI/180;
    double tilt, tinc, swivel, angle, height, hinc;
    // END TESTING

private:
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
};

#endif // MODULE_ROS_H
