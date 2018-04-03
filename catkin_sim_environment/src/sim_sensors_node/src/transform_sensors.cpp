#include <stdlib.h>
#include <ros/ros.h>
#include "transform_sensors.hpp"

/**
 * @brief TransfromSensors constructor
 */
TransformSensors::TransformSensors(std::string modelName) :
modelName(modelName)
{
    setupPublishers();
}

/**
 * @brief Setup publishers for when data changes in each callback
 */
void TransformSensors::setupPublishers()
{
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/MACE/" + modelName + "/kinect/depth/points", 1000);
    laserScanPub = nh.advertise<sensor_msgs::LaserScan>("/MACE/" + modelName + "/scan", 1000);
}

/**
 * @brief Callback function for the laser scans reported from turtlebot
 */
void TransformSensors::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::cout << "Ranges size: " << msg->ranges.size() << std::endl;
}

/**
 * @brief Callback function for the kinect/depth/points from simulated UxV
 */
void TransformSensors::kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::cout << "Point cloud width: " << msg->width << std::endl;
    std::cout << "Point cloud height: " << msg->height << std::endl;

    std::cout << "FRAME ID: " << msg->header.frame_id << std::endl;

    // pointCloudPub.publish(msg);
}