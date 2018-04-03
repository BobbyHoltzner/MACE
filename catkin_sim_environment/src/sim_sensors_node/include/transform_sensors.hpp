#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * @brief TransformSensors class handles transforming sensor data into the appropriate frame
 */
class TransformSensors
{
  public:
    /**
     * @brief TransfromSensors constructor
     */
    TransformSensors(std::string modelName);

    /**
     * @brief Callback function for the laser scans reported from simulated UxV
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    /**
     * @brief Callback function for the kinect/depth/points from simulated UxV
     */
    void kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  private:
    /**
     * @brief Setup publishers for when data changes in each callback
     */
    void setupPublishers();

  private:
    /**
     * @brief Name of the model to publish/subscribe to
     */
    std::string modelName;

    /**
     * @brief container for a ROS node handler
     */
    ros::NodeHandle nh;

    /**
     * @brief container for a ROS publisher to publish point cloud data after transform
     */
    ros::Publisher pointCloudPub;

    /**
     * @brief container for a ROS publisher to publish laser scan after transform
     */
    ros::Publisher laserScanPub;
};