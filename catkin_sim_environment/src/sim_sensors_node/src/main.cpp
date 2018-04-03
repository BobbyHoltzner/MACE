#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include "transform_sensors.hpp"

#include <typeinfo>

int main(int argc, char **argv)
{
    // Initialize ROS and name our node "talker"
    ros::init(argc, argv, "transform_sensors");

    // Strip away the ROS added arguments passed to the node, then parse arguments passed to the node
    std::vector<std::string> args;
    ros::removeROSArgs(argc, argv, args);
    int vehicleID = std::stoi(args[1]);
    std::string modelName = "basic_quadrotor_" + std::to_string(vehicleID);

    // Handle for the process node. Will handle initialization and
    //   cleanup of the node
    ros::NodeHandle nh;

    // TransformSensors object
    TransformSensors sensorTF(modelName);

    // Subscribe to the "scan" topic to listen for any
    //   messages published on that topic.
    // Set the buffer to 500 messages
    // Set the callback to the chatterCallback method
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 500, &TransformSensors::laserCallback, &sensorTF);
    ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(modelName + "/kinect/depth/points", 1000, &TransformSensors::kinectDepthCallback, &sensorTF);

    // Publish the "velocity" topic to the turtlebot
    ros::Publisher velocityPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        /*
            TODO:
                1) If any messages have changed, publish. (May just want to do this in each callback function...)
        */

        // "Spin" a callback in case we set up any callbacks
        ros::spinOnce();

        // Sleep for the remaining time until we hit our 10 Hz rate
        loop_rate.sleep();
    }

    return 0;
}