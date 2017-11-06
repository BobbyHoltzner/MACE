#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    geometry_msgs::Point robotPosition;
    robotPosition.x = 0.0;
    robotPosition.y = 0.0;
    robotPosition.z = 0.0;

    geometry_msgs::Quaternion attitude;
    attitude.x = 0.0;
    attitude.y = 0.0;
    attitude.z = 0.0;
    attitude.w = 1.0;

    gazebo_msgs::ModelState modelState;
    modelState.model_name = (std::string)"mybot";
    modelState.reference_frame = (std::string)"world";

    gazebo_msgs::SetModelState srv;

    while (ros::ok()) {
        robotPosition.x  = cos(angle)*2;
        robotPosition.y = sin(angle)*2;
        attitude = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        geometry_msgs::Pose pose;
        pose.position = robotPosition;
        pose.orientation = attitude;
	modelState.pose = pose;

        srv.request.model_state = modelState;
        if(client.call(srv))
        {
            //this means it was a success
        }
        else{
            //this means it was not a success
        }
        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

