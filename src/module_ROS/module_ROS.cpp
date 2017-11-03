#include <module_ROS.h>

#include "mace_core/module_factory.h"

#include <map>
#include <string>
#include <iostream>

#ifdef ROS_EXISTS
#include <geometry_msgs/Twist.h>
#endif

ModuleROS::ModuleROS() :
    MaceCore::IModuleCommandROS(),
    m_PlanningStateTopic("planningState")
{
    // TESTING:
    counter = 0;
    // END TESTING
}

ModuleROS::~ModuleROS() {
    if(m_timer)
    {
        m_timer->stop();
    }

#ifdef ROS_EXISTS
    ros::shutdown();
#endif
}

void ModuleROS::start() {
#ifdef ROS_EXISTS
    this->setupROS();

    // Start timer:
    m_timer = std::make_shared<ROSTimer>([=]()
    {
        ros::spinOnce();
        counter++;

        if(counter%10 == 0) {
            std::cout << "**** **** **** Fire publisher..." << std::endl;
            DataState::StateLocalPosition pos;
            publishVehiclePosition(counter, pos);
        }
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(ROSTimer::Interval(500));
    m_timer->start(true);

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

    // TODO: On new vehicle position, send to ROS via publishVehiclePosition
    // TODO: Figure out a better way to check for ROS_EXISTS...the way it is right now, everything
    //          in this NewTopic method would have to check if ROS_EXISTS before calling any ROS specific methods

    if(topicName == m_PlanningStateTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_PlanningStateTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == mace::poseTopic::Cartesian_2D_Topic::Name()){
                std::shared_ptr<mace::poseTopic::Cartesian_2D_Topic> component = std::make_shared<mace::poseTopic::Cartesian_2D_Topic>();
                m_PlanningStateTopic.GetComponent(component, read_topicDatagram);
                this->renderState(component->getPose());
            }
            else if(componentsUpdated.at(i) == mace::geometryTopic::Line_2DC_Topic::Name()) {
                std::shared_ptr<mace::geometryTopic::Line_2DC_Topic> component = std::make_shared<mace::geometryTopic::Line_2DC_Topic>();
                m_PlanningStateTopic.GetComponent(component, read_topicDatagram);
                this->renderEdge(component->getLine());
            }
        }
    }

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
    std::map<std::string, std::string> remappings;
    ros::init(remappings,"ROS_Module");
    ros::NodeHandle nh;
    // Subscribers
    laserSub = nh.subscribe <sensor_msgs::LaserScan> ("/scan", 500, &ModuleROS::newLaserScan, this);

    // Publishers
    velocityPub = nh.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1000);

    //State Planning Publisher
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // %Tag(MARKER_INIT)%
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;
    // %EndTag(ID)%

    // %Tag(TYPE)%
        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // %Tag(SCALE)%
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.1;
        points.scale.y = 0.1;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.05;
        line_list.scale.x = 0.05;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
    // %EndTag(COLOR)%

    ros::spinOnce();
}

void ModuleROS::newLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::cout << "Ranges size: " << msg->ranges.size() << std::endl;
}

void ModuleROS::publishVehiclePosition(const int &vehicleID, const DataState::StateLocalPosition &localPos) {
    // Publish the "velocity" topic to the turtlebot

    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    // Initialize the twist message that we will command turtlebot with
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = vehicleID/10;

    // Publish the twist message to anyone listening
    velocityPub.publish(msg);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();
}


void ModuleROS::renderState(const mace::pose::CartesianPosition_2D &state) {
    geometry_msgs::Point p;
    p.x = state.getXPosition();
    p.y = state.getYPosition();
    points.points.push_back(p);
    markerPub.publish(points);
}

void ModuleROS::renderEdge(const mace::geometry::Line_2DC &edge) {
    geometry_msgs::Point startPoint;
    mace::pose::CartesianPosition_2D begin(edge.getBeginLine());

    startPoint.x = begin.getXPosition();
    startPoint.y = begin.getYPosition();

    geometry_msgs::Point endPoint;
    mace::pose::CartesianPosition_2D end(edge.getEndLine());
    endPoint.x = end.getXPosition();
    endPoint.y = end.getYPosition();

    line_list.points.push_back(startPoint);
    line_list.points.push_back(endPoint);
    markerPub.publish(line_list);
}
#endif
