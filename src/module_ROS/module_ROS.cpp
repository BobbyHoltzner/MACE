#include <module_ROS.h>

#include "mace_core/module_factory.h"

#include <map>
#include <string>
#include <iostream>

#ifdef ROS_EXISTS
#include <geometry_msgs/Twist.h>
#endif

ModuleROS::ModuleROS() :
    m_vehicleID(-1),
    airborneInstance(false),
    MaceCore::IModuleCommandROS()
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
    // TODO: Call MACE core getAttachedVehicleID (or whatever we call it)
    //          -Basically to handle if we miss the newly available vehicle topic

    this->setupROS();

    // Add sensors to the ROS instance:
    addSensorsToROS();

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

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> laserSensor = std::make_shared<MaceCore::ModuleParameterStructure>();
    laserSensor->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    laserSensor->AddTerminalParameters("Type", MaceCore::ModuleParameterTerminalTypes::STRING, true, "lidar_scan", {"lidar_scan", "lidar_flash"});
    structure.AddNonTerminal("LaserSensor", laserSensor, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSensor = std::make_shared<MaceCore::ModuleParameterStructure>();
    cameraSensor->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    cameraSensor->AddTerminalParameters("Type", MaceCore::ModuleParameterTerminalTypes::STRING, true, "rgb", {"rgb", "infrared"});
    structure.AddNonTerminal("CameraSensor", cameraSensor, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleROS::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");
    }
    if(params->HasNonTerminal("LaserSensor"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> laserSettings = params->GetNonTerminalValue("LaserSensor");
        std::cout << " **** **** **** **** Sensor name: " << laserSettings->GetTerminalValue<std::string>("Name") << std::endl;
        std::cout << " **** **** **** **** Sensor type: " << laserSettings->GetTerminalValue<std::string>("Type") << std::endl;
        std::tuple<std::string, std::string> sensor = std::make_tuple(laserSettings->GetTerminalValue<std::string>("Name"), laserSettings->GetTerminalValue<std::string>("Type"));
        m_sensors.push_back(sensor);
    }
    if(params->HasNonTerminal("CameraSensor"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> cameraSettings = params->GetNonTerminalValue("CameraSensor");
        std::cout << " **** **** **** **** Sensor name: " << cameraSettings->GetTerminalValue<std::string>("Name") << std::endl;
        std::cout << " **** **** **** **** Sensor type: " << cameraSettings->GetTerminalValue<std::string>("Type") << std::endl;
        std::tuple<std::string, std::string> sensor = std::make_tuple(cameraSettings->GetTerminalValue<std::string>("Name"), cameraSettings->GetTerminalValue<std::string>("Type"));
        m_sensors.push_back(sensor);
    }
}

void ModuleROS::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    UNUSED(topicName);
    UNUSED(senderID);
    UNUSED(componentsUpdated);
    if(senderID == m_vehicleID || !airborneInstance) {

    }


    // TODO: On new vehicle position, send to ROS via publishVehiclePosition
    // TODO: Figure out a better way to check for ROS_EXISTS...the way it is right now, everything
    //          in this NewTopic method would have to check if ROS_EXISTS before calling any ROS specific methods

    std::cout << "NEW TOPIC" << std::endl;
}

void ModuleROS::NewlyAvailableVehicle(const int &vehicleID)
{
    // Set vehicle ID:
    m_vehicleID = vehicleID;
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
    mapSub = nh.subscribe <nav_msgs::OccupancyGrid> ("/map", 500, &ModuleROS::newOccupancyGrid, this);



    // Publishers
    velocityPub = nh.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1000);
//    rosservice call ('set_model', 'model_name', (x,y,z));

    ros::spinOnce();
}

void ModuleROS::addSensorsToROS() {
    // Spawn sensor models in ROS per m_sensors vector:
}

void ModuleROS::newLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::cout << "Ranges size: " << msg->ranges.size() << std::endl;    
}

void ModuleROS::newOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::cout << "Map size: " << std::endl;
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

#endif
