#include <module_ROS.h>

#include "mace_core/module_factory.h"

#include <map>
#include <string>
#include <iostream>

#include <limits>

#ifdef ROS_EXISTS
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <octomap_ros/conversions.h>

#endif

//!
//! \brief ModuleROS Default constructor
//!
ModuleROS::ModuleROS() :
    MaceCore::IModuleCommandROS(),
    m_PlanningStateTopic("planningState"),
    m_VehicleDataTopic("vehicleData")
{
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

//!
//! \brief start Start ROS loop
//!
void ModuleROS::start() {
#ifdef ROS_EXISTS
    this->setupROS();

    // Start timer:
    m_timer = std::make_shared<ROSTimer>([=]()
    {
        // Spin ROS
        ros::spinOnce();
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(ROSTimer::Interval(50));
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

//    structure.AddTerminalParameters("ID", MaceCore::ModuleParameterTerminalTypes::INT, true);

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

//    this->SetID(params->GetTerminalValue<int>("ID"));
}

//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleROS::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{

}


//!
//! \brief New Spooled topic given
//!
//! Spooled topics are stored on the core's datafusion.
//! This method is used to notify other modules that there exists new data for the given components on the given module.
//! \param topicName Name of topic given
//! \param sender Module that sent topic
//! \param componentsUpdated Components in topic that where updated
//! \param target Target moudle (or broadcast)
//!
void ModuleROS::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    int senderID = sender.ID;
    if(topicName == m_PlanningStateTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_PlanningStateTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == mace::poseTopic::Cartesian_2D_Topic::Name()){
                std::shared_ptr<mace::poseTopic::Cartesian_2D_Topic> component = std::make_shared<mace::poseTopic::Cartesian_2D_Topic>();
                m_PlanningStateTopic.GetComponent(component, read_topicDatagram);
                //this->renderState(component->getPose());
            }
            else if(componentsUpdated.at(i) == mace::geometryTopic::Line_2DC_Topic::Name()) {
                std::shared_ptr<mace::geometryTopic::Line_2DC_Topic> component = std::make_shared<mace::geometryTopic::Line_2DC_Topic>();
                m_PlanningStateTopic.GetComponent(component, read_topicDatagram);
                //this->renderEdge(component->getLine());
            }
        }
    }
    else if(topicName == m_VehicleDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // If vehicle does not exist in our map, insert into the map
                insertVehicleIfNotExist(senderID);

                // Write Attitude data to the GUI:
                updateAttitudeData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateLocalPositionTopic> component = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // If vehicle does not exist in our map, insert into the map
                insertVehicleIfNotExist(senderID);

                // Write Position data to the GUI:
                updatePositionData(senderID, component);
            }
        }
    }
}

//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleROS::NewlyAvailableVehicle(const int &vehicleID)
{
    // If vehicle does not exist in our map, insert into the map
    insertVehicleIfNotExist(vehicleID);
}

void ModuleROS::insertVehicleIfNotExist(const int &vehicleID) {
    if(m_vehicleMap.find(vehicleID) == m_vehicleMap.end()) {

        std::cout << " =================== IN IF STATEMENT =================== " << std::endl;

        // Insert vehicle into map:
        DataState::StateLocalPosition localPos;
        DataState::StateAttitude att;
        std::tuple<DataState::StateLocalPosition, DataState::StateAttitude> tmpTuple = std::make_tuple(localPos, att);
        m_vehicleMap.insert(std::make_pair(vehicleID, tmpTuple));

#ifdef ROS_EXISTS
        // Add subscriber(s) for vehicle sensor messages
        std::vector<ros::Subscriber> vehicleSensors;
        std::string modelName = "basic_quadrotor_" + std::to_string(vehicleID);

        for(auto sensor : m_sensors) {
            std::string sensorType = std::get<1>(sensor);
            ros::Subscriber tmpSub;
            if(sensorType == "lidar_scan") {
                tmpSub = nh.subscribe("MACE/" + modelName + "/scan/cloud", 500, &ModuleROS::newPointCloud, this);
            }
            else if(sensorType == "lidar_flash") {
                std::cout << "In if lidar_scan for model: " << modelName << std::endl;
                tmpSub = nh.subscribe <sensor_msgs::PointCloud2> ("MACE/" + modelName + "/kinect/depth/points", 1000, &ModuleROS::newPointCloud, this);
            }
            else if(sensorType == "camera") {
                // TODO
            }
            // Add sensor to list:
            vehicleSensors.push_back(tmpSub);
        }
        // Add sensor list to sensor map:
        m_sensorVehicleMap.insert(std::make_pair(vehicleID, vehicleSensors));
#endif
    }
}

//!
//! \brief updatePositionData Update the position of the corresponding Gazebo model based on position of MACE vehicle
//! \param vehicleID ID of the vehicle to update
//! \param component Position (in the local frame)
//!
void ModuleROS::updatePositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &component)
{
    double x = component->getX();
    double y = component->getY();
    double z = component->getZ();

    if(m_vehicleMap.find(vehicleID) != m_vehicleMap.end()) {
        std::tuple<DataState::StateLocalPosition, DataState::StateAttitude> tmpTuple;
        DataState::StateLocalPosition tmpPos = DataState::StateLocalPosition(component->getCoordinateFrame(), x, y, z);
        DataState::StateAttitude tmpAtt = std::get<1>(m_vehicleMap[vehicleID]);
        tmpTuple = std::make_tuple(tmpPos, tmpAtt);
        m_vehicleMap[vehicleID] = tmpTuple;
    }

#ifdef ROS_EXISTS
    // Send gazebo model state:
    sendGazeboModelState(vehicleID);
#endif
}

//!
//! \brief updateAttitudeData Update the attitude of the corresponding Gazebo model based on attitude of MACE vehicle
//! \param vehicleID ID of the vehicle to update
//! \param component Attitude
//!
void ModuleROS::updateAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component)
{
    double roll = component->roll;
    double pitch = component->pitch;
    double yaw = component->yaw;

    if(m_vehicleMap.find(vehicleID) != m_vehicleMap.end()) {
        std::tuple<DataState::StateLocalPosition, DataState::StateAttitude> tmpTuple;
        DataState::StateLocalPosition tmpPos = std::get<0>(m_vehicleMap[vehicleID]);
        DataState::StateAttitude tmpAtt = DataState::StateAttitude();
        tmpAtt.setAttitude(roll, pitch, yaw);
        tmpTuple = std::make_tuple(tmpPos, tmpAtt);
        m_vehicleMap[vehicleID] = tmpTuple;
    }

#ifdef ROS_EXISTS
    // Send gazebo model state:
    sendGazeboModelState(vehicleID);
#endif
}

////! ========================================================================
////! ======================  ROS Specific functions:  =======================
////! ========================================================================
#ifdef ROS_EXISTS
//!
//! \brief setupROS Setup ROS subscribers, publishers, and node handler
//!
void ModuleROS::setupROS() {
    m_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // TODO: Do I need to use the transform container? Or can I just create a transform before sending the model state to Gazebo?
    m_transform.setOrigin(tf::Vector3(0.0,0.0,1.0));
    m_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    // TODO: Do I need to send this transform before I do anything else? And should I send it for every basic_quadrotor_ID?
//    m_broadcaster.sendTransform(tf::StampedTransform(m_transform,ros::Time::now(),"world","basic_quadrotor/base_link"));

    // TESTING:
    cloudInPub = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 50);
    // END TESTING

    ros::spinOnce();
}

//!
//! \brief newLaserScan Laser scan callback for ROS LaserScan message
//! \param event LaserScan message
//!
void ModuleROS::newLaserScan(const ros::MessageEvent<sensor_msgs::LaserScan const>& event) {
    // ** Message event metadata
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");
    ros::Time receipt_time = event.getReceiptTime();

    std::cout << "**** Message Event ****" << std::endl;
    std::cout << "  Publisher name: " << publisher_name << std::endl;
    std::cout << "  Header -- topic: " << topic << std::endl;
    std::cout << "  receipt_time: " << receipt_time << std::endl;
    std::cout << "**** **** ****" << std::endl;

    // ** Get Laser Scan message from message event:
    const sensor_msgs::LaserScan::ConstPtr& msg = event.getMessage();
    std::cout << "  Ranges size: " << msg->ranges.size() << std::endl;
    std::cout << "  Range min: " << msg->range_min << std::endl;
    std::cout << "  Range max: " << msg->range_max << std::endl;

    double minDistance = std::numeric_limits<double>::max();
    for(int i = 0; i < msg->ranges.size(); i++) {
        if(msg->ranges[i] < minDistance) {
            minDistance = msg->ranges[i];
        }
    }
    std::cout << "  Loop range min: " << minDistance << std::endl;
}

//!
//! \brief newPointCloud Point cloud callback for ROS PointCloud2 message
//! \param msg PointCloud2 message
//!
void ModuleROS::newPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert to Octomap Point Cloud:
    octomap::Pointcloud octoPointCloud;
//    octomap::pointCloud2ToOctomap(*msg, octoPointCloud);

    // TODO: Send converted point cloud to MACE core so path planning can take over.


    std::cout << "Converted PC..." << std::endl;
    std::cout << octoPointCloud.size() << std::endl;

    /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
        }); *///this is a general publication event, however, no one knows explicitly how to handle

    // TODO: Make this NotifyListeners method name better -- this is directly to Mace core
        ModuleROS::NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
            ptr->ROS_NewLaserScan(octoPointCloud);
        }); //this one explicitly calls mace_core and its up to you to handle in core


    // TESTING OCTOMAP VISUALIZATION:
    // TODO: Publish PointCloud2 on correct topic for RViz visualization with octomap plugins
    //      - This is to avoid remapping every vehicle's point cloud topics to the cloudIn topic required by octomap server
    //          - Ideally, for the entire octomap, we don't care about which vehicle its coming from. They should be updating the
    //              same octomap

//    cloudInPub.publish(msg);

    //
}

//!
//! \brief renderState Publish the 2D Cartesian Position to ROS for rendering in RViz
//! \param state 2D Cartesian Position to render
//!
void ModuleROS::renderState(const mace::pose::CartesianPosition_2D &state) {
    geometry_msgs::Point p;
    p.x = state.getXPosition();
    p.y = state.getYPosition();
    points.points.push_back(p);
    markerPub.publish(points);
}

//!
//! \brief renderEdge Publish the 2D line to ROS for rendering in RViz
//! \param edge Edge/line to render
//!
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

//!
//! \brief convertToGazeboCartesian Convert position in local frame to Gazebo's world frame
//! \param localPos MACE local position
//!
void ModuleROS::convertToGazeboCartesian(DataState::StateLocalPosition& localPos) {

    switch (localPos.getCoordinateFrame()) {
    case Data::CoordinateFrameType::CF_GLOBAL:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_LOCAL_NED:
        localPos.setZ(-localPos.getZ());
        break;
    case Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_LOCAL_ENU:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_GLOBAL_INT:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT_INT:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_LOCAL_OFFSET_NED:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_BODY_NED:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_BODY_OFFSET_NED:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT:
        // TODO:
        break;
    case Data::CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT_INT:
        // TODO:
        break;
    default:
        std::cout << "Unknown coordinate system seen when sending to ROS/Gazebo." << std::endl;
    }
}

//!
//! \brief sendGazeboModelState Send the current position and attitude of the corresponding vehicle model to Gazebo
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROS::sendGazeboModelState(const int &vehicleID) {
    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    // robot state
    DataState::StateLocalPosition tmpLocalPos = std::get<0>(m_vehicleMap[vehicleID]);
    DataState::StateAttitude tmpAtt = std::get<1>(m_vehicleMap[vehicleID]);

    convertToGazeboCartesian(tmpLocalPos);

    geometry_msgs::Point robotPosition;
    robotPosition.x  = tmpLocalPos.getPositionX();
    robotPosition.y = tmpLocalPos.getPositionY();
    robotPosition.z = tmpLocalPos.getPositionZ();

    geometry_msgs::Quaternion attitude;
    attitude.x = 0.0;
    attitude.y = 0.0;
    attitude.z = 0.0;
    attitude.w = 1.0;
    attitude = tf::createQuaternionMsgFromRollPitchYaw(tmpAtt.roll, tmpAtt.pitch, tmpAtt.yaw);

    geometry_msgs::Pose pose;
    pose.position = robotPosition;
    pose.orientation = attitude;

    // Set model state and name:
    gazebo_msgs::ModelState modelState;
    modelState.model_name = "basic_quadrotor_" + std::to_string(vehicleID);
    modelState.reference_frame = "world";
    modelState.pose = pose;

    // TODO: Do I need to use the transform container? Or can I just create a transform before sending the model state to Gazebo?
    m_transform.setOrigin(tf::Vector3(robotPosition.x,robotPosition.y,robotPosition.z));
    m_transform.setRotation(tf::Quaternion(attitude.x,attitude.y,attitude.z,attitude.w));
    std::string childLink = modelState.model_name + "/base_link";
    m_broadcaster.sendTransform(tf::StampedTransform(m_transform,ros::Time::now(),"world",childLink));

    m_srv.request.model_state = modelState;

    ros::spinOnce();


    if(m_client.call(m_srv))
    {
        //this means it was a success
        return true;
    }
    else {
        //this means it was not a success
        return false;
    }
}

#endif
