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
#include <nav_msgs/OccupancyGrid.h>
#endif

//!
//! \brief ModuleROS Default constructor
//!
ModuleROS::ModuleROS() :
    MaceCore::IModuleCommandROS(),
    m_PlanningStateTopic("planningState"),
    m_VehicleDataTopic("vehicleData"),
    m_MapTopic("mappingData")
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

    AbstractModule_EventListeners::start();
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
                this->renderState(component->getPose());
            }
            else if(componentsUpdated.at(i) == mace::geometryTopic::Line_2DC_Topic::Name()) {
                std::shared_ptr<mace::geometryTopic::Line_2DC_Topic> component = std::make_shared<mace::geometryTopic::Line_2DC_Topic>();
                m_PlanningStateTopic.GetComponent(component, read_topicDatagram);
                this->renderEdge(component->getLine());
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
    else if(topicName == m_MapTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MapItemTopics::Occupancy2DGrid_Topic::Name()) {
                std::shared_ptr<MapItemTopics::Occupancy2DGrid_Topic> component =  std::shared_ptr<MapItemTopics::Occupancy2DGrid_Topic>();
                m_MapTopic.GetComponent(component, read_topicDatagram);
                std::shared_ptr<Data2DGrid<OctomapWrapper::OccupiedResult>> map = component->getOccupancyMap();
                NewlyCompressedOccupancyMap(*map.get());
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
    std::cout<<"The ROS module is going to see a new vehicle"<<std::endl;
    // If vehicle does not exist in our map, insert into the map
    insertVehicleIfNotExist(vehicleID);
}

void ModuleROS::TestFiring()
{
    std::cout<<"Test firing catches"<<std::endl;
}

void ModuleROS::NewlyCompressedOccupancyMap(const mace::maps::Data2DGrid<mace::maps::OctomapWrapper::OccupiedResult> &map)
{
    std::cout<<"I have made it into the ROS module."<<std::endl;
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.info.resolution = map.getXResolution();
    occupancyGrid.info.origin.position.x = map.getOriginPosition().getXPosition();
    occupancyGrid.info.origin.position.y = map.getOriginPosition().getYPosition();
    occupancyGrid.info.origin.position.z = 0.0;
    occupancyGrid.info.width = map.getSizeX();
    occupancyGrid.info.height = map.getSizeX();
    occupancyGrid.info.origin.orientation.x = 0.0;
    occupancyGrid.info.origin.orientation.y = 0.0;
    occupancyGrid.info.origin.orientation.z = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;

    occupancyGrid.data.resize(map.getNodeCount());
    const float cellMin = 0;
    const float cellMax = 100;
    const float cellRange = cellMax - cellMin;
    mace::maps::GridMapIterator it(&map);
    occupancyGrid.data[0] = 100;
    for(;!it.isPastEnd();++it)
    {
        const mace::maps::OctomapWrapper::OccupiedResult* ptr = map.getCellByIndex(*it);
        switch(*ptr)
        {
        case mace::maps::OctomapWrapper::OccupiedResult::OCCUPIED:
        {
            occupancyGrid.data[*it] = 100;
            break;
        }
        case mace::maps::OctomapWrapper::OccupiedResult::NO_DATA:
        {
            occupancyGrid.data[*it] = 0;
            break;
        }
        default:
        {
            occupancyGrid.data[*it] = -1;
            break;
        }
        }
    }
    compressedMapPub.publish(occupancyGrid);
}

void ModuleROS::NewlyFoundPath(const std::vector<mace::state_space::StatePtr> &path)
{
    geometry_msgs::Point startPoint;
    geometry_msgs::Point endPoint;

    for(int i = 1; i < path.size();i ++)
    {
        std::cout<<"X: "<<path[i]->as<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<path[i]->as<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;

        mace::pose::CartesianPosition_2D begin(*path.at(i-1)->as<mace::pose::CartesianPosition_2D>());
        startPoint.x = begin.getXPosition();
        startPoint.y = begin.getYPosition();

        mace::pose::CartesianPosition_2D end(*path.at(i)->as<mace::pose::CartesianPosition_2D>());
        endPoint.x = end.getXPosition();
        endPoint.y = end.getYPosition();

        path_list.points.push_back(startPoint);
        path_list.points.push_back(endPoint);
    }
    markerPub.publish(path_list);
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
    compressedMapPub = nh.advertise<nav_msgs::OccupancyGrid>("compressedMap",2);
    // END TESTING
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker",0);
    // %Tag(MARKER_INIT)%
            points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = path_list.header.frame_id = "/map";
            points.header.stamp = line_strip.header.stamp = line_list.header.stamp = path_list.header.stamp = ros::Time::now();
            points.ns = line_strip.ns = line_list.ns =  path_list.ns = "points_and_lines";
            points.action = line_strip.action = line_list.action = path_list.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = path_list.pose.orientation.w = 1.0;
        // %EndTag(MARKER_INIT)%

        // %Tag(ID)%
            points.id = 0;
            line_strip.id = 1;
            line_list.id = 2;
            path_list.id = 3;
        // %EndTag(ID)%

        // %Tag(TYPE)%
            points.type = visualization_msgs::Marker::POINTS;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            path_list.type = visualization_msgs::Marker::LINE_LIST;

        // %EndTag(TYPE)%

        // %Tag(SCALE)%
            // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.1;
            points.scale.y = 0.1;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_strip.scale.x = 0.05;
            line_list.scale.x = 0.05;
            path_list.scale.x = 0.05;
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
            // Path list is blue
            path_list.color.b= 1.0;
            path_list.color.a = 1.0;
        // %EndTag(COLOR)%

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

    // TODO: Send converted point cloud to MACE so path planning can take over.

    std::cout << "Converted PC..." << std::endl;
    std::cout << octoPointCloud.size() << std::endl;

    // TESTING OCTOMAP VISUALIZATION:
    // TODO: Publish PointCloud2 on correct topic for RViz visualization with octomap plugins
    //      - This is to avoid remapping every vehicle's point cloud topics to the cloudIn topic required by octomap server
    //          - Ideally, for the entire octomap, we don't care about which vehicle its coming from. They should be updating the
    //              same octomap

    //    cloudInPub.publish(msg);

    //
}
void ModuleROS::renderOccupancyMap()
{
    octomap::OcTree tree;
    visualization_msgs::MarkerArray occupiedVoxels;
    occupiedVoxels.markers.resize(tree.getTreeDepth() + 1);
    for (octomap::OcTree::iterator it = tree->begin(tree.getTreeDepth()), end = tree->end(); it != end; ++it)
    {
        if(tree.isNodeOccupied(*it))
        {
            unsigned int leafIndex = it.getDepth();

            double size = it.getSize();

            double minX, minY, minZ, maxX, maxY, maxZ;

            tree.getMetricMin(minX, minY, minZ);
            tree.getMetricMax(maxX, maxY, maxZ);
            geometry_msgs::Point cube;
            cube.x = it.getX();
            cube.y = it.getY();
            cube.z = it.getZ();
            double height = (1-std::min(std::max((it.getZ() - minZ)/(maxZ - minZ),0),1.0));
            occupiedVoxels.markers[idx].points.push_back(cube);
            occupiedVoxels.markers[idx].colors.push_back(generateColorHeight(height));

        }
    }
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

std_msgs::ColorRGBA ModuleROS::generateColorHeight(const double height)
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    height -= floor(height);
    height *= 6;
    int i;
    double m, n, f;

    i = floor(height);
    f = height - i;
    if (!(i & 1))
      f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
      case 6:
      case 0:
        color.r = v; color.g = n; color.b = m;
        break;
      case 1:
        color.r = n; color.g = v; color.b = m;
        break;
      case 2:
        color.r = m; color.g = v; color.b = n;
        break;
      case 3:
        color.r = m; color.g = n; color.b = v;
        break;
      case 4:
        color.r = n; color.g = m; color.b = v;
        break;
      case 5:
        color.r = v; color.g = m; color.b = n;
        break;
      default:
        color.r = 1; color.g = 0.5; color.b = 0.5;
        break;
    }

    return color;
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
