#include <module_ROS.h>

#include "mace_core/module_factory.h"

#include <map>
#include <string>
#include <iostream>

#include <limits>

#ifdef ROS_EXISTS
#include <geometry_msgs/Twist.h>
#endif

ModuleROS::ModuleROS() :
    MaceCore::IModuleCommandROS(),
    m_PlanningStateTopic("planningState"),
    m_VehicleDataTopic("vehicleData")
{
    // TESTING:
    counter = 0;
//    degree = M_PI/180;
    tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
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

//!
//! \brief start Start ROS loop
//!
void ModuleROS::start() {
#ifdef ROS_EXISTS
    // TODO: Call MACE core getAttachedVehicleID (or whatever we call it)
    //          -Basically to handle if we miss the newly available vehicle topic

    this->setupROS();

    // Start timer:
    m_timer = std::make_shared<ROSTimer>([=]()
    {
        ros::spinOnce();
        counter++;

        if(counter%10 == 0) {
//            std::cout << "**** **** **** Fire publisher..." << std::endl;
            DataState::StateLocalPosition pos;
            publishVehiclePosition(counter, pos);
        }
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

                // Write Attitude data to the GUI:
                //updateAttitudeData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateLocalPositionTopic> component = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Position data to the GUI:
                //updatePositionData(senderID, component);
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
    // Set vehicle ID:
    m_vehicleID = vehicleID;
}

////! ========================================================================
////! ======================  ROS Specific functions:  =======================
////! ========================================================================
#ifdef ROS_EXISTS
//!
//! \brief setupROS Setup ROS subscribers, publishers, and node handler
//!
void ModuleROS::setupROS() {
    ros::NodeHandle nh;
    // Subscribers
    laserSub = nh.subscribe("basic_quadrotor/scan", 500, &ModuleROS::newLaserScan, this);
    pointCloudSub = nh.subscribe <sensor_msgs::PointCloud2> ("basic_quadrotor/kinect/depth/points", 1000, &ModuleROS::newPointCloud, this);

    // Publishers
    velocityPub = nh.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1000);
//    rosservice call ('set_model', 'model_name', (x,y,z));

    m_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    m_transform.setOrigin(tf::Vector3(0.0,0.0,1.0));
    m_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    m_broadcaster.sendTransform(tf::StampedTransform(m_transform,ros::Time::now(),"world","basic_quadrotor/base_link"));

    m_modelState.model_name = (std::string)"basic_quadrotor";
    m_modelState.reference_frame = (std::string)"world";

    ros::spinOnce();
}

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

void ModuleROS::newPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    double distance = std::numeric_limits<double>::max();

    int xPt = 0;
    int yPt = 0;
    geometry_msgs::Point point;
    pixelTo3DPoint(msg, xPt,yPt, point);

    std::cout << "Pixel at (" << point.x << ", " << point.y << ", " << point.z << ") from point cloud pixel: (" << xPt << ", " << yPt << ")" << std::endl;
}

void ModuleROS::pixelTo3DPoint(const sensor_msgs::PointCloud2::ConstPtr& pCloud, const int u, const int v, geometry_msgs::Point &p) {
  // get width and height of 2D point cloud data
  int width = pCloud->width;
  int height = pCloud->height;

  std::cout << "Point cloud width: " << width << std::endl;
  std::cout << "Point cloud height: " << height << std::endl;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud->row_step + u*pCloud->point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud->fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

  p.x = X;
  p.y = Y;
  p.z = Z;

}

void ModuleROS::publishVehiclePosition(const int &vehicleID, const DataState::StateLocalPosition &localPos) {
    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    // robot state
//    const double degree = M_PI/180;
//    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    geometry_msgs::Point robotPosition;
//    robotPosition.x  = cos(angle)*2;
//    robotPosition.y = sin(angle)*2;
//    robotPosition.z = 0.75;
    robotPosition.x  = 0;
    robotPosition.y = 1;
    robotPosition.z = 0.3;

    geometry_msgs::Quaternion attitude;
    attitude.x = 0.0;
    attitude.y = 0.0;
    attitude.z = 0.0;
    attitude.w = 1.0;
    attitude = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    geometry_msgs::Pose pose;
    pose.position = robotPosition;
    pose.orientation = attitude;
    m_modelState.pose = pose;
    m_transform.setOrigin(tf::Vector3(robotPosition.x,robotPosition.y,robotPosition.z));
    m_transform.setRotation(tf::Quaternion(attitude.x,attitude.y,attitude.z,attitude.w));
    m_broadcaster.sendTransform(tf::StampedTransform(m_transform,ros::Time::now(),"world","basic_quadrotor/base_link"));

    m_srv.request.model_state = m_modelState;
    if(m_client.call(m_srv))
    {
        //this means it was a success
    }
    else{
        //this means it was not a success
    }
    // Create new robot state
//    tilt += tinc;
//    if (tilt<-.5 || tilt>0) tinc *= -1;
//    height += hinc;
//    if (height>.2 || height<0) hinc *= -1;
//    swivel += degree;
//    angle += degree/4;

    tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

//    std::cout << " ========= ROS FIRE ======== " << std::endl;

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

void ModuleROS::updatePositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &component)
{
    std::cout << "Update position data" << std::endl;

    double lat = component->getX();
    double lng = component->getY();
    double alt = component->getZ();
}

void ModuleROS::updateAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component)
{
    std::cout << "Update attitude data" << std::endl;

    double roll = component->roll * (180/M_PI);
    double pitch = component->pitch * (180/M_PI);
    double yaw = (component->yaw * (180/M_PI) < 0) ? (component->yaw * (180/M_PI) + 360) : (component->yaw * (180/M_PI));
}

#endif
