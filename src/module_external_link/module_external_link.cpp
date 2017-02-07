#include "module_external_link.h"


ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_SensorFootprintDataTopic("sensorFootprint"),m_MissionDataTopic("externalMission")
{
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleExternalLink::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

}

void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

    std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
    //MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* newWP = new MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>();
    newWP->position.setPosition(35.7470021,-78.8395026,0.0);

    //std::shared_ptr<MissionTopic::MissionItemTopic> newMissionItem = std::make_shared<MissionTopic::MissionItemTopic>(MissionTopic::MissionType::GUIDED);
    //newMissionItem->setMissionItem(newWP);
    //newMissionItem->setVehicleID(1);
    std::shared_ptr<MissionItem::MissionList> newMissionList = std::make_shared<MissionItem::MissionList>();
    newMissionList->insertMissionItem(newWP);
    newMissionList->insertMissionItem(newWP);

    std::shared_ptr<MissionTopic::MissionListTopic> newMissionListTopic = std::make_shared<MissionTopic::MissionListTopic>(MissionTopic::MissionType::MISSION);
    newMissionListTopic->setVehicleID(1);
    newMissionListTopic->setMissionList(newMissionList);

    MaceCore::TopicDatagram topicDatagram;
    m_MissionDataTopic.SetComponent(newMissionListTopic, topicDatagram);

    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(m_MissionDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
    });

    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }

//            if(componentsUpdated.at(i) == DataVehicleArdupilot::VehicleFlightMode::Name()) {
//                std::shared_ptr<DataVehicleArdupilot::VehicleFlightMode> component = std::make_shared<DataVehicleArdupilot::VehicleFlightMode>();
//                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//            }
//            if(componentsUpdated.at(i) == DataVehicleArdupilot::VehicleOperatingStatus::Name()) {
//                std::shared_ptr<DataVehicleArdupilot::VehicleOperatingStatus> component = std::make_shared<DataVehicleArdupilot::VehicleOperatingStatus>();
//                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Local::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
                m_SensorFootprintDataTopic.GetComponent(newSensorV, read_topicDatagram);
            }
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Global>("TestM");
                m_SensorFootprintDataTopic.GetComponent(newSensorV, read_topicDatagram);
            }
        }
    }
}

void ModuleExternalLink::NewlyAvailableVehicle(const int &vehicleID)
{

}

