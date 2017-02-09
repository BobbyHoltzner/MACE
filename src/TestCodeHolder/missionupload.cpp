/*
if(executedOnce == false){

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
        ptr->RequestCurrentVehicleMission(this,1);
    });

executedOnce = true;
std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
//MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* newWP = new MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>();
newWP->position.setPosition(35.7470021,-78.8395026,0.0);
newWP->setVehicleID(senderID);

std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP1 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
//MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* newWP = new MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>();
newWP1->position.setPosition(35.7463033,-78.8386631,0.0);
newWP1->setVehicleID(senderID);

std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP2 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
//MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* newWP = new MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>();
newWP2->position.setPosition(35.7459724,-78.8390923,0.0);
newWP2->setVehicleID(senderID);

std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP3 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
//MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>* newWP = new MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>();
newWP3->position.setPosition(35.7466538,-78.8399184,0.0);
newWP3->setVehicleID(senderID);


MissionItem::MissionList newMissionList;
newMissionList.insertMissionItem(newWP);
newMissionList.insertMissionItem(newWP1);
newMissionList.insertMissionItem(newWP2);
newMissionList.insertMissionItem(newWP3);

std::shared_ptr<MissionTopic::MissionListTopic> newMissionListTopic = std::make_shared<MissionTopic::MissionListTopic>(MissionTopic::MissionType::MISSION);
newMissionListTopic->setVehicleID(1);
newMissionListTopic->setMissionList(newMissionList);

MaceCore::TopicDatagram topicDatagram;
m_MissionDataTopic.SetComponent(newMissionListTopic, topicDatagram);

ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
    ptr->NewTopicDataValues(m_MissionDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
});

}


*/
