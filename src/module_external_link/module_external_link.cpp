#include "module_external_link.h"

#include "module_generic_MAVLINK/controllers/controller_mavlink_generic_set.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

void FinishedMessage(const bool completed, const uint8_t finishCode)
{
    if(completed == false)
    {
        printf("Controller timed out sending message, gave up sending message\n");
    }
    else {
        printf("Controller Received Final ACK with code of %d\n", finishCode);
    }
}

template <typename T>
T* Helper_CreateAndSetUp(ModuleExternalLink* obj, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, uint8_t chan)
{
    T* newController = new T(obj, queue, chan);
    newController->setLambda_DataReceived([obj](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command){obj->ReceivedCommand(sender, command);});
    newController->setLambda_Finished(FinishedMessage);
    return newController;
}


ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission"),
    associatedSystemID(254), airborneInstance(true),
    m_HeartbeatController(NULL)
{

    Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue = new Controllers::MessageModuleTransmissionQueue<mace_message_t>(2000, 3);





    auto controller_SystemMode2 = new ModuleGenericMavlink::MAVLINKControllers::GenericControllerSetRequestRespond<
            mace_message_t,
            MaceCore::TopicDatagram,
            mace_command_system_mode_t,
            mace_system_mode_ack_t,
            MACE_MSG_ID_COMMAND_SYSTEM_MODE,
            MACE_MSG_ID_SYSTEM_MODE_ACK
            >
            (this, queue, m_LinkChan,
                mace_msg_command_system_mode_encode_chan,
                mace_msg_command_system_mode_decode,
                mace_msg_system_mode_ack_encode_chan,
                mace_msg_system_mode_ack_decode
            );
    controller_SystemMode2->FillSetObject([this](const MaceCore::TopicDatagram &commandItem, const MaceCore::ModuleCharacteristic &target, mace_command_system_mode_t &cmd)
    {
        std::shared_ptr<Data::TopicComponents::String> component = std::make_shared<Data::TopicComponents::String>();
        this->m_VehicleTopics.m_CommandSystemMode.GetComponent(commandItem, component);

        strcpy(cmd.mode, component->Str().c_str());
        cmd.target_system = target.ID;
    });
    controller_SystemMode2->FillDataAndAck([this](const mace_command_system_mode_t &msg, std::shared_ptr<MaceCore::TopicDatagram> &data, mace_system_mode_ack_t &ack){

        std::shared_ptr<Data::TopicComponents::String> component = std::make_shared<Data::TopicComponents::String>(std::string(msg.mode));

        data = std::make_shared<MaceCore::TopicDatagram>();
        this->m_VehicleTopics.m_CommandSystemMode.SetComponent(component, *data);

        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
    });
    controller_SystemMode2->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<MaceCore::TopicDatagram> &command){
        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, this->m_VehicleTopics.m_CommandSystemMode.Name(), sender, MaceCore::TIME(), *command);
        });
    });
    m_TopicToControllers.insert({this->m_VehicleTopics.m_CommandSystemMode.Name(), controller_SystemMode2});





    //auto externalLink = new ExternalLink::MissionController(this, queue, m_LinkChan);
    //externalLink->setLambda_DataReceived([this](const MissionKey &key, const std::shared_ptr<MissionList> &list){this->ReceivedMission(*list);});
    //externalLink->setLambda_FetchDataFromKey([this](const OptionalParameter<MissionKey> &key){return this->FetchMissionFromKey(key);});
    //externalLink->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllMissionFromModule(module);});
    //m_Controllers.Add(externalLink);


    m_Controllers.Add(Helper_CreateAndSetUp<Controllers::CommandLand<mace_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<Controllers::CommandTakeoff<mace_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<Controllers::CommandARM<mace_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<Controllers::CommandRTL<mace_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<Controllers::CommandMissionItem<mace_message_t>>(this, queue, m_LinkChan));


    /*
    auto controller_SystemMode = new Controllers::ControllerSystemMode<mace_message_t>(this, queue, m_LinkChan);
    controller_SystemMode->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command){this->ReceivedCommand(sender, command);});
    controller_SystemMode->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(controller_SystemMode);
    */


    auto homeController = new Controllers::ControllerHome<mace_message_t>(this, queue, m_LinkChan);
    homeController->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &key, const std::shared_ptr<CommandItem::SpatialHome> &home){this->ReceivedHome(*home);});
    homeController->setLambda_FetchDataFromKey([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &key){return this->FetchHomeFromKey(key);});
    homeController->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllHomeFromModule(module);});
    homeController->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(homeController);


    auto missionController = new Controllers::ControllerMission<mace_message_t>(this, queue, m_LinkChan);
    missionController->setLambda_DataReceived([this](const MissionKey &key, const std::shared_ptr<MissionList> &list){this->ReceivedMission(*list);});
    missionController->setLambda_FetchDataFromKey([this](const OptionalParameter<MissionKey> &key){return this->FetchMissionFromKey(key);});
    missionController->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllMissionFromModule(module);});
    missionController->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(missionController);


}

void ModuleExternalLink::ReceivedCommand(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command)
{
    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
        ptr->Event_IssueGeneralCommand(this, command);
    });
}

ModuleExternalLink::~ModuleExternalLink()
{

}

std::vector<MaceCore::TopicCharacteristic> ModuleExternalLink::GetEmittedTopics()
{
    std::vector<MaceCore::TopicCharacteristic> topics;

    topics.push_back(this->m_VehicleDataTopic.Characterisic());
    topics.push_back(this->m_MissionDataTopic.Characterisic());
    topics.push_back(this->m_VehicleTopics.m_CommandSystemMode.Characterisic());

    return topics;
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());

    ptr->Subscribe(this, m_VehicleTopics.m_CommandSystemMode.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleExternalLink::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    ConfigureMACEStructure(structure);
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

}



//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ConfigureMACEComms(params);

    m_LinkMarshaler->SpecifyAddedModuleAction(m_LinkName, [this](const char* resourceName, int ID){
        this->ExternalModuleAdded(resourceName, ID);
    });

    m_LinkMarshaler->SpecifyAddedModuleAction(m_LinkName, [this](const char* resourceName, int ID){
        this->ExternalModuleRemoved(resourceName, ID);
    });

    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");

        //m_LinkMarshaler->AddResource(m_LinkName, GROUNDSTATION_STR, associatedSystemID);

        if(airborneInstance == false)
        {

        }
    }

}

//!
//! \brief Event to fire when an external module has been added
//! \param resourceName Name of resource (module) added
//! \param ID ID of module
//!
void ModuleExternalLink::ExternalModuleAdded(const char* resourceName, int ID)
{
    MaceCore::ModuleCharacteristic module;
    module.ID = ID;
    if(strcmp(resourceName, CommsMACE::VEHICLE_STR) == 0)
    {
        module.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
    }
    else if(strcmp(resourceName, CommsMACE::GROUNDSTATION_STR) == 0)
    {
        module.Class = MaceCore::ModuleClasses::GROUND_STATION;
    }
    else {
        printf("%s\n", CommsMACE::GROUNDSTATION_STR);
        throw std::runtime_error("Unknown module class received");
    }

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->ExternalEvent_NewModule(this, module);
    });
}

void ModuleExternalLink::ExternalModuleRemoved(const char* ResourceName, int ID)
{
    throw std::runtime_error("Not Implimented");
}

std::string ModuleExternalLink::createLog(const int &systemID)
{
    loggerCreated = true;
    std::string logname = this->loggingPath + "/ExternalLinkModule" + std::to_string(systemID) + ".txt";
    std::string loggerName = "ExternalLinkModule_" + std::to_string(systemID);
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    mLog = spdlog::basic_logger_mt(logNameArray, logname);
    mLog->set_level(spdlog::level::debug);
    return loggerName;
}

void ModuleExternalLink::TransmitMessage(const mace_message_t &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) const
{
    //broadcast
    if(target.IsSet() == false)
    {
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
    }

    else if(target().Class == MaceCore::ModuleClasses::VEHICLE_COMMS)
    {
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, std::make_tuple<const char*, int>(CommsMACE::VEHICLE_STR, target.Value().ID));
    }
    else if(target().Class == MaceCore::ModuleClasses::GROUND_STATION)
    {
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, std::make_tuple<const char*, int>(CommsMACE::GROUNDSTATION_STR, target.Value().ID));
    }
    else {
        throw std::runtime_error("Unknown target given");
    }
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from the Heartbeat Controller
/// Interface via callback functionality.
///////////////////////////////////////////////////////////////////////////////////////
void ModuleExternalLink::cbiHeartbeatController_transmitCommand(const mace_heartbeat_t &heartbeat)
{
    mace_message_t msg;
    mace_msg_heartbeat_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&heartbeat);
    TransmitMessage(msg);
}


void ModuleExternalLink::ReceivedMission(const MissionItem::MissionList &list)
{
    std::cout<<"The external link module now has received the entire mission."<<std::endl;

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->ExternalEvent_FinishedRXMissionList(this, list);
    });
}


Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn ModuleExternalLink::FetchMissionFromKey(const OptionalParameter<MissionKey> &key)
{
    if(key.IsSet() == false)
    {
        throw std::runtime_error("Key not set in fetchdatafromkey function");
    }

    Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn rtn;

    MissionList list;
    this->getDataObject()->getMissionList(key(), list);
    rtn.push_back(std::make_tuple(key(), list));
    return rtn;
}

Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn ModuleExternalLink::FetchAllMissionFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module)
{
    Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn rtn;


    //Function to fetch missions for given module
    auto func = [this, &rtn](MaceCore::ModuleCharacteristic vehicle)
    {
        //fetch mission
        MissionItem::MissionList currentMission;
        bool exists = this->getDataObject()->getCurrentMission(vehicle.ID, currentMission);

        //if exists append, if it doesn't append empty
        if(exists)
        {
            MissionKey key = currentMission.getMissionKey();
            std::vector<std::tuple<MissionKey, MissionList>> vec = {};
            vec.push_back(std::make_tuple(key, currentMission));
            std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<MissionKey, MissionList>>> tmp = std::make_tuple(vehicle, vec);
            rtn.push_back(tmp);
        }
        else
        {
            rtn.push_back(std::make_tuple(vehicle, std::vector<std::tuple<MissionKey, MissionList>>()));
        }
    };


    //if no module is given then fetch all
    if(module.IsSet() == false || module().ID == 0)
    {
        std::vector<int> vehicles;
        this->getDataObject()->GetLocalVehicles(vehicles);
        for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
            MaceCore::ModuleCharacteristic vehicle;
            vehicle.ID = *it;
            vehicle.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            func(vehicle);
        }
    }
    else {
        func(module());
    }


    return rtn;
}





void ModuleExternalLink::ReceivedHome(const CommandItem::SpatialHome &home)\
{
    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->GVEvents_NewHomePosition(this,home);
    });

    std::shared_ptr<CommandItem::SpatialHome> homePtr = std::make_shared<CommandItem::SpatialHome>(home);
    std::shared_ptr<MissionTopic::MissionHomeTopic> missionTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
    missionTopic->setHome(homePtr);

    MaceCore::TopicDatagram topicDatagram;
    m_MissionDataTopic.SetComponent(missionTopic, topicDatagram);

    MaceCore::ModuleCharacteristic sender;
    sender.ID = home.getOriginatingSystem();
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    //notify listeners of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), sender, MaceCore::TIME(), topicDatagram);
    });
}

Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchKeyReturn ModuleExternalLink::FetchHomeFromKey(const OptionalParameter<MaceCore::ModuleCharacteristic> &key)
{
    if(key.IsSet() == false)
    {
        throw std::runtime_error("Key not set in fetchdatafromkey function");
    }

    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchKeyReturn rtn;

    CommandItem::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(key().ID);
    rtn.push_back(std::make_tuple(key(), home));
    return rtn;
}

Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchModuleReturn ModuleExternalLink::FetchAllHomeFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module)
{
    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchModuleReturn rtn;


    //Function to fetch missions for given module
    auto func = [this, &rtn](MaceCore::ModuleCharacteristic vehicle)
    {
        //fetch mission
        CommandItem::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(vehicle.ID);

        std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> vec = {};
        vec.push_back(std::make_tuple(vehicle, home));
        std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>>> tmp = std::make_tuple(vehicle, vec);
        rtn.push_back(tmp);
    };


    //if no module is given then fetch all
    if(module.IsSet() == false || module().ID == 0)
    {
        std::vector<int> vehicles;
        this->getDataObject()->GetLocalVehicles(vehicles);
        for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
            MaceCore::ModuleCharacteristic vehicle;
            vehicle.ID = *it;
            vehicle.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
            func(vehicle);
        }
    }
    else {
        func(module());
    }


    return rtn;
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleExternalLink::MACEMessage(const std::string &linkName, const mace_message_t &message)
{
    UNUSED(linkName);
    m_Controllers.ForEach<Controllers::IController<mace_message_t>>([message](Controllers::IController<mace_message_t>* ptr) {
       ptr->ReceiveMessage(&message);
    });

    for(auto it = m_TopicToControllers.cbegin() ; it != m_TopicToControllers.cend() ; ++it)
    {
        it->second->ReceiveMessage(&message);
    }

    this->ParseForData(&message);
}



void ModuleExternalLink::HeartbeatInfo(const int &systemID, const mace_heartbeat_t &heartbeatMSG)
{
    if(systemIDMap.find(systemID) == systemIDMap.end())
    {
        //this function should always be called by an external link connected to ground for now
        //KEN this is a hack...but it will function for now
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_UpdateRemoteID(this, systemID);
        });

        std::string loggerName = createLog(systemID);
        systemIDMap.insert({systemID,0});

        //The system has yet to have communicated through this module
        //We therefore have to notify the core that there is a new vehicle

        /*
         * Now handled by ExternalModuleAdded Method
         *
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_NewConstructedVehicle(this, systemID);
        });
        */

        //Request_FullDataSync(systemID);
    }


    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(static_cast<Data::AutopilotType>(heartbeatMSG.autopilot));
    heartbeat.setCompanion((heartbeatMSG.mace_companion>0)? true : false);
    heartbeat.setProtocol(static_cast<Data::CommsProtocol>(heartbeatMSG.protocol));
    heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.mission_state));
    heartbeat.setType(static_cast<Data::SystemType>(heartbeatMSG.type));
    //heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.missionState));
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    PublishVehicleData(systemID,ptrHeartbeat);
}

//!
//! \brief ModuleExternalLink::VehicleCommandMACEACK
//! \param linkName
//! \param systemID
//! \param cmdACK
//!
//Ken Fix This
//void ModuleExternalLink::MACECommandACK(const std::string &linkName, const int &systemID, const mace_command_ack_t &cmdACK)
//{
//    m_CommandController->receivedCommandACK();
//    Data::CommandItemType cmdType = static_cast<Data::CommandItemType>(cmdACK.command);
//    Data::CommandACKType ackType = static_cast<Data::CommandACKType>(cmdACK.result);

//    std::string cmdString = Data::CommandItemTypeToString(cmdType);
//    std::string ackString = Data::CommandACKToString(ackType);

//    std::stringstream ss;
//    ss << "System " << systemID << " " << ackString << " command " << cmdString;
//    std::string newString = ss.str();

//    MaceCore::TopicDatagram topicDatagram;
//    DataGenericItem::DataGenericItem_Text newText;
//    newText.setSeverity(Data::StatusSeverityType::STATUS_INFO);
//    newText.setText(newString);
//    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);

//    m_VehicleDataTopic.SetComponent(ptrStatusText, topicDatagram);
//    //notify listeners of topic
//    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//    });

//}

void ModuleExternalLink::PublishVehicleData(const int &systemID, const std::shared_ptr<Data::ITopicComponentDataObject> &component)
{
    //construct datagram
    MaceCore::TopicDatagram topicDatagram;
    m_VehicleDataTopic.SetComponent(component, topicDatagram);

    MaceCore::ModuleCharacteristic sender;
    sender.ID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    //notify listeners of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), sender, MaceCore::TIME(), topicDatagram);
    });
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from IModuleCommandExternalLink.
//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    if(sender.IsSet() == false) {
        throw std::runtime_error("no sender given");
    }



    //This first segment causes all the topics to be republished
    mace_message_t msg;
    mace_vehicle_sync_t sync;
    sync.target_system = targetSystem;
    mace_msg_vehicle_sync_encode_chan(sender().ID, (int)sender().Class, m_LinkChan, &msg, &sync);

    if(targetSystem == 0)
    {
        TransmitMessage(msg);
    }
    else {
        MaceCore::ModuleCharacteristic target;
        target.ID = targetSystem;
        target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        TransmitMessage(msg, target);

        m_Controllers.Retreive<Controllers::ControllerHome<mace_message_t>>()->Request(sender(), target);

        m_Controllers.Retreive<Controllers::ControllerMission<mace_message_t>>()->RequestCurrentMission(sender(), target);
    }
}

void ModuleExternalLink::Command_SystemArm(const CommandItem::ActionArm &systemArm, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = systemArm.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(target.ID == 0)
    {
        m_Controllers.Retreive<Controllers::CommandARM<mace_message_t>>()->Broadcast(systemArm, sender());
    }
    else {
        m_Controllers.Retreive<Controllers::CommandARM<mace_message_t>>()->Send(systemArm, sender(), target);
    }
}

void ModuleExternalLink::Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout<<"We are trying to change the mode in external link"<<std::endl;
    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleMode.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;
    m_Controllers.Retreive<Controllers::ControllerSystemMode<mace_message_t>>()->Send(vehicleMode, sender(), target);
}

void ModuleExternalLink::Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &vehicleTakeoff, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleTakeoff.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(target.ID == 0)
    {
        m_Controllers.Retreive<Controllers::CommandTakeoff<mace_message_t>>()->Broadcast(vehicleTakeoff, sender());
    }
    else {
        m_Controllers.Retreive<Controllers::CommandTakeoff<mace_message_t>>()->Send(vehicleTakeoff, sender(), target);
    }
}

void ModuleExternalLink::Command_Land(const CommandItem::SpatialLand &vehicleLand, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleLand.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(target.ID == 0)
    {
        m_Controllers.Retreive<Controllers::CommandLand<mace_message_t>>()->Broadcast(vehicleLand, sender());
    }
    else {
        m_Controllers.Retreive<Controllers::CommandLand<mace_message_t>>()->Send(vehicleLand, sender(), target);
    }
}

void ModuleExternalLink::Command_ReturnToLaunch(const CommandItem::SpatialRTL &vehicleRTL, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleRTL.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(target.ID == 0)
    {
        m_Controllers.Retreive<Controllers::CommandRTL<mace_message_t>>()->Broadcast(vehicleRTL, sender());
    }
    else {
        m_Controllers.Retreive<Controllers::CommandRTL<mace_message_t>>()->Send(vehicleRTL, sender(), target);
    }
}

void ModuleExternalLink::Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = command.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(target.ID == 0)
    {
        m_Controllers.Retreive<Controllers::CommandMissionItem<mace_message_t>>()->Broadcast(command, sender());
    }
    else {
        m_Controllers.Retreive<Controllers::CommandMissionItem<mace_message_t>>()->Send(command, sender(), target);
    }
}

void ModuleExternalLink::Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{
    UNUSED(command);
}

void ModuleExternalLink::Command_EmitHeartbeat(const CommandItem::SpatialTakeoff &heartbeat)
{

}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_GetHomePosition(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout<<"External link module saw a get home position request"<<std::endl;

    MaceCore::ModuleCharacteristic target;
    target.ID = vehicleID;
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;


    m_Controllers.Retreive<Controllers::ControllerHome<mace_message_t>>()->Request(sender(), target);
}

void ModuleExternalLink::Command_SetHomePosition(const CommandItem::SpatialHome &systemHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target;
    target.ID = systemHome.getTargetSystem();
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;


    m_Controllers.Retreive<Controllers::ControllerHome<mace_message_t>>()->Send(systemHome, sender(), target);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
    int targetSystem = missionList.getVehicleID();

    MaceCore::ModuleCharacteristic target;
    target.ID = targetSystem;
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    if(status.state == MissionItem::MissionList::COMPLETE)
    {
        throw std::runtime_error("Not Implimented");
        //m_Controllers.Retreive<Controllers::ControllerMission<mace_message_t>>()->UploadMission(missionList, target);
    }
}

void ModuleExternalLink::Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    if(sender.IsSet() == false) {
        throw std::runtime_error("no sender given");
    }

    MaceCore::ModuleCharacteristic target;
    target.ID = key.m_systemID;
    target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    UNUSED(key);
    m_Controllers.Retreive<Controllers::ControllerMission<mace_message_t>>()->RequestMission(key, sender(), target);
}

void ModuleExternalLink::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    //    mace_message_t msg;
    //    mace_mission_set_current_t request;
    //    request.
    //    request.mission_creator = key.m_creatorID;
    //    request.mission_id = key.m_missionID;
    //    request.mission_type = (uint8_t)key.m_missionType;
    //    request.target_system = key.m_systemID;
    //    mace_msg_mission_set_current_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
    //    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    mace_new_onboard_mission_t mission;
    mission.mission_creator = key.m_creatorID;
    mission.mission_id = key.m_missionID;
    mission.mission_type = (uint8_t)key.m_missionType;
    mission.mission_system = key.m_systemID;
    mission.mission_state = (uint8_t)key.m_missionState;

    mace_message_t msg;
    mace_msg_new_onboard_mission_encode_chan(sender().ID, (int)sender().Class, m_LinkChan,&msg,&mission);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    m_Controllers.Retreive<Controllers::ControllerHome<mace_message_t>>()->Broadcast(home, sender());
}

//Ken FIX THIS: I dont know if I should pass the pertinent systemID with the key
void ModuleExternalLink::NewlyAvailableMissionExeState(const MissionItem::MissionKey &key)
{
    MissionItem::MissionList list;
    bool validity = this->getDataObject()->getMissionList(key,list);
    if(validity)
    {
        mace_mission_exe_state_t state;
        Data::MissionExecutionState missionState = list.getMissionExeState();
        state.mission_creator = key.m_creatorID;
        state.mission_id = key.m_missionID;
        state.mission_state = (uint8_t)missionState;
        state.mission_system = key.m_systemID;
        state.mission_type = (uint8_t)key.m_missionType;

        mace_message_t msg;
        mace_msg_mission_exe_state_encode_chan(key.m_systemID,0,m_LinkChan,&msg,&state);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
    }
}

void ModuleExternalLink::NewlyAvailableModule(const MaceCore::ModuleCharacteristic &module)
{
    if(module.Class == MaceCore::ModuleClasses::VEHICLE_COMMS)
    {
        m_LinkMarshaler->AddResource(m_LinkName, CommsMACE::VEHICLE_STR, module.ID);
    }
    if(module.Class == MaceCore::ModuleClasses::GROUND_STATION)
    {
        m_LinkMarshaler->AddResource(m_LinkName, CommsMACE::GROUNDSTATION_STR, module.ID);
    }

    if(airborneInstance)
    {
        this->associatedSystemID = module.ID;

        //this function should always be called by an external link connected to ground for now
        //KEN this is a hack...but it will function for now
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_UpdateRemoteID(this, 254);
        });
    }
}

void ModuleExternalLink::ReceivedMissionACK(const MissionItem::MissionACK &ack)
{
    mace_mission_ack_t missionACK;
    MissionItem::MissionKey key = ack.getMissionKey();
    missionACK.cur_mission_state = (uint8_t)ack.getNewMissionState();
    missionACK.mission_creator = key.m_creatorID;
    missionACK.mission_id = key.m_missionID;
    missionACK.mission_result = (uint8_t)ack.getMissionResult();
    missionACK.mission_system = key.m_systemID;
    missionACK.mission_type = (uint8_t)key.m_missionType;
    missionACK.prev_mission_state = (uint8_t)key.m_missionState;

    mace_message_t msg;
    mace_msg_mission_ack_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&missionACK);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
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
void ModuleExternalLink::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{

    if(this->m_TopicToControllers.find(topicName) == m_TopicToControllers.cend())
    {
        throw std::runtime_error("Attempting to send a topic that external link has no knowledge of");
    }

    Controllers::IController<mace_message_t> *controller = m_TopicToControllers.at(topicName);
    if(target.IsSet())
    {
        if(controller->ContainsAction(Controllers::Actions::SEND) == false)
        {
            throw std::runtime_error("Attempting to send a topic to a controller that has no send action");
        }
        Controllers::IActionSend<MaceCore::TopicDatagram> *sendAction = dynamic_cast<Controllers::IActionSend<MaceCore::TopicDatagram>*>(controller);
        sendAction->Send(data, sender, target());
    }
    else {
        if(controller->ContainsAction(Controllers::Actions::BROADCAST) == false)
        {
            throw std::runtime_error("Attempting to broadcast a topic to a controller that has no broadcast action");
        }
        Controllers::IActionBroadcast<MaceCore::TopicDatagram> *sendAction = dynamic_cast<Controllers::IActionBroadcast<MaceCore::TopicDatagram>*>(controller);
        sendAction->Broadcast(data, sender);
    }
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
void ModuleExternalLink::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    int senderID = sender.ID;
    if(airborneInstance)
    {

        //In relevance to the external link module, the module when receiving a new topic should pack that up for transmission
        //to other instances of MACE
        //example read of vehicle data
        if(topicName == m_VehicleDataTopic.Name())
        {
            //get latest datagram from mace_data
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
            //example of how to get data and parse through the components that were updated
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()) {
                    if(!loggerCreated)
                        createLog(senderID);
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg_Full(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Text::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
            }
        } //end of vehicle data topic
        else if(topicName == m_MissionDataTopic.Name())
        {
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                //get latest datagram from mace_data
                MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);
                if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                    m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == MissionTopic::VehicleTargetTopic::Name()) {
                    std::shared_ptr<MissionTopic::VehicleTargetTopic> component = std::make_shared<MissionTopic::VehicleTargetTopic>();
                    m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
            }
        }
    }//end of if airborne instance
}

