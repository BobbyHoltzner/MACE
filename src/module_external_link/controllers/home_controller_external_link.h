#ifndef HOME_CONTROLLER_EXTERNAL_LINK_H
#define HOME_CONTROLLER_EXTERNAL_LINK_H

#include <iostream>
#include <QDate>
#include "spdlog/spdlog.h"

#include "mace.h"

#include "data/controller_comms_state.h"
#include "data/threadmanager.h"
#include "data/timer.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

#include "mace_core/module_characteristics.h"

#include "generic_mace_controller.h"


namespace ExternalLink {

class HomeController_ExternalLink : public GenericMACEController<
        TransmitQueueWithKeys<MACETransmissionQueue, KeyWithInt<MaceCore::ModuleCharacteristic>>,
        DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>
        >
{

private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_HomeRequestedFrom;

public:
    HomeController_ExternalLink(const MACEControllerInterface* cb, MACETransmissionQueue * queue, int linkChan) :
        GenericMACEController(cb, queue, linkChan)
    {


        ///////////////////////////////////////////////////////////////
        //// DOWNLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddTriggeredLogic<MACE_MSG_ID_SET_HOME_POSITION, mace_set_home_position_t >( mace_msg_set_home_position_decode,
                [this](const mace_set_home_position_t  &msg, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    CommandItem::SpatialHome systemHome;
                    systemHome.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
                    systemHome.position->setX(msg.latitude / pow(10,7));
                    systemHome.position->setY(msg.longitude / pow(10,7));
                    systemHome.position->setZ(msg.altitude / pow(10,3));
                    systemHome.setTargetSystem(msg.target_system);
                    systemHome.setOriginatingSystem(msg.target_system);

                    std::cout<<"Home controller: received an onsolicieted home position"<<std::endl;
                    //onDataReceived(sender, systemHome);

                    //No need to send an ACK when a request wasn't made
                }
        );

        AddResponseLogic<MACE_MSG_ID_HOME_POSITION, MaceCore::ModuleCharacteristic, mace_home_position_t  >( mace_msg_home_position_decode,
                [](const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                   UNUSED(msg);
                   return sender;
                },
                [this](const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &key, const MaceCore::ModuleCharacteristic &sender){

                    std::cout<<"Home controller: received the home position"<<std::endl;
                    MaceCore::ModuleCharacteristic target = sender;

                    if(m_HomeRequestedFrom.find(sender) == m_HomeRequestedFrom.cend())
                    {
                        std::cout<<"Home controller: send Ack"<<std::endl;
                        mace_home_position_ack_t ack;
                        EncodeMessage(mace_msg_home_position_ack_encode_chan, ack, key, target);
                    }
                    else {
                        MaceCore::ModuleCharacteristic target = sender;



                        //mLog->info("Mission Controller received system home item item.");

                        //This is the home position item associated with the vehicle
                        CommandItem::SpatialHome newHome;
                        newHome.position->setX(msg.latitude / pow(10,7));
                        newHome.position->setY(msg.longitude / pow(10,7));
                        newHome.position->setZ(msg.altitude / pow(10,7));
                        newHome.setOriginatingSystem(target.ID);
                        newHome.setTargetSystem(target.ID);

                        //onDataReceived(sender, newHome);

                        std::cout << "Home Controller: Send Ack" << std::endl;
                        mace_home_position_ack_t ack;
                        EncodeMessage(mace_msg_home_position_ack_encode_chan, ack, m_HomeRequestedFrom.at(target), target);
                    }
                }
        );



        ///////////////////////////////////////////////////////////////
        //// UPLOAD BEHAVIOR
        ///////////////////////////////////////////////////////////////

        AddResponseLogic<MACE_MSG_ID_HOME_POSITION_ACK, MaceCore::ModuleCharacteristic, mace_home_position_ack_t >( mace_msg_home_position_ack_decode,
                [](const mace_home_position_ack_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                  UNUSED(msg);
                  return sender;
                },
                [this](const mace_home_position_ack_t  &msg, MaceCore::ModuleCharacteristic &key, const MaceCore::ModuleCharacteristic &sender){

                    std::cout << "Home Controller: Ack Recieved" << std::endl;

                }
        );

        AddResponseLogic<MACE_MSG_ID_MISSION_REQUEST_HOME, MaceCore::ModuleCharacteristic, mace_mission_request_home_t>( mace_msg_mission_request_home_decode,
                [](const mace_mission_request_home_t &msg, const MaceCore::ModuleCharacteristic &sender)
                {
                    MaceCore::ModuleCharacteristic homeInQuestion;
                    homeInQuestion.ID = msg.target_system;
                    homeInQuestion.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

                    return homeInQuestion;
                },
                [this](const mace_mission_request_home_t &msg, MaceCore::ModuleCharacteristic &key, const MaceCore::ModuleCharacteristic &sender){

                    MaceCore::ModuleCharacteristic target = sender;

                    std::vector<std::tuple<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>> homes;
                    FetchDataFromKey(key, homes);

                    for(auto it = homes.cbegin() ; it != homes.cend() ; ++it)
                    {
                        mace_home_position_t homeMACE;
                        homeMACE.latitude = std::get<1>(*it).position->getX() * pow(10,7);
                        homeMACE.longitude = std::get<1>(*it).position->getY() * pow(10,7);
                        homeMACE.altitude = std::get<1>(*it).position->getZ() * pow(10,3);

                        std::cout << "Home Controller: Send Home Position" << std::endl;
                        QueueTransmission(target, 0, [this, homeMACE, key, target](){
                            EncodeMessage(mace_msg_home_position_encode_chan, homeMACE, key, target);
                        });
                    }

                }
        );
    }


    void requestHome(const MaceCore::ModuleCharacteristic &target, const MaceCore::ModuleCharacteristic &sender)
    {
        if(mLog)
        {
            mLog->info("Mission Controller has seen a request home.");
        }

        mace_mission_request_home_t request;
        request.target_system = target.ID;

        m_HomeRequestedFrom.insert({target, sender});

        std::cout << "Home Controller: Send Home Request" << std::endl;
        QueueTransmission(target, MACE_MSG_ID_HOME_POSITION, [this, request, sender, target](){
            EncodeMessage(mace_msg_mission_request_home_encode_chan, request, sender, target);
        });

    }

    void BroadcastHome(const CommandItem::SpatialHome &home, MaceCore::ModuleCharacteristic sender)
    {
        if(home.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
        {
            mace_set_home_position_t setHome;
            setHome.target_system = home.getTargetSystem();
            setHome.latitude = home.position->getX() * pow(10,7);
            setHome.longitude = home.position->getY()* pow(10,7);
            setHome.altitude = home.position->getZ() * 1000.0;


            std::cout << "Home Controller: Boadcast Home" << std::endl;
            EncodeMessage(mace_msg_set_home_position_encode_chan, setHome, sender);

        }
        else
        {
            throw std::runtime_error("Not Implemented");
        }
    }

    void setHome(const CommandItem::SpatialHome &home, MaceCore::ModuleCharacteristic sender)
    {
        if(home.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
        {
            MaceCore::ModuleCharacteristic target;
            target.ID = home.getTargetSystem();
            target.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

            mace_set_home_position_t setHome;
            setHome.target_system = home.getTargetSystem();
            setHome.latitude = home.position->getX() * pow(10,7);
            setHome.longitude = home.position->getY()* pow(10,7);
            setHome.altitude = home.position->getZ() * 1000.0;


            std::cout << "Home Controller: Send Home Set" << std::endl;
            QueueTransmission(target, -1, [this, setHome, sender, target](){
                EncodeMessage(mace_msg_set_home_position_encode_chan, setHome, sender, target);
            });

        }
        else
        {
            throw std::runtime_error("Not Implemented");
        }
    }

};


} //end of namespace DataInterface_MAVLINK

#endif // HOME_CONTROLLER_EXTERNAL_LINK_H
