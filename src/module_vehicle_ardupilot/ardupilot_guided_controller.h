#ifndef ARDUPILOT_GUIDED_CONTROLLER_H
#define ARDUPILOT_GUIDED_CONTROLLER_H

#include <chrono>
#include <thread>
#include <string>
#include <iostream>

#include "data/mission_state.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_vehicle_ardupilot/components.h"
#include "data_vehicle_ardupilot/vehicle_object_ardupilot.h"

#include "comms/comms_marshaler.h"

#include "ardupilot_mission_state.h"

#include <mavlink.h>


class Thread {
public:
    Thread() :
        mThread(NULL)
    {
        std::cout << "Constructor on Generic Thread" << std::endl;
    }

    virtual ~Thread() {
        std::cout << "Destructor on Generic Thread" << std::endl;
        if(mThread)
        {
            mThread->join();
            delete mThread;
        }
    }

    virtual void run() = 0;

    void start() {
        mThread = new std::thread([this]()
        {
            this->run();
        });
    }
protected:
    std::thread *mThread;
};

class Ardupilot_GuidedController : public Thread
{
public:

    Ardupilot_GuidedController(std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleData, Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan);

    ~Ardupilot_GuidedController() {
        std::cout << "Destructor on guidance controller" << std::endl;
        mToExit = true;
    }

    void initializeMissionSequence();

    void updatedHomePostion(const MissionItem::SpatialHome &homePosition);
    void updatedMission(const MissionItem::MissionList &updatedMission);

    void updateAttitudeTopic(const DataState::StateAttitude &attitude);
    void updateGlobalPositionTopic(const DataState::StateGlobalPosition &globalPosition);


    double distanceToTarget();
    void generateControl(const Data::MissionState &currentState);

    virtual void run();

    //The following are communications objects
private:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> vehicleDataObject;

private:
    //FLAGS for the thread:
    bool mToExit;
    bool executionState;
    bool positionUpdated;
    bool attitudeUpdated;
    bool missionUpdated;

    ArdupilotMissionState vehicleMissionState;

    MissionItem::MissionList m_CurrentMission;
    std::string vehicleMode;

    DataState::StateGlobalPosition currentPosition;
    DataState::StateAttitude currentAttitude;

    MissionItem::SpatialHome m_VehicleHome;

};


#endif // ARDUPILOT_GUIDED_CONTROLLER_H
