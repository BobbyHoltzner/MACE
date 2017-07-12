#ifndef MISSION_CONTROLLER_MAVLINK_H
#define MISSION_CONTROLLER_MAVLINK_H

#include <iostream>
#include <QDate>

#include "mavlink.h"

#include "data/threadmanager.h"
#include "data/timer.h"

#include <callback_interface_data_mavlink.h>

typedef void(*CallbackFunctionPtr_MisCount)(void*, mavlink_message_t &);


class MissionController_Interface
{
public:
    virtual void cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count) = 0;
    virtual void cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item) = 0;
    virtual void cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &request) = 0;
};

namespace DataInterface_MAVLINK {

class MissionController_MAVLINK : public Thread
{

private:
    enum commsState{
        NEUTRAL,
        TRANSMITTING,
        RECEIVING
    };


public:
    MissionController_MAVLINK();

    ~MissionController_MAVLINK() {
        std::cout << "Destructor on the mavlink mission controller" << std::endl;
        mToExit = true;
    }

    void run();

    void forceCallback();

    void requestMission();

    void transmitMission();

    void recievedMissionItem(const mavlink_mission_item_t &missionItem);

    void receivedMissionCount(const mavlink_mission_count_t &missionCount);

    void receivedMissionACK(const mavlink_mission_ack_t &missionAck);

    void connectCallback(MissionController_Interface *cb)
    {
        m_CB = cb;
    }

private:
    Timer mTimer;
    bool mToExit;
    int currentRetry;
    int maxRetries;
    int responseTimeout;
private:
    MissionController_Interface *m_CB;

    commsState currentCommsState;
    //commsObject *prevTransmit;
};


template <class T>
class commsObject
{
    enum commsItem{
        ITEMMISSION,
        ITEMCOUNT,
        ITEMLIST
    };

    commsObject(const commsItem &itemType, const T &itemObj)
    {
        this->type = itemType;
        this->obj = itemObj;
    }

    ~commsObject()
    {

    }

    commsItem type;
    T obj;
};


} //end of namespace DataInterface_MAVLINK

#endif // MISSION_CONTROLLER_MAVLINK_H
