#ifndef GENERICMSGDEF_MAVLINK_H
#define GENERICMSGDEF_MAVLINK_H

#include "mavlink.h"

#include "mace_core/vehicle_message.h"
#include "mace_core/vehicle_object.h"


template<class T> class GenericMsgDef_MAVLINK: public AbstractVehicleMessage
{
public:
    GenericMsgDef_MAVLINK(const int &sendersID, const T &msgData){
        std::cout<<"The msg object is being constructed with an ID of: "<<sendersID<<std::endl;
        m_VehicleID = sendersID;
        this->messageData = msgData;
    }

    T getMessageData(){
        return this->messageData;
    }

    void setMessageData(const T &msgData){
        this->messageData = msgData;
    }

    virtual std::string getDescription() const{
        std::string rtnString = "This message contains MAVLINK data";
        return rtnString;
    }

    virtual std::string getMessageType()const{
        return "MAVLINK messaging";
    }

private:
    T messageData;
};

#endif // GENERICMSGDEF_MAVLINK_H
