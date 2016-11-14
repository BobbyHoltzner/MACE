#ifndef VEHICLEMESSAGE_H
#define VEHICLEMESSAGE_H

#include <memory>
#include <string>

//namespace MaceCore
//{

enum VehicleTypeENUM{
    VT_GENERIC,
    VT_FIXED_WING,
    VT_TRICOPTER,
    VT_QUADROTOR,
    VT_HEXACOPTER,
    VT_OCTOCOPTER,
    VT_HELICOPTER
};

class AbstractVehicleMessage{
public:
    enum MsgProtocolENUM{
        PROTOCOL_DJI,
        PROTOCOL_MAVLINK
    };

public:
    //AbstractVehicleMessage(const int &vehicleID);
    int getVehicleID() const;
    virtual ~AbstractVehicleMessage();

    //virtual void getProtocol() const = 0;
    //virtual void setProtocol(VehicleProtocolENUM protocol);
    virtual std::string getMessageType() const = 0;
    virtual std::string getDescription() const = 0;
protected:
    int m_VehicleID;
};

class VehicleMessage
{
public:
    VehicleMessage();
    void setDataObject(const std::shared_ptr<AbstractVehicleMessage> &data);

    std::shared_ptr<AbstractVehicleMessage> getDataObject() const;

private:
    std::shared_ptr<AbstractVehicleMessage> messageData;

};

//} //end of namespace MaceCore
#endif // VEHICLEMESSAGE_H
