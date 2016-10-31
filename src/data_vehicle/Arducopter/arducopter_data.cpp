#include "arducopter_data.h"

using namespace Data;

ArducopterData::ArducopterData(){

}

VehicleProtocol ArducopterData::getProtocolDefinition() const
{
    return Data::PROTOCOL_ARDUPILOT;
}

ArducopterMessageDef ArducopterData::StringToMessageTypeEnum(const std::string &messageString){
    ArducopterMessageDef messageEnum;
    if(messageString == "Arducopter Properties")
        messageEnum = ArducopterMessageDef::MESSAGE_PROPERTIES;
    else
        throw new std::runtime_error("Unknown String seen");

    return messageEnum;
}

std::string ArducopterData::MessageTypeToString(const ArducopterMessageDef &messageType){
    std::string returnString;
    switch (messageType) {
    case ArducopterMessageDef::MESSAGE_PROPERTIES:
        returnString = "Arducopter Properties";
        break;
    default:
        throw new std::runtime_error("Unknown enumeration seen");
        break;
    }

    return returnString;
}
