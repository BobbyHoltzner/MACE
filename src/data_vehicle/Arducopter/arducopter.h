#ifndef ARDUCOPTER_H
#define ARDUCOPTER_H

#include <iostream>
#include <string>

namespace Data{

enum ArducopterMessages
{
    VEHICLE_ATTITUDE //! A data message representing the vehicle attitude state as presented via arudcopter
};

class Arducopter
{
public:
    Arducopter();

    virtual ~Arducopter();

    virtual ArducopterMessages getMessageType() const = 0;

    virtual std::string getMessageDescription() const = 0;


    // //////////////////////////////////////////////////////////////
    // ////////////// STATIC METHODS ////////////////////////////////
    // //////////////////////////////////////////////////////////////

    static std::string MessageTypeToString(const ArducopterMessages &messageType);

    static ArducopterMessages StringToMessageTypeEnum(const std::string &messageString);

};

} //end of namespace data

#endif // ARDUCOPTER_H
