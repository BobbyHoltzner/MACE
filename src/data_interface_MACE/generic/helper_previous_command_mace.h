#ifndef HELPER_PREVIOUS_COMMAND_MACE_H
#define HELPER_PREVIOUS_COMMAND_MACE_H

#include "helper_previous_transmission_base_mace.h"

namespace DataInterface_MACE {

enum commandItemEnum{
    COMMAND_SHORT,
    COMMAND_LONG
};

template <class T>
class PreviousCommand : public PreviousTransmissionBase<commandItemEnum>
{
public:
    PreviousCommand(const commandItemEnum &objType, const T &data):
        PreviousTransmissionBase(objType), obj(data)
    {

    }

    PreviousCommand(const PreviousCommand &rhs)
    {
        this->obj = rhs.obj;
        this->type = rhs.type;
    }

    void setData(const T &data)
    {
        this->obj = data;
    }

    T getData() const
    {
        return this->obj;
    }
private:
    T obj;
};

} //end of namespace DataInterfaceMACE

#endif // HELPER_PREVIOUS_COMMAND_MAVLINK_H
