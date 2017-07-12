#ifndef HELPER_PREVIOUS_TRANSMISSION_H
#define HELPER_PREVIOUS_TRANSMISSION_H

#include "comms_item.h"

class PreviousTransmissionBase
{
public:

    PreviousTransmissionBase(const commsItemEnum &objType):
        type(objType)
    {

    }

    PreviousTransmissionBase(const PreviousTransmissionBase &rhs)
    {
        this->type = rhs.type;
    }

    void setType(const commsItemEnum &objType)
    {
        this->type = objType;
    }

    commsItemEnum getType() const
    {
        return this->type;
    }
private:
    commsItemEnum type;
};

template <class T>
class PreviousTransmission : public PreviousTransmissionBase
{
public:
    PreviousTransmission(const commsItemEnum &objType, const T &data):
        PreviousTransmissionBase(objType), obj(data)
    {

    }

    PreviousTransmission(const PreviousTransmission &rhs)
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

#endif // HELPER_PREVIOUS_TRANSMISSION_H
