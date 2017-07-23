#ifndef HELPER_PREVIOUS_TRANSMISSION_MACE_H
#define HELPER_PREVIOUS_TRANSMISSION_MACE_H

#include "helper_previous_transmission_base_mace.h"

namespace DataInterface_MACE {

enum commsItemEnum{
    ITEM_RXLIST,
    ITEM_RXITEM,
    ITEM_TXCOUNT,
    ITEM_TXITEM
};

template <class T>
class PreviousTransmission : public PreviousTransmissionBase<commsItemEnum>
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

} //end of namespace DataInterface_MACE

#endif // HELPER_PREVIOUS_TRANSMISSION_MACE_H
