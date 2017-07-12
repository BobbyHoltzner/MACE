#ifndef HELPER_PREVIOUS_TRANSMISSION_H
#define HELPER_PREVIOUS_TRANSMISSION_H

class PreviousTransmissionBase
{
public:
    enum commsItem{
        ITEM_RXLIST,
        ITEM_RXITEM,
        ITEM_TXCOUNT,
        ITEM_TXITEM
    };

    PreviousTransmissionBase(const commsItem &objType):
        type(objType)
    {

    }

    PreviousTransmissionBase(const PreviousTransmissionBase &rhs)
    {
        this->type = rhs.type;
    }

    void setType(const commsItem &objType)
    {
        this->type = objType;
    }

    commsItem getType() const
    {
        return this->type;
    }
private:
    commsItem type;
};

template <class T>
class PreviousTransmission : public PreviousTransmissionBase
{
public:
    PreviousTransmission(const commsItem &objType, const T &data):
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
