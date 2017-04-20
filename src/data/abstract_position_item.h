#ifndef ABSTRACT_POSITION_ITEM_H
#define ABSTRACT_POSITION_ITEM_H


namespace Data {

struct localPos{
    float x;
    float y;
    float z;

    void operator =(const localPos &rhs){

    }

    bool operator== (const localPos &rhs) const{

    }

    bool operator!= (const localPos &rhs) const{

    }
};

struct globalPos{
    float latitude;
    float longitude;
    float altitude;

    void operator =(const localPos &rhs){

    }

    bool operator== (const localPos &rhs) const{

    }

    bool operator!= (const localPos &rhs) const{

    }
};

template <class T>
class AbstractPosition
{
public:

public:
    T position;
};

}
#endif // ABSTRACT_POSITION_ITEM_H
