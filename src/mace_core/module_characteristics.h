#ifndef MODULE_CHARACTERISTICS_H
#define MODULE_CHARACTERISTICS_H

#include <unordered_set>

namespace MaceCore
{

enum class ModuleClasses
{
    VEHICLE_COMMS = 0,
    EXTERNAL_LINK,
    GROUND_STATION,
    PATH_PLANNING,
    ROS,
    RTA,
    SENSORS,
    NR_TYPES
};


struct ModuleCharacteristic
{
    int ID;
    ModuleClasses Class;

    bool operator== (const ModuleCharacteristic &rhs) const
    {
        if(this->Class != rhs.Class)
            return false;
        if(this->ID != rhs.ID) {
            return false;
        }
        return true;
    }

    bool operator!= (const ModuleCharacteristic &rhs) const
    {
        return !(*this == rhs);
    }

    bool operator < (const ModuleCharacteristic& rhs) const {
        if(this->Class < rhs.Class)
            return true;
        if(this->ID < rhs.ID) {
            return true;
        }
        return false;
    }
};




struct ModuleCharacteristicCmp {
    bool operator()(const ModuleCharacteristic& lhs, const ModuleCharacteristic& rhs) const {

        return lhs < rhs;
    }
};


}


namespace std
{
template <>
struct hash<MaceCore::ModuleCharacteristic>
{
    std::size_t operator()(const MaceCore::ModuleCharacteristic& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

        std::size_t const h1 ( std::hash<int>{}(k.ID) );
        std::size_t const h2 ( std::hash<int>{}((int)k.Class) );
        return h1 ^ (h2 << 1);
    }
};
}

#endif // MODULE_CHARACTERISTICS_H
