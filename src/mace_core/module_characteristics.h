#ifndef MODULE_CHARACTERISTICS_H
#define MODULE_CHARACTERISTICS_H


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
};

struct ModuleCharacteristicCmp {
    bool operator()(const ModuleCharacteristic& lhs, const ModuleCharacteristic& rhs) const {
        if(lhs.Class < rhs.Class)
            return true;
        if(lhs.ID < rhs.ID) {
            return true;
        }
        return false;
    }
};


struct ModuleCharacteristicEQ {
    bool operator()(const ModuleCharacteristic& lhs, const ModuleCharacteristic& rhs) const {
        if(lhs.Class != rhs.Class)
            return false;
        if(lhs.ID != rhs.ID) {
            return false;
        }
        return true;
    }
};


}

#endif // MODULE_CHARACTERISTICS_H
