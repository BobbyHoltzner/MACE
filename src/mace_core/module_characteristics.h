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

}

#endif // MODULE_CHARACTERISTICS_H
