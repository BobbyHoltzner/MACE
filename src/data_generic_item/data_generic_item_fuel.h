#ifndef DATA_GENERIC_ITEM_FUEL_H
#define DATA_GENERIC_ITEM_FUEL_H

#include <iostream>

namespace DataGenericItem {

class DataGenericItem_Fuel
{
public:
    DataGenericItem_Fuel();

    DataGenericItem_Fuel(const DataGenericItem_Fuel &copyObj);


    void setBatteryVoltage(const double &voltage){
        this->voltage = voltage;
    }
    double getBatteryVoltage() const{
        return voltage;
    }

    void setBatteryCurrent(const double &current){
        this->current = current;
    }
    double getBatteryCurrent() const{
        return current;
    }

    void setBatteryRemaining(const double &batteryRemaing){
        this->batteryRemaing = batteryRemaing;
    }
    double getBatteryRemaining() const{
        return batteryRemaing;
    }

public:
    void operator = (const DataGenericItem_Fuel &rhs)
    {
        this->voltage = rhs.voltage;
        this->current = rhs.current;
        this->batteryRemaing = rhs.batteryRemaing;
    }

    bool operator == (const DataGenericItem_Fuel &rhs) {
        if(this->voltage != rhs.voltage){
            return false;
        }
        if(this->current != rhs.current){
            return false;
        }
        if(this->batteryRemaing != rhs.batteryRemaing){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Fuel &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Vehicle Battery( Voltage: "<<voltage<<", Current: "<<current<<", Remaining %: "<<batteryRemaing<<")";
        return out;
    }

protected:
    double voltage;
    double current;
    double batteryRemaing;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FUEL_H
