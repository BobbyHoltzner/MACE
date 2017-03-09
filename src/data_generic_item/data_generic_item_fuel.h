#ifndef DATA_GENERIC_ITEM_FUEL_H
#define DATA_GENERIC_ITEM_FUEL_H

namespace DataGenericItem {

class DataGenericItem_Fuel
{
public:
    DataGenericItem_Fuel();

    void setBatteryVoltage(const double &voltage){
        this->voltage = voltage;
    }
    double getBatteryVoltage(){
        return voltage;
    }

    void setBatteryCurrent(const double &current){
        this->current = current;
    }
    double getBatteryCurrent(){
        return current;
    }

    void setBatteryRemaining(const double &batteryRemaing){
        this->batteryRemaing = batteryRemaing;
    }
    double getBatteryRemaining(){
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

protected:
    double voltage;
    double current;
    double batteryRemaing;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FUEL_H