#ifndef DATA_GENERIC_ITEM_TEXT_H
#define DATA_GENERIC_ITEM_TEXT_H

#include <iostream>
#include <string>
#include "data/status_severity_type.h"

namespace DataGenericItem {

class DataGenericItem_Text
{
public:
    DataGenericItem_Text();

    DataGenericItem_Text(const DataGenericItem_Text &copyObj);


public:

    void setText(const std::string &dataString){
        this->dataString = dataString;
    }
    std::string getText() const{
        return dataString;
    }

    void setSeverity(const Data::StatusSeverityType &severity){
        this->severity = severity;
    }

    Data::StatusSeverityType getSeverity() const{
        return severity;
    }

public:
    void operator = (const DataGenericItem_Text &rhs)
    {
        this->severity = rhs.severity;
        this->dataString = rhs.dataString;
    }

    bool operator == (const DataGenericItem_Text &rhs) {
        if(this->severity != rhs.severity){
            return false;
        }
        if(this->dataString != rhs.dataString){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Text &rhs) {
        return !(*this == rhs);
    }

//    std::ostream& operator<<(std::ostream &out)
//    {
//        out<<"Status Text( Severity: "<<StatusSeverityTypeToString(severity)<<", Text: "<<dataString<<")";
//        return out;
//    }

protected:
    Data::StatusSeverityType severity;
    std::string dataString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_TEXT_H
