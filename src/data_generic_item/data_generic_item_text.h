#ifndef DATA_GENERIC_ITEM_TEXT_H
#define DATA_GENERIC_ITEM_TEXT_H

#include <string>

namespace DataGenericItem {

class DataGenericItem_Text
{
public:
    enum STATUS_SEVERITY{
        STATUS_EMERGENCY,
        STATUS_ALERT,
        STATUS_CRITICAL,
        STATUS_ERROR,
        STATUS_WARNING,
        STATUS_NOTICE,
        STATUS_INFO,
        STATUS_DEBUG
    };

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

    void setSeverity(const STATUS_SEVERITY &severity){
        this->severity = severity;
    }
    STATUS_SEVERITY getSeverity() const{
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

protected:
    STATUS_SEVERITY severity;
    std::string dataString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_TEXT_H
