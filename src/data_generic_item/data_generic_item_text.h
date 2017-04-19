#ifndef DATA_GENERIC_ITEM_TEXT_H
#define DATA_GENERIC_ITEM_TEXT_H

#include <iostream>
#include <string>

namespace DataGenericItem {

class DataGenericItem_Text
{
public:
    enum STATUS_SEVERITY{
        EMERGENCY,
        ALERT,
        CRITICAL,
        ERROR,
        WARNING,
        NOTICE,
        INFO,
        DEBUG
    };

    static inline std::string StatusSeverityToString(const STATUS_SEVERITY &state) {
        switch (state) {
        case STATUS_SEVERITY::EMERGENCY:
            return "EMERGENCY";
        case STATUS_SEVERITY::ALERT:
            return "ALERT";
        case STATUS_SEVERITY::CRITICAL:
            return "CRITICAL";
        case STATUS_SEVERITY::ERROR:
            return "ERROR";
        case STATUS_SEVERITY::WARNING:
            return "WARNING";
        case STATUS_SEVERITY::NOTICE:
            return "NOTICE";
        case STATUS_SEVERITY::INFO:
            return "INFO";
        case STATUS_SEVERITY::DEBUG:
            return "DEBUG";
        default:
            throw std::runtime_error("Unknown status severity seen");
        }
    }

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

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Status Text( Severity: "<<severity<<", Text: "<<dataString<<")";
        return out;
    }

protected:
    STATUS_SEVERITY severity;
    std::string dataString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_TEXT_H
