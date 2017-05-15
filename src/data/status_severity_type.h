#ifndef STATUS_SEVERITY_TYPE_H
#define STATUS_SEVERITY_TYPE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class StatusSeverityType{
        STATUS_EMERGENCY,
        STATUS_ALERT,
        STATUS_CRITICAL,
        STATUS_ERROR,
        STATUS_WARNING,
        STATUS_NOTICE,
        STATUS_INFO,
        STATUS_DEBUG
    };

    inline std::string StatusSeverityTypeToString(const StatusSeverityType &state) {
        switch (state) {
        case StatusSeverityType::STATUS_EMERGENCY:
            return "EMERGENCY";
        case StatusSeverityType::STATUS_ALERT:
            return "ALERT";
        case StatusSeverityType::STATUS_CRITICAL:
            return "CRITICAL";
        case StatusSeverityType::STATUS_ERROR:
            return "ERROR";
        case StatusSeverityType::STATUS_WARNING:
            return "WARNING";
        case StatusSeverityType::STATUS_NOTICE:
            return "NOTICE";
        case StatusSeverityType::STATUS_INFO:
            return "INFO";
        case StatusSeverityType::STATUS_DEBUG:
            return "DEBUG";
        default:
            throw std::runtime_error("Unknown status severity seen");
        }
    }
}

#endif // STATUS_SEVERITY_TYPE_H
