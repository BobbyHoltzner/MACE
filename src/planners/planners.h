#ifndef PLANNERS_H
#define PLANNERS_H

#include "planners_global.h"

#include "spdlog/spdlog.h"

namespace mace{
namespace planners {

class PLANNERSSHARED_EXPORT Planners
{

public:
    Planners();

    virtual void solve() = 0;
protected:
    void createLog();

protected:
    std::shared_ptr<spdlog::logger> mLog;

};

} //end of namespace planners
} //end of namespace mace

#endif // PLANNERS_H
