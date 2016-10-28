#ifndef MACE_API_H
#define MACE_API_H

#include "module_RTA_NASAPhase2/module_rta_nasaphase2.h"


using namespace std;

class MaceAPI
{

public:
    MaceAPI();

    double addOne(double input);

    double getTestDouble();

    void newVehicle(const std::string &ID);

private:
    double m_testDouble;

    ModuleRTANASAPhase2* m_rtaModule;

};

#endif // MACE_API_H
