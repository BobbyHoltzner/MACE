#include "mace_api.h"

MaceAPI::MaceAPI()
{
    m_testDouble = 10.4;

    m_rtaModule = new ModuleRTANASAPhase2();

}

double MaceAPI::addOne(double input)
{
    double ret = input + 1.0;
    return ret;
}

double MaceAPI::getTestDouble()
{
    return m_testDouble;
}

void MaceAPI::newVehicle(const std::string &ID)
{
    m_rtaModule->NewVehicle(ID);
}
