#ifndef MACE_API_H
#define MACE_API_H

#include <iostream>
#include <functional>
using namespace std;

class MaceAPI
{

public:
    MaceAPI();

    double addOne(double input);

    double getTestDouble();

private:
    double m_testDouble;

};

#endif // MACE_API_H
