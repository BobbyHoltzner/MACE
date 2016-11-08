#ifndef ARDUCOPTERMAIN_H
#define ARDUCOPTERMAIN_H

#include <iostream>

#include "arducopter_collection.h"

namespace Data {

class ArducopterMain
{
public:
    ArducopterMain();

public:
    ArducopterAttitude mAttitude;
    ArducopterGPS mGPS;
private:

};

} //end of namespace Data
#endif // ARDUCOPTERMAIN_H
