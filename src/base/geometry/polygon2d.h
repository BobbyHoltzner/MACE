#ifndef POLYGON2D_H
#define POLYGON2D_H

#include "pose/base_position.h"

//this class should extend either

template <class POSITION>
class Polygon2D : public std::vector<POSITION>
{
public:
    Polygon2D();


};

#endif // POLYGON2D_H
