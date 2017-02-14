#include <iostream>
#include <math.h>
#include "data_generic_state_item/state_item_components.h"
#include <Eigen/Dense>

int main(int argc, char *argv[])
{
    DataState::StateGlobalPosition globalOne(35.7479006,-78.8425295,100);
    DataState::StateGlobalPosition globalTwo(35.7481134,-78.8422962,250);

    Eigen::Vector3f newVector;
    globalOne.translationTransformation(globalTwo, newVector);
    std::cout<<"The new vector is: "<<newVector<<std::endl;
//    double bearing = globalOne.bearingBetween(globalTwo);
//    double distance = globalOne.distanceBetween2D(globalTwo);

//    double distanceX = distance * sin(bearing);
//    std::cout<<"The distance in the X direction is: "<<distanceX<<std::endl;
//    double distanceY = distance * cos(bearing);
//    std::cout<<"The distance in the Y direction is: "<<distanceY<<std::endl;
//    double distanceZ = globalOne.deltaAltitude(globalTwo);
//    std::cout<<"The distance in the Z direction is: "<<distanceZ<<std::endl;

    return 0;
}
