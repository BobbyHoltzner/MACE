#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include "common/common.h"

#include "data_generic_state_item/state_item_components.h"


int main(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);


//    DataState::StateGlobalPosition globalOne(35.7479006,-78.8425295,100);
//    DataState::StateGlobalPosition globalTwo(globalOne);

//    if(globalOne == globalTwo)
//    {
//        std::cout<<"They are equal to eachother"<<std::endl;
//    }

//    DataState::StateGlobalPosition globalThree(globalOne);
//    globalThree.setCoordinateFrame(Data::CoordinateFrame::NWU);

//    if(globalOne == globalThree)
//    {
//        std::cout<<"They are equal to eachother"<<std::endl;
//    }else{
//        std::cout<<"The are not equal to eachother"<<std::endl;
//    }

//    Eigen::Vector3f newVector;
//    globalOne.translationTransformation(globalTwo, newVector);
//    std::cout<<"The new vector is: "<<newVector<<std::endl;
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
