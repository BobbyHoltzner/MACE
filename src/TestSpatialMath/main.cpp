#include <iostream>

#include "data_generic_state_item/state_item_components.h"

int main(int argc, char *argv[])
{
    DataState::StateGlobalPosition globalOne(35.7479006,-78.8425295,100);
    DataState::StateGlobalPosition globalTwo(35.7481134,-78.8422962,250);

    double bearing = globalOne.bearingBetween(globalTwo);
    double distance = globalOne.distanceBetween2D(globalTwo);
    std::cout<<"The distance between is: "<<distance<<std::endl;
    double pi = 3.14159265358979323846;
    double degrees = bearing * (180.0/pi);
    std::cout<<"The bearing between is: "<<degrees<<std::endl;

    return 0;
}
