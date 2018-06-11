#include <QCoreApplication>

#include "base/geometry/polygon_2DC.h"

using namespace mace ;
using namespace geometry;

const char kPathSeparator =
        #ifdef _WIN32
        '\\';
#else
        '/';
#endif


class StateData{
public:

    StateData():
        value(4.5)
    {

    }
    StateData(const StateData &copy)
    {
        std::cout<<"The copy constructor of StateData is being called"<<std::endl;
        this->value = copy.value;
    }

    StateData& operator =(const StateData &rhs)
    {
        this->value = rhs.value;
        return *this;
    }

    double getValue() const {
        this->value;
    }

    void setValue(const double &newValue){
        this->value = newValue;
    }

private:
    double value;
};

class TestCopy
{
public:

    TestCopy()
    {
        data = new StateData();
    }

    TestCopy(const TestCopy &copy)
    {
        data = new StateData(*copy.data);
    }


    StateData getStateData() const
    {
        return *data;
    }


private:
    StateData* data;
};


class TestPointer
{
public:
    TestPointer() = default;
};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    Polygon_2DC newPolygon;
    newPolygon.initializePolygon(10);
    std::vector<int> indices = newPolygon.findUndefinedVertices();
    std::cout<<"The size of the vector is: "<<indices.size()<<std::endl;
    return a.exec();
}
