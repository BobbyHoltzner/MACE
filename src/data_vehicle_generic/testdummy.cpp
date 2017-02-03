#include "testdummy.h"

namespace DataVehicleGeneric
{

const char TestDummy_name[] = "testDummy";
const MaceCore::TopicComponentStructure TestDummy_structure = []{
    MaceCore::TopicComponentStructure structure;

    return structure;
}();

MaceCore::TopicDatagram TestDummy::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;

    return datagram;
}

void TestDummy::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

}

}
