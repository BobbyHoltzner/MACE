#ifndef ARDUPILOTPOSITION_H
#define ARDUPILOTPOSITION_H

namespace Ardupilot{

struct LocalPosition{
    double xPosition;
    double yPosition;
    double zPosition;
    double xVelocity;
    double yVelocity;
    double zVelocity;

};

struct GlobalPosition{

};

class ArdupilotPosition
{
public:
    ArdupilotPosition();
};

} //end of namespace Ardupilot

#endif // ARDUPILOTPOSITION_H
