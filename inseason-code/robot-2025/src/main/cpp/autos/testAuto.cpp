#include "Robot.h"

#include "Config.h"

#include "util/maths.h"

#include "Hardware.h"
#include "State.h"


void Robot::yeetAuto()
{
    printf("hello is the auto??\n");
    queue.AddTask(new SwerveWaypointTask(&pather,frc::Pose2d(15.52_m,5.76_m,0.0_rad),2.0,0.05,0.03,3.5,3.5));
    queue.AddTask(new SwerveLockWheelsTask(&pather));
}