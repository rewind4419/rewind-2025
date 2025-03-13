# Log

3/13

I think the RobotState.cpp thing is actually not needed
because we can make commands non-interruptable
So there's no need to have a check that something is
happening, because if something is then there
is a command being run which can't be cancelled.
Instead, i'll just have it set the robot state
at the start of the sequence
- Sherwin