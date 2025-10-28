#pragma once

#include "queue/Queue.h"
#include <frc/Timer.h>

class TimedTask : public Task {
public:
    Task* inner;
    double timeout;
    double start;

    TimedTask(Task* innerTask, double timeoutSeconds)
        : inner(innerTask), timeout(timeoutSeconds), start(0.0) {}

    void Start() override {
        start = frc::Timer::GetFPGATimestamp().value();
        inner->Start();
    }

    bool Loop() override {
        if (inner->Loop()) { return true; }
        double now = frc::Timer::GetFPGATimestamp().value();
        return (now - start) > timeout;
    }

    void End() override {
        inner->End();
        delete inner;
    }
};


