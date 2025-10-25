#pragma once

#include "queue/Queue.h"
#include <functional>

class ConditionalTask : public Task {
public:
    std::function<bool()> predicate;
    Task* inner;
    bool active = false;

    ConditionalTask(std::function<bool()> pred, Task* innerTask)
        : predicate(pred), inner(innerTask) {}

    void Start() override {
        active = predicate();
        if (active) { inner->Start(); }
    }
    bool Loop() override {
        if (!active) return true; // skip
        return inner->Loop();
    }
    void End() override {
        if (active) { inner->End(); delete inner; }
    }
};


