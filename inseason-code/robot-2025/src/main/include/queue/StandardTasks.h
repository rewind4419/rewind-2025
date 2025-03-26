#pragma once
#include "Queue.h"
#include <functional>

class DelayTask :public Task
{
public:
    int duration;
    long long startTime;
    DelayTask(int duration_ms);

    void Start() override;
    bool Loop() override;
    void End() override;
};

class CustomTask :public Task
{
public:
    std::function<bool()> loopFunc;
    CustomTask(std::function<bool()> func);
    bool Loop() override;
    void End() override;
};

class TaskList : public Task
{
public:
    Queue queue;
    TaskList();
    void AddTask(Task* task);
    bool Loop() override;
    void End() override;
};

class ForkTask : public Task
{
public:
    Task* task0;
    Task* task1;

    bool task0Done = false;
    bool task1Done = false;

    ForkTask(Task* task0, Task* task1);
    void Start() override;
    bool Loop() override;
    void End() override;
};
