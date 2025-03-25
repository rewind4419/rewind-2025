#pragma once
#include "taskQueue.hpp"
#include <functional>

class DelayTask :public Task
{
public:
    int duration;
    long startTime;
    DelayTask(int duration_ms);

    void start() override;
    bool loop() override;
    void end() override;
};

class CustomTask :public Task
{
public:
    std::function<bool()> loopFunc;
    CustomTask(std::function<bool()> func);
    bool loop() override;
    void end() override;
};

class TaskList : public Task
{
public:
    TaskQueue queue;
    TaskList();
    void addTask(Task* task);
    bool loop() override;
    void end() override;
};

class ForkTask : public Task
{
public:
    Task* task0;
    Task* task1;

    bool task0Done = false;
    bool task1Done = false;

    ForkTask(Task* task0, Task* task1);
    void start() override;
    bool loop() override;
    void end() override;
};
