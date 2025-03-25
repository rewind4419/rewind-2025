#pragma once

#include <cstdio>

class Task
{
public:
    virtual ~Task(){};

    virtual void start() {}
    virtual bool loop() {
        return true;
    }
    virtual void end() {}
};

class TaskQueue
{
public:
    //List of the tasks
    Task** tasks;
    Task* activeTask;
    bool active = false;


    int queueSize;
    int queueStart = 0;
    int queueItemCount = 0;

    TaskQueue();
    void addTask(Task* task);
    void nextTask(int recursionDepth = 64);
    void update();

    void clear();
    void destroy();

private:
    Task* dequeue();
};
