#pragma once
#include "taskQueue.hpp"
#include <functional>

class DelayTask:public Task
{
    public:
        int duration;
        long startTime;
        DelayTask(int duration_ms);

        void start();
        bool loop();
        void dispose();
};

class CustomTask:public Task
{
    public:
        std::function<bool()> loopFunc;
        CustomTask(std::function<bool()> func);
        bool loop();
        void dispose();
};


class TaskList: public Task
{
    public:
        TaskQueue queue;
        TaskList();
        void addTask(Task* task);
        bool loop();
        void dispose();
};

class ForkTask: public Task
{
    public:
        Task* task0;
        Task* task1;

        bool task0Done = false;
        bool task1Done = false;
        
        ForkTask(Task* task0, Task* task1);
        void start();
        bool loop();
        void dispose();
};