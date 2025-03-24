#pragma once

#include <cstdio>

class Task
{
    public:
        virtual void start(){}
        virtual bool loop(){
            return true;
        }
        virtual void dispose(){}
};

class TaskQueue
{
    public:
        Task** tasks;
        Task* activeTask;
        bool active = false;


        int queueSize;
        int queueStart = 0;
        int queueItemCount = 0;

        TaskQueue();
        void addTask(Task* task);
        void nextTask();
        void update();

        void clear();
        void destroy();

    private:
        Task* dequeue();
};
