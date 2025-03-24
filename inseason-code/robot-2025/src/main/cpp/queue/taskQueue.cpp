#include <cstdlib>
#include "queue/taskQueue.hpp"


TaskQueue::TaskQueue()
{
    tasks = (Task**)calloc(1024, sizeof(Task*));
    queueSize = 1024;
}

void TaskQueue::addTask(Task* task)
{
    if(queueItemCount < queueSize){
        tasks[(queueStart+queueItemCount)%queueSize] = task;
        queueItemCount ++;
    }
    else {
        throw("QUEUE OVERFLOW!!!!");
    }
}

void TaskQueue::nextTask()
{
    if(active)
    {
        activeTask->dispose();
    }

    if(queueItemCount != 0)
    {
        activeTask = dequeue();
        activeTask->start();
        active = true;
        if(activeTask->loop())
        {
            nextTask();
        }
    }
    else 
    {
        active = false;
    }
}

void TaskQueue::update()
{
    if(!active)
    {
        nextTask();
    }
    if(active)
    {
        if(activeTask->loop())
        {
            nextTask();
        }
    }
}

Task* TaskQueue::dequeue()
{
    Task* out;
    if(queueItemCount != 0)
    {
        out = tasks[queueStart];
        queueItemCount--;
        queueStart++;
        queueStart %= queueSize;
    }
    
    return out;
}

void TaskQueue::clear()
{
    if(active)
    {
        activeTask->dispose();
        active = false;
    }
    for(int i = 0; i < queueItemCount; i++)
    {
        tasks[(queueStart + i) % queueSize]->dispose();
    }
}
void TaskQueue::destroy()
{
    clear();
    free(tasks);
}