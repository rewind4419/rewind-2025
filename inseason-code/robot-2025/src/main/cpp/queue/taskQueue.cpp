#include <cstdlib>
#include "queue/taskQueue.hpp"

int counter = 0;
TaskQueue::TaskQueue()
{
    tasks = (Task**)calloc(1024, sizeof(Task*));
    queueSize = 1024;
    counter++;
}

void TaskQueue::addTask(Task* task)
{
    if (queueItemCount < queueSize) {
        tasks[(queueStart + queueItemCount) % queueSize] = task;
        queueItemCount++;
    }
    else {
        //throw("QUEUE OVERFLOW!!!!");
        printf("Unable to add task, queue is full!\n");
    }
}

void TaskQueue::nextTask(int recursionDepth)
{
    if (active)
    {
        activeTask->end();
        delete activeTask;
    }

    if (queueItemCount != 0)
    {
        activeTask = dequeue();
        printf("what: %i\n", queueItemCount);
        activeTask->start();
        active = true;
        if (recursionDepth > 0)
        {
            if (activeTask->loop())
            {
                nextTask(recursionDepth - 1);
            }
        }
    }
    else
    {
        active = false;
    }
}

void TaskQueue::update()
{
    if (!active)
    {
        nextTask();
    }
    else if (active)
    {
        if (activeTask->loop())
        {
            nextTask();
        }
    }
}

Task* TaskQueue::dequeue()
{
    Task* out = nullptr;
    if (queueItemCount != 0)
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
    if (active)
    {
        activeTask->end();
        active = false;
        delete activeTask;
    }
    for (int i = 0; i < queueItemCount; i++)
    {
        tasks[(queueStart + i) % queueSize]->end();
        delete tasks[(queueStart + i) % queueSize];
    }
}
void TaskQueue::destroy()
{
    clear();
    free(tasks);
    counter--;
    printf("count:%i\n", counter);
}