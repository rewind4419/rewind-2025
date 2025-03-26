#include "queue/StandardTasks.h"
#include <chrono>

DelayTask::DelayTask(int duration_ms) {
    duration = duration_ms;
}

void DelayTask::Start() {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    long long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    startTime = currentTime;
}

bool DelayTask::Loop() {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    long long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    return currentTime >= startTime + duration;
}

void DelayTask::End() {

}

CustomTask::CustomTask(std::function<bool()> func) {
    loopFunc = func;
}

bool CustomTask::Loop() {
    return loopFunc();
}

void CustomTask::End() {

}

TaskList::TaskList() {}

void TaskList::AddTask(Task* task) {
    queue.AddTask(task);
}

bool TaskList::Loop() {
    queue.Update();
    if (queue.IsBusy()) { return false; }
    else { return true; }
}
void TaskList::End() {
    
}

ForkTask::ForkTask(Task* task0, Task* task1) :task0(task0), task1(task1) {}

void ForkTask::Start() {
    task0->Start();
    task1->Start();
}

bool ForkTask::Loop() {
    if (!task0Done)
    {
        task0Done = task0->Loop();
        if (task0Done) { task0->End(); }
    }
    if (!task1Done)
    {
        task1Done = task1->Loop();
        if (task1Done) { task1->End(); }
    }
    return task0Done && task1Done;
}

void ForkTask::End() {
    delete task0;
    delete task1;
}
