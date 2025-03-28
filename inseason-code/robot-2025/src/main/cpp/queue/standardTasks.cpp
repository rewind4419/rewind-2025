#include "queue/StandardTasks.h"
#include <chrono>
#include <frc/Timer.h>

DelayTask::DelayTask(double durationSeconds)
{
    this->duration = durationSeconds;
}

void DelayTask::Start() {
    this->startTime = frc::Timer::GetFPGATimestamp().value();
}

bool DelayTask::Loop() {
    return (frc::Timer::GetFPGATimestamp().value()) >= startTime + duration;
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
