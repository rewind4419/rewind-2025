#include "queue/standardTasks.hpp"
#include <chrono>



DelayTask::DelayTask(int duration_ms) {
    duration = duration_ms;
}
void DelayTask::start() {
    auto now = std::chrono::system_clock::now();
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    startTime = currentTime;
}
bool DelayTask::loop() {
    auto now = std::chrono::system_clock::now();
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    return currentTime >= startTime + duration;
}
void DelayTask::end() {

}


CustomTask::CustomTask(std::function<bool()> func) {
    loopFunc = func;
}
bool CustomTask::loop() {
    return loopFunc();
}
void CustomTask::end() {

}



TaskList::TaskList() {}

void TaskList::addTask(Task* task) {
    queue.addTask(task);
}

bool TaskList::loop() {
    queue.update();
    return !queue.active;
}
void TaskList::end() {
    queue.destroy();
}

ForkTask::ForkTask(Task* task0, Task* task1) :task0(task0), task1(task1) {}
void ForkTask::start() {
    task0->start();
    task1->start();
}
bool ForkTask::loop() {
    if (!task0Done)
    {
        task0Done = task0->loop();
    }
    if (!task1Done)
    {
        task1Done = task1->loop();
    }
    return task0Done && task1Done;
}
void ForkTask::end() {
    task0->end();
    task1->end();
    delete task0;
    delete task1;
}
